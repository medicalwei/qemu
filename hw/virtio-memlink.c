#include "iov.h"
#include "qemu-common.h"
#include "virtio.h"
#include "pc.h"
#include "cpu.h"
#include "virtio-memlink.h"
#include "kvm.h"
#include "exec-memory.h"

#if defined(__linux__)
#include <sys/mman.h>
#endif

#define DEBUG 1

#define MEMLINK_UNUSED 0
#define MEMLINK_USED 1
#define MEMLINK_SHMMAX 67108864

typedef struct Memlink {
	void *host_memory;
	void *offseted_host_memory;
	unsigned int num_gfns;
	uint32_t *gfns;
	unsigned int size;
	unsigned int offset;
	struct Memlink *next, *pprev;
} Memlink;

typedef struct ShmInfo {
	int id;
	char filename[40];
	void *mem;
	void *orig_mem;
	unsigned int usedcount;
} ShmInfo;

typedef struct MemlinkMapItem {
	struct ShmInfo *shm;
	unsigned int offset;
	unsigned int usedcount;
	void *orig;
} MemlinkMapItem;

typedef struct VirtIOMemlink {
	VirtIODevice vdev;
	VirtQueue *create_vq;
	VirtQueue *revoke_vq;
	Memlink *memlink_head;
	MemlinkMapItem *map;
	MemlinkMapItem shm_next;
	VirtQueueElement stats_vq_elem;
	DeviceState *qdev;
	uint32_t state_count;
} VirtIOMemlink;

void * gfn_to_hva(uint32_t gfn, uint32_t *offset_fn);
void * get_shared_memory(VirtIOMemlink *vml, uint32_t gfn);
void put_shared_memory(VirtIOMemlink *vml, unsigned int gfn);

void * gfn_to_hva(uint32_t gfn, uint32_t *offset_fn)
{
	MemoryRegionSection section;
	ram_addr_t pa;

	pa = (ram_addr_t) gfn << VIRTIO_MEMLINK_PFN_SHIFT;
	section = memory_region_find(get_system_memory(), pa, 1);

	if (!section.size || !memory_region_is_ram(section.mr)){
		return NULL;
	}

	if (offset_fn != NULL) {
		*offset_fn = section.offset_within_region
			>> VIRTIO_MEMLINK_PFN_SHIFT;
	}

	return memory_region_get_ram_ptr(section.mr) +
		section.offset_within_region;
}

void * get_shared_memory(VirtIOMemlink *vml, uint32_t gfn)
{
	uint32_t offset_fn;
	void * qemu_hva = gfn_to_hva(gfn, &offset_fn);
	MemlinkMapItem *item = &vml->map[offset_fn];

	if (unlikely(item->shm != NULL)) {
		item->usedcount += 1;
		return item->shm->mem + item->offset;
	}

	if (unlikely(vml->shm_next.shm == NULL)) {
		ShmInfo* shminfo = (ShmInfo *) malloc(sizeof(ShmInfo));
		sprintf(shminfo->filename, "virtio-memlink_%x", rand());
		shminfo->id = shm_open(shminfo->filename, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);

		if (shminfo->id < 0){
			free(shminfo);
			return NULL;
		}

		if (ftruncate(shminfo->id, MEMLINK_SHMMAX) != 0){
			shm_unlink(shminfo->filename);
			free(shminfo);
			return NULL;
		}

		shminfo->mem = mmap(NULL, MEMLINK_SHMMAX, PROT_READ | PROT_WRITE,
			MAP_SHARED, shminfo->id, 0);
		shminfo->orig_mem = valloc(MEMLINK_SHMMAX);
		shminfo->usedcount = 0;

		vml->shm_next.shm = shminfo;
		vml->shm_next.offset = 0;
	}

	item->shm = vml->shm_next.shm;
	item->offset = vml->shm_next.offset;
	item->usedcount += 1;
	item->shm->usedcount += 1;

	mremap(qemu_hva, VIRTIO_MEMLINK_PAGE_SIZE, VIRTIO_MEMLINK_PAGE_SIZE,
			MREMAP_MAYMOVE | MREMAP_FIXED,
			item->shm->orig_mem + item->offset);
	
	mmap(qemu_hva, VIRTIO_MEMLINK_PAGE_SIZE, PROT_READ | PROT_WRITE,
			MAP_SHARED | MAP_FIXED , item->shm->id, item->offset);

	memcpy(qemu_hva, item->shm->orig_mem+item->offset,
			VIRTIO_MEMLINK_PAGE_SIZE);

	vml->shm_next.offset += VIRTIO_MEMLINK_PAGE_SIZE;
	if (vml->shm_next.offset >= MEMLINK_SHMMAX){
		vml->shm_next.shm = NULL;
	}

	return item->shm->mem + item->offset;
}

void put_shared_memory(VirtIOMemlink *vml, uint32_t gfn)
{
	uint32_t offset_fn;
	void * qemu_hva = gfn_to_hva(gfn, &offset_fn);
	MemlinkMapItem *item = &vml->map[offset_fn];
	ShmInfo *shminfo = item->shm;

	if (unlikely(item->usedcount == 0)){
		return;
	}

	item->usedcount -= 1;

	if (unlikely(item->usedcount > 0)){
		return;
	}

	memcpy(item->shm->orig_mem+item->offset, qemu_hva,
			VIRTIO_MEMLINK_PAGE_SIZE);

	mremap(item->shm->orig_mem + item->offset,
			VIRTIO_MEMLINK_PAGE_SIZE,
			VIRTIO_MEMLINK_PAGE_SIZE,
			MREMAP_MAYMOVE | MREMAP_FIXED, qemu_hva);

	item->shm = NULL;

	if (unlikely(shminfo->usedcount == 0)){
		return;
	}

	shminfo->usedcount -= 1;

	if (unlikely(shminfo->usedcount == 0)){
		shm_unlink(shminfo->filename);
		if (vml->shm_next.shm == shminfo){
			vml->shm_next.shm = NULL;
		}
		free(shminfo);
	}
}

static void virtio_memlink_link_address(VirtIOMemlink *vml, Memlink *ml)
{
	unsigned long mem_size = ml->num_gfns << VIRTIO_MEMLINK_PFN_SHIFT;
	int i;

	ml->host_memory = valloc(mem_size);

	for (i=0; i<ml->num_gfns; i++) {
		void * shmem = get_shared_memory(vml, ml->gfns[i]);
		uint32_t offset = i << VIRTIO_MEMLINK_PFN_SHIFT;
		mremap(shmem, 0, VIRTIO_MEMLINK_PAGE_SIZE,
				MREMAP_MAYMOVE | MREMAP_FIXED,
				ml->host_memory + offset);
	}
}

static void virtio_memlink_revoke_address(VirtIOMemlink *vml, Memlink *ml)
{
	int i;

	for (i=0; i<ml->num_gfns; i++) {
		put_shared_memory(vml, ml->gfns[i]);
	}
}

static void virtio_memlink_handle_create(VirtIODevice *vdev, VirtQueue *vq)
{
	VirtQueueElement elem;
	VirtIOMemlink *vml = DO_UPCAST(VirtIOMemlink, vdev, vdev);

	while (virtqueue_pop(vq, &elem)) {
		Memlink *ml;
		int i;

		ml = (Memlink *) malloc(sizeof(Memlink));
		ml->size = ldl_p(elem.out_sg[0].iov_base);

		ml->offset = ldl_p(elem.out_sg[1].iov_base);
		if (ml->offset >= VIRTIO_MEMLINK_PAGE_SIZE) {
			error_report("virtio-memlink invalid offset");
			free(ml);
			continue;
		}

		ml->num_gfns = (ml->size + ml->offset)/VIRTIO_MEMLINK_PAGE_SIZE;
		if ((ml->size + ml->offset)%VIRTIO_MEMLINK_PAGE_SIZE > 0){
			ml->num_gfns += 1;
		}

		if (elem.out_sg[2].iov_len != sizeof(uint32_t) * ml->num_gfns) {
			error_report("virtio-memlink invalid size");
			free(ml);
			continue;
		}

		ml->gfns = (uint32_t *) malloc(sizeof(uint32_t) * ml->num_gfns);

		for (i=0; i<ml->num_gfns; i++) {
			ml->gfns[i] = ldl_p(elem.out_sg[2].iov_base +
					(sizeof(uint32_t)*i));
		}

		virtio_memlink_link_address(vml, ml);
		ml->offseted_host_memory = ml->host_memory + ml->offset;

		Memlink *orig_memlink_head = vml->memlink_head;
		vml->memlink_head = ml;
		ml->next = orig_memlink_head;
		ml->pprev = NULL;
		if (ml->next != NULL){
			ml->next->pprev = ml;
		}

		stq_p(elem.in_sg[0].iov_base, (uint64_t) ml->offseted_host_memory);

		virtqueue_push(vq, &elem, 0);
		virtio_notify(vdev, vq);
	}
}

static void virtio_memlink_handle_revoke(VirtIODevice *vdev, VirtQueue *vq)
{
	VirtQueueElement elem;
	VirtIOMemlink *vml = DO_UPCAST(VirtIOMemlink, vdev, vdev);

	while (virtqueue_pop(vq, &elem)) {
		if (elem.out_sg[0].iov_len != sizeof(uint64_t)){
			error_report("virtio-memlink invalid size header");
			continue;
		}

		void * offseted_host_memory = (void *) ldq_p(elem.out_sg[0].iov_base);

#if DEBUG
		printf("virtio-memlink revoke address: %p\n",
				offseted_host_memory);
#endif

		Memlink *ml;
		for (ml = vml->memlink_head; ml != NULL; ml = ml->next){
			if (ml->offseted_host_memory == offseted_host_memory){
				break;
			}
		}

		if (ml == NULL) {
			error_report("virtio-memlink memlink at the offset not found");
			continue;
		}

		virtio_memlink_revoke_address(vml, ml);

		if (ml->pprev != NULL) {
			ml->pprev->next = ml->next;
		} else {
			vml->memlink_head = ml->next;
		}
		if (ml->next != NULL) {
			ml->next->pprev = ml->pprev;
		}
		free(ml);

		virtqueue_push(vq, &elem, 0);
		virtio_notify(vdev, vq);
	}
}

static void virtio_memlink_get_config(VirtIODevice *vdev, uint8_t *config_data)
{
#if DEBUG
	printf("virtio_memlink_get_config\n");
#endif
}

static void virtio_memlink_set_config(VirtIODevice *vdev, const uint8_t *config_data)
{
#if DEBUG
	printf("virtio_memlink_set_config\n");
#endif
}

static uint32_t virtio_memlink_get_features(VirtIODevice *vdev, uint32_t f)
{
#if DEBUG
	printf("virtio_memlink_get_features\n");
#endif
	return 0;
}

static void virtio_memlink_save(QEMUFile *f, void *opaque)
{
#if DEBUG
	printf("virtio_memlink_save\n");
#endif
}

static int virtio_memlink_load(QEMUFile *f, void *opaque, int version_id)
{
#if DEBUG
	printf("virtio_memlink_load\n");
#endif
	// TODO
	return 0;
}

VirtIODevice *virtio_memlink_init(DeviceState *dev)
{
	VirtIOMemlink *s;

	s = (VirtIOMemlink *)virtio_common_init("virtio-memlink",
			VIRTIO_ID_MEMLINK,
			0, sizeof(VirtIOMemlink));

	s->vdev.get_config = virtio_memlink_get_config;
	s->vdev.set_config = virtio_memlink_set_config;
	s->vdev.get_features = virtio_memlink_get_features;

	s->create_vq = virtio_add_queue(&s->vdev, 1024,
			virtio_memlink_handle_create);
	s->revoke_vq = virtio_add_queue(&s->vdev, 1024,
			virtio_memlink_handle_revoke);

	s->qdev = dev;
	register_savevm(dev, "virtio-memlink", -1, 1,
			virtio_memlink_save, virtio_memlink_load, s);

	/* TODO: deal with high memory */
	size_t map_size = ram_size/VIRTIO_MEMLINK_PAGE_SIZE
			*sizeof(MemlinkMapItem) * 2;
	s->map = (MemlinkMapItem *) malloc(map_size);
	if (s->map == NULL) {
		error_report("virtio-memlink: cannot allocate shm map.");
        	exit(1);
	}
	memset(s->map, 0, map_size);

	s->memlink_head = NULL;

#if DEBUG
	printf("virtio_memlink_init\n");
	printf("map_size: %lu\n", map_size);
#endif
	return &s->vdev;
}

void virtio_memlink_exit(VirtIODevice *vdev)
{
	VirtIOMemlink *s = DO_UPCAST(VirtIOMemlink, vdev, vdev);

	free(s->map);
	/* TODO: cleanup map, memlinks and shms */

	unregister_savevm(s->qdev, "virtio-memlink", s);
	virtio_cleanup(vdev);
#if DEBUG
	printf("virtio_memlink_exit\n");
#endif
}
