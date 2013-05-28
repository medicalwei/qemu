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
#include <sys/ipc.h>
#include <sys/shm.h>
#endif

#define DEBUG 1

#define MEMLINK_UNUSED 0
#define MEMLINK_USED 1
#define MEMLINK_SHMMAX 33554432 /* default value orz */

typedef struct Memlink{
	void *host_memory;
	void *offseted_host_memory;
	unsigned int num_gfns;
	uint32_t *gfns;
	unsigned int size;
	unsigned int offset;
	Memlink *next;
} Memlink;

typedef struct MemlinkMapItem
{
	ShmInfo *shm;
	unsigned int offset;
	unsigned int usedcount;
	void *orig;
} MemlinkMapItem;

typedef struct ShmInfo
{
	int id;
	void *mem;
	void *orig_mem;
	unsigned int usedcount;
} ShmInfo;


typedef struct VirtIOMemlink
{
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


inline void * gfn_to_hva(uint32_t gfn)
{
	MemoryRegionSection section;
	ram_addr_t pa;

	pa = (ram_addr_t) gfn << VIRTIO_MEMLINK_PFN_SHIFT;
	section = memory_region_find(get_system_memory(), pa, 1);

	if (!section.size || !memory_region_is_ram(section.mr)){
		return NULL;
	}

	return memory_region_get_ram_ptr(section.mr) +
		section.offset_within_region;
}

void * get_shared_memory(VirtIOMemlink *vml, uint32_t gfn)
{
	MemlinkMapItem *item = &(vml->map[gfn]);
	if (unlikely(item->shm != NULL)) {
		item->usedcount += 1;
		return item->shm->mem + item->offset;
	}

	if (unlikely(shm_next.shm == NULL)) {
		ShmInfo* shminfo = (ShmInfo *) malloc(sizeof ShmInfo);
		shminfo->id = shmget(IPC_PRIVATE, MEMLINK_SHMMAX,
				IPC_CREAT | SHM_NORESERVE | IPC_EXCL | 0666);
		if (shminfo->id == -1){
			free(shminfo);
			return NULL;
		}
		shminfo->mem = shmat(item->shmid, NULL, 0);
		/* TODO: check memory leak with valloc then mremap */
		shminfo->orig_mem = valloc(MEMLINK_SHMMAX);
		shminfo->usedcount = 0;

		vml->shm_next.shm = shminfo;
		vml->shm_next.offset = 0;
	}

	item->shm = vml->shm_next.shm;
	item->offset = vml->shm_next.offset;
	item->usedcount += 1;
	item->shm->usedcount += 1;

	void * qemu_hva = gfn_to_hva(gfn);

	mremap(qemu_hva, VIRTIO_MEMLINK_PAGE_SIZE, VIRTIO_MEMLINK_PAGE_SIZE,
			MREMAP_MAYMOVE | MREMAP_FIXED,
			item->shm->orig_mem + item->offset);
	
	/* http://lkml.indiana.edu/hypermail/linux/kernel/0401.1/0819.html 
	 * shmat+mremap with old_len=0 technique from DOSEMU. */
	mremap(item->shm->mem+item->offset, 0, VIRTIO_MEMLINK_PAGE_SIZE,
			MREMAP_MAYMOVE | MREMAP_FIXED, qemu_hva);

	vml->shm_next.offset += VIRTIO_MEMLINK_PAGE_SIZE;
	if (vml->shm_next.offset >= MEMLINK_SHMMAX){
		vml->shm_next.shm = NULL;
	}

	return item->shm->mem + item->offset;
}

void put_shared_memory(VirtIOMemlink *vml, unsigned int gfn)
{
	MemlinkMapItem *item = &(vml->map[gfn]);
	MemlinkMapItem *shminfo = item->shm;

	if (unlikely(item->usedcount == 0)){
		return;
	}

	item->usedcount -= 1;

	if (unlikely(item->usedcount > 0)){
		return;
	}

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
		shmdt(shminfo->mem);
		shmctl(shminfo->id, IPC_RMID, NULL);
		if (vml->shm_next.shm == shminfo){
			vml->shm_next.shm = NULL;
		}
		free(shminfo);
	}
}

static void virtio_memlink_link_address(VirtIOMemlink *vml, Memlink *ml)
{
	unsigned long mem_size = ml->num_pfns << VIRTIO_MEMLINK_PFN_SHIFT;
	int i;

	ml->host_memory = valloc(mem_size);

	for (i=0; i<ml->num_pfns; i++) {
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

	for (i=0; i<ml->num_pfns; i++) {
		put_shared_memory(vml, ml->gfns[i]);
	}
}

static void virtio_memlink_handle_create(VirtIODevice *vdev, VirtQueue *vq)
{
	VirtQueueElement elem;
	VirtIOMemlink *vml = DO_UPCAST(VirtIOMemlink, vdev, vdev);

	while (virtqueue_pop(vq, &elem)) {
		Memlink *ml;
		int memlink_id;
		int i;

		memlink_id = virtio_get_unused_memlink(vml);
		ml = malloc(sizeof(Memlink));
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

		ml->gfns = malloc(sizeof(uint32_t) * ml->num_gfns);

		for (i=0; i<ml->num_gfns; i++) {
			ml->gfns[i] = ldl_p(elem.out_sg[2].iov_base +
					(sizeof(uint32_t)*i));
		}

		virtio_memlink_link_address(vml, ml);
		ml->offseted_host_memory = ml->host_memory + ml->offset;

		Memlink *orig_memlink_head = vml->memlink_head;
		vml->memlink_head = ml;
		ml->next = orig_memlink_head;

		/* this is test area. TODO: remove test area */
		for (i=0; i<ml->size/4; i+=1024) {
			printf("%d ", *((int *) ml->offseted_host_memory+i));
		}
		printf("\n");

		for (i=0; i<ml->size/4; i++) {
			*((int *) ml->offseted_host_memory + i) =
				ml->size/4 - i;
		}

		for (i=0; i<ml->size/4; i+=1024) {
			printf("%d ", *((int *) ml->offseted_host_memory+i));
		}
		printf("\n");

		stl_p(elem.in_sg[0].iov_base, memlink_id);

		virtqueue_push(vq, &elem, 0);
		virtio_notify(vdev, vq);
	}
}

static void virtio_memlink_handle_revoke(VirtIODevice *vdev, VirtQueue *vq)
{
	VirtQueueElement elem;
	VirtIOMemlink *vml = DO_UPCAST(VirtIOMemlink, vdev, vdev);

	while (virtqueue_pop(vq, &elem)) {
		int memlink_id;

		if (elem.out_sg[0].iov_len != sizeof(void *)){
			error_report("virtio-memlink invalid size header");
			continue;
		}

		void * offseted_host_memory = ldl_p(elem.out_sg[0].iov_base);

#if DEBUG
		printf("virtio-memlink revoke address: %d\n",
				offseted_host_memory);
#endif

		for (Memlink *ml = vml->memlink_head;
				ml != NULL; ml = ml->next){
			if (ml->offseted_host_memory == offseted_host_memory){
				break;
			}
		}

		if (ml == NULL) {
			error_report("virtio-memlink memlink at the offset not found");
			continue;
		}

		virtio_memlink_revoke_address(ml);

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
	int i;

	s = (VirtIOMemlink *)virtio_common_init("virtio-memlink",
			VIRTIO_ID_MEMLINK,
			0, sizeof(VirtIOMemlink));

	s->vdev.get_config = virtio_memlink_get_config;
	s->vdev.set_config = virtio_memlink_set_config;
	s->vdev.get_features = virtio_memlink_get_features;

	s->create_vq = virtio_add_queue(&s->vdev, 1024, virtio_memlink_handle_create);
	s->revoke_vq = virtio_add_queue(&s->vdev, 1024, virtio_memlink_handle_revoke);

	s->qdev = dev;
	register_savevm(dev, "virtio-memlink", -1, 1,
			virtio_memlink_save, virtio_memlink_load, s);

	for(i=0; i<MEMLINK_MAX_LINKS; i++){
		s->memlinks[i].status = MEMLINK_UNUSED;
	}
#if DEBUG
	printf("virtio_memlink_init\n");
#endif
	return &s->vdev;
}

void virtio_memlink_exit(VirtIODevice *vdev)
{
	VirtIOMemlink *s = DO_UPCAST(VirtIOMemlink, vdev, vdev);

	unregister_savevm(s->qdev, "virtio-memlink", s);
	virtio_cleanup(vdev);
#if DEBUG
	printf("virtio_memlink_exit\n");
#endif
}
