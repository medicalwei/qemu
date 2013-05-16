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
	int status;
	void *host_memory; /* pointer to exchanged host memory. */
	void *linked_hva; /* offseted address of memlink */
	void **hvas; /* pointer to pointers to HVAs. */
	size_t size;
	size_t num_pfns;
	int *seg_ids; /* default 32M is not enough at all */
	int offset;
} Memlink;

typedef struct VirtIOMemlink
{
	VirtIODevice vdev;
	VirtQueue *create_vq;
	VirtQueue *revoke_vq;
	Memlink memlinks[MEMLINK_MAX_LINKS];
	VirtQueueElement stats_vq_elem;
	DeviceState *qdev;
	uint32_t state_count;
} VirtIOMemlink;

static int virtio_get_unused_memlink(struct VirtIOMemlink *vml)
{
	int i;
	for(i=0; i<MEMLINK_MAX_LINKS; i++){
		if(vml->memlinks[i].status == MEMLINK_UNUSED){
			return i;
		}
	}
	return -ENOSPC;
}

static void virtio_memlink_link_address(struct Memlink *ml)
{
	unsigned long seg_count;
	unsigned long mem_size = ml->num_pfns << VIRTIO_MEMLINK_PFN_SHIFT;
	unsigned long i;

	if (ml->status != MEMLINK_UNUSED) {
		error_report("virtio-memlink requesting link on used memory");
		/* TODO: not to exit the program. instead just return message */
		exit(1);
	}

	ml->status = MEMLINK_USED;

	seg_count = mem_size/MEMLINK_SHMMAX + ((mem_size%MEMLINK_SHMMAX>0)?1:0);
	ml->seg_ids = malloc(sizeof(int)*seg_count);
	ml->host_memory = valloc(mem_size);

	for(i=0; i<seg_count; i++){
		unsigned long seg_start, seg_size, page_per_seg, page_start, j;
		void *seg_memory, *host_seg_memory, *original_memory;

		printf("%lu\n", i);

		seg_start = i*MEMLINK_SHMMAX;
		page_start = seg_start >> VIRTIO_MEMLINK_PFN_SHIFT;
		seg_size = mem_size-seg_start;
		if (seg_size > MEMLINK_SHMMAX) {
			seg_size = MEMLINK_SHMMAX;
		}
		page_per_seg = seg_size >> VIRTIO_MEMLINK_PFN_SHIFT;

		ml->seg_ids[i] = shmget(IPC_PRIVATE, seg_size, IPC_CREAT | SHM_NORESERVE | IPC_EXCL | 0666);
		if (ml->seg_ids[i] < 0){
			error_report("virtio-memlink shmget error");
			perror("error");
			exit(1);
		}

		/* ask for 2 pointers, one for host, one for guest */
		seg_memory = shmat(ml->seg_ids[i], NULL, 0);
		host_seg_memory = shmat(ml->seg_ids[i], NULL, 0);
		if (seg_memory == (void *) -1 || host_seg_memory == (void*) -1){
			error_report("virtio-memlink shmat error");
			perror("error");
			exit(1);
		}

		original_memory = valloc(seg_size);

		for (j=0; j<page_per_seg; j++) {
			printf("j:%lu, page_start+j:%lu, ml->hvas: %p\n", j, page_start+j, ml->hvas[page_start+j]);
			mremap(ml->hvas[page_start+j], 4096, 4096, MREMAP_MAYMOVE | MREMAP_FIXED, original_memory+(j<<VIRTIO_MEMLINK_PFN_SHIFT));
			mremap(seg_memory+(j<<VIRTIO_MEMLINK_PFN_SHIFT), 4096, 4096, MREMAP_MAYMOVE | MREMAP_FIXED, ml->hvas[page_start+j]);
		}

		memcpy(host_seg_memory, original_memory, seg_size);
		mremap(host_seg_memory, seg_size, seg_size, MREMAP_MAYMOVE | MREMAP_FIXED, ml->host_memory+seg_start);
		free(original_memory);
	}
}

static void virtio_memlink_revoke_address(struct Memlink *ml)
{
	unsigned long seg_count;
	unsigned long mem_size = ml->num_pfns << VIRTIO_MEMLINK_PFN_SHIFT;
	unsigned long i;

	if (ml->status != MEMLINK_USED) {
		error_report("virtio-memlink revoking link on unused memory");
		exit(1);
	}

	seg_count = mem_size/MEMLINK_SHMMAX + ((mem_size%MEMLINK_SHMMAX>0)?1:0);

	for(i=0; i<seg_count; i++){
		unsigned long seg_start, seg_size, page_start, j;
		void *seg_memory, *host_seg_memory, *original_memory;

		seg_start = i*MEMLINK_SHMMAX;
		page_start = seg_start >> 12;
		seg_size = mem_size-seg_start;
		if (seg_size > MEMLINK_SHMMAX) {
			seg_size = MEMLINK_SHMMAX;
		}

		host_seg_memory = valloc(seg_size);
		seg_memory = valloc(seg_size);
		original_memory = valloc(seg_size);

		mremap(ml->host_memory+seg_start, seg_size, seg_size, MREMAP_MAYMOVE | MREMAP_FIXED, host_seg_memory);
		memcpy(original_memory, host_seg_memory, seg_size);

		for (j=0; j<ml->num_pfns; j++) {
			mremap(ml->hvas[page_start+j], 4096, 4096, MREMAP_MAYMOVE | MREMAP_FIXED, seg_memory+(j<<VIRTIO_MEMLINK_PFN_SHIFT));
			mremap(original_memory+(j<<VIRTIO_MEMLINK_PFN_SHIFT), 4096, 4096, MREMAP_MAYMOVE | MREMAP_FIXED, ml->hvas[page_start+j]);
		}

		/* remove shared memory */
		shmdt(host_seg_memory);
		shmdt(seg_memory);
		shmctl(ml->seg_ids[i], IPC_RMID, NULL);

		free(ml->hvas);
	}

	ml->status = MEMLINK_UNUSED;
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
		ml = &vml->memlinks[memlink_id];
		ml->size = ldl_p(elem.out_sg[0].iov_base);

		ml->offset = ldl_p(elem.out_sg[1].iov_base);
		if (ml->offset < 0 || ml->offset >= VIRTIO_MEMLINK_PAGE_SIZE) {
			error_report("virtio-memlink invalid offset");
#if DEBUG
			printf("offset %d\n", ml->offset);
#endif
			exit(1);
		}

		ml->num_pfns = (ml->size + ml->offset)/VIRTIO_MEMLINK_PAGE_SIZE;
		if ((ml->size + ml->offset)%VIRTIO_MEMLINK_PAGE_SIZE > 0){
			ml->num_pfns += 1;
		}

		if (elem.out_sg[2].iov_len != sizeof(uint32_t) * ml->num_pfns) {
			error_report("virtio-memlink invalid size");
#if DEBUG
			printf("size %lu, offset %d, iov_len %lu\n", ml->size, ml->offset, elem.out_sg[2].iov_len);
#endif
			exit(1);
		}

		ml->hvas = malloc(sizeof(void *) * ml->num_pfns);

		for (i=0; i<ml->num_pfns; i++) {
			uint32_t gfn;
			MemoryRegionSection section;
			ram_addr_t pa;

			gfn = ldl_p(elem.out_sg[2].iov_base+(sizeof(uint32_t)*i));
			pa = (ram_addr_t) gfn << VIRTIO_MEMLINK_PFN_SHIFT;
			section = memory_region_find(get_system_memory(), pa, 1);

			if (!section.size || !memory_region_is_ram(section.mr)){
				error_report("virtio-memlink memory_region_find error");
				exit(1);
			}

			ml->hvas[i] = memory_region_get_ram_ptr(section.mr) + section.offset_within_region;
		}

		virtio_memlink_link_address(ml);
		ml->linked_hva = ml->host_memory + ml->offset;

		/* this is test area. TODO: remove test area */
		for (i=0; i<ml->size/4; i++) {
			printf("%d ", *((int *) ml->linked_hva+i));
		}
		printf("\n");

		for (i=0; i<ml->size/4; i++) {
			*((int *) ml->linked_hva + i) = ml->size/4 - i;
		}

		for (i=0; i<ml->size/4; i++) {
			printf("%d ", *((int *) ml->linked_hva+i));
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

		if (elem.out_sg[0].iov_len != sizeof(int)){
			error_report("virtio-memlink invalid size header");
			exit(1);
		}

		memlink_id = ldl_p(elem.out_sg[0].iov_base);

		if (memlink_id < 0 || memlink_id > MEMLINK_MAX_LINKS) {
			error_report("virtio-memlink invalid id");
			exit(1);
		}

#if DEBUG
		printf("virtio-memlink revoke id: %d\n", memlink_id);
#endif

		virtio_memlink_revoke_address(&vml->memlinks[memlink_id]);

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
