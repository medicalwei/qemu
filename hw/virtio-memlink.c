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

typedef struct Memlink{
	  int status;
    void * host_pointer; /* pointer to exchanged host memory. */
    void ** guest_pointers; /* pointers to HVAs. */
    size_t size;
    uint32_t max_size;
		MemoryRegion *mr;
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

static void virtio_memlink_link_address(struct Memlink *memlink)
{
	  int count;
		if (memlink->status != MEMLINK_UNUSED) {
			error_report("virtio-memlink requesting link on used memory");
			exit(1);
		}
		memlink->status = MEMLINK_USED;
	  memlink->host_pointer = valloc(4096*memlink->size); /* FIXME: page size constant */
	  void *dummy = valloc(4096*memlink->size); /* dummy memory */

		for(count=0; count<memlink->size; count++){
		    mremap(memlink->guest_pointers[count], 4096, 4096,
						   MREMAP_MAYMOVE | MREMAP_FIXED, memlink->host_pointer+count*4096);
		    mremap(dummy+count*4096, 4096, 4096,
						   MREMAP_MAYMOVE | MREMAP_FIXED, memlink->guest_pointers[count]);
		}
}

static void virtio_memlink_revoke_address(struct Memlink *memlink)
{
	  int count;
		if (memlink->status != MEMLINK_USED) {
			error_report("virtio-memlink revoking link on unused memory");
			exit(1);
		}

		for(count=0; count<memlink->size; count++){
		    mremap(memlink->host_pointer+count*4096, 4096, 4096,
						   MREMAP_MAYMOVE | MREMAP_FIXED, memlink->guest_pointers[count]);
		}

		free(memlink->guest_pointers);
		memlink->status = MEMLINK_UNUSED;
}

static void virtio_memlink_handle_create(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement elem;
    MemoryRegionSection section;
    VirtIOMemlink *vml = DO_UPCAST(VirtIOMemlink, vdev, vdev);

    while (virtqueue_pop(vq, &elem)) {
        int count = 0;
        uint32_t pfn;
				Memlink *ml;
				int memlink_id;

        #if DEBUG
            printf("link pfns:");
        #endif


				if (elem.out_sg[0].iov_len != sizeof(int) || elem.out_sg[1].iov_len != sizeof(uint32_t)){
					error_report("virtio-memlink invalid size header");
					exit(1);
				}

				memlink_id = ldl_p(elem.out_sg[0].iov_base);
				if (memlink_id < 0 || memlink_id > MEMLINK_MAX_LINKS) {
					error_report("virtio-memlink invalid id");
					exit(1);
				}

				ml = &vml->memlinks[memlink_id];

				ml->max_size = ldl_p(elem.out_sg[1].iov_base);
				if (elem.out_sg[2].iov_len != sizeof(uint32_t) * ml->max_size) {
					error_report("virtio-memlink invalid size");
					exit(1);
				}

				ml->guest_pointers = valloc(sizeof(ram_addr_t) * ml->max_size);
				ml->size = 0;

        #if DEBUG
            printf(" (size %u)", ml->max_size);
        #endif

				for (count = 0; count < ml->max_size; count++){
            ram_addr_t pa;
            ram_addr_t addr;

            pfn = ldl_p(elem.out_sg[2].iov_base+sizeof(pfn)*count);

            #if DEBUG
                printf(" %d", pfn);
            #endif

            pa = (ram_addr_t)ldl_p(&pfn) << VIRTIO_MEMLINK_PFN_SHIFT;

            section = memory_region_find(get_system_memory(), pa, 1);
            if (!section.size || !memory_region_is_ram(section.mr))
                continue;

						/* TODO: mr may not be the same in one request */
						ml->mr = section.mr;

            addr = section.offset_within_region;

						ml->guest_pointers[ml->size] = memory_region_get_ram_ptr(section.mr) + addr;
						ml->size += 1;
						if(ml->size >= ml->max_size) break;
        }

        #if DEBUG
            printf("\n");
        #endif

				printf("%d ", *((int *) ml->guest_pointers[0]));
				printf("%d ", *((int *) ml->guest_pointers[1]));
				printf("%d ", *((int *) ml->guest_pointers[2]));
				printf("%d\n", *((int *) ml->guest_pointers[3]));

        virtio_memlink_link_address(ml);

				// XXX: test
				printf("%d ", *((int *) ml->host_pointer));
				printf("%d ", *((int *) ml->host_pointer + 1024));
				printf("%d ", *((int *) ml->host_pointer + 2048));
				printf("%d\n", *((int *) ml->host_pointer + 3072));

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
