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

typedef struct Memlink{
    void * host_pointer; /* pointer to exchanged host memory. */
    void ** guest_pointers; /* pointers to HVAs. */
    uint32_t size;
    uint32_t max_size;
} Memlink;

typedef struct VirtIOMemlink
{
    VirtIODevice vdev;
    VirtQueue *vq;
    Memlink memlink;
    VirtQueueElement stats_vq_elem;
    DeviceState *qdev;
    uint32_t state_count;
} VirtIOMemlink;

typedef struct MemlinkReturn{
    uint32_t fn;
    int32_t val;
} MemlinkCreateReturn;

static void virtio_memlink_link_address(void * page_addr)
{
}

static void virtio_memlink_handle_output(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement elem;
    MemoryRegionSection section;
    uint32_t pfn;

    while (virtqueue_pop(vq, &elem)) {
        size_t offset = 0;
        while (iov_to_buf(elem.out_sg, elem.out_num, offset, &pfn, 4) == 4) {
            ram_addr_t pa;
            ram_addr_t addr;

            pa = (ram_addr_t)ldl_p(&pfn) << VIRTIO_MEMLINK_PFN_SHIFT;
            offset += 4;

            section = memory_region_find(get_system_memory(), pa, 1);
            if (!section.size || !memory_region_is_ram(section.mr))
                continue;

            addr = section.offset_within_region;
            virtio_memlink_link_address(memory_region_get_ram_ptr(section.mr) + addr);

            offset += 4;
        }
    }
}

static void virtio_memlink_get_config(VirtIODevice *vdev, uint8_t *config_data)
{
}

static void virtio_memlink_set_config(VirtIODevice *vdev, const uint8_t *config_data)
{
}

static uint32_t virtio_memlink_get_features(VirtIODevice *vdev, uint32_t f)
{
    return 0;
}

static void virtio_memlink_save(QEMUFile *f, void *opaque)
{
}

static int virtio_memlink_load(QEMUFile *f, void *opaque, int version_id)
{
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

    s->vq = virtio_add_queue(&s->vdev, 128, virtio_memlink_handle_output);

    s->qdev = dev;
    register_savevm(dev, "virtio-memlink", -1, 1,
                    virtio_memlink_save, virtio_memlink_load, s);

    return &s->vdev;
}

void virtio_memlink_exit(VirtIODevice *vdev)
{
    VirtIOMemlink *s = DO_UPCAST(VirtIOMemlink, vdev, vdev);

    unregister_savevm(s->qdev, "virtio-memlink", s);
    virtio_cleanup(vdev);
}
