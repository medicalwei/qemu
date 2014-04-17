#include "iov.h"
#include "qemu-common.h"
#include "virtio.h"
#include "pci.h"

#include "memlink.h"
#include "virtio-memlink.h"

#define DEBUG 0

typedef struct MemlinkListItem
{
    Memlink ml;
    QLIST_ENTRY(MemlinkListItem) next;
} MemlinkListItem;

typedef struct VirtIOMemlink {
    VirtIODevice vdev;
    VirtQueue *create_vq;
    VirtQueue *revoke_vq;
    QLIST_HEAD(, MemlinkListItem) memlink_head;
    VirtQueueElement stats_vq_elem;
    DeviceState *qdev;
    uint32_t state_count;
} VirtIOMemlink;

static void virtio_memlink_handle_create(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement elem;
    VirtIOMemlink *vml = DO_UPCAST(VirtIOMemlink, vdev, vdev);

    while (virtqueue_pop(vq, &elem)) {
        MemlinkListItem *ml;

        ml = (MemlinkListItem *) malloc(sizeof(MemlinkListItem));

        ml->ml.offset = ldl_p(elem.out_sg[1].iov_base);
        if (ml->ml.offset >= TARGET_PAGE_SIZE) {
            error_report("virtio-memlink invalid offset");
            free(ml);
            continue;
        }

        ml->ml.num_gfns = elem.out_sg[2].iov_len / sizeof(uint32_t);
        ml->ml.gfns = (uint32_t *) malloc(elem.out_sg[2].iov_len);

        iov_to_buf(&elem.out_sg[2], 1, 0, &ml->ml.gfns, elem.out_sg[2].iov_len);

        memlink_link_address(&ml->ml);

        QLIST_INSERT_HEAD(&vml->memlink_head, ml, next);

        stq_p(elem.in_sg[0].iov_base, (uint64_t) ml->ml.offseted_host_memory);

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
        printf("virtio-memlink revoke address: %p\n", offseted_host_memory);
#endif

        MemlinkListItem *ml;

        QLIST_FOREACH(ml, &vml->memlink_head, next) {
            if (ml->ml.offseted_host_memory == offseted_host_memory){
                break;
            }
        }

        if (ml == NULL) {
            error_report("virtio-memlink memlink at the offset not found");
            continue;
        }

        memlink_unlink_address(&ml->ml);
        QLIST_REMOVE(ml, next);
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

    s->create_vq = virtio_add_queue(&s->vdev, 1024, virtio_memlink_handle_create);
    s->revoke_vq = virtio_add_queue(&s->vdev, 1024, virtio_memlink_handle_revoke);

    s->qdev = dev;
    register_savevm(dev, "virtio-memlink", -1, 1,
            virtio_memlink_save, virtio_memlink_load, s);

    QLIST_INIT(&s->memlink_head);

    memlink_init();

#if DEBUG
    printf("virtio_memlink_init\n");
    printf("map_size: %lu\n", map_size);
#endif
    return &s->vdev;
}

void virtio_memlink_exit(VirtIODevice *vdev)
{
    VirtIOMemlink *s = DO_UPCAST(VirtIOMemlink, vdev, vdev);

    unregister_savevm(s->qdev, "virtio-memlink", s);
    virtio_cleanup(vdev);

    memlink_exit();
#if DEBUG
    printf("virtio_memlink_exit\n");
#endif
}
