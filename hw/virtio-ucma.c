#include "iov.h"
#include "qemu-common.h"
#include "virtio.h"
#include "cpu.h"
#include "virtio-ucma.h"

#include <poll.h>
#include <rdma/rdma_user_cm.h>

#define RDMA_CMD_MAX_SIZE 512

#define DEBUG 0
#define DEBUG_PRINT_CURRENT_LINE \
    if (DEBUG) printf("%s:%d %s\n", __FILE__, __LINE__, __FUNCTION__); \


typedef struct VirtIOUCMA {
    VirtIODevice vdev;

    VirtQueue *write_vq;
    VirtQueue *device_vq;
    VirtQueue *poll_vq;

    DeviceState *qdev;
} VirtIOUCMA;

struct VirtQueueFdHandlerData{
    VirtIODevice *vdev;
    VirtQueue *vq;
    VirtQueueElement elem;
};

const char *virtucma_cmd_name[] = {
#define VIRTUCMA_EACH_CMD_NAME(__cmd) [__cmd] = #__cmd

    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_CREATE_ID),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_DESTROY_ID),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_BIND_ADDR),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_RESOLVE_ADDR),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_RESOLVE_ROUTE),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_QUERY_ROUTE),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_CONNECT),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_LISTEN),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_ACCEPT),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_REJECT),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_DISCONNECT),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_INIT_QP_ATTR),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_GET_EVENT),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_GET_OPTION),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_SET_OPTION),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_NOTIFY),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_JOIN_MCAST),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_LEAVE_MCAST),
    VIRTUCMA_EACH_CMD_NAME(RDMA_USER_CM_CMD_MIGRATE_ID)
};

static void virtucma_replace_response_if_any(struct rdma_ucm_cmd_hdr *hdr, void *payload, void *out)
{
#define VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(__cmd, __struct) \
    if (hdr->cmd == __cmd) {\
        ((struct __struct *) payload)->response = (__u64) out;\
        return;\
    }\

    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_CREATE_ID,    rdma_ucm_create_id)
    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_DESTROY_ID,   rdma_ucm_destroy_id)
    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_BIND_ADDR,    rdma_ucm_bind_addr)
    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_QUERY_ROUTE,  rdma_ucm_query_route)
    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_INIT_QP_ATTR, rdma_ucm_init_qp_attr)
    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_JOIN_MCAST,   rdma_ucm_join_mcast)
    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_LEAVE_MCAST,  rdma_ucm_destroy_id)
    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_GET_EVENT,    rdma_ucm_get_event)
    VIRTUCMA_CHECK_CMD_AND_REPLACE_RESPONSE(RDMA_USER_CM_CMD_MIGRATE_ID,   rdma_ucm_migrate_id)
    return;
}

static void virtucma_handle_write_dowrite(void *v){
    struct VirtQueueFdHandlerData *t = v;
    int fd, ret;
    void *out, *payload;
    char in[RDMA_CMD_MAX_SIZE];
    struct rdma_ucm_cmd_hdr *hdr = (struct rdma_ucm_cmd_hdr *) in;

    fd = ldl_p(t->elem.out_sg[0].iov_base);
    memcpy(in, t->elem.out_sg[1].iov_base, t->elem.out_sg[1].iov_len);
    payload = (void *) in + sizeof(*hdr);
    out = (void *) t->elem.in_sg[1].iov_base;

    virtucma_replace_response_if_any(hdr, payload, out);

    if (DEBUG)
        printf("DEBUG: write cmd: %s, fd: %d\n", virtucma_cmd_name[hdr->cmd], fd);

    ret = write(fd, in, t->elem.out_sg[1].iov_len);

    if (ret == -EAGAIN) {
        /* return earlier and redo it again. */
        return;
    }

    stl_p(t->elem.in_sg[0].iov_base, ret);

    virtqueue_push(t->vq, &t->elem, sizeof(int) + RDMA_CMD_MAX_SIZE);
    virtio_notify(t->vdev, t->vq);
    if (DEBUG)
        printf("DEBUG: end of %s, fd: %d\n", virtucma_cmd_name[hdr->cmd], fd);

    g_free(t);
    qemu_set_fd_handler(fd, NULL, NULL, NULL);
}

static void virtio_ucma_handle_write(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement elem;
    int fd;
    struct rdma_ucm_cmd_hdr *hdr;

    DEBUG_PRINT_CURRENT_LINE
    while(virtqueue_pop(vq, &elem)) {
        hdr = (struct rdma_ucm_cmd_hdr *) elem.out_sg[1].iov_base;
        struct VirtQueueFdHandlerData *t = g_malloc0(sizeof *t);
        fd = ldl_p(elem.out_sg[0].iov_base);
        t->vdev = vdev;
        t->vq = vq;
        memcpy(&t->elem, &elem, sizeof elem);

        if (hdr->cmd == RDMA_USER_CM_CMD_GET_EVENT) {
            qemu_set_fd_handler(fd, virtucma_handle_write_dowrite, NULL, (void *) t);
        } else {
            virtucma_handle_write_dowrite((void *) t);
        }
    }
}

static unsigned int virtucma_open_device(VirtQueueElement *elem){
    int fd;

    DEBUG_PRINT_CURRENT_LINE
    fd = open("/dev/infiniband/rdma_cm", O_RDWR);
    if (DEBUG)
        printf("DEBUG fd: %d\n", fd);
    qemu_set_fd_handler2(fd, NULL, NULL, NULL, NULL);
    stl_p(elem->in_sg[0].iov_base, fd);
    return sizeof fd;
};

static unsigned int virtucma_close_device(VirtQueueElement *elem){
    int fd;

    DEBUG_PRINT_CURRENT_LINE
    fd = ldl_p(elem->out_sg[1].iov_base);
    qemu_set_fd_handler2(fd, NULL, NULL, NULL, NULL);
    if (DEBUG)
        printf("DEBUG fd: %d\n", fd);
    close(fd);
    return 0;
};

static unsigned int (*virtucma_cmd_callbacks[]) (VirtQueueElement *) = {
    [VIRTUCMA_OPEN_DEVICE] = virtucma_open_device,
    [VIRTUCMA_CLOSE_DEVICE] = virtucma_close_device
};

static void virtio_ucma_handle_device(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement elem;
    virtucma_cmd cmd;
    int cb_size;

    DEBUG_PRINT_CURRENT_LINE
    while (virtqueue_pop(vq, &elem)) {
        cmd = ldl_p(elem.out_sg[0].iov_base);
        cb_size = virtucma_cmd_callbacks[cmd](&elem);
        virtqueue_push(vq, &elem, cb_size);
    }
    virtio_notify(vdev, vq);
}

static void virtucma_handle_poll_read(void *v){
    int fd;
    struct VirtQueueFdHandlerData *t = v;

    DEBUG_PRINT_CURRENT_LINE
    fd = ldl_p(t->elem.out_sg[0].iov_base);
    stl_p(t->elem.in_sg[0].iov_base, POLLIN | POLLRDNORM);
    virtqueue_push(t->vq, &t->elem, sizeof(unsigned int));
    virtio_notify(t->vdev, t->vq);
    g_free(t);
    qemu_set_fd_handler(fd, NULL, NULL, NULL);
}

static void virtio_ucma_handle_poll(VirtIODevice *vdev, VirtQueue *vq){
    VirtQueueElement elem;
    int fd;

    DEBUG_PRINT_CURRENT_LINE
    while (virtqueue_pop(vq, &elem)) {
        struct VirtQueueFdHandlerData *t = g_malloc0(sizeof *t);
        fd = ldl_p(elem.out_sg[0].iov_base);
        t->vdev = vdev;
        t->vq = vq;
        memcpy(&t->elem, &elem, sizeof elem);
        qemu_set_fd_handler(fd, virtucma_handle_poll_read, NULL, (void *) t);
    }
}

static void virtio_ucma_get_config(VirtIODevice *vdev, uint8_t *config_data)
{
}

static void virtio_ucma_set_config(VirtIODevice *vdev, const uint8_t *config_data)
{
}

static uint32_t virtio_ucma_get_features(VirtIODevice *vdev, uint32_t f)
{
    return 0;
}

static void virtio_ucma_save(QEMUFile *f, void *opaque)
{
}

static int virtio_ucma_load(QEMUFile *f, void *opaque, int version_id)
{
    return 0;
}

VirtIODevice *virtio_ucma_init(DeviceState *dev)
{
    VirtIOUCMA *s;

    s = (VirtIOUCMA *)virtio_common_init("virtio-ucma",
            VIRTIO_ID_UCMA,
            0, sizeof(VirtIOUCMA));

    s->vdev.get_config = virtio_ucma_get_config;
    s->vdev.set_config = virtio_ucma_set_config;
    s->vdev.get_features = virtio_ucma_get_features;

    s->write_vq  = virtio_add_queue(&s->vdev, 1024, virtio_ucma_handle_write);
    s->device_vq = virtio_add_queue(&s->vdev, 1024, virtio_ucma_handle_device);
    s->poll_vq   = virtio_add_queue(&s->vdev, 1024, virtio_ucma_handle_poll);

    s->qdev = dev;
    register_savevm(dev, "virtio-ucma", -1, 1,
            virtio_ucma_save, virtio_ucma_load, s);

    return &s->vdev;
}

void virtio_ucma_exit(VirtIODevice *vdev)
{
    VirtIOUCMA *s = DO_UPCAST(VirtIOUCMA, vdev, vdev);

    unregister_savevm(s->qdev, "virtio-ucma", s);
    virtio_cleanup(vdev);
}
