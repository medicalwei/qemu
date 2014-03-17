#include "iov.h"
#include "qemu-common.h"
#include "virtio.h"
#include "cpu.h"
#include "memory.h"
#include "exec-memory.h"
#include "virtio-ib.h"

#include <poll.h>
#include <sys/mman.h>
#include <dirent.h>
#include <infiniband/verbs.h>
#include <infiniband/driver.h>

#define IB_UVERBS_CMD_MAX_SIZE 16384
#define VIRTIB_MAX_SYSFS_DEVS 10
#define VIRTIB_UVERBS_DEV_PATH "/dev/infiniband/uverbs0"

typedef struct VirtIOIB
{
    VirtIODevice vdev;
    VirtQueue *write_vq;
    VirtQueue *read_vq;
    VirtQueue *device_vq;
    VirtQueue *event_vq;
    VirtQueue *event_poll_vq;

    DeviceState *qdev;
    int status;
} VirtIOIB;

struct ibv_sysfs_dev {
    char                  sysfs_name[IBV_SYSFS_NAME_MAX];
    char                  ibdev_name[IBV_SYSFS_NAME_MAX];
    char                  sysfs_path[IBV_SYSFS_PATH_MAX];
    char                  ibdev_path[IBV_SYSFS_PATH_MAX];
    struct ibv_sysfs_dev *next;
    int                   abi_ver;
    int                   have_driver;
};

struct ibv_sysfs_dev *sysfs_list;

struct VirtQueueFdHandlerData{
    VirtIODevice *vdev;
    VirtQueue *vq;
    VirtQueueElement elem;
};

static void virtib_get_config(VirtIODevice *vdev, uint8_t *config)
{
}

static void virtib_set_config(VirtIODevice *vdev, const uint8_t *config)
{
}

static void virtib_save(QEMUFile *f, void *opaque)
{
    VirtIOIB *ib = opaque;
    virtio_save(&ib->vdev, f);
}

static int virtib_load(QEMUFile *f, void *opaque, int version_id)
{
    VirtIOIB *ib = opaque;
    virtio_load(&ib->vdev, f);

    return 0;
}

static uint32_t virtib_get_features(VirtIODevice *vdev, uint32_t features)
{
    return 0;
}

static void virtib_reset(VirtIODevice *vdev)
{
}

static void virtib_set_status(struct VirtIODevice *vdev, uint8_t status)
{
}

static void virtib_handle_write(VirtIODevice *vdev, VirtQueue *vq)
{
    /* Element Segments
     * out_sg[0] int32_t: fd
     * out_sg[1] VAR_LEN: ibv command
     *  in_sg[0] int32_t: write response
     *  in_sg[1] VAR_LEN: data response
     */
    VirtQueueElement elem;

    while(virtqueue_pop(vq, &elem)){
        int fd, resp;
        char in[IB_UVERBS_CMD_MAX_SIZE];
        struct ibv_query_params *hdr = (void *) in;

        fd = ldl_p(elem.out_sg[0].iov_base);
        memcpy(in, elem.out_sg[1].iov_base, elem.out_sg[1].iov_len);

        if (hdr->out_words > 0){
            hdr->response = (__u64) elem.in_sg[1].iov_base;
        }

        resp = write(fd, in, elem.out_sg[1].iov_len);

        stl_p(elem.in_sg[0].iov_base, resp);
        virtqueue_push(vq, &elem, sizeof(int) + IB_UVERBS_CMD_MAX_SIZE);
    }
    virtio_notify(vdev, vq);
}

static void virtib_handle_read(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement elem;

    int ret_len = 0;
    int fd;

    while(virtqueue_pop(vq, &elem)){
        fd = open((char*)elem.out_sg[0].iov_base, O_RDONLY);

        if(fd < 0){
            error_report("virtio-ib: read driver system file failed\n");
            return;
        }

        ret_len = read(fd, elem.in_sg[0].iov_base, elem.in_sg[0].iov_len);

        close(fd);

        stq_p(elem.in_sg[1].iov_base, ret_len);
        virtqueue_push(vq, &elem, ret_len);
    }
    virtio_notify(vdev, vq);

    return;
}

static int get_sysfs_devs(struct ibv_sysfs_dev *sysfs_dev_list, struct ibv_sysfs_dev *sysfs)
{
    struct ibv_sysfs_dev *sysfs_dev = NULL;
    int i = 0, j = 0;

    for(sysfs_dev = sysfs_dev_list; sysfs_dev; sysfs_dev = sysfs_dev->next){
        memcpy(&sysfs[i], sysfs_dev, sizeof(struct ibv_sysfs_dev));
        i++;
    }

    for (j = i; j < VIRTIB_MAX_SYSFS_DEVS; j++){
        sysfs[j].have_driver = -1;
    }

    if (i > 0)
        return 0;
    else{
        error_report("virtio-ib: no InfiniBand driver loaded\n");
        return -1;
    }

}

static unsigned int virtib_device_find_sysfs(VirtQueueElement *elem){
    /* Element Segments
     * out_sg[0] __s32: VIRTIB_DEVICE_FIND_SYSFS
     *  in_sg[0] __s32: get_sysfs_devs return value
     *  in_sg[1] sizeof(struct ibv_sysfs_dev)*VIRTIB_MAX_SYSFS_DEVS:
     *                  sysfs list
     */
    int resp = get_sysfs_devs(sysfs_list, elem->in_sg[1].iov_base);
    stl_p(elem->in_sg[0].iov_base, resp);
    return sizeof(resp) + VIRTIB_MAX_SYSFS_DEVS*sizeof(struct ibv_sysfs_dev);
}

static unsigned int virtib_device_open(VirtQueueElement *elem){
    /* Element Segments
     * out_sg[0] int32_t: VIRTIB_DEVICE_OPEN
     *  in_sg[0] int32_t: fd opened
     */
    int fd = open(VIRTIB_UVERBS_DEV_PATH, O_RDWR);
    stl_p(elem->in_sg[0].iov_base, fd);
    return sizeof fd;
}

static unsigned int virtib_device_close(VirtQueueElement *elem){
    /* Element Segments
     * out_sg[0] int32_t: VIRTIB_DEVICE_CLOSE
     * out_sg[1] int32_t: fd to be closed
     *  in_sg[0] int32_t: close result
     */
    int fd = ldl_p(elem->out_sg[1].iov_base);
    int resp = close(fd);
    stl_p(elem->in_sg[0].iov_base, resp);
    return sizeof(resp);
}

static unsigned int virtib_device_mmap(VirtQueueElement *elem){
    /* Element Segments
     * out_sg[0]  int32_t: VIRTIB_DEVICE_MMAP
     * out_sg[1]  int32_t: fd
     * out_sg[2] uint64_t: offset
     *  in_sg[0] uint64_t: mmap return value
     */
    int32_t    fd     = (int32_t) ldl_p(elem->out_sg[1].iov_base);
    off_t      offset = (off_t)   ldl_p(elem->out_sg[2].iov_base);

    uint64_t resp;

    resp = (uint64_t) mmap(NULL, TARGET_PAGE_SIZE, PROT_WRITE, MAP_SHARED, fd, offset);

    stq_p(elem->in_sg[0].iov_base, resp);
    return sizeof(resp);
}

static unsigned int virtib_device_munmap(VirtQueueElement *elem){
    /* Element Segments
     * out_sg[0]  int32_t: VIRTIB_DEVICE_MUNMAP
     * out_sg[1] uint64_t: address
     *  in_sg[0]  int32_t: mmap return value
     */
    void *addr = (void *) ldq_p(elem->out_sg[1].iov_base);
    int32_t resp;

    resp = (int32_t) munmap(addr, TARGET_PAGE_SIZE);
    stq_p(elem->in_sg[0].iov_base, resp);
    return sizeof(resp);
}

static unsigned int virtib_device_mcopy(VirtQueueElement *elem){
    /* Element Segments
     * out_sg[0]  int32_t: VIRTIB_DEVICE_MCOPY
     * out_sg[1] uint64_t: destination host virtual address
     * out_sg[2] variable: written data
     * out_sg[3] uint32_t: byte count
     */
    unsigned long *dst = (void *) ldq_p(elem->out_sg[1].iov_base);
    unsigned long *src = (void *) elem->out_sg[2].iov_base;
    unsigned int bytecnt = ldl_p(elem->out_sg[3].iov_base);

    while (bytecnt > 0) {
        *dst++ = *src++;
        *dst++ = *src++;
        bytecnt -= 2 * sizeof (long);
    }

    return 0;
}

static unsigned int virtib_device_massign(VirtQueueElement *elem){
    /* Element Segments
     * out_sg[0]  int32_t: VIRTIB_DEVICE_MASSIGN or VIRTIB_DEVICE_MASSIGN_LONG
     * out_sg[1] uint64_t: destination host virtual address
     * out_sg[2] uint32_t or uint64_t: data to be assigned
     */
    int32_t cmd = ldl_p(elem->out_sg[0].iov_base);
    if (cmd == VIRTIB_DEVICE_MASSIGN_LONG) {
        uint64_t *dst = (void *) ldq_p(elem->out_sg[1].iov_base);
        *dst = ldq_p(elem->out_sg[2].iov_base);
    } else if (cmd == VIRTIB_DEVICE_MASSIGN) {
        uint32_t *dst = (void *) ldq_p(elem->out_sg[1].iov_base);
        *dst = ldl_p(elem->out_sg[2].iov_base);
    }
    return 0;
}

static unsigned int (*virtib_device_cmd_callbacks[]) (VirtQueueElement *) = {
    [VIRTIB_DEVICE_FIND_SYSFS]   = virtib_device_find_sysfs,
    [VIRTIB_DEVICE_OPEN]         = virtib_device_open,
    [VIRTIB_DEVICE_CLOSE]        = virtib_device_close,
    [VIRTIB_DEVICE_MMAP]         = virtib_device_mmap,
    [VIRTIB_DEVICE_MUNMAP]       = virtib_device_munmap,
    [VIRTIB_DEVICE_MCOPY]        = virtib_device_mcopy,
    [VIRTIB_DEVICE_MASSIGN]      = virtib_device_massign,
    [VIRTIB_DEVICE_MASSIGN_LONG] = virtib_device_massign,
};

static void virtib_handle_device(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement elem;

    while(virtqueue_pop(vq, &elem)){
        int cmd = ldl_p(elem.out_sg[0].iov_base);
        unsigned int cb_size = virtib_device_cmd_callbacks[cmd](&elem);
        virtqueue_push(vq, &elem, cb_size);
    }
    virtio_notify(vdev, vq);

    return;
}

static void virtib_event_poll(void *opaque)
{
    struct VirtQueueFdHandlerData *t = opaque;
    int fd, cmd;
    ssize_t len, ret;

    fd = ldl_p(t->elem.out_sg[0].iov_base);
    cmd = (__s32) ldl_p(t->elem.out_sg[1].iov_base);

    switch(cmd){
        case VIRTIB_EVENT_READ:
            len = (__u32) ldl_p(t->elem.out_sg[2].iov_base);
            ret = read(fd, t->elem.in_sg[1].iov_base, len);
            stl_p(t->elem.in_sg[0].iov_base, ret);
            virtqueue_push(t->vq, &t->elem, sizeof(ret) + ret);
            break;
        case VIRTIB_EVENT_POLL:
            stl_p(t->elem.in_sg[0].iov_base, POLLIN | POLLRDNORM);
            virtqueue_push(t->vq, &t->elem, sizeof(unsigned int));
            break;
    }
    virtio_notify(t->vdev, t->vq);
    g_free(t);
    qemu_set_fd_handler(fd, NULL, NULL, NULL);
}

static void virtib_handle_event(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement elem;
    int fd;
    __s32 cmd;
    struct VirtQueueFdHandlerData *t;

    while (virtqueue_pop(vq, &elem)) {
        fd = ldl_p(elem.out_sg[0].iov_base);
        cmd = ldl_p(elem.out_sg[1].iov_base);
        switch(cmd){
            case VIRTIB_EVENT_READ:
            case VIRTIB_EVENT_POLL:
                t = g_malloc0(sizeof *t);
                t->vdev = vdev;
                t->vq = vq;
                memcpy(&t->elem, &elem, sizeof elem);
                qemu_set_fd_handler(fd, virtib_event_poll, NULL, (void *) t);
                break;
            case VIRTIB_EVENT_CLOSE:
                qemu_set_fd_handler(fd, NULL, NULL, NULL);
                close(fd);
                virtqueue_push(vq, &elem, 0);
                virtio_notify(vdev, vq);
                break;
        }
    }
}

static int find_sysfs_devs(void)
{
    char class_path[IBV_SYSFS_PATH_MAX];
    DIR *class_dir;
    struct dirent *dent;
    struct ibv_sysfs_dev *sysfs_dev = NULL;
    char value[8];
    int ret = 0;

    snprintf(class_path, sizeof class_path, "/sys/class/infiniband_verbs");

    class_dir = opendir(class_path);
    if (!class_dir)
        return ENOMEM;

    while ((dent = readdir(class_dir))) {
        struct stat buf;

        if (dent->d_name[0] == '.')
            continue;

        if (!sysfs_dev)
            sysfs_dev = malloc(sizeof *sysfs_dev);
        if (!sysfs_dev) {
            ret = ENOMEM;
            goto out;
        }

        snprintf(sysfs_dev->sysfs_path, sizeof sysfs_dev->sysfs_path,
                "%s/%s", class_path, dent->d_name);

        if (stat(sysfs_dev->sysfs_path, &buf)) {
            fprintf(stderr, "MLX4 Warning: couldn't stat '%s'.\n",
                    sysfs_dev->sysfs_path);
            continue;
        }

        if (!S_ISDIR(buf.st_mode))
            continue;

        snprintf(sysfs_dev->sysfs_name, sizeof sysfs_dev->sysfs_name,
                "%s", dent->d_name);

        if (ibv_read_sysfs_file(sysfs_dev->sysfs_path, "ibdev",
                    sysfs_dev->ibdev_name,
                    sizeof sysfs_dev->ibdev_name) < 0) {
            fprintf(stderr, "MLX4 Warning: no ibdev class attr for '%s'.\n",
                    dent->d_name);
            continue;
        }

        snprintf(sysfs_dev->ibdev_path, sizeof sysfs_dev->ibdev_path,
                "%s/class/infiniband/%s", ibv_get_sysfs_path(),
                sysfs_dev->ibdev_name);

        sysfs_dev->next        = sysfs_list;
        sysfs_dev->have_driver = 0;
        if (ibv_read_sysfs_file(sysfs_dev->sysfs_path, "abi_version",
                    value, sizeof value) > 0)
            sysfs_dev->abi_ver = strtol(value, NULL, 10);
        else
            sysfs_dev->abi_ver = 0;

        sysfs_list = sysfs_dev;
        sysfs_dev      = NULL;
    }

out:
    if (sysfs_dev)
        free(sysfs_dev);

    closedir(class_dir);
    return ret;
}


VirtIODevice *virtio_ib_init(DeviceState *dev)
{
    VirtIOIB *ib;

    ib = (VirtIOIB *)virtio_common_init("virtio-ib",
            VIRTIO_ID_IB,
            0, sizeof(VirtIOIB));

    ib->vdev.get_config = virtib_get_config;
    ib->vdev.set_config = virtib_set_config;
    ib->vdev.get_features = virtib_get_features;
    ib->vdev.reset = virtib_reset;
    ib->vdev.set_status = virtib_set_status;
    ib->write_vq = virtio_add_queue(&ib->vdev, 1024, virtib_handle_write);
    ib->read_vq = virtio_add_queue(&ib->vdev, 1024, virtib_handle_read);
    ib->device_vq = virtio_add_queue(&ib->vdev, 1024, virtib_handle_device);
    ib->event_vq = virtio_add_queue(&ib->vdev, 1024, virtib_handle_event);

    ib->qdev = dev;
    ib->status = 99;

    if (find_sysfs_devs())
        error_report("virtio-ib: Can't not find InfiniBand!\n");

    register_savevm(dev, "virtio-ib", 0, 2,
            virtib_save, virtib_load, ib);

    return &ib->vdev;
}

void virtio_ib_exit(VirtIODevice *vdev)
{
    VirtIOIB *ib = DO_UPCAST(VirtIOIB, vdev, vdev);
    unregister_savevm(ib->qdev, "virtio-ib", ib);
}
