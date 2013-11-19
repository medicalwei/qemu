#include "virtio-ib.h"
#include "qemu-error.h"

#define DEBUG 1

typedef struct VirtIOIB
{
    VirtIODevice vdev;
    VirtQueue *write_vq;
    VirtQueue *read_vq;
    VirtQueue *device_vq;
    struct {
        VirtQueueElement elem;
        ssize_t len;
    } request;
    struct ibv_sysfs_dev *sysfs_dev_list;
    DeviceState *qdev;
    int status;
    int pid2fd[32768];
} VirtIOIB;

struct ibv_sysfs_dev *sysfs_list;

struct virtio_ib_config
{
    int config;
};

static VirtIOIB *to_virtio_ib(VirtIODevice *vdev)
{
    return (VirtIOIB *)vdev;
}

static void virtio_ib_get_config(VirtIODevice *vdev, uint8_t *config)
{
    VirtIOIB *ib = to_virtio_ib(vdev);
    struct virtio_ib_config cfg;

    stw_p(&cfg.config, ib->status);
    memcpy((void*) config, &cfg.config, sizeof(cfg));

    #if DEBUG
        printf("virtio_ib_get_config\n");
    #endif
}

static void virtio_ib_set_config(VirtIODevice *vdev, const uint8_t *config)
{
    /*VirtIOIB *ib = to_virtio_ib(vdev);
    struct virtio_ib_config cfg;*/
    #if DEBUG
        printf("virtio_ib_set_config\n");
    #endif
}

static void virtio_ib_save(QEMUFile *f, void *opaque)
{
    VirtIOIB *ib = opaque;
    virtio_save(&ib->vdev, f);
    #if DEBUG
        printf("virtio_ib_save\n");
    #endif
}

static int virtio_ib_load(QEMUFile *f, void *opaque, int version_id)
{
    VirtIOIB *ib = opaque;
    virtio_load(&ib->vdev, f);

    #if DEBUG
        printf("virtio_ib_load\n");
    #endif
    return 0;
}

static uint32_t virtio_ib_get_features(VirtIODevice *vdev, uint32_t features)
{
    //VirtIOIB *ib = to_virtio_ib(vdev);
    
    #if DEBUG
        printf("virtio_ib_get_features\n");
    #endif
    return features;
}

static uint32_t virtio_ib_bad_features(VirtIODevice *vdev)
{
    uint32_t features = 0;
    #if DEBUG
        printf("virtio_ib_bad_features\n");
    #endif
    return features;
}

static void virtio_ib_set_features(VirtIODevice *vdev, uint32_t features)
{
    //VirtIOIB *ib = to_virtio_ib(vdev);
    #if DEBUG
        printf("virtio_ib_set_features\n");
    #endif
}

static void virtio_ib_reset(VirtIODevice *vdev)
{
    VirtIOIB *ib = to_virtio_ib(vdev);
    int i;

    for (i = 0; i < 32768; i++)
        ib->pid2fd[i] = -1;

    #if DEBUG
        printf("virtio_ib_reset\n");
    #endif
}

static void virtio_ib_set_status(struct VirtIODevice *vdev, uint8_t status)
{
    //VirtIOIB *ib = to_virtio_ib(vdev);
    #if DEBUG
        printf("virtio_ib_set_status\n");
    #endif
}

static void virtio_ib_handle_write(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtIOIB *vib = to_virtio_ib(vdev);
    VirtQueueElement elem;
    struct vib_cmd_hdr hdr;
    char *cmd;
    void *addr;
    int cmd_size = 0;
    int ret_len = 0;
    int resp;
    int fd = 0;
    int i;

    while(virtqueue_pop(vq, &elem)){

        INIT_VIB_ELEM;

        if(!memcmp(&vib->request.elem, &elem, sizeof(elem)))
            error_report("virtio-ib can not get read request\n");

        GET_CMD_AND_HDR(&hdr, elem, cmd, cmd_size, sizeof(hdr));

        #if DEBUG
            printf("[virtio_ib_handle_write] cmd:%d\n", hdr.command);
        #endif

        /*Get opening device fd*/
        if(hdr.command > 1 && hdr.command < 45)
            memcpy(&fd, elem.out_sg[1].iov_base, sizeof(int));

        switch(hdr.command){
            case IB_USER_VERBS_CMD_GET_CONTEXT:
            case IB_USER_VERBS_CMD_QUERY_DEVICE:
                GET_INDEX(elem.out_sg[1].iov_base);
                fd = vib->pid2fd[i];
                CHANGE_RESP_ADDR(struct ibv_get_context, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_QUERY_PORT:
                CHANGE_RESP_ADDR(struct ibv_query_port, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_ALLOC_PD:
                CHANGE_RESP_ADDR(struct ibv_alloc_pd, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_DEALLOC_PD:
            case IB_USER_VERBS_CMD_DESTROY_AH:
            case IB_USER_VERBS_CMD_DEREG_MR:
            case IB_USER_VERBS_CMD_REQ_NOTIFY_CQ:
            case IB_USER_VERBS_CMD_MODIFY_QP:
            case IB_USER_VERBS_CMD_ATTACH_MCAST:
            case IB_USER_VERBS_CMD_DETACH_MCAST:
            case IB_USER_VERBS_CMD_MODIFY_SRQ:
                ret_len = sizeof(int);
                break;
            case IB_USER_VERBS_CMD_CREATE_AH: 
                CHANGE_RESP_ADDR(struct ibv_create_ah, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_REG_MR:
                CHANGE_RESP_ADDR(struct ibv_reg_mr, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_CREATE_COMP_CHANNEL:
                CHANGE_RESP_ADDR(struct ibv_create_comp_channel, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_CREATE_CQ:
                CHANGE_RESP_ADDR(struct ibv_create_cq, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_RESIZE_CQ:
                CHANGE_RESP_ADDR(struct ibv_resize_cq, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_DESTROY_CQ:
                CHANGE_RESP_ADDR(struct ibv_destroy_cq, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_CREATE_QP:
                CHANGE_RESP_ADDR(struct ibv_create_qp, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_QUERY_QP:
                CHANGE_RESP_ADDR(struct ibv_query_qp, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_DESTROY_QP:
                CHANGE_RESP_ADDR(struct ibv_destroy_qp, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_CREATE_SRQ:
                CHANGE_RESP_ADDR(struct ibv_create_srq, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_QUERY_SRQ:
                CHANGE_RESP_ADDR(struct ibv_query_srq, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_DESTROY_SRQ:
                CHANGE_RESP_ADDR(struct ibv_destroy_srq, cmd, elem.in_sg[1]);
                break;
            case IB_USER_VERBS_CMD_MMAP:
                GET_INDEX(elem.out_sg[1].iov_base);
                fd = vib->pid2fd[i];
                addr = mmap(NULL, ((struct vib_mmap*)cmd)->page_size, ((struct vib_mmap*)cmd)->prot, ((struct vib_mmap*)cmd)->flags, fd, ((struct vib_mmap*)cmd)->off);
                resp = sizeof(struct vib_mmap);
                memcpy(elem.in_sg[0].iov_base, &resp, sizeof(int));
                memcpy(elem.in_sg[1].iov_base, &addr, sizeof(void*));
                ret_len = sizeof(void*);
                break;
            case IB_USER_VERBS_CMD_UNMAP:
                memcpy(&addr, elem.out_sg[1].iov_base, sizeof(void*));
                munmap((void *) addr, *((int*)(elem.out_sg[2].iov_base)));
                break;
            case IB_USER_VERBS_CMD_GET_EVENT:
                memcpy(&fd, elem.out_sg[1].iov_base, sizeof(fd));
                ret_len = ((struct vib_cmd*)cmd)->out_words*4;
                resp = read(fd, elem.in_sg[1].iov_base, ret_len);
            default:
                error_report("virtio-ib wrong write command\n");
                break;
        }

        if (hdr.command < 45){
            resp = write(fd, cmd, cmd_size);
            memcpy(elem.in_sg[0].iov_base, &resp, sizeof(resp));
            if(!ret_len)
                ret_len = hdr.out_words * 4;
        }

        elem.in_sg[0].iov_len = sizeof(int);
        virtqueue_push(vq, &elem, ret_len);
    }
    virtio_notify(vdev, vq);
}

static void virtio_ib_handle_read(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtIOIB *vib = to_virtio_ib(vdev);
    VirtQueueElement elem;

    int ret_len = 0;
    int fd;

    while(virtqueue_pop(vq, &elem)){
        INIT_VIB_ELEM;

        if(!memcmp(&vib->request.elem, &elem, sizeof(elem)))
            error_report("virtio-ib can not get write request\n");

        #if DEBUG
            printf("read path:%s\n", (char*)(elem.out_sg[0].iov_base));
        #endif

        fd = open((char*)(elem.out_sg[0].iov_base), O_RDONLY);

        if(fd < 0){
            error_report("virtio-ib read driver system file failed\n");
            return;
        }

        ret_len = read(fd, (char*)elem.in_sg[0].iov_base, sizeof(char) * 1024);

        #if DEBUG
            printf("%s, %d\n", (char*)elem.in_sg[0].iov_base, ret_len);
        #endif

        close(fd);

        stl_p(elem.in_sg[1].iov_base, ret_len);
        virtqueue_push(vq, &elem, ret_len);
    }
    virtio_notify(vdev, vq);
    
    return;
}

static int get_sysfs_devs(struct ibv_sysfs_dev *sysfs_dev_list, struct ibv_sysfs_dev *sysfs)
{
    struct ibv_sysfs_dev *sysfs_dev = NULL;
    int i = 0;    
    int j = 0;

    #if DEBUG
        printf("get_sysfs_devs\n");
    #endif

    for(sysfs_dev = sysfs_dev_list; sysfs_dev; sysfs_dev = sysfs_dev->next){
        memcpy(&sysfs[i], sysfs_dev, sizeof(struct ibv_sysfs_dev));
        i++;
    }

    for (j = i; j < 10; j++){
        sysfs[j].have_driver = -1;
    }

    if (i > 0)
        return 0;
    else{
        error_report("virtio-ib no InfiniBand driver loaded\n");
        return -1;
    }

}

static void virtio_ib_handle_device(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtIOIB *vib = to_virtio_ib(vdev);
    VirtQueueElement elem;
    struct vib_cmd_hdr hdr;
    int resp = 0;
    int ret_len = 0;
    int i;
    int fd;
    void *addr;
    unsigned long *addr1, *addr2;

    while(virtqueue_pop(vq, &elem)){

        INIT_VIB_ELEM;

        if (!memcmp(&elem, &vib->request.elem, sizeof(elem))) {
            error_report("virtio-ib can not get device request\n");
        }

        memcpy(&hdr, elem.out_sg[0].iov_base, sizeof(hdr));

        #if DEBUG
            printf("[virtio_ib_handle_device] cmd:%d\n", hdr.command);
        #endif

        switch(hdr.command){
            case IB_USER_VERBS_CMD_FIND_SYSFS:
                resp = get_sysfs_devs(vib->sysfs_dev_list, elem.in_sg[1].iov_base);
                ret_len = 10*sizeof(struct ibv_sysfs_dev); 
                elem.in_sg[1].iov_len = ret_len;
                break;
            case IB_USER_VERBS_CMD_OPEN_DEV:
                GET_INDEX(elem.out_sg[1].iov_base);
                resp = open("/dev/infiniband/uverbs0", O_RDWR);
                vib->pid2fd[i] = resp;
                memcpy(elem.in_sg[0].iov_base, &resp, sizeof(resp));
                ret_len = sizeof(int);
                break;
            case IB_USER_VERBS_CMD_CLOSE_DEV_FD:
                memcpy(&i, elem.out_sg[2].iov_base, sizeof(int));
                memcpy(&fd, elem.out_sg[1].iov_base, sizeof(fd));
                if (i > -1)
                    vib->pid2fd[i] = 0;
                close(fd);
                resp = 0;
                ret_len = sizeof(int); 
                break;
            case IB_USER_VERBS_CMD_RING_DOORBELL:
                memcpy(&addr, elem.out_sg[1].iov_base, sizeof(void*));
                memcpy(&i, elem.out_sg[3].iov_base, sizeof(int));
                switch(i){
                    case 1:
                        *(uint32_t *) ((void*)addr) = *(uint32_t *) elem.out_sg[2].iov_base;
                        break;
                     case 2:/*FIX*/
                        *(volatile uint64_t *) ((void*)addr) = *(uint64_t*) elem.out_sg[2].iov_base;
                        break;
                    case 3:
                        *(volatile uint32_t *) ((void*)addr) = *(uint32_t *) elem.out_sg[2].iov_base;
                        break;
                    default:
                        break;
                }
                break;
            case IB_USER_VERBS_CMD_BUF_COPY:
                i = (unsigned) elem.out_sg[1].iov_len;
                memcpy(&addr, elem.out_sg[2].iov_base, sizeof(void*));
                addr1 = (unsigned long *) addr;//dst
                addr2 = (unsigned long *)(elem.out_sg[1].iov_base);//src
                while (i > 0) {
                    *addr1++ = *addr2++;
                    *addr1++ = *addr2++;
                    i -= 2 * sizeof (long);
                }
                ret_len = sizeof(int);
                break;
            default:
                error_report("virtio-ib wrong command for device\n");
                break; 
        }

        memcpy(elem.in_sg[0].iov_base, &resp, sizeof(int));
        elem.in_sg[0].iov_len = sizeof(int);
        virtqueue_push(vq, &elem, ret_len);
    }
    virtio_notify(vdev, vq);

    return;
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
    int i;

    #if DEBUG
        printf("virtio_ib_init\n");
    #endif

    ib = (VirtIOIB *)virtio_common_init("virtio-ib", VIRTIO_ID_IB,
                                          sizeof(struct virtio_ib_config),
                                          sizeof(VirtIOIB));
    ib->vdev.get_config = virtio_ib_get_config;
    ib->vdev.set_config = virtio_ib_set_config;
    ib->vdev.get_features = virtio_ib_get_features;
    ib->vdev.set_features = virtio_ib_set_features;
    ib->vdev.bad_features = virtio_ib_bad_features;
    ib->vdev.reset = virtio_ib_reset;
    ib->vdev.set_status = virtio_ib_set_status;
    ib->write_vq = virtio_add_queue(&ib->vdev, 1024, virtio_ib_handle_write);
    ib->read_vq = virtio_add_queue(&ib->vdev, 1024, virtio_ib_handle_read);
    ib->device_vq = virtio_add_queue(&ib->vdev, 1024, virtio_ib_handle_device);

    ib->qdev = dev;
    ib->status = 99;

    if (find_sysfs_devs())
        error_report("virtio-ib Can't not find InfiniBand!\n");

    ib->sysfs_dev_list = sysfs_list;

    for (i = 0; i < 32768; i++)
        ib->pid2fd[i] = -1;

    register_savevm(dev, "virtio-ib", 0, 2,
                    virtio_ib_save, virtio_ib_load, ib);

    return &ib->vdev;
}

void virtio_ib_exit(VirtIODevice *vdev)
{
    VirtIOIB *ib = to_virtio_ib(vdev);
    unregister_savevm(ib->qdev, "virtio-ib", ib);
}
