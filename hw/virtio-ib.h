#ifndef _QEMU_VIRTIO_IB_H
#define _QEMU_VIRTIO_IB_H

#include <qemu-common.h>
#include "virtio.h"
#include "qemu-error.h"
#include "pci.h"

#include <infiniband/verbs.h>
#include <infiniband/arch.h>
#include <infiniband/driver.h>
#include "mlx4.h"

#include "string.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <dirent.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <linux/fb.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <alloca.h>

#define VIRTIO_ID_IB 99
#define INIT_VIB_ELEM                                             \
    do {                                                    \
           memset(&vib->request.elem, 0, sizeof(elem));         \
    }while(0)

#define GET_INDEX(ptr) memcpy(&i, (ptr), sizeof(i));   

#define GET_CMD_AND_HDR(header, pop_data, command, csize, hdr_size)            \
    do{                                 \
        (csize)   = (pop_data).out_sg[0].iov_len;                              \
        (command) = (char*) (pop_data).out_sg[0].iov_base;                     \
        memcpy((header), (command), (hdr_size));                               \
    }while(0)

#define CHANGE_RESP_ADDR(type, command, data) ((type*)(command))->response = (uintptr_t) (data).iov_base;

struct ibv_sysfs_dev {
        char                    sysfs_name[IBV_SYSFS_NAME_MAX];
        char                    ibdev_name[IBV_SYSFS_NAME_MAX];
        char                    sysfs_path[IBV_SYSFS_PATH_MAX];
        char                    ibdev_path[IBV_SYSFS_PATH_MAX];
        struct ibv_sysfs_dev   *next;
        int                     abi_ver;
        int                     have_driver;
};

struct vib_cmd_hdr{
        __u32 command;
        __u16 in_words;
        __u16 out_words;
};

struct vib_cmd{
    __u32 command;
    __u16 in_words;
    __u16 out_words;
    __u64 response;
};

enum{
        IB_USER_VERBS_CMD_FIND_SYSFS = 1000,
        IB_USER_VERBS_CMD_OPEN_DEV,
        IB_USER_VERBS_CMD_MMAP,
        IB_USER_VERBS_CMD_UNMAP,
        IB_USER_VERBS_CMD_RING_DOORBELL,
        IB_USER_VERBS_CMD_BUF_COPY,
        IB_USER_VERBS_CMD_CLOSE_DEV_FD,
        IB_USER_VERBS_CMD_GET_EVENT
};

struct vib_mmap{
        __u32 command;
        __u32 page_size;
        __u32 prot;
        __u32 flags;
        __u32 off;
};

#endif
