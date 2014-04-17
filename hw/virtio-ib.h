#ifndef _QEMU_VIRTIO_IB_H
#define _QEMU_VIRTIO_IB_H

#include <infiniband/kern-abi.h>
#define VIRTIO_ID_IB 99

enum{
        VIRTIB_DEVICE_FIND_SYSFS = 1000,
        VIRTIB_DEVICE_OPEN,
        VIRTIB_DEVICE_CLOSE,
        VIRTIB_DEVICE_MMAP,
        VIRTIB_DEVICE_MUNMAP,
};

enum{
        VIRTIB_EVENT_READ,
        VIRTIB_EVENT_POLL,
        VIRTIB_EVENT_CLOSE,
};
#endif
