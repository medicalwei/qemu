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

struct virtib_create_cq {
	struct ibv_create_cq		ibv_cmd;
	__u64				buf_addr;
	__u64				db_addr;
};

struct virtib_resize_cq {
	struct ibv_resize_cq		ibv_cmd;
	__u64				buf_addr;
};

struct virtib_create_srq {
	struct ibv_create_srq		ibv_cmd;
	__u64				buf_addr;
	__u64				db_addr;
};

struct virtib_create_qp {
	struct ibv_create_qp		ibv_cmd;
	__u64				buf_addr;
	__u64				db_addr;
	__u8				log_sq_bb_count;
	__u8				log_sq_stride;
	__u8				sq_no_prefetch;	/* was reserved in ABI 2 */
	__u8				reserved[5];
};

struct virtib_create_ah {
	struct ibv_create_ah		ibv_cmd;
};
#endif
