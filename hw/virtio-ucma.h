#ifndef _QEMU_VIRTIO_UCMA_H
#define _QEMU_VIRTIO_UCMA_H

#define VIRTIO_ID_UCMA 98

enum {
	VIRTUCMA_OPEN_DEVICE,
	VIRTUCMA_POLL_DEVICE,
	VIRTUCMA_CLOSE_DEVICE,
};
typedef unsigned int virtucma_cmd;

#endif
