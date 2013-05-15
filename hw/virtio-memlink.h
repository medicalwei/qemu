#ifndef _QEMU_VIRTIO_MEMLINK_H
#define _QEMU_VIRTIO_MEMLINK_H

#include "virtio.h"
#include "pci.h"

/* The ID for virtio_memlink */
#define VIRTIO_ID_MEMLINK 65535

/* Size of a PFN in the memlink interface. */
#define VIRTIO_MEMLINK_PAGE_SIZE 4096
#define VIRTIO_MEMLINK_PFN_SHIFT 12

#define MEMLINK_MAX_LINKS 32

#endif

