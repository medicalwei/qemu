#ifndef _QEMU_VIRTIO_MEMLINK_H
#define _QEMU_VIRTIO_MEMLINK_H

#include "virtio.h"
#include "pci.h"

/* The ID for virtio_memlink */
#define VIRTIO_ID_MEMLINK 65535

/* Define virtio-memlink functions */
#define VIRTIO_MEMLINK_CREATE 1
#define VIRTIO_MEMLINK_LINK   2
#define VIRTIO_MEMLINK_DELETE 3
#endif

/* Size of a PFN in the balloon interface. */
#define VIRTIO_MEMLINK_PFN_SHIFT 12
