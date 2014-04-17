#ifndef _QEMU_MEMLINK_H
#define _QEMU_MEMLINK_H

#define MEMLINK_SHMMAX 268435456

typedef struct Memlink {
    void *host_memory;
    void *offseted_host_memory;
    unsigned int num_gfns;
    uint32_t *gfns;
    unsigned int offset;
} Memlink;

void memlink_link_address(Memlink *ml);
void memlink_unlink_address(Memlink *ml);
void memlink_init(void);
void memlink_exit(void);
#endif
