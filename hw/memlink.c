#include "qemu-common.h"
#include "kvm.h"
#include "exec-memory.h"
#include "memlink.h"
#include "qerror.h"

#if defined(__linux__)
#include <sys/mman.h>
#endif

typedef struct ShmInfo {
    int id;
    char filename[40];
    void *mem;
    void *orig_mem;
    unsigned int usedcount;
} ShmInfo;

typedef struct MemlinkMapItem {
    struct ShmInfo *shm;
    unsigned int offset;
    unsigned int usedcount;
    void *orig;
} MemlinkMapItem;

typedef struct MemlinkMap {
    MemlinkMapItem *map;
    MemlinkMapItem shm_next;
    unsigned int usedcount;
} MemlinkMap;

static MemlinkMap memlink_map = {.map = NULL, .shm_next.shm = NULL};

static void * gfn_to_hva(uint32_t gfn, uint32_t *offset_fn)
{
    MemoryRegionSection section;
    ram_addr_t pa;

    pa = (ram_addr_t) gfn << TARGET_PAGE_BITS;
    section = memory_region_find(get_system_memory(), pa, 1);

    if (!section.size || !memory_region_is_ram(section.mr)){
        return NULL;
    }

    if (offset_fn != NULL) {
        *offset_fn = section.offset_within_region >> TARGET_PAGE_BITS;
    }

    return memory_region_get_ram_ptr(section.mr) +
        section.offset_within_region;
}

static void * get_shared_memory(uint32_t gfn)
{
    uint32_t offset_fn;
    void * qemu_hva = gfn_to_hva(gfn, &offset_fn);
    MemlinkMapItem *item = &memlink_map.map[offset_fn];

    if (unlikely(item->shm != NULL)) {
        item->usedcount += 1;
        return item->shm->mem + item->offset;
    }

    if (unlikely(memlink_map.shm_next.shm == NULL)) {
        ShmInfo* shminfo = (ShmInfo *) malloc(sizeof(ShmInfo));
        sprintf(shminfo->filename, "virtio-memlink_%x", rand());
        shminfo->id = shm_open(shminfo->filename, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);

        if (shminfo->id < 0){
            free(shminfo);
            return NULL;
        }

        if (ftruncate(shminfo->id, MEMLINK_SHMMAX) != 0){
            shm_unlink(shminfo->filename);
            free(shminfo);
            return NULL;
        }

        shminfo->mem = mmap(NULL, MEMLINK_SHMMAX, PROT_READ | PROT_WRITE, MAP_SHARED, shminfo->id, 0);
        shminfo->orig_mem = mmap(NULL, MEMLINK_SHMMAX, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

        shminfo->usedcount = 0;

        memlink_map.shm_next.shm = shminfo;
        memlink_map.shm_next.offset = 0;
    }

    item->shm = memlink_map.shm_next.shm;
    item->offset = memlink_map.shm_next.offset;
    item->usedcount += 1;
    item->shm->usedcount += 1;

    mremap(qemu_hva, TARGET_PAGE_SIZE, TARGET_PAGE_SIZE, MREMAP_MAYMOVE | MREMAP_FIXED,
           item->shm->orig_mem + item->offset);

    mmap(qemu_hva, TARGET_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, item->shm->id,
         item->offset);

    memcpy(qemu_hva, item->shm->orig_mem+item->offset, TARGET_PAGE_SIZE);

    memlink_map.shm_next.offset += TARGET_PAGE_SIZE;
    if (memlink_map.shm_next.offset >= MEMLINK_SHMMAX){
        memlink_map.shm_next.shm = NULL;
    }

    return item->shm->mem + item->offset;
}

static void put_shared_memory(uint32_t gfn)
{
    uint32_t offset_fn;
    void * qemu_hva = gfn_to_hva(gfn, &offset_fn);
    MemlinkMapItem *item = &memlink_map.map[offset_fn];
    ShmInfo *shminfo = item->shm;

    if (unlikely(item->usedcount == 0)){
        return;
    }

    item->usedcount -= 1;

    if (unlikely(item->usedcount > 0)){
        return;
    }

    memcpy(item->shm->orig_mem+item->offset, qemu_hva, TARGET_PAGE_SIZE);

    mremap(item->shm->orig_mem + item->offset, TARGET_PAGE_SIZE, TARGET_PAGE_SIZE,
           MREMAP_MAYMOVE | MREMAP_FIXED, qemu_hva);

    item->shm = NULL;

    if (unlikely(shminfo->usedcount == 0)){
        return;
    }

    shminfo->usedcount -= 1;

    if (unlikely(shminfo->usedcount == 0)){
        shm_unlink(shminfo->filename);
        if (memlink_map.shm_next.shm == shminfo){
            memlink_map.shm_next.shm = NULL;
        }
        free(shminfo);
    }
}

void memlink_link_address(Memlink *ml)
{
    unsigned long mem_size = ml->num_gfns << TARGET_PAGE_BITS;
    int i;

    ml->host_memory = mmap(NULL, mem_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    for (i=0; i<ml->num_gfns; i++) {
        void * shmem = get_shared_memory(ml->gfns[i]);
        uint32_t offset = i << TARGET_PAGE_BITS;
        mremap(shmem, 0, TARGET_PAGE_SIZE, MREMAP_MAYMOVE | MREMAP_FIXED, ml->host_memory + offset);
    }
}

void memlink_unlink_address(Memlink *ml)
{
    int i;

    for (i=0; i<ml->num_gfns; i++) {
        put_shared_memory(ml->gfns[i]);
    }
}

void memlink_init(void)
{
    if (memlink_map.map != NULL){
        memlink_map.usedcount += 1;
        return;
    }

    size_t map_size = (ram_size >> TARGET_PAGE_BITS)*sizeof(MemlinkMapItem)*2;
    memlink_map.map = (MemlinkMapItem *) malloc(map_size);
    if (memlink_map.map == NULL) {
        error_report("virtio-memlink: cannot allocate shm map.");
        exit(1);
    }
    memset(memlink_map.map, 0, map_size);
    memlink_map.usedcount = 1;
}

void memlink_exit(void)
{
    if (memlink_map.map == NULL){
        return;
    }

    memlink_map.usedcount -= 1;
    if (memlink_map.usedcount > 0){
        return;
    }

    free(memlink_map.map);
    memlink_map.map = NULL;
}
