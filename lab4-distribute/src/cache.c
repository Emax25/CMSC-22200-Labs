/*
 * CMSC 22200, Fall 2016
 *
 * ARM pipeline timing simulator
 *
 */

#include "cache.h"
#include <stdlib.h>
#include <stdio.h>


cache_t *cache_new(int sets, int ways)
{
    cache_t *cache = (cache_t *)malloc(sizeof(cache_t));
    cache->num_sets = sets;
    cache->num_ways = ways;
    cache->accesses = 0;
    cache->cycles = 0;
    cache->waiting = false;

    cache->sets = (cache_block_t **)malloc(sizeof(cache_block_t *) * sets);

    for (int i = 0; i < sets; i++) {
        cache->sets[i] = (cache_block_t *)calloc(ways, sizeof(cache_block_t));
    }

    return cache;
}

void cache_destroy(cache_t *c)
{
    for (int i = 0; i < c->num_sets; i++){
        free(c->sets[i]);
    }
    free(c->sets);
    free(c);
}

int cache_update(cache_t *c, uint64_t addr)
{
    int set_idx;
    uint64_t tag;
    
    if (c->num_sets == 64){
        set_idx = (addr >> 5) & 0x3F;
        tag = addr >> 11;
    }
    else{
        set_idx = (addr >> 5) & 0xFF;
        tag = addr >> 13;
    }

    c->accesses++;
    cache_block_t *set = c->sets[set_idx];

    for (int i = 0; i < c->num_ways; i++) {
        if (set[i].valid && set[i].tag == tag) {
            set[i].last_used = c->accesses;
            return 0; 
        }
    }

    return 1; 
}

void cache_insert(cache_t *c, uint64_t addr){
    int set_idx;
    uint64_t tag;
    
    if (c->num_sets == 64){
        set_idx = (addr >> 5) & 0x3F;
        tag = addr >> 11;
    }
    else{
        set_idx = (addr >> 5) & 0xFF;
        tag = addr >> 13;
    }

    cache_block_t *set = c->sets[set_idx];

    int lru_idx = 0;
    uint64_t last_used = set[0].last_used;
    
    for (int i = 1; i < c->num_ways; i++) {
        if (!set[i].valid) {
            set[i].valid = true;
            set[i].tag = tag;
            set[i].last_used = c->accesses;
            return;
        }
        if (set[i].last_used < last_used) {
            last_used = set[i].last_used;
            lru_idx = i;
        }
    }

    set[lru_idx].tag = tag;
    set[lru_idx].valid = true;
    set[lru_idx].last_used = c->accesses;
}

bool same_block(cache_t *c, uint64_t addr, uint64_t target){
    int targ_set = (target>> 5) & 0x3F;
    int curr_set = (addr >> 5) & 0x3F;
    
    return targ_set == curr_set;
}
