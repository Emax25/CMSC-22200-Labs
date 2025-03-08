/*
 * CMSC 22200, Fall 2016
 *
 * ARM pipeline timing simulator
 *
 */
#ifndef _CACHE_H_
#define _CACHE_H_

#include <stdint.h>
#include "stdbool.h"

typedef struct {
    bool valid;
    uint64_t tag;
    uint64_t last_used; 
} cache_block_t;

typedef struct
{
    int num_sets;
    int num_ways;
    uint64_t accesses;
    uint64_t misses;
    int cycles;

    bool waiting;

    cache_block_t **sets;
} cache_t;

cache_t *cache_new(int sets, int ways);
void cache_destroy(cache_t *c);
int cache_update(cache_t *c, uint64_t addr);
void cache_insert(cache_t *c, uint64_t addr);

#endif
