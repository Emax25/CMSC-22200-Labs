/***************************************************************/
/*                                                             */
/*   ARM Instruction Level Simulator                           */
/*                                                             */
/*   CMSC-22200 Computer Architecture                          */
/*   University of Chicago                                     */
/*                                                             */
/***************************************************************/

#include "bp.h"
#include <stdlib.h>
#include <stdio.h>

#define PHT_SIZE 256
#define BTB_SIZE 1024
#define GHR_MASK 0xFF


void bp_init(bp_t *bp)
{
    bp->ghr_bits = 8;
    bp->ghr = 0;
    bp->pht = calloc(PHT_SIZE, sizeof(uint8_t));

    bp->btb_size = BTB_SIZE;
    bp->btb_bits = 10;
    bp->btb_tag = calloc(BTB_SIZE, sizeof(uint64_t));
    bp->btb_dest = calloc(BTB_SIZE, sizeof(uint64_t));
    bp->btb_valid = calloc(BTB_SIZE, sizeof(uint8_t));
    bp->btb_cond = calloc(BTB_SIZE, sizeof(uint8_t));
}

uint64_t bp_predict(bp_t *bp, uint64_t PC, int *prediction)
{
    int btb_index = (PC >> 2) & (BTB_SIZE - 1); 
    if (bp->btb_valid[btb_index] && bp->btb_tag[btb_index] == PC) {
        if (bp->btb_cond[btb_index] == 0) {
            *prediction = 1;
            return bp->btb_dest[btb_index];
        } 
        else {
            int pht_index = (bp->ghr ^ ((PC >> 2) & GHR_MASK)) & GHR_MASK; 
            if (bp->pht[pht_index] >= 2) {
                *prediction = 1;
                return bp->btb_dest[btb_index];
            }
        }
    } 
    else *prediction = -1;
    return PC + 4;
}

void bp_update(bp_t *bp, uint64_t PC, uint64_t target, int taken, int is_cond) 
{
    int pht_index = (bp->ghr ^ ((PC >> 2) & GHR_MASK)) & GHR_MASK;

    if (taken && bp->pht[pht_index] < 3) bp->pht[pht_index]++;
    else if (!taken && bp->pht[pht_index] > 0) bp->pht[pht_index]--;

    if (is_cond) {
        bp->ghr = ((bp->ghr << 1) | (taken ? 1 : 0)) & GHR_MASK;
    }

    int btb_index = (PC >> 2) & (BTB_SIZE - 1);
    bp->btb_tag[btb_index] = PC;
    bp->btb_dest[btb_index] = target;
    bp->btb_valid[btb_index] = 1;
    bp->btb_cond[btb_index] = is_cond;
}
