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


void bp_init(bp_t *bp)
{
    bp->ghr_bits = 8;
    bp->ghr = 0;
    bp->pht = calloc(PHT_SIZE, sizeof(uint8_t));

    bp->btb_size = BTB_SIZE;
    bp->btb_bits = 10;
    bp->btb_tag = calloc(BTB_SIZE, sizeof(uint64_t));
    bp->btb_dest = calloc(BTB_SIZE, sizeof(uint64_t));
    bp->btb_valid = calloc(BTB_SIZE, sizeof(bool));
    bp->btb_cond = calloc(BTB_SIZE, sizeof(bool));
}

void bp_predict(bp_t *bp, uint64_t *PC)
{
    int btb_index = (*PC >> 2) & 0x3FF; 
    if (bp->btb_valid[btb_index] && bp->btb_tag[btb_index] == *PC) {
        if (!bp->btb_cond[btb_index]) {
            *PC = bp->btb_dest[btb_index];
        } 
        else {
            int pht_index = (bp->ghr ^ ((*PC >> 2) & 0xFF)); 
            if (bp->pht[pht_index] >= 2) {
                *PC = bp->btb_dest[btb_index];
            }
            else *PC += 4;
        }
    } 
    else *PC += 4;
}

void bp_update(bp_t *bp, uint64_t PC, uint64_t target, bool taken, bool is_cond) 
{    
    if (is_cond) {
        int pht_index = (bp->ghr ^ ((PC >> 2) & 0xFF));

        if (taken && bp->pht[pht_index] < 3) bp->pht[pht_index]++;
        else if (!taken && bp->pht[pht_index] > 0) bp->pht[pht_index]--;

        bp->ghr = ((bp->ghr << 1) | (taken ? 1 : 0)) & 0xFF;
    }
    else if (!taken) target = PC + 4; 

    int btb_index = (PC >> 2) & 0x3FF;
    if (!bp->btb_valid[btb_index] || bp->btb_tag[btb_index] != PC)
    {
        bp->btb_tag[btb_index] = PC;
        bp->btb_dest[btb_index] = target;
        bp->btb_valid[btb_index] = true;
        bp->btb_cond[btb_index] = is_cond;
    }
}

bool predicted(uint64_t prediction, uint64_t target,  bool taken)
{
    if (!taken){
        target += 4; 
    }
    return prediction == target;
}
