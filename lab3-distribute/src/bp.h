/***************************************************************/
/*                                                             */
/*   ARM Instruction Level Simulator                           */
/*                                                             */
/*   CMSC-22200 Computer Architecture                          */
/*   University of Chicago                                     */
/*                                                             */
/***************************************************************/

#ifndef _BP_H_
#define _BP_H_

#include <stdint.h>
#include "stdbool.h"

typedef struct
{
     /* gshare */
     int ghr_bits;
     uint32_t ghr;
     uint8_t *pht; 
 
     /* BTB */
     int btb_size;
     int btb_bits;
     uint64_t *btb_tag;
     uint64_t *btb_dest;
     uint8_t *btb_valid;
     uint8_t *btb_cond;
} bp_t;


void bp_init(bp_t *bp);
void bp_predict(bp_t *bp, uint64_t *PC);
void bp_update(bp_t *bp, uint64_t PC, uint64_t target, bool taken, bool is_cond);

#endif
