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
#define PHT_SIZE 256
#define BTB_SIZE 1024

typedef struct
{
     /* gshare */
     int ghr_bits;
     uint32_t ghr;
     uint8_t *pht; 
 
     /* BTB */
     int btb_size;
     int btb_bits;
     uint64_t btb_tag[BTB_SIZE];
     uint64_t btb_dest[BTB_SIZE];
     bool *btb_valid;
     bool btb_cond[BTB_SIZE];
} bp_t;


void bp_init(bp_t *bp);
void bp_predict(bp_t *bp, uint64_t *PC);
void bp_update(bp_t *bp, uint64_t PC, uint64_t target, bool taken, bool is_cond);
bool predicted(uint64_t prediction, uint64_t target,  bool taken);
void bp_free(bp_t *bp);

#endif
