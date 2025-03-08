/***************************************************************/
/*                                                             */
/*   ARM Instruction Level Simulator                           */
/*                                                             */
/*   CMSC-22200 Computer Architecture                          */
/*   University of Chicago                                     */
/*                                                             */
/***************************************************************/

#ifndef _PIPE_H_
#define _PIPE_H_

#include "bp.h"
#include "cache.h"
#include "shell.h"
#include "stdbool.h"
#include <limits.h>

/* Represents an operation travelling through the pipeline. */
typedef struct Pipe_Op {
	uint8_t type; 
	uint16_t opcode; 
	int32_t address; 
	uint8_t Rm; 
	uint8_t Rn;
	uint8_t Rt;
	uint16_t immediate; 
	uint8_t misc; // shamt or op
	uint32_t word;  
	bool flagSet; 
	bool mod_reg;
	bool is_load;
	bool is_store;
	bool will_jump;
	bool is_bubble; 
	uint32_t PC; 
} Pipe_Op;

/* Represents the current state of the pipeline. */
typedef struct Pipe_State {
	/* register file state */
	int64_t REGS[ARM_REGS];
	int FLAG_N;        /* flag N */
	int FLAG_Z;        /* flag Z */

	/* program counter in fetch stage */
	uint64_t PC;

	/* place other information here as necessary */
    
	/* branch predictor */
    bp_t *bp; 
    cache_t *icache;
    cache_t *dcache;
} Pipe_State;

/* Represents the pipeline register between the IF and DE stage. */
typedef struct Pipe_Reg_IFtoDE {
	Pipe_Op operation; 
	uint64_t PC;
	bool is_bubble; 
} Pipe_Reg_IFtoDE;

/* Represents the pipeline register between the DE and EX stage. */
typedef struct Pipe_Reg_DEtoEX {
	Pipe_Op operation; 
	int64_t REGS[ARM_REGS];
	int FLAG_N;
	int FLAG_Z;
	uint64_t PC;
	bool isJump; 
	bool is_bubble;
} Pipe_Reg_DEtoEX;

/* Represents the pipeline register between the EX and MEM stage. */
typedef struct Pipe_Reg_EXtoMEM {
	Pipe_Op operation; 
	int64_t REGS[ARM_REGS];
	int FLAG_N;
	int FLAG_Z;
	uint64_t PC;
	bool willJump; 
	bool is_bubble;
    bool flushed;
} Pipe_Reg_EXtoMEM;

/* Represents the pipeline register between the MEM and WB stage. */
typedef struct Pipe_Reg_MEMtoWB {
	Pipe_Op operation; 
	int64_t REGS[ARM_REGS];
	int FLAG_N;
	int FLAG_Z;
	uint64_t PC;
	bool is_bubble;
} Pipe_Reg_MEMtoWB;

/* Represents an instruction type as a tuple */
typedef struct {
	uint32_t type; // To hold the mask definition type
	int value;     // To hold the associated integer value
} ins_type;

extern int RUN_BIT;
extern int HLT;
extern int STALL;

/* global variable -- pipeline state */
extern Pipe_State pipe;

/* global variables -- pipeline registers*/
extern Pipe_Reg_IFtoDE IF_DE; 
extern Pipe_Reg_DEtoEX DE_EX; 
extern Pipe_Reg_EXtoMEM EX_MEM; 
extern Pipe_Reg_MEMtoWB MEM_WB; 

/* called during simulator startup */
void pipe_init();

/* this function calls the others */
void pipe_cycle();

/* each of these functions implements one stage of the pipeline */
void pipe_stage_fetch();
void pipe_stage_decode();
void pipe_stage_execute();
void pipe_stage_mem();
void pipe_stage_wb();

/* Helper functions */
Pipe_Op initialize_operation(); 
void initialize_pipe_registers();
bool find_operation(uint8_t type, uint16_t opcode, uint32_t word);
void delay(int milliseconds); 


bool decode_R(uint32_t word, uint16_t opcode); 
bool decode_I(uint32_t word, uint16_t opcode); 
bool decode_D(uint32_t word, uint16_t opcode);
bool decode_B(uint32_t word, uint16_t opcode);
bool decode_CB(uint32_t word, uint16_t opcode);
bool decode_IW(uint32_t word, uint16_t opcode);

void incr_PC();
void forward_WB_EX(Pipe_Op operation);
void forward_MEM_EX(Pipe_Op operation);
void flush_pipeline(); 
void free_pipeline();


#endif