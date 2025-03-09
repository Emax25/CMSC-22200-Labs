/*
* CMSC 22200
*
* ARM pipeline timing simulator
*/

#include "pipe.h"
#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>

#define RTYPE 6
#define ITYPE 1
#define DTYPE 2
#define BTYPE 3
#define CTYPE 4
#define IWTYPE 5
#define BUBBLE 10

/* global pipeline state */
Pipe_State pipe;

/* global pipeline registers*/
Pipe_Reg_IFtoDE IF_DE;
Pipe_Reg_DEtoEX DE_EX;
Pipe_Reg_EXtoMEM EX_MEM;
Pipe_Reg_MEMtoWB MEM_WB;

int RUN_BIT;
int HLT;
int STALL;
static int prints = true; 
static int prints2 = false; 

/* static constant list of intruction type tuples */
static const ins_type TYPE_LIST[6] = {
    {BTYPE, 6},
    {CTYPE, 8},
    {ITYPE, 10},
    {RTYPE, 11},
    {DTYPE, 11},  
    {IWTYPE, 11}
};

#define TYPE_LIST_SIZE (sizeof(TYPE_LIST) / sizeof(TYPE_LIST[0]))

void pipe_init()
{
    memset(&pipe, 0, sizeof(Pipe_State));
    pipe.PC = 0x00400000;
    RUN_BIT = TRUE;
    HLT = FALSE;
    STALL = FALSE;
    initialize_pipe_registers();
    pipe.bp = malloc(sizeof(bp_t));
    bp_init(pipe.bp);
    pipe.icache = cache_new(64, 4);
    pipe.dcache = cache_new(256, 8);
}

void pipe_cycle()
{  
    pipe_stage_wb();
    if(RUN_BIT) {
        pipe_stage_mem();
        if (!STALL)
        {   
            pipe_stage_execute();
            pipe_stage_decode();
            pipe_stage_fetch();
            incr_PC();
        }
        else if (pipe.icache->waiting){
            pipe.icache->cycles--;
        }
    }
    else{
        free_pipeline();
    }
    if (prints) printf("Pipe Cycle: %0lX\n", pipe.PC); 
}

void flush_pipeline() {
    EX_MEM.flushed = true;
    IF_DE.operation = initialize_operation();
    IF_DE.operation.is_bubble = true;
    // DE_EX.operation = initialize_operation();
    // DE_EX.operation.is_bubble = true;
}

void incr_PC(){
    if (IF_DE.sec_stall) IF_DE.sec_stall = false;
    else if (!HLT && !pipe.icache->waiting && !EX_MEM.flushed) { 
        bp_predict(pipe.bp, &pipe.PC);
    }
}

void delay(int milliseconds) {
    clock_t start_time = clock();
    while (clock() < start_time + milliseconds * CLOCKS_PER_SEC / 1000);
}

void pipe_stage_wb()
{   
    if (STALL)
    {
        EX_MEM.operation.is_bubble = true;
        STALL = false;
    }
    if(MEM_WB.operation.is_bubble) {
        if(prints) printf("In WB      | BUBBLE\n");
        return; 
    }
    if(prints) printf("In WB      | word: %0X\n", MEM_WB.operation.word);

    // Update pipe
    Pipe_Op operation = MEM_WB.operation; 
    forward_WB_EX(operation);

    uint8_t type = operation.type;
    if (operation.mod_reg)
    {
        uint8_t Rt = operation.Rt;
        pipe.REGS[Rt] = MEM_WB.REGS[Rt];
    }
    if (operation.flagSet) {
        pipe.FLAG_N = MEM_WB.FLAG_N; 
        pipe.FLAG_Z = MEM_WB.FLAG_Z;
        if(prints2) printf("PIPE Flag_Z: %0X\n", pipe.FLAG_Z);
        if(prints2) printf("PIPE Flag_N: %0X\n", pipe.FLAG_N);
    }

    if (prints2) printf("Pipe.PC: %0lX\n", pipe.PC); 

    if (MEM_WB.operation.opcode != 0) {
        stat_inst_retire += 1; 
    }
    if (operation.opcode == 0x6A2) {
        RUN_BIT = FALSE;
    }

}

void pipe_stage_mem()
{
    Pipe_Op operation = EX_MEM.operation;
    int64_t *regs = EX_MEM.REGS;
    EX_MEM.flushed = false;
    if (pipe.dcache->waiting){
        pipe.dcache->cycles--;
        if (pipe.dcache->cycles <= 0){
            pipe.dcache->waiting = false;
            operation.is_bubble = false;
            int64_t DT_address = operation.address;
            uint8_t Rn = operation.Rn;
            cache_insert(pipe.dcache, regs[Rn] + DT_address);
        }
        else{
            MEM_WB.operation.is_bubble = true;
            STALL = true;
            return;
        }
    }
    if (operation.is_bubble) {
        MEM_WB.operation = EX_MEM.operation; 
        if(prints) printf("In MEM     | BUBBLE\n");

        return;
    }
 
    uint64_t PC = EX_MEM.PC; 
    uint8_t type = operation.type; 
    uint16_t opcode = operation.opcode;

    if (!EX_MEM.stalled) forward_MEM_EX(operation);

    if (type == DTYPE) {
        int64_t DT_address = operation.address;
        uint8_t op = operation.misc;
        uint8_t Rn = operation.Rn;
        uint8_t Rt = operation.Rt;

        if (cache_update(pipe.dcache, regs[Rn] + DT_address) == 1){
            pipe.dcache->waiting = true;
            MEM_WB.operation.is_bubble = true;
            pipe.dcache->cycles = 50;
            STALL = false;
            EX_MEM.stalled = true;
            return;
        }

        switch(opcode) {
            case 0x7C2:  // LDUR
            {
                uint64_t low_half = mem_read_32(regs[Rn] + DT_address);        
                uint64_t high_half = mem_read_32(regs[Rn] + DT_address + 4);
                regs[Rt] = (high_half << 32) | low_half;
                operation.mod_reg = true;
                break;
            }
            case 0x5C2:  // LDUR (32-bit)
                regs[Rt] = mem_read_32(regs[Rn] + DT_address); 
                operation.mod_reg = true;
                break;   
            case 0x1C2:  // LDURB
                regs[Rt] = (mem_read_32(regs[Rn] + DT_address) & 0xFF);
                operation.mod_reg = true;
                break; 
            case 0x3C2:  // LDURH
                regs[Rt] = (mem_read_32(regs[Rn] + DT_address) & 0xFFFF);
                operation.mod_reg = true;
                break;
            case 0x7C0:  // STUR
            {
                uint64_t wr_addr = regs[Rn] + DT_address;
                int32_t first_half = regs[Rt] & 0xFFFFFFFF;
                int32_t second_half = (regs[Rt] >> 32) & 0xFFFFFFFF;

                mem_write_32(wr_addr, first_half);
                mem_write_32(wr_addr + 4, second_half);
                break;
            }
            case 0x5C0:  // STUR (32-bit)
            {
                int32_t write = regs[Rt] & 0xFFFFFFFF;

                mem_write_32(regs[Rn] + DT_address, write);  
                break;
            }
            case 0x1C0:  // STURB
            {
                uint64_t wr_addr = regs[Rn] + DT_address;
                uint8_t write = regs[Rt] & 0xFF;
                int32_t word = (mem_read_32(wr_addr) & 0xFFFFFF00) + write; 
                mem_write_32(wr_addr, word);
                break;
            }
            case 0x3C0:  // STURH
            {
                uint64_t wr_addr = regs[Rn] + DT_address;
                uint16_t write = regs[Rt] & 0xFFFF;
                int32_t word = (mem_read_32(wr_addr) & 0xFFFF0000) + write; 
                mem_write_32(wr_addr, word);
                break;
            }
            default:
                printf("ERROR: Unknown Instruction in DTYPE\n");
                RUN_BIT = FALSE;
        }

    }
    
    EX_MEM.stalled = false;
    memcpy(MEM_WB.REGS, regs, ARM_REGS * sizeof(int64_t));
    MEM_WB.operation = operation; 
    MEM_WB.PC = PC; 
    MEM_WB.FLAG_N = EX_MEM.FLAG_N; 
    MEM_WB.FLAG_Z = EX_MEM.FLAG_Z; 
    if(prints) printf("In MEM     | word: %0X\n", EX_MEM.operation.word);
}

void pipe_stage_execute()
{
    EX_MEM.flushed = false;
    if (DE_EX.operation.is_bubble){
        if (!pipe.dcache->waiting) EX_MEM.operation = DE_EX.operation; 
        if(prints) printf("In EXECUTE | BUBBLE\n");
        return;
    }
    Pipe_Op operation = DE_EX.operation; 
    uint8_t type = operation.type; 
    uint16_t opcode = operation.opcode;
    int64_t *regs = DE_EX.REGS;
    uint64_t PC = DE_EX.PC; 
    int FLAG_Z = DE_EX.FLAG_Z; 
    int FLAG_N = DE_EX.FLAG_N; 
    
    if (type == CTYPE) {
        int64_t COND_BR_address = operation.address;
        int8_t Rt = operation.Rt;
        switch(opcode) {
            case 0x5A8 ... 0x5AF:  // CBNZ
                if(regs[Rt] != 0) {
                    PC = PC + COND_BR_address; 
                    operation.will_jump = true;
                } 
                break; 
            case 0x5A0 ... 0x5A7:  // CBZ
                if(regs[Rt] == 0) {
                    operation.will_jump = true;
                    PC = PC + COND_BR_address; 
                } 
                break; 
            case 0x2A0 ... 0x2A7:       // B.cond 
                if (Rt == 0) { //BEQ
                    if (FLAG_Z) {
                        PC = PC + COND_BR_address; 
                        operation.will_jump = true;
                    } 
                    break;
                } else if (Rt == 1) { // BNE
                    if (!FLAG_Z) {
                        PC = (int64_t) (PC + COND_BR_address); 
                        operation.will_jump = true;
                    } 
                    break;
                } else if (Rt == 12) { // BGT
                    if(!FLAG_N) {
                        PC = (int64_t) (PC + COND_BR_address); 
                        operation.will_jump = true;
                    } 
                    break;
                } else if (Rt == 11) { // BLT 
                    if(FLAG_N) {
                        PC = (int64_t) (PC + COND_BR_address); 
                        operation.will_jump = true;
                    }
                    break; 
                } else if (Rt == 10) { // BGE 
                    if(FLAG_Z || !FLAG_N) {
                        PC = (int64_t) (PC + COND_BR_address); 
                        operation.will_jump = true;
                    } 
                    break; 
                } else if (Rt == 13) { // BLE 
                    if(FLAG_Z || FLAG_N) {
                        PC = (int64_t) (PC + COND_BR_address); 
                        operation.will_jump = true;
                    } 
                    break; 
                }
                break;
            default:
                printf("ERROR: Unknown Instruction in CTYPE");
                RUN_BIT = FALSE;
        }
    }
    else if (type == ITYPE) {

        uint16_t ALU_immediate = operation.immediate;
        uint8_t Rn = operation.Rn;
        uint8_t Rt = operation.Rt;
        uint8_t shamt = ALU_immediate & 0x3F; // Extract bits [4:0] for shamt (5 bits).
        int8_t shift = (ALU_immediate >> 6) & 0xFF; 
        operation.mod_reg = true;


        switch(opcode) {
            case 0x488 ... 0x489:   // ADDI
                regs[Rt] = regs[Rn] + ALU_immediate;
                break; 
            case 0x588 ... 0x589:   // ADDIS
                regs[Rt] = regs[Rn] + ALU_immediate;
                FLAG_Z = (regs[Rt] == 0);       
                FLAG_N = (regs[Rt] < 0);
                operation.flagSet = true; 
                break; 
            case 0x688 ... 0x689:    // SUBI
                regs[Rt] = regs[Rn] - ALU_immediate;
                break; 
            case 0x788 ... 0x789:   // SUBIS
                regs[Rt] = regs[Rn] - ALU_immediate;
                FLAG_Z = (regs[Rt] == 0);
                FLAG_N = (regs[Rt] < 0);
                operation.flagSet = true; 
                break; 
            case 0x69A ... 0x69B:             // LSR
                if (shamt == 0x3F) { // LSR
                    regs[Rt] = regs[Rn] >> shift;  
                    break;
                } else { // LSL
                    shift = ((-shift) % 64 + 64) % 64;
                    regs[Rt] = regs[Rn] << shift; 
                    break;
                }
            default: 
                printf("ERROR: Unknown Instruction in ITYPE\n");
        }
    }
    else if (type == RTYPE)
    {
        uint8_t Rt = operation.Rt;
        uint8_t Rm = operation.Rm;
        uint8_t Rn = operation.Rn;
        uint8_t shamt = operation.misc;
        switch (opcode)
        {
            case 0x458:             // ADD
                regs[Rt] = regs[Rn] + regs[Rm];
                operation.mod_reg = true;
                break;
            case 0x558:             // ADDS
                regs[Rt] = regs[Rn] + regs[Rm];
                FLAG_Z = (regs[Rt] == 0);
                FLAG_N = (regs[Rt] < 0);
                operation.flagSet = true; 
                operation.mod_reg = true;
                break;
            case 0x450:             // AND     
                regs[Rt] = regs[Rn] & regs[Rm];
                operation.mod_reg = true;
                break;
            case 0x750:             // ANDS
                regs[Rt] = regs[Rn] & regs[Rm];
                FLAG_Z = (regs[Rt] == 0);
                FLAG_N = (regs[Rt] < 0);
                operation.flagSet = true; 
                operation.mod_reg = true;
                break;
            case 0x650:             // EOR
                regs[Rt] = regs[Rn] ^ regs[Rm];
                operation.mod_reg = true;
                break;
            case 0x550:             // ORR
                regs[Rt] = regs[Rn] | regs[Rm];
                operation.mod_reg = true;
                break;
            case 0x658:             // SUB
                regs[Rt] = regs[Rn] - regs[Rm];
                operation.mod_reg = true;
                break;
            case 0x758:             // SUBS
                regs[Rt] = regs[Rn] - regs[Rm];
                FLAG_Z = (regs[Rt] == 0);
                FLAG_N = (regs[Rt] < 0);
                operation.flagSet = true; 
                operation.mod_reg = true;
                break;
            case 0x4D8:             // MUL
                regs[Rt] = regs[Rn] * regs[Rm];
                operation.mod_reg = true;
                break;
            case 0x6B0:             // BR
                PC = (uint64_t) regs[Rn]; 
                operation.will_jump = true;
                break;
            default:
                printf("ERROR: Uknown Instruction");
                RUN_BIT = FALSE;
        }
    }
    else if (type == IWTYPE)
    {
        uint8_t Rt = operation.Rt;
        uint16_t immediate = operation.immediate; 
        switch (opcode)
        {
            case 0x694 ... 0x697:   // MOVZ
                if (prints2) printf("Moving immediate, %0X, to register rt, %d\n", immediate, Rt); 
                regs[Rt] = immediate;
                operation.mod_reg = true;
                break;
            case 0x6A2:             // HLT
                HLT = TRUE; 
                break;
            default:
                printf("ERROR: Uknown Instruction");
                RUN_BIT = FALSE;
        }
    }
    else if (type == BTYPE)
    {
        // Use *signed* 64-bit here too
        int64_t branchOffset = operation.address;
        switch (opcode)
        {
            case 0x0A0 ... 0x0BF:   // B
                PC = PC + branchOffset;
                operation.will_jump = true;
                break;
            default:
                printf("ERROR: Unknown Instruction");
                RUN_BIT = FALSE;
        }
    }

    if(prints) printf("In EXECUTE | word: %0X\n", operation.word);

    if (!DE_EX.stalled){
        bp_update(pipe.bp, DE_EX.PC, PC, operation.will_jump, type == CTYPE);

        uint64_t target = PC;
        if (!operation.will_jump) target += 4;

        if (!predicted(IF_DE.PC, target) && PC != 0) {
            flush_pipeline();
            if (pipe.icache->waiting){
                if (!same_block(pipe.icache, pipe.PC, target)){
                    pipe.icache->waiting = false;
                    pipe.icache->cycles = 0;
                }
                else{
                    EX_MEM.flushed = false;
                }
            }
            pipe.PC = target;
        }
    }

    DE_EX.stalled = true;

    if (pipe.dcache->waiting) return; 

    DE_EX.stalled = false;
    memcpy(EX_MEM.REGS, regs, ARM_REGS * sizeof(int64_t));
    EX_MEM.REGS[31] = 0;
    EX_MEM.operation = operation; 
    EX_MEM.FLAG_N = FLAG_N; 
    EX_MEM.FLAG_Z = FLAG_Z; 
    EX_MEM.PC = PC; 
}

void pipe_stage_decode()
{
    if (IF_DE.operation.is_bubble){
        if (!pipe.dcache->waiting) DE_EX.operation = IF_DE.operation; 
        if(prints) printf("In DECODE  | BUBBLE\n");
        return;
    }

    uint16_t opcode;
    ins_type type_tuple;
    uint32_t type;
    int op_len; 
    uint32_t word = IF_DE.operation.word;


    for(int i = 0; i < TYPE_LIST_SIZE; i++) {
        ins_type type_tuple = TYPE_LIST[i]; 
        type = type_tuple.type; 
        op_len = type_tuple.value;  
        opcode = word >> (32 - op_len);
        opcode = opcode << (11 - op_len);
        bool status = find_operation(type, opcode, word); 
        if (status) {
            break; 
        }
    }
    
    if(prints) printf("In DECODE  | word: %0X, opcode: %0X\n", word, opcode);
    if (pipe.dcache->waiting) return;

    memcpy(DE_EX.REGS, pipe.REGS, ARM_REGS * sizeof(int64_t));
    DE_EX.PC = IF_DE.PC; 
    DE_EX.FLAG_N = pipe.FLAG_N;
    DE_EX.FLAG_Z = pipe.FLAG_Z;
    DE_EX.operation = IF_DE.operation;
}

void forward_MEM_EX(Pipe_Op operation) {
    if (operation.is_load && (operation.Rt == DE_EX.operation.Rn || operation.Rt == DE_EX.operation.Rm))
    {
        STALL = true;
    }
    else if (operation.is_store && DE_EX.operation.is_load)
    {
        uint64_t wr_addr = EX_MEM.REGS[operation.Rn] + operation.address;
        uint64_t ld_addr = DE_EX.REGS[DE_EX.operation.Rn] + DE_EX.operation.address;
        if (ld_addr > wr_addr - 4 && ld_addr < wr_addr + 4)
        {
            STALL = true;
        }
    }
    if (operation.mod_reg && DE_EX.operation.is_store && operation.Rt == DE_EX.operation.Rt)
    {
        DE_EX.REGS[DE_EX.operation.Rt] = EX_MEM.REGS[operation.Rt];
    }
    if (operation.mod_reg && operation.Rt == DE_EX.operation.Rn)
    {
        DE_EX.REGS[DE_EX.operation.Rn] = EX_MEM.REGS[operation.Rt];
    }
    if (operation.mod_reg && operation.Rt == DE_EX.operation.Rm)
    {
        DE_EX.REGS[DE_EX.operation.Rm] = EX_MEM.REGS[operation.Rt];
    }
    if (operation.flagSet)
    {
        DE_EX.FLAG_N = EX_MEM.FLAG_N; 
        DE_EX.FLAG_Z = EX_MEM.FLAG_Z; 
    }
    if (DE_EX.operation.type == CTYPE) {
        if (DE_EX.operation.Rt == EX_MEM.operation.Rt) {
            DE_EX.REGS[DE_EX.operation.Rt] = EX_MEM.REGS[operation.Rt]; 
        }
    }

}

void forward_WB_EX(Pipe_Op operation) {

    if (operation.mod_reg && operation.Rt == DE_EX.operation.Rn)
    {
        DE_EX.REGS[DE_EX.operation.Rn] = MEM_WB.REGS[operation.Rt];
    }
    if (operation.mod_reg && operation.Rt == DE_EX.operation.Rm)
    {
        DE_EX.REGS[DE_EX.operation.Rm] = MEM_WB.REGS[operation.Rt];
    }
    if (operation.flagSet)
    {
        DE_EX.FLAG_N = MEM_WB.FLAG_N; 
        DE_EX.FLAG_Z = MEM_WB.FLAG_Z; 
    }
    if (operation.mod_reg && DE_EX.operation.is_store && operation.Rt == DE_EX.operation.Rt)
    {
        DE_EX.REGS[DE_EX.operation.Rt] = MEM_WB.REGS[operation.Rt];
    }
}

void pipe_stage_fetch()
{
    if (EX_MEM.flushed){
        IF_DE.operation.is_bubble = true;
        return;
    }
    if (pipe.icache->waiting){
        pipe.icache->cycles--;
        if (pipe.icache->cycles <= 0){
            pipe.icache->waiting = false;
            cache_insert(pipe.icache, pipe.PC);
        }
        else{
            IF_DE.operation.is_bubble = true;
            return;
        }
    }

    if (IF_DE.stalled){
        IF_DE.PC = IF_DE.stalled_PC;
        IF_DE.sec_stall = true;
        IF_DE.stalled = false;
    }

    if (cache_update(pipe.icache, pipe.PC) == 1){
        pipe.icache->waiting = true;
        IF_DE.operation.is_bubble = true;
        pipe.icache->cycles = 50;
        return;
    }
    else IF_DE.PC = pipe.PC;  

    if (pipe.dcache->waiting){
        IF_DE.stalled_PC = pipe.PC;
        IF_DE.stalled = true;
        return;
    } 

    // if (IF_DE.stalled){
    //     IF_DE.PC = IF_DE.stalled_PC;
    //     IF_DE.sec_stall = true;
    //     IF_DE.stalled = false;
    // }

    IF_DE.operation = initialize_operation(); 
    IF_DE.operation.word = mem_read_32(IF_DE.PC);
    IF_DE.operation.PC = IF_DE.PC; 
    if(prints) printf("In Fetch   | word: %0X\n", IF_DE.operation.word);
}

bool find_operation(uint8_t type, uint16_t opcode, uint32_t word)
{  
    if (type == BTYPE)
    {   
        switch(opcode)
        {
            case 0x0A0 ... 0x0BF:   // B
                break;
            default:
                return false; 
        }
        return decode_B(word, opcode); 
    }
    else if (type == CTYPE)
    {
        switch(opcode)
        {
            case 0x5A8 ... 0x5AF:   // CBNZ
                break;
            case 0x5A0 ... 0x5A7:   // CBZ
                break;
            case 0x2A0 ... 0x2A7:   // B.cond 
                break;
            default:
                return false; 
        }
        return decode_CB(word, opcode);
    }
    else if (type == ITYPE)
    {
        switch(opcode)
        {
            case 0x488 ... 0x489:   // ADDI
                break;
            case 0x588 ... 0x589:   // ADDIS
                break;
            case 0x688 ... 0x689:   // SUBI
                break;
            case 0x788 ... 0x789:   // SUBIS 
                break;
            case 0x69A ... 0x69B:   // LSL or LSR
                break;
            default:
                return false;
        }
        return decode_I(word, opcode);
    }
    else if (type == RTYPE)
    {
        switch(opcode)
        {
            case 0x458:             // ADD
                break;
            case 0x558:             // ADDS
                break;
            case 0x450:             // AND     
                break;
            case 0x750:             // ANDS
                break;
            case 0x650:             // EOR
                break;
            case 0x550:             // ORR
                break;
            case 0x658:             // SUB
                break;
            case 0x758:             // SUBS
                break;
            case 0x4D8:             // MUL
                break;
            case 0x6B0:             // BR
                break;
            default:
                return false;
        }
        return decode_R(word, opcode);
    }
    else if (type == DTYPE)
    {
        switch(opcode)
        {
            case 0x7C2:             // LDUR
                IF_DE.operation.is_load = true;
                break;
            case 0x5C2:             // LDUR (32-bit)
                IF_DE.operation.is_load = true;
                break;
            case 0x1C2:             // LDURB
                IF_DE.operation.is_load = true;
                break;
            case 0x3C2:             // LDURH
                IF_DE.operation.is_load = true;
                break;
            case 0x7C0:             // STUR
                IF_DE.operation.is_store = true;
                break;
            case 0x5C0:             // STUR (32-bit)
                IF_DE.operation.is_store = true;
                break;
            case 0x1C0:             // STURB
                IF_DE.operation.is_store = true;
                break;
            case 0x3C0:             // STURH
                IF_DE.operation.is_store = true;
                break;
            default:
                return false;
        }
        return decode_D(word, opcode);
    }
    else if (type == IWTYPE)
    {
        switch(opcode)
        {
            case 0x694 ... 0x697:   // MOVZ
                break;
            case 0x6a2:             // HLT
                break;
            default:
                return false;
        }
        return decode_IW(word, opcode);
    }
    return false;
}

Pipe_Op initialize_operation()
{ 
    Pipe_Op operation; 
    operation.address = 0; 
    operation.immediate = 0;
    operation.misc = 0;
    operation.opcode = 0; 
    operation.Rm = 0;
    operation.Rn = 0;
    operation.Rt = 0; 
    operation.word = 0;
    operation.flagSet = false; 
    operation.mod_reg = false;
    operation.is_load = false; 
    operation.is_store = false; 
    operation.will_jump = false; 
    return operation; 
}

void initialize_pipe_registers()
{
    memset(&IF_DE, 0, sizeof(Pipe_Reg_IFtoDE));
    memset(&DE_EX, 0, sizeof(Pipe_Reg_DEtoEX));
    memset(&EX_MEM, 0, sizeof(Pipe_Reg_EXtoMEM));
    memset(&MEM_WB, 0, sizeof(Pipe_Reg_MEMtoWB));
}

bool decode_R(uint32_t word, uint16_t opcode) {
    /* Decoding helper function for R format words */
    IF_DE.operation.opcode = opcode;
    IF_DE.operation.type = RTYPE;
    IF_DE.operation.Rm = (word >> 16) & 0x1F; 
    IF_DE.operation.misc = (word >> 10) & 0x3F; 
    IF_DE.operation.Rn = (word >> 5) & 0x1F; 
    IF_DE.operation.Rt = word & 0x1F;
    return true;
}

bool decode_D(uint32_t word, uint16_t opcode) {
    /* Decoding helper function for D format words */
    IF_DE.operation.opcode = opcode;
    IF_DE.operation.type = DTYPE; 
    IF_DE.operation.address = ((word >> 12) & 0x1FF);
    IF_DE.operation.misc = (word >> 10) & 0x3;
    IF_DE.operation.Rn = (word >> 5) & 0x1F;
    IF_DE.operation.Rt = word & 0x1F;
    return true;
}

bool decode_B(uint32_t word, uint16_t opcode) {

    IF_DE.operation.opcode = opcode;
    IF_DE.operation.type   = BTYPE;
    int32_t imm26 = word & 0x03FFFFFF; 
    if (imm26 & (1 << 25)) {
        imm26 |= 0xFC000000;
    }
    imm26 <<= 2;
    IF_DE.operation.address = imm26;
    return true;
}

bool decode_CB(uint32_t word, uint16_t opcode) {
    IF_DE.operation.opcode = opcode; 
    IF_DE.operation.type = CTYPE;
    int32_t imm19 = (word >> 5) & 0x7FFFF;
    if (imm19 & (1 << 18)) {
        imm19 |= 0xFFF80000;  // Set bits 31:19 to extend the sign
    }
    imm19 <<= 2;
    int64_t address = (int64_t) imm19;
    IF_DE.operation.address = address;
    IF_DE.operation.Rt = word & 0x1F;
    return true;
}

bool decode_IW(uint32_t word, uint16_t opcode) {
    /* Decoding helper function for IW format words */
    IF_DE.operation.opcode = opcode;
    IF_DE.operation.type = IWTYPE; 
    IF_DE.operation.immediate = (word >> 5) & 0xFFFF; 
    IF_DE.operation.Rt = word & 0x1F;
    return true;
}

bool decode_I(uint32_t word, uint16_t opcode) {
    /* Decoding helper function for I format words */
    IF_DE.operation.opcode = opcode; 
    IF_DE.operation.type = ITYPE; 
    IF_DE.operation.immediate = (word >> 10) & 0xFFF; 
    IF_DE.operation.Rn = (word >> 5) & 0x1F;
    IF_DE.operation.Rt = word & 0x1F;
    return true;
}

void free_pipeline(){
    bp_free(pipe.bp);
    free(pipe.bp);
    pipe.bp = NULL;
    cache_destroy(pipe.icache);
    cache_destroy(pipe.dcache);
}