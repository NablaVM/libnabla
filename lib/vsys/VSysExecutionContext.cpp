#include "VSysExecutionContext.hpp"
#include "VSysMachine.hpp"
#include <assert.h>

#include <iostream>

#include "util.hpp"
#include "endian.hpp"

#if BYTE_ORDER == BIG_ENDIAN
    #warning "VSysExecutionContext: BIG_ENDIAN Systems have not been fully tested"
#endif 

#if BYTE_ORDER == PDP_ENDIAN
    #warning "VSysExecutionContext: PDP_ENDIAN Systems have not been fully tested"
#endif

namespace NABLA
{
namespace VSYS
{
    // ----------------------------------------------------------------
    //
    // ----------------------------------------------------------------

    ExecutionContext::ExecutionContext(Machine &owner,
                                       uint64_t startAddress, 
                                       Memory<NABLA_VSYS_SETTINGS_GLOBAL_MEMORY_BYTES> &global_memory, 
                                       std::vector< std::vector<uint64_t> > &functions) :
                                                                                         owner(owner),
                                                                                         contextCompleted(false),
                                                                                         global_memory(global_memory),
                                                                                         functions(functions),
                                                                                         currentInstructionBlock(startAddress),
                                                                                         switchingFunction(false)

    {
        //  For HUGE projects that run on VSYS this will be a lot of overhead. It will have to iterate over all functions. If 
        //  theres a small number thats fine, but if its 10000+ it will be a headache. A 'loader' could be made that populates 
        //  the entry function, and populates 'on the fly' as things get called (if they get called)
        for(uint64_t i = 0; i < functions.size(); i++)
        {
            InstructionBlock ib;
            ib.instruction_pointer = 0;
            ib.instructions = &functions[i];
            ib.recursion_counter = 0;
            ib.recursion_flag = false;
            contextFunctions.push_back(
                ib
            );
        }

        for(uint8_t i = 0; i < 16; i ++)
        {
            registers[i] = 0;
        }
    }

    // ----------------------------------------------------------------
    //
    // ----------------------------------------------------------------

    ExecutionContext::~ExecutionContext()
    {

    }

    // ----------------------------------------------------------------
    //
    // ----------------------------------------------------------------

    bool ExecutionContext::isContextComplete() const
    {
        return contextCompleted;
    }

    // ----------------------------------------------------------------
    //
    // ----------------------------------------------------------------

    ExecutionReturns ExecutionContext::stepExecution(uint64_t steps)
    {
        for(uint64_t cycle = 0; cycle < steps; cycle++)
        {
            uint64_t poi = this->contextFunctions[this->currentInstructionBlock].instruction_pointer;

            if(poi >=  this->contextFunctions[this->currentInstructionBlock].instructions->size())
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "Current instruction stack empty, attempting to force a return" << std::endl;
#endif
                if(!attempt_return())
                {
                    this->contextCompleted = true;
                    return ExecutionReturns::ALL_EXECUTION_COMPLETE;
                }

                /*
                    If no more intructions, and we were able to return we will find ourselves here.
                    We treat this are like the bottom of the loop, but we don't check for hardware
                    execution because we are out of instructions.. so clearly nothing is going on 
                */
                finalize_cycle();
                continue;
            }

            uint64_t ins = this->contextFunctions[this->currentInstructionBlock].instructions->at(poi);
            
            // The full first byte of the instruction
            uint8_t operation =  UTIL::extract_byte(ins, 7);

            // The 'opcode' of the instruction (first 6 bits)
            uint8_t op = (operation & 0xFC);

            // The 'id' of the isntruction (key information that tells us how to decode the rest of the instruction)
            uint8_t id = (operation & 0x03);

            // Left hand side of an operation (arg 2 of asm instruction)
            int64_t lhs = 0;

            // Right hand side of an operation (arg 3 of an asm instruction)
            int64_t rhs = 0;

            switch(op)
            {     
                case INS_NOP:
                {
                    // No op
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "NOP" << std::endl;
#endif
                    break;
                }
                case INS_LSH :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = (lhs << rhs);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "LSH : Result : " << this->registers[dest] << std::endl;
#endif
                    break;
                } 

                case INS_RSH :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = (lhs >> rhs);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "RSH : Result : " << this->registers[dest] << std::endl;
#endif
                    break;
                } 

                case INS_AND :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = (lhs & rhs);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "AND : Result : " << this->registers[dest] << std::endl;
#endif
                    break;
                } 

                case INS_OR  :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = (lhs | rhs);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "OR : Result : " << this->registers[dest] << std::endl;
#endif
                    break;
                } 

                case INS_XOR :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = (lhs ^ rhs);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "XOR : Result : " << this->registers[dest] << std::endl;
#endif
                    break;
                } 

                case INS_NOT :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = (~lhs);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "NOT : Result : " << this->registers[dest] << std::endl;
#endif
                    break;
                } 

                case INS_ADD  :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    assert(dest < 16);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = lhs+rhs;

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_ADD : Result : " << this->registers[dest] << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }          
                case INS_SUB  :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    assert(dest < 16);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = lhs-rhs;

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_SUB : Result : " << this->registers[dest] << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }          
                case INS_MUL  :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    assert(dest < 16);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);
                    this->registers[dest] = lhs*rhs;

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_MUL : Result : " << this->registers[dest] << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }          
                case INS_DIV  :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    assert(dest < 16);
                    get_arith_lhs_rhs(id, ins, &lhs, &rhs);

                    if(rhs == 0){ this->registers[dest] = 0; }
                    else { this->registers[dest] = lhs/rhs; }
                    
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_DIV : Result : " << this->registers[dest] << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }          
                case INS_ADDD :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    assert(dest < 16);

                    get_arith_lhs_rhs(0, ins, &lhs, &rhs);
                
                    this->registers[dest] = UTIL::convert_double_to_uint64( UTIL::convert_uint64_to_double(lhs) + UTIL::convert_uint64_to_double(rhs));

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_ADDD : Result : " << UTIL::convert_uint64_to_double(lhs) + UTIL::convert_uint64_to_double(rhs) << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }          
                case INS_SUBD :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    assert(dest < 16);

                    get_arith_lhs_rhs(0, ins, &lhs, &rhs);
                
                    this->registers[dest] = UTIL::convert_double_to_uint64( UTIL::convert_uint64_to_double(lhs) - UTIL::convert_uint64_to_double(rhs));

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_SUBD : Result : " << UTIL::convert_uint64_to_double(lhs) - UTIL::convert_uint64_to_double(rhs) << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }          
                case INS_MULD :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    assert(dest < 16);

                    get_arith_lhs_rhs(0, ins, &lhs, &rhs);
                
                    this->registers[dest] = UTIL::convert_double_to_uint64( UTIL::convert_uint64_to_double(lhs) * UTIL::convert_uint64_to_double(rhs));

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_MULD : Result : " << UTIL::convert_uint64_to_double(lhs) * UTIL::convert_uint64_to_double(rhs) << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }          
                case INS_DIVD :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    assert(dest < 16);

                    get_arith_lhs_rhs(0, ins, &lhs, &rhs);
                
                    this->registers[dest] = UTIL::convert_double_to_uint64( UTIL::convert_uint64_to_double(lhs) / UTIL::convert_uint64_to_double(rhs));

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_DIVD : Result : " << UTIL::convert_uint64_to_double(lhs) / UTIL::convert_uint64_to_double(rhs) << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }          
                case INS_BGT  :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);
                    
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BGT : " << (lhs > rhs) << std::endl;
#endif
                    if(lhs > rhs){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BGTE :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);
                    
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BGTE : " << (lhs >= rhs) << std::endl;
#endif
                    if(lhs >= rhs){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BLT  :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);
                    
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BLT : " << (lhs < rhs) << std::endl;
#endif
                    if(lhs < rhs){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BLTE :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);
                    
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BLTE : " << (lhs <= rhs) << std::endl;
#endif
                    if(lhs <= rhs){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BEQ  :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BEQ : " << (lhs == rhs) << std::endl;
#endif
                    if(lhs == rhs){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BNE  :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);
                
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BNE : " << (lhs != rhs) << std::endl;
#endif
                    if(lhs != rhs){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BGTD :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    double lhs_d = UTIL::convert_uint64_to_double(lhs);
                    double rhs_d = UTIL::convert_uint64_to_double(rhs);

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BGTD : " << (lhs_d > rhs_d) << std::endl;
#endif
                    if(lhs_d > rhs_d){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BGTED:
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    double lhs_d = UTIL::convert_uint64_to_double(lhs);
                    double rhs_d = UTIL::convert_uint64_to_double(rhs);

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BGTED : " << (lhs_d >= rhs_d) << std::endl;
#endif
                    if(lhs_d >= rhs_d){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BLTD :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    double lhs_d = UTIL::convert_uint64_to_double(lhs);
                    double rhs_d = UTIL::convert_uint64_to_double(rhs);

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BLTD : " << (lhs_d < rhs_d) << std::endl;
#endif
                    if(lhs_d < rhs_d){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BLTED:
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    double lhs_d = UTIL::convert_uint64_to_double(lhs);
                    double rhs_d = UTIL::convert_uint64_to_double(rhs);

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BLTED : " << (lhs_d <= rhs_d) << std::endl;
#endif
                    if(lhs_d <= rhs_d){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BEQD :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    double lhs_d = UTIL::convert_uint64_to_double(lhs);
                    double rhs_d = UTIL::convert_uint64_to_double(rhs);

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BEQD : " << (UTIL::check_double_equal(lhs_d, rhs_d)) << std::endl;
#endif
                    if(UTIL::check_double_equal(lhs_d, rhs_d)){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_BNED :
                {
                    lhs = this->registers[UTIL::extract_byte(ins, 6)];
                    rhs = this->registers[UTIL::extract_byte(ins, 5)];

                    double lhs_d = UTIL::convert_uint64_to_double(lhs);
                    double rhs_d = UTIL::convert_uint64_to_double(rhs);

                    uint64_t branchAddr = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16 | 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 2);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "BNED : " << !(UTIL::check_double_equal(lhs_d, rhs_d)) << std::endl;
#endif
                    if(!UTIL::check_double_equal(lhs_d, rhs_d)){ this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = branchAddr; continue; }
                    break;
                }          
                case INS_MOV  :
                {
                    lhs = UTIL::extract_byte(ins, 6);

                    // Move register value into another register
                    if(id == 0)
                    {
                        rhs = UTIL::extract_byte(ins, 5);
                        this->registers[lhs] = this->registers[rhs];
                    }
                    // Move numerival value into a register
                    else if (id == 1)
                    {
                        int32_t rval = (uint32_t)UTIL::extract_two_bytes(ins, 5) << 16 | 
                                       (uint32_t)UTIL::extract_two_bytes(ins, 3);

                        this->registers[lhs] =  rval;
                    }
                    else
                    {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                        std::cerr << "Error: Unknown ID bits expressed in INS_MOV" << std::endl;
#endif
                        return ExecutionReturns::UNKNOWN_INSTRUCTION;
                    }

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "MOV result : " << (int64_t)this->registers[lhs] << std::endl;
#endif
                    break;
                }
                case INS_LDB  :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    uint8_t stackSouce = UTIL::extract_byte(ins, 5);
                    uint64_t sourceAddress;

                    // Depending on the 'id' of the instruction we need to change where we are getting the value
                    // for the stack offset.
                    if(id == 0)
                    {
                        sourceAddress = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16| 
                                        (uint64_t)UTIL::extract_two_bytes(ins, 2);

                    }
                    else if (id == 1)
                    {
                        uint8_t sourceReg  = UTIL::extract_byte(ins, 4);
                        sourceAddress = this->registers[sourceReg];
                    }
                    else
                    {
                        std::cerr << "Invalid 'ldb' instruction : ID = " << id << std::endl;
                        return ExecutionReturns::UNKNOWN_INSTRUCTION;
                    }
                    
                    // Now that we have the source address and other required information, we can move on and actually do the load
                    bool okay = false;
                    uint8_t val;
                    if(stackSouce == GLOBAL_STACK)
                    {
                        okay = this->global_memory.get_8(sourceAddress, val);
                    }
                    else if ( stackSouce == LOCAL_STACK )
                    {
                        okay = this->contextFunctions[currentInstructionBlock].function_memory.get_8(
                            sourceAddress, val
                        );
                    }
                    assert(okay);
                    registers[dest] = val;

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "LDB : Result : " << this->registers[dest] << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }
                case INS_LDW  :
                {
                    uint8_t dest =  UTIL::extract_byte(ins, 6);
                    uint8_t stackSouce = UTIL::extract_byte(ins, 5);
                    uint64_t sourceAddress;

                    // Depending on the 'id' of the instruction we need to change where we are getting the value
                    // for the stack offset.
                    if(id == 0)
                    {
                        sourceAddress = (uint64_t)UTIL::extract_two_bytes(ins, 4) << 16| 
                                        (uint64_t)UTIL::extract_two_bytes(ins, 2);

                    }
                    else if (id == 1)
                    {
                        uint8_t sourceReg  = UTIL::extract_byte(ins, 4);
                        sourceAddress = this->registers[sourceReg];
                    }
                    else
                    {
                        std::cerr << "Invalid 'ldw' instruction : ID = " << id << std::endl;
                        return ExecutionReturns::UNKNOWN_INSTRUCTION;
                    }
                    
                    // Now that we have the source address and other required information, we can move on and actually do the load
                    bool okay = false;
                    uint64_t val;
                    if(stackSouce == GLOBAL_STACK)
                    {
                        okay = this->global_memory.get_64(sourceAddress, val);
                    }
                    else if ( stackSouce == LOCAL_STACK )
                    {
                        okay = this->contextFunctions[currentInstructionBlock].function_memory.get_64(
                            sourceAddress, val
                        );
                    }
                    assert(okay);
                    registers[dest] = val;

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "LDW : Result : " << this->registers[dest] << " | Dest : " << (int)dest << std::endl;
#endif
                    break;
                }         
                case INS_STB  :
                {
                    uint8_t  stackDest = UTIL::extract_byte(ins, 6);
                    uint64_t destAddress;
                    uint8_t  sourceReg;

                    // Depending on the 'id' of the instruction we need to change where we are getting the value
                    // for the stack offset.
                    if(id == 0)
                    {
                        sourceReg =  UTIL::extract_byte(ins, 1);
                        destAddress = (uint64_t)UTIL::extract_two_bytes(ins, 5) << 16| 
                                      (uint64_t)UTIL::extract_two_bytes(ins, 3);
                    }
                    else if (id == 1)
                    {
                        sourceReg = UTIL::extract_byte(ins, 4);
                        uint8_t destReg   = UTIL::extract_byte(ins, 5);
                        destAddress = this->registers[destReg];
                    }
                    else
                    {
                        std::cerr << "Invalid 'stb' instruction : ID = " << id << std::endl;
                        return ExecutionReturns::UNKNOWN_INSTRUCTION;
                    }

                    // Now that we have the destination information, perform the store
                    bool okay = false;
                    uint8_t val =  registers[sourceReg] & 0x00000000000000FF;
                    if(stackDest == GLOBAL_STACK)
                    {
                        okay = this->global_memory.set_8(destAddress, val);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "STB : GLOBAL STACK : VAL : " << (int)val << " | Dest : " << destAddress << std::endl;
#endif
                    }
                    else if ( stackDest == LOCAL_STACK )
                    {
                        okay = this->contextFunctions[currentInstructionBlock].function_memory.set_8(
                            destAddress, val
                        );
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "STB : LOCAL STACK : VAL : " << (int)val << " | Dest : " << destAddress << std::endl;
#endif
                    }
                    if(!okay)
                    {
                        std::cerr << "STB Error : DEST [" << destAddress << "] VAL [" << (int)val  << "]" << std::endl; 
                        return ExecutionReturns::EXECUTION_ERROR;
                    }
                    break;
                } 
                case INS_STW  :
                {
                    uint8_t  stackDest = UTIL::extract_byte(ins, 6);
                    uint64_t destAddress;
                    uint8_t  sourceReg;

                    // Depending on the 'id' of the instruction we need to change where we are getting the value
                    // for the stack offset.
                    if(id == 0)
                    {
                        sourceReg =  UTIL::extract_byte(ins, 1);
                        destAddress = (uint64_t)UTIL::extract_two_bytes(ins, 5) << 16| 
                                      (uint64_t)UTIL::extract_two_bytes(ins, 3);
                    }
                    else if (id == 1)
                    {
                        sourceReg = UTIL::extract_byte(ins, 4);
                        uint8_t destReg   = UTIL::extract_byte(ins, 5);
                        destAddress = this->registers[destReg];
                    }
                    else
                    {
                        std::cerr << "Invalid 'stw' instruction : ID = " << id << std::endl;
                        return ExecutionReturns::UNKNOWN_INSTRUCTION;
                    }

                    // Now that we have the destination information, perform the store
                    bool okay = false;
                    uint64_t val =  registers[sourceReg];

                    if(stackDest == GLOBAL_STACK)
                    {
                        okay = this->global_memory.set_64(destAddress, val);
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "STW : GLOBAL STACK : VAL : " << val << " | Dest : " << destAddress << std::endl;
#endif
                    }
                    else if ( stackDest == LOCAL_STACK )
                    {
                        okay = this->contextFunctions[currentInstructionBlock].function_memory.set_64(
                            destAddress, val
                        );
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "STW : LOCAL STACK : VAL : " << val << " | Dest : " << destAddress << std::endl;
#endif
                    }
                    if(!okay)
                    {
                        std::cerr << "STW Error : DEST [" << destAddress << "] VAL [" << val << "]" << std::endl; 
                        return ExecutionReturns::EXECUTION_ERROR;
                    }
                    break;
                }        
                case INS_PUSH  :
                {
                    uint8_t destStack = UTIL::extract_byte(ins, 6);
                    uint8_t sourceReg = UTIL::extract_byte(ins, 5);

                    bool okay = false;
                    if(destStack == GLOBAL_STACK)
                    {
                        okay = this->global_memory.push_8((this->registers[sourceReg] & 0x00000000000000FF));
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "PUSH : GLOBAL STACK : VAL : " << (int)(this->registers[sourceReg] & 0x00000000000000FF) << std::endl;
#endif
                    }
                    else if (destStack == LOCAL_STACK )
                    {
                        okay = this->contextFunctions[currentInstructionBlock].function_memory.push_8(
                            (this->registers[sourceReg] & 0x00000000000000FF)
                        );
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "PUSH : LOCAL STACK : VAL : " << (int)(this->registers[sourceReg] & 0x00000000000000FF) << std::endl;
#endif
                    }
                    if(!okay)
                    {
                        std::cerr << "PUSH Error " << std::endl; 
                        return ExecutionReturns::EXECUTION_ERROR;
                    }
                    break;
                }  
                case INS_PUSHW :
                {
                    uint8_t destStack = UTIL::extract_byte(ins, 6);
                    uint8_t sourceReg = UTIL::extract_byte(ins, 5);

                    bool okay = false;
                    if(destStack == GLOBAL_STACK)
                    {
                        okay = this->global_memory.push_64(this->registers[sourceReg]);
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "PUSHW : GLOBAL STACK : VAL : " << this->registers[sourceReg] << std::endl;
#endif
                    }
                    else if (destStack == LOCAL_STACK )
                    {
                        okay = this->contextFunctions[currentInstructionBlock].function_memory.push_64(
                            this->registers[sourceReg]
                        );
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "PUSHW : LOCAL STACK : VAL : " << this->registers[sourceReg] << std::endl;
#endif
                    }
                    if(!okay)
                    {
                        std::cerr << "PUSHW Error " << std::endl; 
                        return ExecutionReturns::EXECUTION_ERROR;
                    }
                    break;
                }      
                case INS_POP  :
                {
                    uint8_t destReg     = UTIL::extract_byte(ins, 6);
                    uint8_t sourceStack = UTIL::extract_byte(ins, 5);

                    bool okay = false;
                    uint8_t val;
                    if(sourceStack == GLOBAL_STACK)
                    {
                        okay = this->global_memory.pop_8(val);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "POP : GLOBAL STACK : VAL : " << (int)val << " | Dest : " << (int)destReg << std::endl;
#endif
                    }
                    else if (sourceStack == LOCAL_STACK )
                    {
                        okay = this->contextFunctions[currentInstructionBlock].function_memory.pop_8(
                            val
                        );
                     
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "POP : LOCAL STACK : VAL : " << (int)val << " | Dest : " << (int)destReg << std::endl;
#endif
                    }
                    if(!okay)
                    {
                        std::cerr << "POP Error " << std::endl; 
                        return ExecutionReturns::EXECUTION_ERROR;
                    }
                    this->registers[destReg] = val;
                    break;
                }    
                case INS_POPW  :
                {
                    uint8_t destReg     = UTIL::extract_byte(ins, 6);
                    uint8_t sourceStack = UTIL::extract_byte(ins, 5);

                    bool okay = false;
                    uint64_t val;
                    if(sourceStack == GLOBAL_STACK)
                    {
                        okay = this->global_memory.pop_64(val);
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "POPW : GLOBAL STACK : VAL : " << val << " | Dest : " << (int)destReg << std::endl;
#endif
                    }
                    else if (sourceStack == LOCAL_STACK )
                    {
                        okay = this->contextFunctions[currentInstructionBlock].function_memory.pop_64(
                            val
                        );
                        
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "POPW : LOCAL STACK : VAL : " << val << " | Dest : " << (int)destReg << std::endl;
#endif
                    }
                    
                    if(!okay)
                    {
                        std::cerr << "POPW Error " << std::endl; 
                        return ExecutionReturns::EXECUTION_ERROR;
                    }
                    this->registers[destReg] = val;
                    break;
                }          
                case INS_SIZE :
                {
                    uint8_t destReg         = UTIL::extract_byte(ins, 6);
                    uint8_t stackInQuestion = UTIL::extract_byte(ins, 5);

                    if(stackInQuestion == GLOBAL_STACK)
                    {
                        this->registers[destReg] = this->global_memory.getSize();
                    }
                    else if (stackInQuestion == LOCAL_STACK )
                    {
                        this->registers[destReg] = this->contextFunctions[
                            currentInstructionBlock
                            ].function_memory.getSize();
                    }
                    else
                    {
                        std::cerr << "Invalid 'size' instruction" << std::endl;
                        return ExecutionReturns::UNKNOWN_INSTRUCTION;
                    }
                    
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "SIZE : Result : " << this->registers[destReg] << " | Dest : " << (int)destReg << std::endl;
#endif
                    break;
                }
                case INS_JUMP :
                {
                    uint64_t destAddress = (uint64_t)UTIL::extract_two_bytes(ins, 6) << 16| 
                                        (uint64_t)UTIL::extract_two_bytes(ins, 4);

                    this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = destAddress; 

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "JUMP : " << destAddress << std::endl;
#endif
                    continue;
                }       
                case INS_AT_JUMP :
                {
                    uint64_t destAddress = (uint64_t)UTIL::extract_two_bytes(ins, 6) << 16| 
                                        (uint64_t)UTIL::extract_two_bytes(ins, 4);

                    uint64_t current_pointer = this->contextFunctions[this->currentInstructionBlock].instruction_pointer;

                    this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = destAddress; 

                    this->contextFunctions[this->currentInstructionBlock].jumpStack.push(current_pointer);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "@JUMP : " << destAddress << std::endl;
#endif
                    continue;
                }             
                case INS_YIELD:
                {
                    if(this->callStack.empty())
                    {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                        std::cout << "Callstack empty on yield. Exiting" << std::endl;
#endif
                        this->contextCompleted = true;
                        return ExecutionReturns::OKAY;
                    }

                    uint64_t ret_roi = this->callStack.top(); this->callStack.pop();

                    uint64_t func_to = this->callStack.top(); this->callStack.pop();

                    // Increase the current instruction pointer so when we're called again we start off where we left

                    this->contextFunctions[currentInstructionBlock].instruction_pointer++;

                    this->currentInstructionBlock = func_to;

                    this->contextFunctions[currentInstructionBlock].instruction_pointer = ret_roi;

                    this->switchingFunction = true;

                    break;
                }
                case INS_CS_SF :
                {
                    // Call Stack Store function ( The function to return to when next return hits)
                    uint64_t func_from =  (uint64_t)UTIL::extract_two_bytes(ins, 6) << 16| 
                                          (uint64_t)UTIL::extract_two_bytes(ins, 4);

                    this->callStack.push(func_from);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_CS_SF : From : " << func_from << std::endl;
#endif
                    break;
                }
                case INS_CS_SR :
                {
                    // Call Stack Store Region Of Interest ( Instruction Pointer )
                    uint64_t roi =  (uint64_t)UTIL::extract_two_bytes(ins, 6) << 16| 
                                    (uint64_t)UTIL::extract_two_bytes(ins, 4);
                                        
                    this->callStack.push(roi);
                    
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_CS_SR : ROI : " << roi << std::endl;
#endif
                    break;
                }
                case INS_CALL :
                {
                    // Call
                    uint64_t destAddress =  (uint64_t)UTIL::extract_two_bytes(ins, 6) << 16| 
                                            (uint64_t)UTIL::extract_two_bytes(ins, 4);

                    // Detect recursion
                    if(currentInstructionBlock == destAddress)
                    {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_CALL : Recursion detected : " << destAddress << std::endl;
#endif
                        this->contextFunctions[currentInstructionBlock].recursion_flag = true;
                        this->contextFunctions[currentInstructionBlock].recursion_counter++;

                        // Inc ip for return
                        this->contextFunctions[currentInstructionBlock].instruction_pointer++;

                        // Save instruction pointer
                        this->contextFunctions[currentInstructionBlock].instruction_pointer_history.push(
                            this->contextFunctions[currentInstructionBlock].instruction_pointer
                        );

                        // Save block memory
                        this->contextFunctions[currentInstructionBlock].function_memory_history.push(
                            this->contextFunctions[currentInstructionBlock].function_memory
                        );

                        // Save jump stack
                        this->contextFunctions[currentInstructionBlock].jump_history.push(
                            this->contextFunctions[currentInstructionBlock].jumpStack
                        );

                        // Reset instruction pointer for call into self
                        this->contextFunctions[currentInstructionBlock].instruction_pointer = 0;

                        // Reset function memory
                        this->contextFunctions[currentInstructionBlock].function_memory.reset();

                        // Clear jump stack
                        this->contextFunctions[currentInstructionBlock].jumpStack = {};
                    }

                    currentInstructionBlock = destAddress;

                    this->switchingFunction = true;

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_CALL : Destination : " << destAddress << std::endl;
#endif
                    break;
                }      
                case INS_PCALL :
                {
                    // P Call
                    uint64_t destAddress =  (uint64_t)UTIL::extract_two_bytes(ins, 6) << 16| 
                                            (uint64_t)UTIL::extract_two_bytes(ins, 4);

                    if(currentInstructionBlock == destAddress)
                    {
                        std::cout << "pcall recursion detected; this is not supported" << std::endl;
                        return ExecutionReturns::EXECUTION_ERROR;
                    }

                    owner.queueNewExecutionContext(destAddress);
                    
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_PCALL : Destination : " << destAddress << std::endl;
#endif
                    break;
                }
                case INS_RET  :
                {      
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_RET" << std::endl;
#endif
                    // Attempt a return. If it returns, true will be signaled
                    if(!attempt_return())
                    {
                        this->contextCompleted = true;
                        return ExecutionReturns::ALL_EXECUTION_COMPLETE;
                    }
                    break; // Yes
                }   
                case INS_AT_RET  :
                {      
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_AT_RET" << std::endl;
#endif
                    if(this->contextFunctions[this->currentInstructionBlock].jumpStack.empty())
                    {
                        std::cerr << "Jump stack empty when attempting to execute INS_AT_RET" << std::endl;
                        return ExecutionReturns::EXECUTION_ERROR;
                    }

                    uint64_t dest = this->contextFunctions[this->currentInstructionBlock].jumpStack.top(); 
                    this->contextFunctions[this->currentInstructionBlock].jumpStack.pop();

                    this->contextFunctions[
                        this->currentInstructionBlock
                        ].instruction_pointer = dest; 
                    break; // Yes
                }                
                case INS_EXIT :
                {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "INS_EXIT" << std::endl;
#endif
                    this->contextCompleted = true;
                    return ExecutionReturns::ALL_EXECUTION_COMPLETE;
                }    
                default:
                {
                    if(this->contextFunctions[currentInstructionBlock].instruction_pointer == 
                        this->contextFunctions[currentInstructionBlock].instructions->size())
                    {
                        this->contextCompleted = true;
                        return ExecutionReturns::OKAY;
                    }

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "UNKNOWN INSTRUCTION" << std::endl;
#endif
                    return ExecutionReturns::UNKNOWN_INSTRUCTION;
                    break; 
                }
            }

            // Check for hardware execution. If it fails we need to fail.
            if(!hardware_execution_check())
            {

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "Hardware execution check failed : UNKNOWN INSTRUCTION " << std::endl;
#endif
                return ExecutionReturns::UNKNOWN_INSTRUCTION;
            }

            finalize_cycle();
        }

        return ExecutionReturns::OKAY;
    }

    // ----------------------------------------------------------------
    //
    // ----------------------------------------------------------------

    void ExecutionContext::finalize_cycle()
    {
        // ----------------------------------------------------------------------------

        //  Increase the instruction pointer if we aren't explicitly told not to
        //
        if(!this->switchingFunction)
        {
            this->contextFunctions[currentInstructionBlock].instruction_pointer++;
        }
        else
        {
        // This was only to ensure we didn't inc the ip, and since we didn't we will un-flag this
        // so we can step through the next (funky fresh) function
            this->switchingFunction = false;
        }
    }

    // ----------------------------------------------------------------
    //
    // ----------------------------------------------------------------

    bool ExecutionContext::hardware_execution_check()
    {
        //  Check action registers to see if a device needs to be called
        // ----------------------------------------------------------------------------

        if(this->registers[10] != 0)
        {
            uint8_t device_id = UTIL::extract_byte(this->registers[10], 7);
            if(owner.externalDeviceMap.find(device_id) == owner.externalDeviceMap.end())
            {

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cerr << "Invalid device id in register 10 : " << (int)device_id << std::endl;
#endif
                return false;
            }

            if( nullptr == owner.externalDeviceMap[device_id])
            {

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cerr << "Somehow device " << device_id << " was a nullptr!" << std::endl;
                return false;
#endif
            }

            // Ask device to perform some action 
            owner.externalDeviceMap[device_id]->execute(registers, global_memory);
        }

        // Even if nothing happens, we need to indicate everything is okay
        return true;
    }

    // ----------------------------------------------------------------
    //
    // ----------------------------------------------------------------

    void ExecutionContext::get_arith_lhs_rhs(uint8_t id, uint64_t ins, int64_t * lhs, int64_t * rhs)
    {
        if(id == 0)
        {
            *lhs =   this->registers[UTIL::extract_byte(ins, 5)];
            *rhs =   this->registers[UTIL::extract_byte(ins, 4)];
        }
        else if (id == 1)
        {
            *lhs =  this->registers[UTIL::extract_byte(ins, 5)];
            *rhs = (int16_t)UTIL::extract_two_bytes(ins, 4);
        }
        else if (id == 2)
        {
            *lhs =  (int16_t)UTIL::extract_two_bytes(ins, 5);
            *rhs =  this->registers[UTIL::extract_byte(ins, 3)];
        }
        else if (id == 3)
        {
            *lhs =  (int16_t)UTIL::extract_two_bytes(ins, 5);
            *rhs =  (int16_t)UTIL::extract_two_bytes(ins, 3);
        }
    }

    // ----------------------------------------------------------------
    //
    //  Development Note: 
    //      If we threaded things, or somehow parallelized this, things would be much faster (potentially)
    //      during returns. There could be a lot of information to pop from stacks that will hinder
    //      execution time
    //
    // ----------------------------------------------------------------

    bool ExecutionContext::attempt_return()
    {
        if(this->callStack.empty())
        {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
            std::cout << "Callstack empty. Exiting" << std::endl;
#endif
            return false;
        }

        // No matter what, this should be true rn
        this->switchingFunction = true;

        uint64_t ret_roi = this->callStack.top(); this->callStack.pop();
        uint64_t func_to = this->callStack.top(); this->callStack.pop();

        //  Recursive or not, we need to go to the region of interest
        //

        //  Detect a recursive return
        //  
        if(func_to == currentInstructionBlock)
        {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
            std::cout << "Return from recursive call" << std::endl;
#endif
            // Dec the counter
            this->contextFunctions[currentInstructionBlock].recursion_counter--;

            // Reset the old instruction pointer
            this->contextFunctions[currentInstructionBlock].instruction_pointer = 
                this->contextFunctions[currentInstructionBlock].instruction_pointer_history.top();

            // Remove the instruction pointer on top from history
            this->contextFunctions[currentInstructionBlock].instruction_pointer_history.pop();

            // Reset to the old memory
            this->contextFunctions[currentInstructionBlock].function_memory = 
                this->contextFunctions[currentInstructionBlock].function_memory_history.top();

            // Remove the memory on top from history
            this->contextFunctions[currentInstructionBlock].function_memory_history.pop();

            // Reset to the old jump stack
            this->contextFunctions[currentInstructionBlock].jumpStack = 
                this->contextFunctions[currentInstructionBlock].jump_history.top();

            // Pop the old jump stack
            this->contextFunctions[currentInstructionBlock].jump_history.pop();

            // If we are entering back into origin unset recursion flag
            if(this->contextFunctions[currentInstructionBlock].recursion_counter <= 0)
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
            std::cout << "Re-entering recursive origin" << std::endl;
#endif
                this->contextFunctions[currentInstructionBlock].recursion_counter = 0;
                this->contextFunctions[currentInstructionBlock].recursion_flag = false;
            }

            //  We are done here
            //
            return true;
        }

        //  If we are exiting the origin function, then we can clear everything
        //
        if(this->contextFunctions[currentInstructionBlock].recursion_flag == false)
        {
            // Clear out the function's local call stack
            while( this->contextFunctions[currentInstructionBlock].function_memory.hasData() )
            {
                uint8_t t;
                this->contextFunctions[currentInstructionBlock].function_memory.pop_8(t);
            }

            // Clear out function jump stack
            while( !this->contextFunctions[this->currentInstructionBlock].jumpStack.empty() )
            {
                this->contextFunctions[this->currentInstructionBlock].jumpStack.pop();
            }

            this->contextFunctions[currentInstructionBlock].instruction_pointer = 0;
            this->currentInstructionBlock = func_to;
            this->contextFunctions[currentInstructionBlock].instruction_pointer = ret_roi;
        }
        return true;
    }
}
}