#include "VSysExternalDataStore.hpp"

#include "util.hpp"

#include <iostream>

namespace NABLA
{
namespace VSYS
{
namespace EXTERNAL
{
    constexpr int NABLA_DS_DEVICE_ALLOCATE  =  0;
    constexpr int NABLA_DS_DEVICE_FREE      =  1;
    constexpr int NABLA_DS_DEVICE_COPY      =  5;
    constexpr int NABLA_DS_DEVICE_STORE     = 10;
    constexpr int NABLA_DS_DEVICE_LOAD      = 20;
    constexpr int NABLA_DS_DEVICE_SIZE      = 30;
    constexpr int NABLA_DS_DEVICE_RESET     = 50;


    constexpr int NABLA_DS_DEVICE_LOAD_TYPE_SPECIFIC  = 0;
    constexpr int NABLA_DS_DEVICE_LOAD_TYPE_DUMP      = 1;

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------

    DataStore::DataStore() : address_counter(0)
    {

    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------
    
    DataStore::~DataStore()
    {
        for(auto & el : data_store)
        {
            if(el.second != nullptr)
            {
                el.second->clear();
                delete el.second;
            }
        }
        data_store.clear();
    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------

    uint64_t DataStore::get_address()
    {
        if(freed_address_pool.empty())
        {
            return address_counter++;
        }

        uint64_t addr = freed_address_pool.top();
        freed_address_pool.pop();

        return addr;
    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------

    void DataStore::free_address(uint64_t address)
    {
        // Locate the item for deletion
        auto ds_item = data_store.find(address);

        if(ds_item == data_store.end())
        {
            // Item doesn't exist
            return;
        }

        // Free the memory
        if(ds_item->second != nullptr)
        {
            ds_item->second->clear();
            delete ds_item->second;
        }

        // Erase the item from the map
        data_store.erase(ds_item);

        // Store the address for later use
        freed_address_pool.push(address);
    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------

    void DataStore::clear_memory()
    {
        // Delete any existing items
        for(auto & item : data_store)
        {
            if(item.second != nullptr)
            {
                item.second->clear();
                delete item.second;
            }
        }

        // Clear map
        data_store.clear();

        // Clear pool stack
        while(!freed_address_pool.empty())
        {
            freed_address_pool.pop();
        }

        // Reset address counter
        address_counter = 0;
    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------

    uint64_t DataStore::alloc_bytes(uint32_t bytes, bool & okay)
    {
        uint64_t address = get_address();

        try
        {
            // Allocate 'bytes' number of bytes, with a '0' value
            data_store[address] = new std::vector<uint8_t>(bytes, 0); 
        }
        catch(std::bad_alloc&) 
        {
            // Indicate error
            okay = false;

            // Check if for some reason the item was made
            if(data_store.find(address) != data_store.end())
            {
                // If it was erase it, no memory should have been allocated
                data_store.erase(address);
            }

            // Give the address back to the pool
            freed_address_pool.push(address);

            return 0;
        }

        // Indicate success
        okay = true;

        // Give back address
        return address;
    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------
    
    bool DataStore::copy_data(uint64_t addr_from, uint64_t addr_to)
    {
        auto ds_from = data_store.find(addr_from);
        auto ds_to   = data_store.find(addr_to);

        if(ds_from == data_store.end() || 
           ds_to   == data_store.end() )
        {
            return false;
        }

        // Clear destination
        ds_to->second->clear();

        // Copy over data
        ds_to->second->insert(ds_to->second->end(), ds_from->second->begin(), ds_from->second->end());

        return true;
    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------
    
    void DataStore::execute(int64_t (&registers)[16], Memory<NABLA_VSYS_SETTINGS_GLOBAL_MEMORY_BYTES> &global_memory)
    {
        uint8_t subid = UTIL::extract_byte(registers[10], 6);

        switch(subid)
        {
            //  Allocate
            //
            case NABLA_DS_DEVICE_ALLOCATE:
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "NABLA_DS_DEVICE_ALLOCATE" << std::endl;
#endif
                uint32_t bytes = (uint32_t)UTIL::extract_two_bytes(registers[10], 5) << 16 | 
                                 (uint32_t)UTIL::extract_two_bytes(registers[10], 3);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "\tINFO: Asked to alloc : " << bytes << " bytes" << std::endl;
#endif
                bool okay = false;

                uint64_t address = alloc_bytes(bytes, okay);

                if(!okay)
                {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "\tFAIL: Failed to allocate bytes" << std::endl;
#endif
                    // Indicate error
                    registers[11] = 1;
                }
                else
                {
                    // Indicate success
                    registers[11] = 0;

                    // Give back address to new alloc
                    registers[12] = address;
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "\tSUCCESS: New address = " << registers[12] << std::endl;
#endif
                }
                break;
            }
            //  Free
            //
            case NABLA_DS_DEVICE_FREE:
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "NABLA_DS_DEVICE_FREE" << std::endl;
#endif
                free_address(registers[11]);
                registers[11] = 0;
                break;
            }
            //  Copy
            //
            case NABLA_DS_DEVICE_COPY:
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "NABLA_DS_DEVICE_COPY" << std::endl;
#endif
                // Copy data, store result as success (0) or failure (1)
                registers[11] = (copy_data(registers[11], registers[12])) ? 0 : 1; 
                break;
            }
            //  Store
            //
            case NABLA_DS_DEVICE_STORE:
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "NABLA_DS_DEVICE_STORE" << std::endl;
#endif
                uint64_t address_to = registers[11];
                uint32_t gs_start_addr = (uint32_t)UTIL::extract_two_bytes(registers[12], 7) << 16 |
                                         (uint32_t)UTIL::extract_two_bytes(registers[12], 5);
                uint32_t gs_end_addr   = (uint32_t)UTIL::extract_two_bytes(registers[12], 3) << 16 |
                                         (uint32_t)UTIL::extract_two_bytes(registers[12], 1);

#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "\tADDRESS : " << address_to << std::endl;
                std::cout << "\tGS START: " << gs_start_addr << std::endl;
                std::cout << "\tGS END  : " << gs_end_addr << std::endl;
#endif
                auto ds_to = data_store.find(address_to);

                // If no exist, fail
                if(ds_to == data_store.end())
                {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "\tFAIL: Address not found" << std::endl;
#endif
                    registers[10] = 0;
                    registers[11] = 1;
                    registers[12] = 0;
                    return;
                }

                // If idx wonky, or size not correct, fail
                if(gs_end_addr < gs_start_addr || 
                   std::abs((int64_t)gs_end_addr - (int64_t)gs_start_addr) > ds_to->second->size())
                {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "\tFAIL: Idx wonky, or size is incorrect" << std::endl;
#endif
                    registers[10] = 0;
                    registers[11] = 1;
                    registers[12] = 0;
                    return;
                }

                // Store data
                uint64_t var_idx = 0; // Index in storage unit
                for(uint32_t idx = gs_start_addr; idx < gs_end_addr; idx++)
                {
                    uint8_t data; 

                    if(!global_memory.get_8(idx, data))
                    {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                        std::cout << "\tFAIL: Failed to get data from GS. idx: " << idx << " | var idx : " << var_idx << std::endl;
#endif
                        registers[10] = 0;
                        registers[11] = 1;
                        registers[12] = 0;
                        return;
                    }

                    ds_to->second->at(var_idx++) = data;
                }

                // Success
                registers[11] = 0;
                registers[12] = 0;
                break;
            }
            //  Load
            //
            case NABLA_DS_DEVICE_LOAD:
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "NABLA_DS_DEVICE_LOAD" << std::endl;
#endif
                uint8_t load_type = UTIL::extract_byte(registers[10], 5);

                uint64_t address_to = registers[11];

                auto ds_from = data_store.find(address_to);

                // If no exist, fail
                if(ds_from == data_store.end())
                {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "\tFAIL: Address not found" << std::endl;
#endif
                    registers[10] = 0;
                    registers[11] = 1;
                    registers[12] = 0;
                    return;
                }

                switch(load_type)
                {
                    //  Load specific 
                    //
                    case NABLA_DS_DEVICE_LOAD_TYPE_SPECIFIC:
                    {
                        uint32_t gs_start_addr = (uint32_t)UTIL::extract_two_bytes(registers[12], 7) << 16 |
                                                 (uint32_t)UTIL::extract_two_bytes(registers[12], 5);
                        uint32_t gs_end_addr   = (uint32_t)UTIL::extract_two_bytes(registers[12], 3) << 16 |
                                                 (uint32_t)UTIL::extract_two_bytes(registers[12], 1);

                        // If idx wonky, or size not correct, fail
                        if(gs_end_addr < gs_start_addr || 
                        std::abs((int64_t)gs_end_addr - (int64_t)gs_start_addr) < ds_from->second->size())
                        {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                            std::cout << "\tFAIL: Idx wonky, or size is incorrect" << std::endl;
                            std::cout << "\tGS START: " << gs_start_addr << std::endl;
                            std::cout << "\tGS END  : " << gs_end_addr << std::endl;
#endif
                            registers[10] = 0;
                            registers[11] = 1;
                            registers[12] = 0;
                            return;
                        }

                        uint32_t gs_idx = gs_start_addr;
                        for(auto & element : *ds_from->second)
                        {
                            if(!global_memory.set_8(gs_idx++, element))
                            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                                std::cout << "\tFAIL: Failed to set byte in gs at :" << gs_idx << std::endl;
#endif
                                registers[10] = 0;
                                registers[11] = 1;
                                registers[12] = 0;
                                return;
                            }
                        }
                        break;
                    }
                    //  Dump data into GS
                    //
                    case NABLA_DS_DEVICE_LOAD_TYPE_DUMP:
                    {
                        // Go through elements and put them on the GS
                        for(auto & element : *ds_from->second)
                        {
                            if(!global_memory.push_8(element))
                            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                                std::cout << "\tFAIL: Failed to push byte to gs" << std::endl;
#endif
                                registers[10] = 0;
                                registers[11] = 1;
                                registers[12] = 0;
                                return;
                            }
                        }
                        break;
                    }
                    default:
                    {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                        std::cout << "DS_DEVICE_UNKNOWN_LOAD_SUB_INSTRUCTION" << std::endl;
#endif
                        registers[11] = 1;
                        registers[10] = 0;
                        return;
                    }
                }


                // Success
                registers[11] = 0;
                registers[12] = 0;
                break;
            }
            // Reset 
            //
            case NABLA_DS_DEVICE_RESET:
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "NABLA_DS_DEVICE_RESET" << std::endl;
#endif
                clear_memory();
                break;
            }
            // Size
            //
            case NABLA_DS_DEVICE_SIZE:
            {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "NABLA_DS_DEVICE_SIZE" << std::endl;
#endif

                uint64_t address_to = registers[11];
                auto ds_from = data_store.find(address_to);

                if(ds_from == data_store.end())
                {
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                    std::cout << "\tFAIL : Unable to locate ID : " << address_to << std::endl;
#endif
                    registers[10] = 0;
                    registers[11] = 1;
                    return;
                }  

                registers[11] = 0;
                registers[12] = ds_from->second->size();
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "\tSUCCESS : Size of " << address_to << " is : " << registers[12] << std::endl;
#endif
                break;
            }
            default:
#ifdef NABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT
                std::cout << "DS_DEVICE_UNKNOWN_INSTRUCTION" << std::endl;
#endif
                registers[11] = 1;
                break;
        }
        registers[10] = 0;
    }
}
}
}