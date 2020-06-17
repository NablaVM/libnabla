#ifndef NABLA_HARDWARE_DEVICE_DATA_STORE_HPP
#define NABLA_HARDWARE_DEVICE_DATA_STORE_HPP

#include "VSysExternalIf.hpp"

#include <stdint.h>
#include <map>
#include <vector>
#include <stack>

namespace NABLA
{
namespace VSYS
{
namespace EXTERNAL
{
    //! \brief An external 'device' that adds Data Storage functionality
    class DataStore : public ExternalIf 
    {
    public:

        //! \brief Create the external device
        DataStore();

        //! \brief Destroy the external device
        ~DataStore();

        // From ExternalIf
        virtual void execute(int64_t (&registers)[16], Memory<NABLA_VSYS_SETTINGS_GLOBAL_MEMORY_BYTES> &global_memory) override;

    private:

        uint64_t address_counter;

        uint64_t get_address();

        void free_address(uint64_t address);

        void clear_memory();

        uint64_t alloc_bytes(uint32_t bytes, bool & okay);

        bool copy_data(uint64_t addr_from, uint64_t addr_to);

        std::stack<uint64_t> freed_address_pool;
        std::map<uint64_t, std::vector<uint8_t>*> data_store;
    };
}
}
}

#endif