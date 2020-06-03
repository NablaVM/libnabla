
#include "endian.hpp"
#include <iostream>
#include <random>
#include <limits>
#include <bitset>
#include "CppUTest/TestHarness.h"

namespace 
{
    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------
    
    uint16_t getRandomU16(uint16_t low, uint16_t high)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(low, high);
        return dis(gen);
    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------
    
    uint32_t getRandomU32(uint32_t low, uint32_t high)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(low, high);
        return dis(gen);
    }

    // ---------------------------------------------------------------
    // 
    // ---------------------------------------------------------------
    
    uint64_t getRandomU64(uint64_t low, uint64_t high)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(low, high);
        return dis(gen);
    }
}

TEST_GROUP(EndianTests)
{   
};

// ---------------------------------------------------------------
// 
// ---------------------------------------------------------------


TEST(EndianTests, NonBigendianTests)
{
    for(int i = 0; i < 100; i++)
    {
        uint16_t r16 = getRandomU16(1024, 60000 );
        uint32_t r32 = getRandomU32(1024, 100000);
        uint64_t r64 = getRandomU64(1024, 100000);

        uint16_t le_r16 = ENDIAN::conditional_to_le_16(r16);
        uint32_t le_r32 = ENDIAN::conditional_to_le_32(r32);
        uint64_t le_r64 = ENDIAN::conditional_to_le_64(r64);

#if BYTE_ORDER == LITTLE_ENDIAN
        CHECK_TRUE(r16 == le_r16);
        CHECK_TRUE(r32 == le_r32);
        CHECK_TRUE(r64 == le_r64);
#else
        CHECK_FALSE(r16 == le_r16);
        CHECK_FALSE(r32 == le_r32);
        CHECK_FALSE(r64 == le_r64);
#endif
    }
}

