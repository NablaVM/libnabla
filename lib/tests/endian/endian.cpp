
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

#if ((BYTE_ORDER == LITTLE_ENDIAN) || (BYTE_ORDER == PDP_ENDIAN))

TEST(EndianTests, AllEndianTests)
{
    for(int i = 0; i < 100; i++)
    {
        uint16_t r16 = getRandomU16(1024, std::numeric_limits<uint16_t>::max());
        uint32_t r32 = getRandomU32(1024, std::numeric_limits<uint32_t>::max());
        uint64_t r64 = getRandomU64(1024, std::numeric_limits<uint64_t>::max());

        uint16_t be_r16 = endian_conditional_to_be_16(r16);
        uint32_t be_r32 = endian_conditional_to_be_32(r32);
        uint64_t be_r64 = endian_conditional_to_be_64(r64);

        CHECK_FALSE(r16 == be_r16);
        CHECK_FALSE(r32 == be_r32);
        CHECK_FALSE(r64 == be_r64);
    }
}

#else

TEST(EndianTests, AllEndianTests)
{
    for(int i = 0; i < 100; i++)
    {
        std::cout << "STARTING BIG ENDIAN" << std::endl;

        uint16_t r16 = getRandomU16(1024, std::numeric_limits<uint16_t>::max());
        std::cout << "r16" << (int)r16 << std::endl;

        uint32_t r32 = getRandomU32(1024, std::numeric_limits<uint32_t>::max());
        std::cout << "r32" << r32 << std::endl;

        uint64_t r64 = getRandomU64(1024, std::numeric_limits<uint64_t>::max());
        std::cout << "r64" << r64 << std::endl;

        uint16_t be_r16 = endian_conditional_to_be_16(r16); std::cout << "-- 0 --" << std::endl;
        uint32_t be_r32 = endian_conditional_to_be_32(r32); std::cout << "-- 1 --" << std::endl;
        uint64_t be_r64 = endian_conditional_to_be_64(r64); std::cout << "-- 2 --" << std::endl;

        CHECK_TRUE(r16 == be_r16);
        CHECK_TRUE(r32 == be_r32);
        CHECK_TRUE(r64 == be_r64);

        std::cout << "ITR : " << i << " complete " << std::endl;
    }
}

#endif

