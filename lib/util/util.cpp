#include "util.hpp"
#include <cmath>
#include <assert.h>

#include "endian.hpp"
#include <iostream>
namespace
{

    constexpr uint64_t IEEE_SIGN_MASK       = 0x8000000000000000;
    constexpr uint16_t IEEE_EXPONENT_MASK   = 0x7FF;
    constexpr unsigned IEEE_EXPONENT_SHIFT  = 52;
    constexpr uint64_t IEEE_MANTISSA_MASK   = 0x000FFFFFFFFFFFFF;

    // Union for easy conversions between doubles and uint64_ts
    union DoubleExtract
    {
        uint64_t bin;
        double d;
    };
}

// --------------------------------------------------------------
//
// --------------------------------------------------------------

uint8_t util_extract_byte(uint64_t data, uint8_t idx)
{
    return (data >> (8*idx)) & 0xff;
}

// --------------------------------------------------------------
//
// --------------------------------------------------------------

uint16_t util_extract_two_bytes(uint64_t data, uint8_t idx)
{

    assert(idx > 0);
    return (data >> (8*(idx-1))) & 0xffff;
}

// --------------------------------------------------------------
//
// --------------------------------------------------------------

double util_convert_uint64_to_double(uint64_t val)
{
    // Extract from our value
    DoubleExtract d; d.bin = val;

    // Return double
    return d.d;
}

// --------------------------------------------------------------
//
// --------------------------------------------------------------

uint64_t util_convert_double_to_uint64(double val)
{
    // Extract from our value
    DoubleExtract d; d.d = val;

    // Return uint64_t
    return d.bin;
}

// --------------------------------------------------------------
//
// --------------------------------------------------------------

uint8_t util_check_double_equal(double lhs, double rhs)
{
    double precision = 0.00001;
    if (((lhs - precision) < rhs) && 
        ((lhs + precision) > rhs))
    {
        return 1;
    }
    return 0;
}

// --------------------------------------------------------------
//
// --------------------------------------------------------------

DoubleDeconstructed util_deconstruct_double(double val)
{
    DoubleDeconstructed deconstructed;

    DoubleExtract de; de.d = val;

    return DoubleDeconstructed { std::signbit(val), 
                                 ((de.bin >> IEEE_EXPONENT_SHIFT) & IEEE_EXPONENT_MASK),
                                             de.bin & IEEE_MANTISSA_MASK };
}

// --------------------------------------------------------------
//
// --------------------------------------------------------------

double util_construct_double(struct DoubleDeconstructed dd, bool &okay)
{
    okay = false;

    if(dd.exponent & ~IEEE_EXPONENT_MASK)
    {
        return 0.0;
    }

    if(dd.mantissa & ~IEEE_MANTISSA_MASK)
    {
        return 0.0;
    }

    DoubleExtract de;

    de.bin = (dd.sign ? IEEE_SIGN_MASK : 0x00)
           | (uint64_t)(dd.exponent & IEEE_EXPONENT_MASK) << IEEE_EXPONENT_SHIFT
           | (uint64_t)(dd.mantissa & IEEE_MANTISSA_MASK);

    okay = true;
    double d = de.d;
    return d;
}