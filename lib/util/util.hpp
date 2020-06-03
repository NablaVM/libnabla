#ifndef NABLA_LIBC_UTIL
#define NABLA_LIBC_UTIL

#include <stdint.h>

//! \brief A deconstructed double
struct DoubleDeconstructed
{
    bool sign;
    uint64_t exponent;
    uint64_t mantissa;
};

//! \brief Extract a byte from a uint64_t
//! \param data The data do get the byte from
//! \param idx  The index of the byte to get
//! \returns The byte extracted
//! \note This method assumes data is in little endian
uint8_t util_extract_byte(uint64_t data, uint8_t idx);

//! \brief Extract two bytes from a uint64_t
//! \param data The data do get the byte from
//! \param idx  The index of the two byte to get
//! \returns The bytes extracted
//! \note This method assumes data is in little endian, but ensures the result
//!       is in host endianess. This is done with VSysExecutionContext in mind
uint16_t util_extract_two_bytes(uint64_t data, uint8_t idx);

//! \brief Convert a uint64_t to a double
//! \param val The value to convert to a double
//! \returns A double that was constructed from val
double util_convert_uint64_to_double(uint64_t val);

//! \brief Convert a double to a uint64_t 
//! \param val The value to convert to a uint64_t
//! \returns A uint64_t that was constructed from val
uint64_t util_convert_double_to_uint64(double val);

//! \brief Check if two doubles are mostly equal
//! \param lhs Left hand operand
//! \param rhs Right hand operand
//! \retval 1 The doubles are equal
//! \retval 0 The doubles are not equal
//! \note Uses a precision of 0.00001
uint8_t util_check_double_equal(double lhs, double rhs);

// [ The blow functions are intended for future updates - Not used as of 3/ June/ 2020 ]

//! \brief Deconstruct a double into its base components
//! \param val The double to deconstruct
//! \returns Struct containing double components
DoubleDeconstructed util_deconstruct_double(double val);

//! \brief Construct a double from deconstructed components
//! \param dd The deconstructed double
//! \param okay [out] Tells us if constriuction works
//! \returns Constructed double
double util_construct_double(struct DoubleDeconstructed dd, bool &okay);

#endif