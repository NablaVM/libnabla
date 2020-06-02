
#ifndef NABLA_IEEE_HEADER_GUARD
#define NABLA_IEEE_HEADER_GUARD


#ifdef __APPLE__ 

// Because APPLE has to be difficult
union ieee754_double
  {
    double d;
    /* This is the IEEE 754 double-precision format.  */
    struct
      {
        /* Together these comprise the mantissa.  */
        unsigned int mantissa1:32;
        unsigned int mantissa0:20;
        unsigned int exponent:11;
        unsigned int negative:1;
      } ieee;
  };

#else 

#include <ieee754.h>

#endif // Apple

#endif