/** \file vector.h

    \brief Dynamic array templated class
*/
#ifndef VECTOR_H
#define VECTOR_H

#ifdef _MSC_VER
    #pragma warning(push)
    // Microsoft Visual C++ vector library generates "unreachable code" sometimes.
    #pragma warning(disable: 4702) // unreachable code
    #pragma warning(disable: 4530) // C++ exception handler used, but unwind semantics are not enabled. Specify /EHsc
#endif

#include <vector>
 
#ifdef _MSC_VER
    #pragma warning(pop) 
#endif

#include "Core/wrapperMacros.h"

#define VECTOR  std::vector

#endif