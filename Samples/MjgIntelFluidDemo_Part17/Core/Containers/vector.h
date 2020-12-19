/** \file vector.h

    \brief Dynamic array templated class
*/
#ifndef VECTOR_H
#define VECTOR_H

// Microsoft Visual C++ vector library generates "unreachable code" sometimes.
#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4702) // unreachable code
#endif

#include <vector>
 
#ifdef _MSC_VER
    #pragma warning(pop) 
#endif

#include "Core/wrapperMacros.h"

#define VECTOR  std::vector

#endif