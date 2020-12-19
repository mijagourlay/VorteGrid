/** \file slist.h

    \brief Singly-linked list templated class
*/
#ifndef SLIST_H
#define SLIST_H

#ifdef _MSC_VER
    #pragma warning(push)
    // Microsoft Visual C++ vector library generates "unreachable code" sometimes.
    #pragma warning(disable: 4702) // unreachable code
    #pragma warning(disable: 4530) // C++ exception handler used, but unwind semantics are not enabled. Specify /EHsc
#endif

#include <list>
 
#ifdef _MSC_VER
    #pragma warning(pop) 
#endif

#include "Core/wrapperMacros.h"

#define SLIST  std::list

#endif