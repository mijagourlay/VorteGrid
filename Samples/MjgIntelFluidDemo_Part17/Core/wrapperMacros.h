//#pragma optimize( "" , off )
/** \file wrapperMacros.h

    \brief Convenience macros

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef WRAPPER_MACROS_H
#define WRAPPER_MACROS_H

/// Whether to enable fluid-body simulation.
/// It can be useful to disable the simulation when diagnosing problems and profiling performance.
#define ENABLE_FLUID_BODY_SIMULATION 1

#if defined( _DEBUG )
#include <assert.h>
#endif

// Use "fast" floating-point model (as opposed to strict or precise)
#pragma warning( disable: 4068 )
#pragma float_control(except, off)
#pragma fenv_access(off)
#pragma float_control(precise, off)
// The following line is needed on Itanium processors
#pragma fp_contract(on)

// Microsoft Visual C++ vector library generates "unreachable code" sometimes.
#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4702) // unreachable code
#endif

#include <vector>
#include <list>
 
#ifdef _MSC_VER
    #pragma warning(pop) 
#endif

using namespace std ;

// Macros to toggle whether to use home-grown or standard containers.
#define SLIST               std::list

#define Iterator            iterator
#define ConstIterator       const_iterator
#define Reverse_Iterator    reverse_iterator
#define Begin               begin
#define End                 end
#define RBegin              rbegin
#define REnd                rend
#define Size                size
#define Empty               empty
#define Front               front
#define Back                back
#define PushBack            push_back
#define PopBack             pop_back
#define Reserve             reserve
#define Resize              resize
#define Capacity            capacity
#define Clear               clear
#define Erase               erase

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
