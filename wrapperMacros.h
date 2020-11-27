/*! \file wrapperMacros.h

    \brief Convenience macros

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef WRAPPER_MACROS_H
#define WRAPPER_MACROS_H

// Use "fast" floating-point model (as opposed to strict or precise)
#pragma warning( disable: 4068 )
#pragma float_control(except, off)
#pragma fenv_access(off)
#pragma float_control(precise, off)
// The following line is needed on Itanium processors
#pragma fp_contract(on)

#include <list>
#include <vector>

using namespace std ;

#define SList               list
#define Vector              vector
#define Iterator            iterator
#define Reverse_Iterator    reverse_iterator
#define Begin               begin
#define End                 end
#define RBegin              rbegin
#define REnd                rend
#define Size                size
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

#define MIN2(x,y)               (((x)<(y))?(x):(y))
#define MAX2(x,y)               (((x)>(y))?(x):(y))
#define POW3(x)                 ((x)*(x)*(x))
#define CLAMP( x , min , max )  MIN2( MAX2( x , min ) , max )
static const float PI     = 3.1415926535897932384626433832795f ;
static const float TWO_PI = 2.0f * PI ;

// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

inline float sechf( const float & x ) { return 1.0f / coshf( x ) ; }

#endif
