/*! \file wrapperMacros.h

    \brief Convenience macros

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef WRAPPER_MACROS_H
#define WRAPPER_MACROS_H

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

#define MIN2(x,y)               (((x)<(y))?(x):(y))
#define MIN3(x,y,z)             MIN2( MIN2( (x) , (y) ) , (z) )
#define MAX2(x,y)               (((x)>(y))?(x):(y))
#define MAX3(x,y,z)             MAX2( MAX2( (x) , (y) ) , (z) )
#define POW3(x)                 ((x)*(x)*(x))
#define CLAMP( x , min , max )  MIN2( MAX2( x , min ) , max )
static const float PI       = 3.1415926535897932384626433832795f ;
static const float TWO_PI   = 2.0f * PI ;
static const float RAD2DEG  = (180.0f / PI) ;

#if ! defined( DEBUG_ONLY )
    #if defined( _DEBUG )
        #define DEBUG_ONLY( expr ) expr
    #else
        #define DEBUG_ONLY( expr )
    #endif
#endif


#if defined( _DEBUG )
    #define ASSERT( expr ) assert( expr )
#else
    #define ASSERT( expr )
#endif


// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

inline float sechf( const float & x ) { return 1.0f / coshf( x ) ; }

#endif
