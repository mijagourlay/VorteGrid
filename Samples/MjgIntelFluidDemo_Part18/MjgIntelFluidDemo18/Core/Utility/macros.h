/** \file macros.h

    \brief useful macros

    \author Written and Copyright (C) 1999-2013 by Michael J. Gourlay
*/
/** \page core_page Core

    \section core_intro Foundation classes

    These classes provide basic functionality which any application can build upon.

    - Memory
        - Allocator - memory management with leak, corruption and fragmentation detection
    - Containers
        - SList - (sequence container) singly linked list
        - Stack - fixed-size stack
        - HashEntry - base class for HashMap key data
        - HashMap - (associative container) dictionary with constant-time access to entries
        - RString - reference-counted string
    - Math
        - Vec3 - 3-vector
        - Vec4 - 4-vector
        - Mat44 - 4x4 matrix
    - RTTI - lightweight run-time type information

    \section Utility routines

    - scan - text scanning routines
    - util - miscellany

    \author Written and Copyright 2005-2012 Michael J. Gourlay; All rights reserved.

*/
#ifndef MACROS_H
#define MACROS_H

#include <cassert>

#ifndef NULLPTR
#   define NULLPTR 0
#endif


#if ( ! defined( UNIT_TEST ) ) && defined( _DEBUG )
#define UNIT_TEST
#endif

/// Determine whether to perform stringent run-time checking like type, bounds and normalized vectors.
#if ! defined( PEDANTIC )
    #if defined( _DEBUG )
        #define PEDANTIC 0
    #else
        #define PEDANTIC 0
    #endif
#endif

#ifdef __cplusplus
// Microsoft Visual C++ 7 (a.k.a. .NET) defines "min" as a macro, which conflicts with std::min.
/// Return the smaller of 2 given values.
template <class T> T Min2( const T & x , const T & y ) { return x < y ? x : y ; }

/// Return the larger of 2 given values.
template <class T> T Max2( const T & x , const T & y ) { return x > y ? x : y ; }

/// Minimum of 3 values.
#define MIN3(x,y,z)         Min2( Min2( (x) , (y) ) , (z) )

/// Maximum of 3 values.
#define MAX3(x,y,z)         Max2( Max2( (x) , (y) ) , (z) )

/// Clamp value to lie within provided min, max bounds.
template < class T >
T Clamp( const T & x , const T & min , const T & max )
{
    return Min2( Max2( x , min ) , max ) ;
}


/// Swap two values.
template<typename T> inline void Swap( T & a, T & b )
{
    const T tmp = a ;
    a = b ;
    b = tmp ;
}
#endif




/// Whether machine storage is little-endian (least significant bytes stored first).
#define LITTLE_ENDIAN_ARCH (*((short*)"A") == 'A')


/// Convert enum to string.
#define STRING_FROM_TOKEN(tok,var)  { if((var) == (tok)) { return #tok ; } }


/// Convert enum to string.
#define CASE_STRING_FROM_TOKEN(tok)  case tok: return #tok ; break ;


/// Convert string to enum.
#define TOKEN_FROM_STRING(tok,var)  { if(!strcmp(#tok,(var))) { return tok ; } }


/// Convert 4-byte string to integer.
#define INT_FROM_STRING4( str4 ) ( (str4)[0] + ( (str4)[1] <<  8 ) + ( (str4)[2] << 16 ) + ( (str4)[3] << 24 ) )

#if ! defined( DEBUG_ONLY )
    #if defined( _DEBUG )
        /// Compile the expression since this is a debug build.
        #define DEBUG_ONLY( expr ) expr
        #define NON_DEBUG_ONLY( expr )
    #else
        /// Ignore the expression since this is NOT a debug build.
        #define DEBUG_ONLY( expr )
        #define NON_DEBUG_ONLY( expr ) expr
    #endif
#endif

#define UNUSED_PARAM( param ) ( (void) param ) ;

#define SET_BREAKPOINT_HERE { int i ; i = 0 ; ++ i ; }

#if defined( WIN32 )
    #define DEBUG_BREAK() __debugbreak()
#else
    #define DEBUG_BREAK() *(int*)(0) = 0 ;
#endif

#if defined( _DEBUG )
    #if defined( __GNUC__ )
        // MinGW ASSERT exits from the program a little too gracefully -- no call stack from the debugger.
        // Crashing actually provides more info.
        #define ASSERT( condition ) { if( ! ( condition ) ) { int * pNull = 0 ; * pNull = 0 ; } }
    #else
        #define ASSERT( condition ) assert( condition )
    #endif
#else
    #define ASSERT( condition )
#endif

#define FAIL() ASSERT( false )

#if PEDANTIC
    #define PARANOID( condition ) ASSERT( condition )
#else
    #define PARANOID( condition )
#endif

#if defined( __GNUC__ ) // GNU C++ compiler requires a different syntax for defining static member variables of template classes.
    #define TEMPLATE_DECL template <class T> 
#else
    #define TEMPLATE_DECL
#endif

#if ! defined( _XBOX )
    typedef unsigned char   BYTE ;
    typedef short           SHORT ;
    typedef unsigned short  WORD ;
    typedef unsigned long   DWORD ;
    typedef int             BOOL ;
    typedef float           FLOAT ;
#endif

#endif
