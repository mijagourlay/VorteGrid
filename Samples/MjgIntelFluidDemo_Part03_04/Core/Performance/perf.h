/*! \file perf.h

    \brief Performance counter macros

    \author Copyright 2009 UCF/Fiea/MJG; All rights reserved.
*/
#ifndef PERF_H
#define PERF_H

#include <stdio.h>

#if defined( WIN32 )
    #include <windows.h>
#else
    #error "unknown platform"
#endif


/*! \brief Whether to print profile data

    Printing profile data as it is collected each frame can
    cause a lot of output to spew, which can slow the process.
    Unless you want per-frame data, keep this disabled for
    most of the time, then enable it when you want to
    see the data, e.g. after collecting a lot of samples.

*/
extern bool gPrintProfileData ;


// Macros --------------------------------------------------------------

#if defined( PROFILE ) || defined( _DEBUG )
    #define QUERY_PERFORMANCE 1
#else
    #define QUERY_PERFORMANCE 0
#endif

/*! \brief Macros to query performance counters

    To use these macros, place QUERY_PERFORMANCE_ENTER before the start,
    and QUERY_PERFORMANCE_EXIT after the end, of a block of code you want
    to profile.  For example:

    \verbatim
        QUERY_PERFORMANCE_ENTER ;
        MyRoutine1() ;
        MyRoutine2() ;
        QUERY_PERFORMANCE_EXIT( LabelForMyRoutines ) ;
    \endverbatim

    The argument passed into QUERY_PERFORMANCE_EXIT acts as a label.
    It should be any valid C-style identifier, although its name
    has no particular meaning, so it does not have to be the name
    of a routine or variable.

    You can also nest these blocks, for example:

    \verbatim
        QUERY_PERFORMANCE_ENTER ;
        MyRoutine1() ;
            QUERY_PERFORMANCE_ENTER ;
            MyRoutine2() ;
            QUERY_PERFORMANCE_EXIT( LabelForMyRoutine2 ) ;
        QUERY_PERFORMANCE_EXIT( LabelForBothOfMyRoutines ) ;
    \endverbatim

    You must place the ENTER and EXIT macros in the same lexical scope.
    Failing to do so will result in cryptic compiler errors.

*/
#if QUERY_PERFORMANCE
#define QUERY_PERFORMANCE_ENTER                                                             \
    {                                                                                       \
        LARGE_INTEGER QP_qwTicksPerSec ;                                                    \
        QueryPerformanceFrequency( & QP_qwTicksPerSec ) ;                                   \
        const float QP_milliSecondsPerTick = 1000.0f / float( QP_qwTicksPerSec.QuadPart ) ; \
        LARGE_INTEGER QP_iTimeEnter  ;                                                      \
        QueryPerformanceCounter( & QP_iTimeEnter ) ;

#define QUERY_PERFORMANCE_EXIT( strName )                                                                                               \
        {                                                                                                                               \
            LARGE_INTEGER iTimeExit ;                                                                                                   \
            QueryPerformanceCounter( & iTimeExit ) ;                                                                                    \
            LARGE_INTEGER iDuration ;                                                                                                   \
            iDuration.QuadPart = iTimeExit.QuadPart - QP_iTimeEnter.QuadPart ;                                                          \
            const float fDuration = QP_milliSecondsPerTick * float( iDuration.QuadPart ) ;                                              \
            static LARGE_INTEGER strName ## _DurationTotal ;                                                                            \
            strName ## _DurationTotal.QuadPart += iDuration.QuadPart ;                                                                  \
            static unsigned strName ## _Count = 0 ;                                                                                     \
            ++ strName ## _Count ;                                                                                                      \
            const float fDurAvg = QP_milliSecondsPerTick * float( strName ## _DurationTotal.QuadPart ) / float( strName ## _Count ) ;   \
            if( gPrintProfileData )                                                                                                     \
                fprintf( stderr , # strName " = %g (%g) ms\n", fDuration , fDurAvg ) ;                                                  \
        }                                                                                                                               \
    }
#else
#define QUERY_PERFORMANCE_ENTER
#define QUERY_PERFORMANCE_EXIT( strName )
#endif


// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
