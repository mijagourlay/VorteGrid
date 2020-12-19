/** \file perf.h

    \brief Performance counter utilities

    \author Copyright 2008-2012 MJG; All rights reserved.
*/
#ifndef PERF_H
#define PERF_H

#include "Core/Utility/macros.h"
#include <stdio.h>

#if defined( _XBOX )
    #include <xtl.h>
#elif defined( WIN32 )
    #include <windows.h>
#else
    #error "unknown platform"
#endif


/** Whether to print profile data.

    Printing profile data as it is collected each frame can
    cause a lot of output to spew, which can slow the process.
    Unless you want per-frame data, keep this disabled for
    most of the time, then enable it when you want to
    see the data, e.g. after collecting a lot of samples.

*/
extern bool gPrintProfileData ;


// Macros --------------------------------------------------------------
#if defined( _DEBUG ) && ! defined( PROFILE )
    #define PROFILE 1
#endif

#if defined( PROFILE )
    /// Enable performance profiling macros.
    #define QUERY_PERFORMANCE 1
#else
    /// Disable performance profiling macros.
    #define QUERY_PERFORMANCE 0
#endif

/** Macros to query performance counters.

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
            fprintf( stderr , # strName " , last= , %g , avg= , %g , #runs= , %u \n", fDuration , fDurAvg , strName ## _Count ) ;         \
        }                                                                                                                               \
    }
#else
#define QUERY_PERFORMANCE_ENTER
#define QUERY_PERFORMANCE_EXIT( strName )
#endif


#if defined( PROFILE ) || defined( _DEBUG )
    #define PERF_BLOCK( uLabel ) PerfBlock perfBlock_ ## uLabel ## _( # uLabel ) ;
#else
    #define PERF_BLOCK( uLabel )
#endif

// Types -----------------------------------------------------------------------

/** Performance counter block.

    Create an object of this class at the beginning of any code to profile,
    in its own scope.  When the scope ends, the block profiling ends automatically.

*/
class PerfBlock
{
    public:
        /** Initialize a performance profiling block.

            \note   This default constructor should not be used by client code,
                    and will cause assertions and errors in if used inappropriately.

        */
        PerfBlock( void )
            : mDurationInclusive()
            , mLabelSelf( 0 )
            , mNumCalls( 0 )
            , mLabelCallers( 0 )
            , mLabelCallersCapacity( 0 )
        {
            mDurationInclusive.QuadPart = 0 ;
        }


        /** Enter a performance profiling block.
        */
        PerfBlock( const char strLabel[5] )
        {
            // TODO: keep track of duration spent inside profiling routines.
            unsigned int uLabel = INT_FROM_STRING4( strLabel ) ;
            ASSERT( uLabel != 0 ) ;
            ASSERT( uLabel != 'MAIN' ) ;    // 'MAIN' is a special outer-most block that clients should not use
            ASSERT( sCallStackSize > 0 ) ; // 0th entry contains special 'MAIN' entry
            ASSERT( sCallStackSize < MAX_CALL_STACK_CAPACITY ) ;
            // Query performance counter to get current time (enter time) and push it onto stack.
            QueryPerformanceCounter( & sCallStack[ sCallStackSize ].mTimeEntered ) ;
            // Initialize callee duration to zero.  Callee durations will accumulate here to calculate exclusive duration.
            sCallStack[ sCallStackSize ].mDurationCallees.QuadPart = 0 ;
            // Record label for tis block
            sCallStack[ sCallStackSize ].mLabel = uLabel ;
            sCallStackSize ++ ;
            ASSERT( ( sCallStackSize >= MAX_CALL_STACK_CAPACITY ) || ( sCallStack[ sCallStackSize ].mLabel == 0 ) ) ;
        }


        /** Exit a performance profiling block.

            \todo   Store dynamics array of callers or callees.
        */
        ~PerfBlock()
        {
            ASSERT( sListSize < MAX_BLOCK_LIST_CAPACITY ) ;

            // TODO: keep track of duration spent inside profiling routines.

            if( 1 == sCallStackSize )
            {   // Destructor is getting called on static list.
                // This branch should only execute when the application is terminating, never during normal execution.
                // Note that this makes the assertion below useless in practice but still serves to indicate normal conditions.
                // Set a breakpoint here and make sure it only hits when terminating the application.
                // We could avoid this situation by using a container which only calls dtor on populated entries, not on dummy entries.
                // That might be worth doing to avoid the extra "if" in here that should never execute during profile build.
                return ;
            }

            // Pop the stack
            -- sCallStackSize ;
            ASSERT( sCallStackSize > 0 ) ; // 0th entry contains special 'MAIN' entry

            // Query performance counter to get exit time
            LARGE_INTEGER iTimeExit ;
            QueryPerformanceCounter( & iTimeExit ) ;
            // Find matching block record or append one if this is a new block.
            unsigned int iBlock = FindBlock( sCallStack[ sCallStackSize ].mLabel ) ;
            // Store inclusive duration for this performance block.
            sCallList[ iBlock ].mDurationInclusive.QuadPart += iTimeExit.QuadPart - sCallStack[ sCallStackSize ].mTimeEntered.QuadPart ;
            // Store number of calls to this block.
            sCallList[ iBlock ].mNumCalls ++ ;
            // Accumulate callee inclusive duration for use later calculating caller exclusive duration.
            sCallStack[ sCallStackSize - 1 ].mDurationCallees.QuadPart += sCallList[ sListSize ].mDurationInclusive.QuadPart ;

            // Mark one-past-end of stack as empty
            sCallStack[ sCallStackSize ].mLabel = 0 ;
        }

        static void Flush( void ) ;
        static void StartUpdate( void ) ;
        static void EnableProfiling( int numFrames = 1 ) ;
        static void UnitTest( void ) ;

    private:

        void Clear( void )
        {
            mDurationInclusive.QuadPart = 0 ;
            mLabelSelf                  = 0 ;
            mNumCalls                   = 0 ;
            if( mLabelCallers )
            {
                free( mLabelCallers ) ;
            }
            mLabelCallers               = 0 ;
            mLabelCallersCapacity       = 0 ;
        }

        struct PerfEntry
        {
            LARGE_INTEGER   mTimeEntered        ;
            LARGE_INTEGER   mDurationCallees    ;
            unsigned        mLabel              ;
        } ;

        static int FindBlock( unsigned int uLabel ) ;

        static const int        MAX_BLOCK_LIST_CAPACITY = 512   ; ///< Maximum number enter/exit blocks per update
        static const int        MAX_CALL_STACK_CAPACITY = 31    ; ///< Maximum number of nested labels (approximately the same as callstack depth)

        LARGE_INTEGER           mDurationInclusive      ;
        unsigned int            mLabelSelf              ;
        unsigned int            mNumCalls               ;
        unsigned int          * mLabelCallers           ;   // TODO: make a dynamic list
        unsigned int            mLabelCallersCapacity   ;   // TODO: make a dynamic list

        static bool             sStartProfilingNextFrame    ;   ///< Start capturing profile data on next call to StartUpdate
        static bool             sIsProfileInProgress        ;   ///< Whether profiling is in progress
        static int              sNumFramesToProfile         ;   ///< Number of frames to profile
        static float            sFrameDurationMsAvg         ;   ///< Average frame duration in milliseconds

        // Performance info for...
        //  ...Current frame
        static PerfBlock        sCallList[ MAX_BLOCK_LIST_CAPACITY ] ;
        static unsigned int     sListSize ;

        //  ...Current call stack
        static PerfEntry        sCallStack[ MAX_CALL_STACK_CAPACITY ] ;
        static unsigned int     sCallStackSize ;

        // TODO: Average
        // TODO: Worst frame
} ;

// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

#endif
