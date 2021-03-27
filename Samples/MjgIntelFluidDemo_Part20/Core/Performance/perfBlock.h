/** \file perfBlock.h

    \brief Performance profiling for a lexical scope block.

    \author Written and copyright 2005-2016 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PERF_BLOCK_H
#define PERF_BLOCK_H

#pragma warning(disable: 4996) // fopen may be unsafe.

#if defined( _XBOX )
    #include <xtl.h>
#elif defined( WIN32 )
    #include <windows.h>    // For LARGE_INTEGER
    #ifdef min
        #undef min
    #endif
    #ifdef max
        #undef max
    #endif
#else
    #error "unknown platform"
#endif

#include <stdio.h>

// Macros --------------------------------------------------------------

#if defined( _DEBUG )   // Enable profiling for debug builds -- mainly to test profiling library, not because profiles of debug builds are useful.
#   if ! defined( PROFILE )
#       define PROFILE 1
#   endif
#endif

#if PROFILE
    #define PERF_BLOCK_MAIN( threadName )                       PerfBlock perfBlock_MAIN( __FILE__ , __LINE__ , threadName ) ;
    #define PERF_BLOCK( label )                                 PerfBlock perfBlock_ ## label ## _( # label , __FILE__ , __LINE__ ) ;
    #define PERF_BLOCK_ENABLE_PROFILING( numFrames )            PerfBlock::EnableProfilingThisThread( numFrames ) ;
    #define PERF_BLOCK_CROSS_FRAME_BOUNDARY( tallyEveryFrame )  PerfBlock::CrossFrameBoundaryThisThread( tallyEveryFrame ) ;
#else
    #define PERF_BLOCK_MAIN( threadName )
    #define PERF_BLOCK_STATIC_MAIN()
    #define PERF_BLOCK( label )
    #define PERF_BLOCK_ENABLE_PROFILING( numFrames )
    #define PERF_BLOCK_CROSS_FRAME_BOUNDARY( tallyEveryFrame )
#endif

// Use this as an argument to PERF_BLOCK_ENABLE_PROFILING, to continue profile indefinitely
#define PROFILE_FOREVER -1

#define PANIC( condition ) if( ! ( condition ) ) { unsigned * crash = 0 ; * crash = 0xbaaddeed ; }

// Types -----------------------------------------------------------------------

struct PerfTally ;

typedef void (*PerfLogFunc)( const char * strFormat , ... ) ;




/** Performance logging functor that outputs to stdout.
*/
class PerfLoggerStdout
{
public:
    PerfLoggerStdout() { }

    ~PerfLoggerStdout()
    {
        fflush( stdout ) ;
    }

    static void LogFunc( const char * strFormat , ... )
    {
        va_list argList ;
        va_start( argList, strFormat ) ;
        vfprintf( stdout , strFormat , argList ) ;
        va_end( argList ) ;
        fflush( stdout ) ;
    }
} ;




/** Simple example performance logging function.

    This class is a singleton, i.e. there can be at most one object of this class.

    Example usage:

        PerfLoggerFileStream perfLogger( "perf.csv" ) ;
        PerfBlock::SetLogFunc( PerfLoggerFileStream::LogFunc ) ;
        PERF_BLOCK_MAIN( "main" ) ;  // Must have exactly one PERF_BLOCK_MAIN per thread.
        ...code to profile...

*/
class PerfLoggerFileStream
{
public:
    PerfLoggerFileStream( const char * logFilename )
    {
        PANIC( 0 == sFilePointer ) ; // File is already open.  Tried to create more than one PerfLoggerFileStream object at a time in a single process.

        sFilePointer = fopen( logFilename , "w" ) ;

        if( 0 == sFilePointer )
        {   // File failed to open.  Probably lack access permission, possibly because file is already open in another process, such as Excel.
            // Check errno.
            const int errorNumber = errno ; (void) errorNumber ;
            PANIC( 0 != sFilePointer ) ;
        }
    }

    ~PerfLoggerFileStream()
    {
        fclose( sFilePointer ) ;
        sFilePointer = 0 ;
    }

    static void LogFunc( const char * strFormat , ... )
    {
        PANIC( 0 != sFilePointer ) ; // Tried to log when no log file exists.

        va_list argList ;
        va_start( argList, strFormat ) ;
        vfprintf( sFilePointer , strFormat , argList ) ;
        va_end( argList ) ;
        fflush( sFilePointer ) ;
    }

private:
    static FILE * sFilePointer ;    ///< Log file pointer
} ;




/** Performance profiling for a lexical scope block.

    Create an object of this class at the beginning of any code to profile, in
    its own scope.  The profiling block ends where the lexical block ends.

    Use PERF_BLOCK_MAIN and PERF_BLOCK macros to create those objects.

    Creating a PerfBlock object starts a timer, and destroying that object stops
    the timer and records the duration in a corresponding PerfTally object.

    There is a many-to-one mapping between PerfBlock objects (which are
    ephemeral and live on the stack only for the duration of the block they
    time) and PerfTally objects (which persist for the lifetime of the process,
    after their corresponding block is first executed).

    The profiling model is such that each thread takes care of its own startup and shutdown.
    There is no concept of a “master thread” in this code, and data necessary for finalizing and aggregating stats are stored in thread-local-storage (TLS).

    There are 2 intended use cases:
    -   Live
        -   Each thread aggregates stats per frame.
        -   This is meant for “live” reporting of various stats, e.g. if you wanted to telemetrize it.
    -   Automatic
        -   Each thread runs for a predefined number of frames, then finalizes and aggregates stats.
        -   This is meant for benchmarking, where you’d run a thread for a fixed number of frames, to get repeatable stats.

    Notice “TerminateAndLogAllThreads”.  It is meant for the case where the profiling run terminates before Finalize and Aggregate get called automatically.

*/
class PerfBlock
{
    public:
        PerfBlock( const char * filename , unsigned line , const char * threadName ) ;
        PerfBlock( const char * label , const char * filename , unsigned line ) ;
        ~PerfBlock() ;

        static void EnableProfilingThisThread( int numFrames = 1 ) ;
        static bool IsProfileInProgressThisThread();
        static void CrossFrameBoundaryThisThread( bool tallyEveryFrame ) ;
        static PerfTally * GetMainTallyThisThread() ;

        static void TerminateAndLogAllThreads() ;

        static void SetLogFunc( PerfLogFunc func ) ;

        static void UnitTest() ;

    private:
        inline void RecordBlockExit() ;
        inline void EndBlock() ;
        PerfBlock * GetMain() const ;

        static void Aggregate( struct PerfBlockPerThreadInfo * perThreadInfo ) ;
        static void Finalize( struct PerfBlockPerThreadInfo * perThreadInfo , bool log ) ;
        static void TerminateAndLog( struct PerfBlockPerThreadInfo * perThreadInfo ) ;

        LARGE_INTEGER   mCtorEnterTime  ;   /// Time (in ticks) when the PerfBlock constructor entered -- start time for the perf block, including perf overhead.
        LARGE_INTEGER   mCtorExitTime   ;   /// Time (in ticks) when the PerfBlock constructor exited -- used to keep track of perf overhead.
        PerfBlock *     mCaller         ;   /// PerfBlock that called (i.e. contains) this one.
        PerfTally *     mPerfTallyHead  ;   /// First PerfTally object associated with this block.  This is the head of a linked list.
} ;

// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

#endif
