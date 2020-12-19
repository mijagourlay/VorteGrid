/** \file perfBlock.h

    \brief Performance profiling for a lexical scope block.

    \author Written and copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
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
    #define PERF_BLOCK_MAIN( threadName ) PerfBlock perfBlock_MAIN( __FILE__ , __LINE__ , threadName ) ;
    #define PERF_BLOCK( label ) PerfBlock perfBlock_ ## label ## _( # label , __FILE__ , __LINE__ ) ;
    #define PERF_BLOCK_ENABLE_PROFILING( numFrames ) PerfBlock::EnableProfiling( numFrames ) ;
    #define PERF_BLOCK_CROSS_FRAME_BOUNDARY( tallyEveryFrame ) PerfBlock::CrossFrameBoundary( tallyEveryFrame ) ;
#else
    #define PERF_BLOCK_MAIN( threadName )
    #define PERF_BLOCK_STATIC_MAIN()
    #define PERF_BLOCK( label )
    #define PERF_BLOCK_ENABLE_PROFILING( numFrames )
    #define PERF_BLOCK_CROSS_FRAME_BOUNDARY( tallyEveryFrame )
#endif

// Use this as an argument to PERF_BLOCK_ENABLE_PROFILING, to continue profile indefinitely
#define PROFILE_FOREVER -1


// Types -----------------------------------------------------------------------

struct PerfTally ;

typedef void (*PerfLogFunc)( const char * strFormat , ... ) ;




/** Simple example performance logging functor.

    This class is a singleton, i.e. there can be at most one object of this class.


    Example usage:

        PerfLoggerExample perfLogger( "perf.csv" ) ;
        PerfBlock::SetLogFunc( PerfLoggerExample::LogFunc ) ;
        PERF_BLOCK_MAIN( "main" ) ;  // Must have exactly one PERF_BLOCK_MAIN per thread.
        ...code to profile...

*/
class PerfLoggerExample
{
public:
    PerfLoggerExample( const char * logFilename )
    {
        if( 0 != sFilePointer ) { unsigned * crash = 0 ; * crash = 0xbaaddeed ; }   // File is already open.  Tried to create more than one PerfLoggerExample object at a time in a single process.

        sFilePointer = fopen( logFilename , "w" ) ;

        if( 0 == sFilePointer )
        {   // File failed to open.  Probably lack access permission, possibly because file is already open in another process, such as Excel.
            // Check errno.
            const int errorNumber = errno ; (void) errorNumber ;
            unsigned * crash = 0 ; * crash = 0xbaadf00d ;
        }
    }

    ~PerfLoggerExample()
    {
        fclose( sFilePointer ) ;
        sFilePointer = 0 ;
    }

    static void LogFunc( const char * strFormat , ... )
    {
        if( 0 == sFilePointer ) { unsigned * crash = 0 ; * crash = 0xbaadcafe ; }   // Tried to log when no log file exists.

        va_list argList ;
        va_start( argList, strFormat ) ;
        vfprintf( sFilePointer , strFormat , argList ) ;
        va_end( argList ) ;
    }

private:
    static FILE * sFilePointer ;    ///< Log file pointer
} ;




/** Performance profiling for a lexical scope block.

    Create an object of this class at the beginning of any code to profile, in
    its own scope.  The profiling block ends where the lexical block ends.

    Creating a PerfBlock object starts a timer, and destroying that object stops
    the timer and records the duration in a corresponding PerfTally object.

    There is a many-to-one mapping between PerfBlock objects (which are
    ephemeral and live on the stack only for the duration of the block they
    time) and PerfTally objects (which persist for the lifetime of the process,
    after their corresponding block is first executed).

*/
class PerfBlock
{
    public:
        PerfBlock( const char * filename , unsigned line , const char * threadName ) ;
        PerfBlock( const char * label , const char * filename , unsigned line ) ;
        ~PerfBlock() ;

        static void EnableProfiling( int numFrames = 1 ) ;
        static bool IsProfileInProgress();
        static void CrossFrameBoundary( bool tallyEveryFrame ) ;
        static void Aggregate() ;
        static void Finalize( bool log ) ;
        static void TerminateAndLog() ;
        static PerfTally * GetMainTally() ;

        static void SetLogFunc( PerfLogFunc func ) ;
        static void LogAllThreads() ;

        static void UnitTest() ;

    private:
        inline void RecordBlockExit() ;
        inline void EndBlock() ;
        PerfBlock * GetMain() const ;

        LARGE_INTEGER   mCtorEnterTime  ;   /// Time (in ticks) when the PerfBlock constructor entered -- start time for the perf block, including perf overhead.
        LARGE_INTEGER   mCtorExitTime   ;   /// Time (in ticks) when the PerfBlock constructor exitted -- used to keep track of perf overhead.
        PerfBlock *     mCaller         ;   /// PerfBlock that called (i.e. contains) this one.
        PerfTally *     mPerfTallyHead  ;   /// First PerfTally object associated with this block.  This is the head of a linked list.
} ;

// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

#endif
