/** \file perfBlock.cpp

    \brief Performance profiling for a lexical scope block.

    \author Written and copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "Core/Utility/macros.h"

#include "Core/File/debugPrint.h"

#include "perfTally.h"
#include "perfBlock.h"

#if defined( _DEBUG )
    #include "Core/File/debugPrint.h"
    #define PERF_PRINTF DebugPrintf
#else
    #include <stdio.h>
    #define PERF_PRINTF printf
#endif

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------

THREAD_LOCAL_STORAGE bool           sStartProfilingNextFrame                = false     ;
THREAD_LOCAL_STORAGE bool           sIsProfileInProgress                    = false     ;
THREAD_LOCAL_STORAGE int            sFrameCounter                           = 0         ;   /// Number of frames that have occurred since EnableProfiling or last call to Finalize.
THREAD_LOCAL_STORAGE int            sNumFramesToProfile                     = 0         ;   /// Number of frames remaining until profiling automatically terminates.
THREAD_LOCAL_STORAGE PerfBlock *    sTlsOutermostPerfBlock                  = NULLPTR   ;   /// "MAIN", outer-most caller block for this thread.
THREAD_LOCAL_STORAGE PerfBlock *    sTlsCaller                              = NULLPTR   ;   /// Current inner-most caller block for this thread.

AtomicBool                          sMainPerfBlockLock                                  ;   /// Mutex for synchronizing access to sMainPerfBlocks, sMainPerfBlockCount.
static const size_t                 sMaxNumMainPerfBlocks                   = 128       ;   /// Maximum number of threads supported.
PerfBlock *                         sMainPerfBlocks[ sMaxNumMainPerfBlocks ]            ;   /// Container of "main" PerfBlock objects, one for each thread.
size_t                              sMainPerfBlockCount                     = 0         ;   /// Number of populated elements in sMainPerfBlocks.
AtomicBool                          sPerfLogLock                                        ;   /// Mutex for synchronizing logging output.
PerfLogFunc                         sPerfLogFunc                            = NULLPTR   ;   /// Global shared callback for logging perf info
/* static */ FILE *                 PerfLoggerExample::sFilePointer         = NULLPTR   ;

// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------

#if PROFILE
static void PushMainPerfBlock( PerfBlock * perfBlock )
{
    ASSERT( sMainPerfBlockCount < sMaxNumMainPerfBlocks ) ;
    sMainPerfBlocks[ sMainPerfBlockCount ] = perfBlock ;
}
#endif

// Public functions ------------------------------------------------------------

/** Create the main performance block for this thread.
*/
PerfBlock::PerfBlock( const char * filename , unsigned line , const char * threadName )
{
#if PROFILE
    // Query performance counter to get ctor entry time to tally profile overhead.
    QueryPerformanceCounter( & mCtorEnterTime ) ;

    // Assign members inside ctor rather than using initializer, to account for profiling overhead.

    ASSERT( NULLPTR == sTlsCaller ) ; // This ctor establishes the main PerfBlock for this thread.

    mCaller = NULLPTR ;  // MAIN block has no caller.

    // Create persistent tally object associated with this PerfBlock object.
    mPerfTallyHead  = PerfTally::New( threadName , filename , line ) ;

    ASSERT( NULLPTR == sTlsOutermostPerfBlock ) ;
    ASSERT( NULLPTR == sTlsCaller ) ;

    sTlsOutermostPerfBlock  = this  ;
    sTlsCaller              = this  ;  // Set new calling scope for nested profile blocks.

    // Synchronously add this PerfBlock (which is the main block for the current thread) to the list of main PerfBlocks.
    {
        ScopedSpinLock lock( sMainPerfBlockLock ) ;
        PushMainPerfBlock( this ) ;
    }

    // Query performance counter to get block entry time.
    QueryPerformanceCounter( & mCtorExitTime ) ;
#else
    (void) filename , line , threadName ;
#endif
}




/** Enter a non-main performance profiling block.
*/
PerfBlock::PerfBlock( const char * label , const char * filename , unsigned line )
{
#if PROFILE

    if( NULLPTR == sTlsCaller )
    {   // Not inside the main block yet, so do nothing.
        // This can happen if we're running construction code prior to hitting the main loop function, or there is no main block on the current thread.
        return ;
    }

    // Query performance counter to get ctor entry time to tally profile overhead.
    QueryPerformanceCounter( & mCtorEnterTime ) ;

    // Assign members inside ctor rather than using initializer, to account for profiling overhead.

    ASSERT( label != NULLPTR ) ;
    ASSERT( strcmp( label , "MAIN" ) ) ; // "MAIN" is a special outer-most block that clients should not use.
    ASSERT( sTlsCaller != NULLPTR ) ; // Main PerfBlock must have already been set up for this thread.

    mCaller         = sTlsCaller ;  // Remember calling scope to restore it later.

    // Find persistent tally object associated with this PerfBlock object.
    mPerfTallyHead  = mCaller->mPerfTallyHead->FindOrAppendCallee( label , filename , line ) ;
    ASSERT( mPerfTallyHead != NULL ) ;

    ASSERT( sTlsOutermostPerfBlock != NULLPTR ) ;
    ASSERT( sTlsCaller != NULLPTR ) ;

    sTlsCaller      = this      ;  // Set new calling scope for nested profile blocks.

    // Query performance counter to get block entry time.
    QueryPerformanceCounter( & mCtorExitTime ) ;
#else
    (void) label , filename , line ;
#endif
}




/** Record the exit of a performance block.

    Only call this for outermost blocks (i.e. the thread main block).
    Do not call this directly for non-outermost blocks.

    \see EndBlock.
    
*/
inline void PerfBlock::RecordBlockExit()
{
#if PROFILE
    if( NULLPTR == sTlsCaller )
    {   // Not inside the main block yet, so do nothing.
        // This can happen if we're running construction code prior to hitting the main loop function, or there is no main block on the current thread.
        return;
    }

    //ASSERT( sTlsOutermostPerfBlock == sTlsCaller ) ; // Current block should be outer-most.

    // Query performance counter to get block exit time.
    LARGE_INTEGER dtorEnterTime ;
    QueryPerformanceCounter( & dtorEnterTime ) ;

    LONGLONG blockDuration = dtorEnterTime.QuadPart - mCtorExitTime.QuadPart ;

    mPerfTallyHead->mInContextAggregate.mTotalDurationInclusive  += blockDuration ;
    mPerfTallyHead->mInContextAggregate.mNumCalls ++ ;

    mPerfTallyHead->mInContextAggregate.mMaxDurationInclusive = Max2( mPerfTallyHead->mInContextAggregate.mMaxDurationInclusive , blockDuration ) ;

    // Query performance counter to get dtor exit time -- to tally profile overhead.
    LARGE_INTEGER dtorExitTime ;
    QueryPerformanceCounter( & dtorExitTime ) ;
    mPerfTallyHead->mInContextAggregate.mTotalDurationWithOverhead += dtorExitTime.QuadPart - mCtorEnterTime.QuadPart ;
#endif
}




/** Exit a performance profiling block.

    Only call this for non-outermost blocks (i.e. the thread main block).
    Do not call this directly for outermost blocks.

    \see RecordBlockExit.

*/
inline void PerfBlock::EndBlock()
{
#if PROFILE
    if( NULLPTR == sTlsCaller )
    {   // Not inside the main block yet, so do nothing.
        // This can happen if we're running construction code prior to hitting the main loop function, or there is no main block on the current thread.
        return;
    }

    // Restore previous calling scope.
    sTlsCaller = mCaller ;

    if( sIsProfileInProgress )
    {
        RecordBlockExit() ;
    }
#endif
}




/** Exit a performance profiling block.
*/
PerfBlock::~PerfBlock()
{
#if PROFILE
    EndBlock() ;
#endif
}




/** Enabling profiling starting next frame, lasting the given number of frames.
*/
/* static */ void PerfBlock::EnableProfiling( int numFramesToProfile )
{
#if PROFILE
    sNumFramesToProfile         = numFramesToProfile ;
    sFrameCounter               = 0 ;
    sStartProfilingNextFrame    = true ;
#else
    UNUSED_PARAM( numFramesToProfile ) ;
#endif
}




/** Return whether or not profiling is in progress
*/
/*static*/ bool PerfBlock::IsProfileInProgress()
{
#if PROFILE
    return sIsProfileInProgress;
#else
    return false;
#endif
}




/** Cross a profiling frame bouncary for current thread.

    This routine marks the start of a block of code that runs cyclically.

    If you want to profile code that is called only once, call
    CrossFrameBoundary just before and just after that code.
*/
/* static */ void PerfBlock::CrossFrameBoundary( bool tallyEveryFrame )
{
#if PROFILE
    ASSERT( sTlsCaller != NULLPTR ) ; // Caller should have established a main PerfBlock for current thread.
    ASSERT( sTlsOutermostPerfBlock == sTlsCaller ) ; // Current block should be outer-most.

    if( sIsProfileInProgress )
    {   // Profile is running.
        ++ sFrameCounter ;

        if( sNumFramesToProfile > 0 )
        {   // We're not in 'run forever' mode and profiling is still running
            -- sNumFramesToProfile ;    // Decrement expiration counter.
        }

        if( 0 == sNumFramesToProfile )
        {   // Reached end of profiling duration.
            sTlsOutermostPerfBlock->RecordBlockExit() ; // Force outermost block to finalize itself.
            sIsProfileInProgress = false ;
            Finalize( true ) ;  // Call Finalize after RecordBlockExit on outermost block.
        }
        else if( tallyEveryFrame )
        {
            Aggregate() ;
        }
    }

    if( sStartProfilingNextFrame )
    {   // Somebody previously requested to start profiling.  Do it now.
        sIsProfileInProgress        = true ;
        sStartProfilingNextFrame    = false ;
    }
#else
    UNUSED_PARAM( tallyEveryFrame ) ;
#endif
}




/** Aggregate performance profile statistics for this thread.
*/
/* static */ void PerfBlock::Aggregate()
{
#if PROFILE
    ASSERT( sTlsCaller != NULLPTR ) ; // Caller should have established a main PerfBlock for current thread.
    ASSERT( sTlsOutermostPerfBlock->mPerfTallyHead != NULLPTR ) ; // Make sure tally object was allocated and assigned for this block.
    ASSERT( sTlsOutermostPerfBlock == sTlsCaller ) ; // Current block should be outer-most. Otherwise, calling Aggregate from inside a block would mean the process is not at the end of a "frame" and aggregation would be incomplete.

    // The outermost block is the static one for the thread, which (if set up correctly) never leaves scope.
    // It therefore has no EndBlock, so force it to record the exit of a block.
    // Note that it is possible for mTotalDurationInclusive to be non-zero here, if Finalize was previously called
    // from CrossFrameBoundary (i.e. due to an automatic tally at the end of a profiling interval).
    // It is also possible for sIsProfileInProgress to be either true or false here, regardless of the value of
    // mTotalDurationInclusive (zero or not), depending on the recent history of calls to CrossFrameBoundary, etc.
    if( sIsProfileInProgress )
    {
        sTlsOutermostPerfBlock->RecordBlockExit() ;
    }

    sTlsOutermostPerfBlock->mPerfTallyHead->Tally( sFrameCounter ) ;

    sFrameCounter = 0 ;
#endif
}




/** Aggregate, then reset, performance profile statistics for this thread, and optionally log results to a file stream.
*/
/* static */ void PerfBlock::Finalize( bool log )
{
#if PROFILE
    Aggregate() ;

    if( log )
    {
        ScopedSpinLock logLock( sPerfLogLock ) ; // Obtain lock on log.
        sTlsOutermostPerfBlock->mPerfTallyHead->Log( 0 , sPerfLogFunc ) ;
    }
    sTlsOutermostPerfBlock->mPerfTallyHead->Reset() ;
#else
    UNUSED_PARAM( log ) ;
#endif
}




/** Terminate profiling, aggregeate and log results.

    This is useful for when a profiling session was in progress but the process terminates early,
    for example due to the user exiting the process.

*/
/* static */ void PerfBlock::TerminateAndLog()
{
#if PROFILE
    ASSERT( sTlsCaller != NULLPTR ) ; // Caller should have established a main PerfBlock for current thread.

    // Remember current inner-most caller block for this thread.
    // Technically this should not be necessary since TerminateAndLog should only be called in an exit handler (or equivalent).
    // But people do not always obey the prescribed usage pattern, so this protects against that somewhat.
    PerfBlock *    actualCaller = sTlsCaller ;

    sTlsCaller = sTlsOutermostPerfBlock ; // Spoof current block to be outer-most. This lets Aggregate not assert.

    if( sIsProfileInProgress && ( sFrameCounter > 0 ) )
    {   // Profile is running and at least one frame has executed.
        sTlsOutermostPerfBlock->RecordBlockExit() ; // Force outermost block to finalize itself.
        sIsProfileInProgress = false ;
        Finalize( true ) ;  // Call Finalize after RecordBlockExit on outermost block.
    }

    sTlsCaller = actualCaller ; // Restore record of actual place in call stack.
#endif
}




/** Get main PerfTally object associated with this thread.
*/
/* static */ PerfTally * PerfBlock::GetMainTally()
{
#if PROFILE
    ASSERT( sTlsOutermostPerfBlock != NULLPTR ) ;
    return sTlsOutermostPerfBlock->mPerfTallyHead ;
#else
    return NULLPTR ;
#endif
}




/** Get main PerfBlock (i.e. root of call tree, i.e. main block for thread) for given PerfBlock.
*/
PerfBlock * PerfBlock::GetMain() const
{
    const PerfBlock * root = this ;
    while( root->mCaller != NULL )
    {
        root = root->mCaller ;
    }
    return const_cast< PerfBlock * >( root ) ;
}




/** Register a custom logging function.
*/
/*static*/ void PerfBlock::SetLogFunc(PerfLogFunc func)
{
    ScopedSpinLock lock( sPerfLogLock ) ; // Obtain lock.
    sPerfLogFunc = func;
}




/** Dump performance logs for all threads.
*/
/* static */ void PerfBlock::LogAllThreads()
{
    ScopedSpinLock logLock( sPerfLogLock ) ; // Obtain lock on log.

    ScopedSpinLock mainPerfBlockLock( sPerfLogLock ) ; // Obtain lock on sMainPerfBlock
    for( size_t iBlock = 0 ; iBlock < sMainPerfBlockCount ; ++ iBlock )
    {
        PerfBlock * perfBlock     = sMainPerfBlocks[ iBlock ] ;
        PerfBlock * perfBlockMain = perfBlock->GetMain() ;
        PerfTally * perfTally     = perfBlockMain->mPerfTallyHead ;
        perfTally->Log( 0 , sPerfLogFunc ) ;
    }
}




#if defined( UNIT_TEST )

#include <math.h>

void TestFrame()
{
    static volatile float acc = 0.0f ;
    PERF_BLOCK_CROSS_FRAME_BOUNDARY( false ) ;
    {
        PERF_BLOCK( for_i ) ;
        for( int i = 0 ; i < 3 ; ++ i )
        {
            {
                PERF_BLOCK( for_iu ) ;
                for( unsigned u = 0 ; u < 0x1111 ; ++ u )
                {
                    float fu = float( u ) + 1.0f ;
                    acc *= 1.0f / sqrt( fu ) ;
                    acc += 1.0f ;
                }
            }
            {
                PERF_BLOCK( for_iv ) ;
                for( unsigned v = 0 ; v < 0x2222 ; ++ v )
                {
                    float fv = float( v ) + 2.0f ;
                    acc *= 1.0f / sqrt( fv ) ;
                    acc += 2.0f ;
                }
            }
        }
    }
    {
        PERF_BLOCK( for_j ) ;
        for( int j = 0 ; j < 5 ; ++ j )
        {
            {
                PERF_BLOCK( for_ju ) ;
                for( unsigned u = 0 ; u < 0x3333 ; ++ u )
                {
                    float fu = float( u ) + 5.0f ;
                    acc *= 1.0f / sqrt( fu ) ;
                    acc += 5.0f ;
                }
            }
            {
                PERF_BLOCK( for_jv ) ;
                for( unsigned v = 0 ; v < 0x5555 ; ++ v )
                {
                    float fv = float( v ) + 10.0f ;
                    acc *= 1.0f / sqrt( fv ) ;
                    acc += 10.0f ;
                }
            }
        }
    }
    {
        PERF_BLOCK( for_k ) ;
        for( int k = 0 ; k < 8 ; ++ k )
        {
            {
                PERF_BLOCK( for_ku ) ;
                for( unsigned u = 0 ; u < 0x7777 ; ++ u )
                {
                    float fu = float( u ) + 10.0f ;
                    acc *= 1.0f / sqrt( fu ) ;
                    acc += 20.0f ;
                }
            }
            {
                PERF_BLOCK( for_kv ) ;
                for( unsigned v = 0 ; v < 0xffff ; ++ v )
                {
                    float fv = float( v ) + 20.0f ;
                    acc *= 1.0f / sqrt( fv ) ;
                    acc += 50.0f ;
                }
            }
        }
    }
}




void PerfBlock::UnitTest()
{
    PERF_BLOCK_MAIN( "UnitTestThread" ) ;  // Must have exactly one PERF_BLOCK_MAIN per thread.

    {   // Automatically finalize only at end of profiling duration.
        PERF_BLOCK_ENABLE_PROFILING( 3 ) ;   // Profile next 3 frames.
        for( int frame = 0 ; frame < 5 ; ++ frame )
        {
            TestFrame() ;
        }
    }

    {   // Finalize after each frame to get updates every frame.  Useful for live reporting.
        PerfBlock::EnableProfiling( 3 ) ;   // Profile next 3 frames.
        for( int frame = 0 ; frame < 5 ; ++ frame )
        {
            TestFrame() ;
            PerfBlock::Aggregate() ;
            const PerfTally * perfTallyMain  = PerfBlock::GetMainTally() ;
            if( perfTallyMain )
            {
                const PerfTally * perfTallyForI  = perfTallyMain->Find( "for_i" ) ;
                const PerfTally * perfTallyForJ  = perfTallyMain->Find( "for_j" ) ;
                const PerfTally * perfTallyForKu = perfTallyMain->Find( "for_ku" ) ;
                PERF_PRINTF(
                    "main:%g i:%g j:%g ku:%g\n"
                    ,   perfTallyMain->mInContextAggregate.mAverageDurationInclusive  * PerfTally::MilliSecondsPerTick()
                    ,   perfTallyForI->mInContextAggregate.mAverageDurationInclusive  * PerfTally::MilliSecondsPerTick()
                    ,   perfTallyForJ->mInContextAggregate.mAverageDurationInclusive  * PerfTally::MilliSecondsPerTick()
                    ,   perfTallyForKu->mInContextAggregate.mAverageDurationInclusive * PerfTally::MilliSecondsPerTick()
                    ) ;
            }
        }
    }

    return ;
}

#endif
