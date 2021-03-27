/** \file perfBlock.cpp

    \brief Performance profiling for a lexical scope block.

    \author Written and copyright 2005-2016 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
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

struct PerfBlockPerThreadInfo
{
    bool           _startProfilingNextFrame ;
    bool           _isProfileInProgress     ;
    int            _frameCounter            ;   /// Number of frames that have occurred since EnableProfilingThisThread or last call to FinalizeThisThread.
    int            _numFramesToProfile      ;   /// Number of frames remaining until profiling automatically terminates.
    PerfBlock *    _outermostPerfBlock      ;   /// "MAIN", outer-most caller block for this thread.
    PerfBlock *    _caller                  ;   /// Current inner-most caller block for this thread.
} ;

// Private variables -----------------------------------------------------------

THREAD_LOCAL_STORAGE PerfBlockPerThreadInfo sTls = { false , false , 0 , 0, NULLPTR , NULLPTR } ;

// Supporting having a single global TerminateAndLogAllThreads, that generates a report from each thread, e.g. upon termination of the process.
SpinLock                            sPerThreadInfoLock                                  ;   /// Mutex for synchronizing access to sPerThreadInfos, sPerThreadInfoCount.
static const size_t                 sMaxNumPerThreadInfos                   = 128       ;   /// Maximum number of threads supported.
PerfBlockPerThreadInfo *            sPerThreadInfos[ sMaxNumPerThreadInfos ]            ;   /// Container of "main" PerfBlock objects, one for each thread.
size_t                              sPerThreadInfoCount                     = 0         ;   /// Number of populated elements in sPerThreadInfos.

SpinLock                            sPerfLogLock                                        ;   /// Mutex for synchronizing logging output.
PerfLogFunc                         sPerfLogFunc                            = PerfLoggerStdout::LogFunc ;   /// Global shared callback for logging perf info
/* static */ FILE *                 PerfLoggerFileStream::sFilePointer      = NULLPTR   ;

static PerfTally::PerfLogFormat     sPerfLogFormat                          = PerfTally::PERF_LOG_FORMAT_TABLE ;

// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------

#if PROFILE
static void PushPerThreadInfo( PerfBlockPerThreadInfo * perThreadInfo )
{
    ASSERT( sPerThreadInfoCount < sMaxNumPerThreadInfos ) ;
    sPerThreadInfos[ sPerThreadInfoCount ] = perThreadInfo ;
    ++ sPerThreadInfoCount ;
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

    ASSERT( NULLPTR == sTls._caller ) ; // This ctor establishes the main PerfBlock for this thread.

    mCaller = NULLPTR ;  // MAIN block has no caller.

    // Create persistent tally object associated with this PerfBlock object.
    mPerfTallyHead  = PerfTally::New( threadName , filename , line ) ;

    ASSERT( NULLPTR == sTls._outermostPerfBlock ) ;
    ASSERT( NULLPTR == sTls._caller ) ;

    sTls._outermostPerfBlock  = this  ;
    sTls._caller              = this  ;  // Set new calling scope for nested profile blocks.

    // Synchronously add this PerfBlock (which is the main block for the current thread) to the list of main PerfBlocks.
    {
        ScopedSpinLock lock( sPerThreadInfoLock ) ;
        PushPerThreadInfo( & sTls ) ;
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

    if( NULLPTR == sTls._caller )
    {   // Not inside the main block yet, so do nothing.
        // This can happen if we're running construction code prior to hitting the main loop function, or there is no main block on the current thread.
        // This could also happen if this routine is running in an ephemeral worker thread that has no "MAIN" perf block, so the _caller for this thread is effectively not initialized.
        // The latter could be due to using a data-parallel library like TBB.  If so, the info for this block will not be recorded for all but the original thread.
        // If you want to capture that info, code could be added here to create a main PerfBlock, and/or this could be aggregated with other calls of the same routine but from the main thread.
        // Beware, though, that appending this to the main thread's PerfBlock would require locking that block to avoid a race condition.
        return ;
    }

    // Query performance counter to get ctor entry time to tally profile overhead.
    QueryPerformanceCounter( & mCtorEnterTime ) ;

    // Assign members inside ctor rather than using initializer, to account for profiling overhead.

    ASSERT( label != NULLPTR ) ;
    ASSERT( strcmp( label , "MAIN" ) ) ; // "MAIN" is a special outer-most block that clients should not use.
    ASSERT( sTls._caller != NULLPTR ) ; // Main PerfBlock must have already been set up for this thread.

    mCaller         = sTls._caller ;  // Remember calling scope to restore it later.

    // Find persistent tally object associated with this PerfBlock object.
    mPerfTallyHead  = mCaller->mPerfTallyHead->FindOrAppendCallee( label , filename , line ) ;
    ASSERT( mPerfTallyHead != NULL ) ;

    ASSERT( sTls._outermostPerfBlock != NULLPTR ) ;
    ASSERT( sTls._caller != NULLPTR ) ;

    sTls._caller      = this      ;  // Set new calling scope for nested profile blocks.

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
    if( NULLPTR == sTls._caller )
    {   // Not inside the main block yet, so do nothing.
        // This can happen if we're running construction code prior to hitting the main loop function, or there is no main block on the current thread.
        return;
    }

    //ASSERT( sTls._outermostPerfBlock == sTls._caller ) ; // Current block is usually outer-most.  But not always, hence commented out.

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
    if( NULLPTR == sTls._caller )
    {   // Not inside the main block yet, so do nothing.
        // This can happen if we're running construction code prior to hitting the main loop function, or there is no main block on the current thread.
        return;
    }

    // Restore previous calling scope.
    sTls._caller = mCaller ;

    if( NULLPTR == mCaller )
    {   // This is the main PerfBlock, and it is exiting.
        // Reset the sTls._OutermostPerfBlock, to allow subsequent PerfBlocks to run.
        // This is especially useful for running multiple PerfBlock unit tests sequentially in the same process.
        sTls._outermostPerfBlock = NULLPTR ;
    }

    if( sTls._isProfileInProgress )
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
/* static */ void PerfBlock::EnableProfilingThisThread( int numFramesToProfile )
{
#if PROFILE
    sTls._numFramesToProfile      = numFramesToProfile ;
    sTls._frameCounter            = 0 ;
    //sTls._startProfilingNextFrame    = true ;
    sTls._isProfileInProgress     = true ;
#else
    UNUSED_PARAM( numFramesToProfile ) ;
#endif
}




/** Return whether or not profiling is in progress
*/
/*static*/ bool PerfBlock::IsProfileInProgressThisThread()
{
#if PROFILE
    return sTls._isProfileInProgress;
#else
    return false;
#endif
}




/** Cross a profiling frame boundary for current thread.

    This routine marks the end of a block of code that runs cyclically.

    If you want to profile code that is called only once, call
    CrossFrameBoundaryThisThread just before and just after that code.
*/
/* static */ void PerfBlock::CrossFrameBoundaryThisThread( bool tallyEveryFrame )
{
#if PROFILE
    ASSERT( sTls._caller != NULLPTR ) ; // Caller should have established a main PerfBlock for current thread.
    ASSERT( sTls._outermostPerfBlock == sTls._caller ) ; // Current block should be outer-most, i.e. a main PerfBlock.  Do not call CrossFrameBoundaryThisThread from inside the scope of a (non-main) PerfBlock.

    if( sTls._isProfileInProgress )
    {   // Profile is running.
        ++ sTls._frameCounter ;

        if( sTls._numFramesToProfile > 0 )
        {   // We're not in 'run forever' mode and profiling is still running
            -- sTls._numFramesToProfile ;    // Decrement expiration counter.
            if( 0 == sTls._numFramesToProfile )
            {   // Reached end of profiling duration.
                sTls._outermostPerfBlock->RecordBlockExit() ; // Force outermost block to finalize itself.
                sTls._isProfileInProgress = false ;
                Finalize( & sTls , true ) ;  // Call FinalizeThisThread after RecordBlockExit on outermost block.
            }
        }

        if( tallyEveryFrame && ( sTls._frameCounter > 0 ) )
        {
            Aggregate( & sTls ) ;
        }
    }

    //if( sTls._startProfilingNextFrame )
    //{   // Somebody previously requested to start profiling.  Do it now.
    //    sTls._isProfileInProgress        = true ;
    //    sTls._startProfilingNextFrame    = false ;
    //}
#else
    UNUSED_PARAM( tallyEveryFrame ) ;
#endif
}




/** Aggregate performance profile statistics for given thread.
*/
/* static */ void PerfBlock::Aggregate( PerfBlockPerThreadInfo * pti )
{
#if PROFILE
    ASSERT( pti->_caller != NULLPTR ) ; // Caller should have established a main PerfBlock for current thread.
    ASSERT( pti->_outermostPerfBlock->mPerfTallyHead != NULLPTR ) ; // Make sure tally object was allocated and assigned for this block.
    ASSERT( pti->_outermostPerfBlock == pti->_caller ) ; // Current block should be outer-most. Otherwise, calling AggregateThisThread from inside a block would mean the process is not at the end of a "frame" and aggregation would be incomplete.

    // The outermost block is the static one for the thread, which (if set up correctly) never leaves scope.
    // It therefore has no EndBlock, so force it to record the exit of a block.
    // Note that it is possible for mTotalDurationInclusive to be non-zero here, if FinalizeThisThread was previously called
    // from CrossFrameBoundaryThisThread (i.e. due to an automatic tally at the end of a profiling interval).
    // It is also possible for sIsProfileInProgress to be either true or false here, regardless of the value of
    // mTotalDurationInclusive (zero or not), depending on the recent history of calls to CrossFrameBoundaryThisThread, etc.
    if( pti->_isProfileInProgress )
    {
        pti->_outermostPerfBlock->RecordBlockExit() ;
    }

    pti->_outermostPerfBlock->mPerfTallyHead->Tally( pti->_frameCounter ) ;
#else
    UNUSED_PARAM( pti ) ;
#endif
}




/** Aggregate, then reset, performance profile statistics for this thread, and optionally log results to a file stream.
*/
/* static */ void PerfBlock::Finalize( PerfBlockPerThreadInfo * pti , bool log )
{
#if PROFILE
    Aggregate( pti ) ;

    pti->_frameCounter = 0 ;

    if( log )
    {
        ScopedSpinLock logLock( sPerfLogLock ) ; // Obtain lock on log.
        pti->_outermostPerfBlock->mPerfTallyHead->LogThread( 1 , sPerfLogFunc , sPerfLogFormat ) ;
        pti->_outermostPerfBlock->mPerfTallyHead->LogFooter( sPerfLogFunc , sPerfLogFormat ) ;
    }
    pti->_outermostPerfBlock->mPerfTallyHead->Reset() ;
#else
    UNUSED_PARAM( pti ) ;
    UNUSED_PARAM( log ) ;
#endif
}




/** Terminate profiling, aggregate and log results for given thread.

    This is useful for when a profiling session was in progress but the process terminates early,
    for example due to the user exiting the process.

*/
/* static */ void PerfBlock::TerminateAndLog( PerfBlockPerThreadInfo * pti )
{
#if PROFILE
    ASSERT( pti->_caller != NULLPTR ) ; // Caller should have established a main PerfBlock for current thread.

    // Remember current inner-most caller block for this thread.
    // Technically this should not be necessary since TerminateAndLog should only be called in an exit handler (or equivalent).
    // But people do not always obey the prescribed usage pattern, so this protects against that somewhat.
    PerfBlock *    actualCaller = pti->_caller ;

    pti->_caller = pti->_outermostPerfBlock ; // Spoof current block to be outer-most. This lets AggregateThisThread not assert.

    if( pti->_isProfileInProgress && ( pti->_frameCounter > 0 ) )
    {   // Profile is running and at least one frame has executed.
        pti->_outermostPerfBlock->RecordBlockExit() ; // Force outermost block to finalize itself.
        pti->_isProfileInProgress = false ;
        Finalize( pti , true ) ;  // Call FinalizeThisThread after RecordBlockExit on outermost block.
    }

    pti->_caller = actualCaller ; // Restore record of actual place in call stack.
#else
    UNUSED_PARAM( pti ) ;
#endif
}




/** Get main PerfTally object associated with this thread.
*/
/* static */ PerfTally * PerfBlock::GetMainTallyThisThread()
{
#if PROFILE
    ASSERT( sTls._outermostPerfBlock != NULLPTR ) ;
    return sTls._outermostPerfBlock->mPerfTallyHead ;
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




/** Terminate profiling, aggregate and log results for given thread.

    This is useful for when a profiling session was in progress but the process terminates early,
    for example due to the user exiting the process.
*/
/* static */ void PerfBlock::TerminateAndLogAllThreads()
{
    // BEWARE: This routine ends up holding multiple locks (sPerThreadInfoLock and sPerfLogLock).  Make sure EVERYWHERE that these locks are always obtained in the same order, otherwise deadlock can occur.
    ScopedSpinLock perThreadInfoLock( sPerThreadInfoLock ) ; // Obtain lock on sPerThreadInfos, sPerThreadInfoCount
    ScopedSpinLock lock( sPerfLogLock ) ; // Obtain lock on log.

    for( size_t iThread = 0 ; iThread < sPerThreadInfoCount ; ++ iThread )
    {   // For each thread...
        TerminateAndLog( sPerThreadInfos[ iThread ] ) ;
    }

    PerfTally::LogFooter( sPerfLogFunc , sPerfLogFormat ) ;
}




#if defined( UNIT_TEST )

#include <math.h>

void TestFrame( bool tallyEveryFrame )
{
    static volatile float acc = 0.0f ;
    {
        PERF_BLOCK( for_i ) ;
        for( int i = 0 ; i < 3 ; ++ i )
        {
            {
                PERF_BLOCK( for_iu ) ;
                for( unsigned u = 0 ; u < 0x1111 ; ++ u )
                {
                    float fu = float( u ) + 1.0f ;
                    acc *= 1.0f / sqrtf( fu ) ;
                    acc += 1.0f ;
                }
            }
            {
                PERF_BLOCK( for_iv ) ;
                for( unsigned v = 0 ; v < 0x2222 ; ++ v )
                {
                    float fv = float( v ) + 2.0f ;
                    acc *= 1.0f / sqrtf( fv ) ;
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
                    acc *= 1.0f / sqrtf( fu ) ;
                    acc += 5.0f ;
                }
            }
            {
                PERF_BLOCK( for_jv ) ;
                for( unsigned v = 0 ; v < 0x5555 ; ++ v )
                {
                    float fv = float( v ) + 10.0f ;
                    acc *= 1.0f / sqrtf( fv ) ;
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
                    acc *= 1.0f / sqrtf( fu ) ;
                    acc += 20.0f ;
                }
            }
            {
                PERF_BLOCK( for_kv ) ;
                for( unsigned v = 0 ; v < 0xffff ; ++ v )
                {
                    float fv = float( v ) + 20.0f ;
                    acc *= 1.0f / sqrtf( fv ) ;
                    acc += 50.0f ;
                }
            }
        }
    }
    PERF_BLOCK_CROSS_FRAME_BOUNDARY( tallyEveryFrame ) ;
}




// Manually finalize at end of profiling run.
void PerfBlock_UnitTest_ManualFinalize()
{
    PERF_BLOCK_MAIN( "PerfBlock_UnitTest_ManualFinalize" ) ;  // Must have exactly one PERF_BLOCK_MAIN per thread.

    PERF_BLOCK_ENABLE_PROFILING( -1 ) ;   // Profile indefinitely.
    for( int frame = 0 ; frame < 5 ; ++ frame )
    {
        TestFrame( false ) ;
    }

    PerfBlock::TerminateAndLogAllThreads() ;
}




// Automatically finalize only at end of profiling duration.
void PerfBlock_UnitTest_AutoFinalize()
{
    PERF_BLOCK_MAIN( "PerfBlock_UnitTest_AutoFinalize" ) ;  // Must have exactly one PERF_BLOCK_MAIN per thread.

    PERF_BLOCK_ENABLE_PROFILING( 3 ) ;   // Profile next 3 frames.
    for( int frame = 0 ; frame < 5 ; ++ frame )
    {
        TestFrame( false ) ;
    }
}




// Finalize after each frame to get updates every frame.  Useful for live reporting.
void PerfBlock_UnitTest_LiveReporting()
{
    PERF_BLOCK_MAIN( "PerfBlock_UnitTest_LiveReporting" ) ;  // Must have exactly one PERF_BLOCK_MAIN per thread.

    PERF_BLOCK_ENABLE_PROFILING( 0 ) ;   // Profile indefinitely.
    for( int frame = 0 ; frame < 5 ; ++ frame )
    {   // Run for 5 frames.
        TestFrame( true ) ;
        const PerfTally * perfTallyMain  = PerfBlock::GetMainTallyThisThread() ;
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




/** Finalize after each frame to get updates every frame, and also terminate profiling after finite number of frames.

    Note that after 2 iterations, this live tally will show all zeros.
    That is because at the end of the 3rd (final) iteration, PerfBlock automatically finalizes the tally, which happens before this.
    If this seems odd, consider that the scenario is itself odd: The caller specified both that it wants automatic finalization after finite frames,
    and also wants live tallying.  It would be more typical to want either an indefinite number of frames with live tallying (for live reporting)
    or a finite number of frames (for a benchmark).  This unit test exercises both at the same time, which is not a typical use case.
*/
void PerfBlock_UnitTest_MixAutoFinalizeWithLiveReporting()
{
    PERF_BLOCK_MAIN( "PerfBlock_UnitTest_MixAutoFinalizeWithLiveReporting" ) ;  // Must have exactly one PERF_BLOCK_MAIN per thread.

    PERF_BLOCK_ENABLE_PROFILING( 3 ) ;   // Profile next 3 frames.
    for( int frame = 0 ; frame < 5 ; ++ frame )
    {   // Run for 5 frames.
        TestFrame( true ) ;
        const PerfTally * perfTallyMain  = PerfBlock::GetMainTallyThisThread() ;
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




void PerfBlock::UnitTest()
{
    PerfBlock_UnitTest_ManualFinalize() ;

    PerfBlock_UnitTest_AutoFinalize() ;

    PerfBlock_UnitTest_LiveReporting() ;

    PerfBlock_UnitTest_MixAutoFinalizeWithLiveReporting() ;

    return ; // Do-nothing statement. Makes it easier to set breakpoint here.
}

#endif
