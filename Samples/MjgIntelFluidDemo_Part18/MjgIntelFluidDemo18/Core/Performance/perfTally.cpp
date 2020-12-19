/** \file perfTally.cpp

    \brief Tally of runtime performance measurements for a block being profiled.

    \author Written and copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <new>  // For placement new.

#include <stdio.h>

#if defined( _DEBUG )
    #include "Core/File/debugPrint.h"
    #define PERF_PRINTF DebugPrintf
#else
    #define PERF_PRINTF printf
#endif

// NOTE: I doubt sPerfTallyCount needs to be atomic now that sPerfTalliesLock synchronizes access to it.
#if _MSC_VER && ( _MSC_VER < 1700 ) // Older compiler that lacks atomic.
    typedef volatile int    AtomicInt ;
    AtomicInt   sPerfTallyCount = 0 ;   /// Used by linear allocator to index into sPerfTallies pool.
#else   // Modern compiler that has atomic.
    #include <atomic>
    typedef std::atomic_int AtomicInt ;
    AtomicInt   sPerfTallyCount ;   /// Used by linear allocator to index into sPerfTallies pool.
#endif

#include "Core/Utility/macros.h"

#include "perfTally.h"

// Types -----------------------------------------------------------------------

// Private variables -----------------------------------------------------------

// Allocate PerfTally objects linearly to reduce profiling overhead.
static const size_t MAX_NUM_PERF_TALLIES    = 2048                  ;   ///< Maximum number of PerfTally objects for entire process.
PerfTally           sPerfTallies[ MAX_NUM_PERF_TALLIES ]            ;   ///< Global pool of PerfTally objects, shared across entire process.  NOTE: If sPerfTallyCount and sPerfTallies were TLS, it wouldn't need a lock.  Would need to make PerfTally POD without ctor.
AtomicBool          sPerfTalliesLock                                ;   ///< Mutex for synchronizing access to sPerfTallies and sPerfTallyCount.

static const size_t         MAX_NUM_PERF_AGG_STATS  = 1024                  ;   ///< Maximum number of PerfAggregatedStatsWithId objects for entire process.
PerfAggregatedStatsWithId   sPerfAggregatedStats[ MAX_NUM_PERF_AGG_STATS ]  ;   ///< Global pool for PerfAggregatedStatsWithId objects, shared across entire process.
size_t                      sPerfAggregatedStatsCount = 0                   ;   ///< Used by linear allocator to index into sPerfAggregatedStats pool.

// Private functions -----------------------------------------------------------

/** Return the address of a string with the given number of spaces.
*/
const char * Indent( const int indent )
{
    static const unsigned NUM_SPACES = 128 ;
    static const char spaces[ NUM_SPACES + 1 ] = "                                                                                                                                " ;
    ASSERT( indent <= NUM_SPACES ) ;
    return & spaces[ NUM_SPACES - indent ] ;
}


// Public functions ------------------------------------------------------------




// -----------------------------------------------------------------------------
// PerfAggregatedStatsWithId
// -----------------------------------------------------------------------------




/** Find PerfAggregatedStatsWithId object associated with the given identifier.
*/
PerfAggregatedStatsWithId * PerfAggregatedStatsWithId::Find( const PerfBlockIdentifier * id )
{
    // Synchronously search global pool.
    ScopedSpinLock  lock( sPerfTalliesLock ) ;

    for( size_t idx = 0 ; idx < sPerfAggregatedStatsCount ; ++ idx )
    {   // For each PerfAggregatedStats in global pool...
        PerfAggregatedStatsWithId * tuple = & sPerfAggregatedStats[ idx ] ;
        if( ( tuple->mId ) && ! strcmp( tuple->mId->mLabel , id->mLabel ) )
        {   // Found object with sought label.  That should uniquely identify block but only if labels are globally unique across process.
            if( tuple->mId->mLine == id->mLine )
            {   // Found sought object.
                if( strcmp( tuple->mId->mFilename , id->mFilename ) )
                {   // Label is not unique.  Compiler probably used differnt paths for the same source file.  Maybe #include is not consistent.
                    DEBUG_BREAK() ;
                }
                return tuple ;
            }
            else
            {   // Label is not unique.  Probably from copy-pasta, in which case label should be changed.
                DEBUG_BREAK() ;
            }
        }
    }

    return NULLPTR ; // Did not find sought object.
}




/** Allocate and initialize a new PerfAggregatedStatsWithId object.
*/
PerfAggregatedStatsWithId * PerfAggregatedStatsWithId::New()
{
    // Synchronously allocate PerfAggregatedStats from global pool.
    ScopedSpinLock  lock( sPerfTalliesLock ) ;

    // Allocate a new object.
    ASSERT( sPerfAggregatedStatsCount < MAX_NUM_PERF_AGG_STATS ) ;
    if( sPerfAggregatedStatsCount >= MAX_NUM_PERF_AGG_STATS ) { int * crash = NULLPTR ; * crash = 0xbada110c ; }

    if( sPerfAggregatedStatsCount < MAX_NUM_PERF_AGG_STATS )
    {   // Pool has room to allocate another object.
        PerfAggregatedStatsWithId * tuple = new ( & sPerfAggregatedStats[ sPerfAggregatedStatsCount ] ) PerfAggregatedStatsWithId() ;
        ++ sPerfAggregatedStatsCount ;

        return tuple ;
    }

    FAIL() ;

    return NULLPTR ;  // Pool ran out of objects.  Maybe increase MAX_NUM_PERF_AGG_STATS or fallback to heap allocation.
}




/** Find PerfAggregatedStats object associated with the given identifier, or allocate a new one if none such exists.

    The idea is that the first time a PerfTally is encountered with the given identifier,
    a new cross-context PerfAggregatedStatsWithId object is allocated.  Subsequently, other PerfTally
    objects with the same identifier reuse that PerfAggregatedStatsWithId object.

*/
PerfAggregatedStatsWithId * PerfAggregatedStatsWithId::FindOrNew( const PerfBlockIdentifier * id )
{
    PerfAggregatedStatsWithId * object = PerfAggregatedStatsWithId::Find( id ) ;
    if( NULLPTR == object )
    {
        object = PerfAggregatedStatsWithId::New() ;
        object->mId = id ;
    }
    return object ;
}




// -----------------------------------------------------------------------------
// PerfTally
// -----------------------------------------------------------------------------




/** Allocate and initialize a new PerfTally object.
*/
PerfTally * PerfTally::New( const char * label , const char * filename , unsigned line )
{
    // Synchronously allocate PerfTally from global pool.
    ScopedSpinLock  lock( sPerfTalliesLock ) ;

    // Allocate a new object.
    ASSERT( sPerfTallyCount < MAX_NUM_PERF_TALLIES ) ;
    if( sPerfTallyCount >= MAX_NUM_PERF_TALLIES ) { int * crash = NULLPTR ; * crash = 0xbada110c ; }

    if( sPerfTallyCount < MAX_NUM_PERF_TALLIES )
    {   // Pool has room to allocate another object.
        PerfTally * newPerfTally = new ( & sPerfTallies[ sPerfTallyCount ] ) PerfTally( label , filename , line ) ;
        ++ sPerfTallyCount ;

        return newPerfTally ;
    }

    FAIL() ;

    return NULLPTR ;  // Pool ran out of objects.  Maybe increase MAX_NUM_PERF_TALLIES or fallback to heap allocation.
}




/** Find a callee PerfTally object at the given filename and line, or add one with the given label, under this one.

    \param label    Human-readable label (string) to associate with the performance profiling block.

    \param filename Name of the source file in which the performance profiling block resides.

    \param line     Line within source file where the performance profiling block starts.
*/
PerfTally * PerfTally::FindOrAppendCallee( const char * label , const char * filename , unsigned line )
{
    PerfTally * lastCallee = NULLPTR ;
    for( PerfTally * callee = mFirstCallee ; callee != NULLPTR ; callee = callee->mNextSiblingCallee )
    {   // For each callee under this block...
        if(     ( line     == callee->mId.mLine     )
            &&  ( filename == callee->mId.mFilename ) )
        {   // Found callee matching this one.
            ASSERT( callee->mId.mLabel == label ) ; // Paranoid sanity check.  Should not have multiple labels at a single <file,line>.
            return callee ;
        }
        lastCallee = callee ;
    }

    // Failed to find matching callee.  Need to allocate a new one.
    PerfTally * newPerfTally = New( label , filename , line ) ;

    if( 0 == lastCallee )
    {   // This tally had no callees yet.
        ASSERT( NULLPTR == mFirstCallee ) ;   // Paranoid sanity check.  Can fail if race condition.
        mFirstCallee = newPerfTally ;
    }
    else
    {   // This tally had callees, just not the sought one.
        ASSERT( NULLPTR == lastCallee->mNextSiblingCallee ) ; // Paranoid sanity check.
        lastCallee->mNextSiblingCallee = newPerfTally ;
    }
    return newPerfTally ;
}




/** Find a PerfTally object with the given label, at or under this one.
*/
const PerfTally * PerfTally::Find( const char * label ) const
{
    if( /* Premature optimization for probably uncommon case: ( mId.mLabel == label ) || */ ! strcmp( mId.mLabel , label ) )
    {   // This object has sought label.
        return this ;
    }
    for( PerfTally * callee = mFirstCallee ; callee != 0 ; callee = callee->mNextSiblingCallee )
    {   // For each callee under this block...
        const PerfTally * perfTally = callee->Find( label ) ;
        if( perfTally != NULLPTR )
        {   // Found object with sought label.
            return perfTally ;
        }
    }
    return NULLPTR ; // Failed to find label.
}




/** Tally cross-context performance statistics for this block and its callees

    Aggregate duration and #calls for each unique label.
    For InContext, "total" means "across calls within a callstack".
    For CrossContext, "total" means "across calls across all callstacks".

    For example, if a routine Foo is called from Bar 5 times and Gronk 7 times, then Foo
    shows up with 2 totals, one under Bar (with 5 calls) and one under Gronk (with 7 calls).
    CrossContext aggregate indicates Foo was called 5+7 times.

*/
void PerfTally::TallyCrossContext( int frameCount )
{
    ASSERT( mCrossContextAggregate != NULLPTR ) ;

    mCrossContextAggregate->mTotalDurationWithOverhead  += mInContextAggregate.mTotalDurationWithOverhead   ;
    mCrossContextAggregate->mTotalDurationInclusive     += mInContextAggregate.mTotalDurationInclusive      ;
    mCrossContextAggregate->mMaxDurationInclusive       += mInContextAggregate.mMaxDurationInclusive        ;
    mCrossContextAggregate->mNumCalls                   += mInContextAggregate.mNumCalls                    ;

    mCrossContextAggregate->mTotalDurationExclusive     += mInContextAggregate.mTotalDurationExclusive      ;
    mCrossContextAggregate->mCalleeProfileOverheadSum   += mInContextAggregate.mCalleeProfileOverheadSum    ;
    mCrossContextAggregate->mTotalProfileOverheadSum    += mInContextAggregate.mTotalProfileOverheadSum     ;

    if( mCrossContextAggregate->mNumCalls > 0 )
    {   // Compute summary per-call statistics.
        double oneOverNumCalls = 1.0 / double( mCrossContextAggregate->mNumCalls ) ;
        mCrossContextAggregate->mAverageDurationInclusive = double( mCrossContextAggregate->mTotalDurationInclusive ) * oneOverNumCalls ;
        mCrossContextAggregate->mAverageDurationExclusive = double( mCrossContextAggregate->mTotalDurationExclusive ) * oneOverNumCalls ;
    }
    else
    {
        mCrossContextAggregate->mAverageDurationInclusive  = 0 ;
        mCrossContextAggregate->mAverageDurationExclusive  = 0 ;
    }

    ASSERT( frameCount > 0 ) ;
    double oneOverNumFrames = 1.0 / double( frameCount ) ;
    mCrossContextAggregate->mPerFrameDurationInclusive = double( mCrossContextAggregate->mTotalDurationInclusive ) * oneOverNumFrames ;
    mCrossContextAggregate->mPerFrameDurationExclusive = double( mCrossContextAggregate->mTotalDurationExclusive ) * oneOverNumFrames ;
}




/** Tally performance statistics for this block and its callees.
*/
void PerfTally::Tally( int frameCount )
{
    /** Create and assign cross-context tally objects for each PerfTally object in this call tree.
        
        Each PerfBlock can occur in the call tree multiple times, depending on from where it was called.
        Each PerfBlock also has a single cross-context tally object associated with it, which aggregates
        performance statistics across all callstack contexts.  This routine visits each PerfTally in the
        tree and either finds or (if it does not yet exist) creates the unique cross-context tally object.

    */
    if( NULLPTR == mCrossContextAggregate )
    {   //  This block does not yet know its cross-context aggregation object.
        PerfAggregatedStatsWithId * tuple   = PerfAggregatedStatsWithId::FindOrNew( & mId ) ;
        mCrossContextAggregate = & tuple->mStats ;
    }

    LONGLONG calleeDurationSum     = 0 ;
    LONGLONG calleeProfOverheadSum = 0 ;

    for( PerfTally * callee = mFirstCallee ; callee != 0 ; callee = callee->mNextSiblingCallee )
    {   // For each callee under this block...
        callee->Tally( frameCount ) ;   // Tally stats for that callee.
        calleeDurationSum     += callee->mInContextAggregate.mTotalDurationInclusive  ;
        calleeProfOverheadSum += callee->mInContextAggregate.mTotalProfileOverheadSum ;
    }

    // Compute total exclusive duration for this block.
    mInContextAggregate.mTotalDurationExclusive = mInContextAggregate.mTotalDurationInclusive - calleeDurationSum ;

    // Estimate profiling overhead duration.
    // Segregate callee and self profile overhead.
    // Inclusive durations include callee profile overhead, but omit self
    // profile overhead.  Caller needs to know total (self plus callee) overhead
    // but self needs to use callee overhead only to adjust inclusive tally.
    mInContextAggregate.mCalleeProfileOverheadSum = calleeProfOverheadSum ;
    const LONGLONG myProfOverhead = mInContextAggregate.mTotalDurationWithOverhead - mInContextAggregate.mTotalDurationInclusive ;
    mInContextAggregate.mTotalProfileOverheadSum = myProfOverhead + calleeProfOverheadSum ;

    ASSERT( mInContextAggregate.mTotalDurationInclusive   >= mInContextAggregate.mCalleeProfileOverheadSum ) ;

    if( mInContextAggregate.mNumCalls > 0 )
    {   // Compute summary per-call statistics.
        double oneOverNumCalls = 1.0 / double( mInContextAggregate.mNumCalls ) ;
        mInContextAggregate.mAverageDurationInclusive = double( mInContextAggregate.mTotalDurationInclusive ) * oneOverNumCalls ;
        mInContextAggregate.mAverageDurationExclusive = double( mInContextAggregate.mTotalDurationExclusive ) * oneOverNumCalls ;
    }
    else
    {
        mInContextAggregate.mAverageDurationInclusive = 0;
        mInContextAggregate.mAverageDurationExclusive = 0;
    }

    ASSERT( frameCount > 0 ) ;
    double oneOverNumFrames = 1.0 / double( frameCount ) ;
    mInContextAggregate.mPerFrameDurationInclusive = double( mInContextAggregate.mTotalDurationInclusive ) * oneOverNumFrames ;
    mInContextAggregate.mPerFrameDurationExclusive = double( mInContextAggregate.mTotalDurationExclusive ) * oneOverNumFrames ;

    // Tally cross-context performance stats.  This must be done AFTER tallying in-context quantities (above).
    TallyCrossContext( frameCount ) ;
}




/** Reset performance tally counters, to facilitate restarting another performance capture interval.
*/
void PerfTally::Reset()
{
    for( PerfTally * callee = mFirstCallee ; callee != 0 ; callee = callee->mNextSiblingCallee )
    {   // For each callee under this block...
        callee->Reset() ;
    }

    mInContextAggregate.Reset() ;
}




/** Get number of seconds per tick.

    Tick is the unit of time reported by performance the counter.
*/
double PerfTally::SecondsPerTick()
{
    LARGE_INTEGER perfCtrTickPerSec ;
    QueryPerformanceFrequency( & perfCtrTickPerSec ) ;
    const double secondsPerTick = 1.0 / double( perfCtrTickPerSec.QuadPart ) ;
    return secondsPerTick ;
}




/** Get number of milliseconds per tick.

    \see SecondsPerTick
*/
double PerfTally::MilliSecondsPerTick()
{
    static const double millisecondsPerSecond = 1000.0 ;
    return millisecondsPerSecond * SecondsPerTick() ;
}




/** Log header (title text) for performance data.
*/
void PerfTally::LogHeader( PerfLogFunc logFunc )
{
    static const char * header = "IndentedLabel,Filename,Line,Label,Depth,"
                                   "InContext TotDurIncl,AvgDurInc,TotDurExcl,AvgDurExc,MaxDurInc,ProfOvrheadTot,ProfOvrheadCallees,NumCalls,"
                                "CrossContext TotDurIncl,AvgDurInc,TotDurExcl,AvgDurExc,MaxDurInc,ProfOvrheadTot,ProfOvrheadCallees,NumCalls,"
                                "\n" ;
    if( logFunc )
    {   // Caller passed in a log function so use it.
        logFunc( header ) ;
    }
    else
    {   // Caller did not pass in a log function so print to a default stream.
        PERF_PRINTF( header ) ;
        fflush( stdout ) ;
        fflush( stderr ) ;
    }
}




/** Log performance statistics for this block and its callees.

    Note that times reported here are in milliseconds, whereas durations recorded in variables are in seconds.

*/
void PerfTally::Log( const int indent , PerfLogFunc logFunc )
{
    if( 0 == indent )
    {   // This is the first call to Log so log header info.
        LogHeader( logFunc ) ;
    }

    const double avgCalleeOverhead_inCtx = mInContextAggregate.mNumCalls > 0 ? double( mInContextAggregate.mCalleeProfileOverheadSum ) / double( mInContextAggregate.mNumCalls ) : 0 ;

    ASSERT( ( 0 == mInContextAggregate.mNumCalls ) || ( mInContextAggregate.mAverageDurationInclusive >= avgCalleeOverhead_inCtx ) ) ;

    const double milliSecondsPerTick = MilliSecondsPerTick() ;

    const double totDurIn_inCtx = double( mInContextAggregate.mTotalDurationInclusive   - mInContextAggregate.mCalleeProfileOverheadSum ) * milliSecondsPerTick ;
    const double avgDurIn_inCtx = double( mInContextAggregate.mAverageDurationInclusive - avgCalleeOverhead_inCtx                       ) * milliSecondsPerTick ;
    const double totDurEx_inCtx = double( mInContextAggregate.mTotalDurationExclusive                                                   ) * milliSecondsPerTick ;
    const double avgDurEx_inCtx = double( mInContextAggregate.mAverageDurationExclusive                                                 ) * milliSecondsPerTick ;
    const double maxDurIn_inCtx = double( mInContextAggregate.mMaxDurationInclusive     - avgCalleeOverhead_inCtx                       ) * milliSecondsPerTick ;
    const double profOhTt_inCtx = double( mInContextAggregate.mTotalProfileOverheadSum                                                  ) * milliSecondsPerTick ;
    const double profOhCl_inCtx = double( mInContextAggregate.mCalleeProfileOverheadSum                                                 ) * milliSecondsPerTick ;

    const double avgCalleeOverhead_crCtx = mCrossContextAggregate->mNumCalls > 0 ? double( mCrossContextAggregate->mCalleeProfileOverheadSum ) / double( mCrossContextAggregate->mNumCalls ) : 0 ;

    ASSERT( ( 0 == mCrossContextAggregate->mNumCalls ) || ( mCrossContextAggregate->mAverageDurationInclusive >= avgCalleeOverhead_crCtx ) ) ;

    const double totDurIn_crCtx = double( mCrossContextAggregate->mTotalDurationInclusive   - mCrossContextAggregate->mCalleeProfileOverheadSum ) * milliSecondsPerTick ;
    const double avgDurIn_crCtx = double( mCrossContextAggregate->mAverageDurationInclusive - avgCalleeOverhead_crCtx                           ) * milliSecondsPerTick ;
    const double totDurEx_crCtx = double( mCrossContextAggregate->mTotalDurationExclusive                                                       ) * milliSecondsPerTick ;
    const double avgDurEx_crCtx = double( mCrossContextAggregate->mAverageDurationExclusive                                                     ) * milliSecondsPerTick ;
    const double maxDurIn_crCtx = double( mCrossContextAggregate->mMaxDurationInclusive     - avgCalleeOverhead_crCtx                           ) * milliSecondsPerTick ;
    const double profOhTt_crCtx = double( mCrossContextAggregate->mTotalProfileOverheadSum                                                      ) * milliSecondsPerTick ;
    const double profOhCl_crCtx = double( mCrossContextAggregate->mCalleeProfileOverheadSum                                                     ) * milliSecondsPerTick ;

    // Use comma-separated values, for easy import into a spreadsheet program.
    const char * formatStr = /* indent-label */ "%s%s," /* filename , line */ "%s,%i," /* label , depth */ "%s,%i,"
                            /*    in-context stats */ /* inclusive tot,avg */ "%g,%g," /* exclusive tot,avg */ "%g,%g," /* max */ "%g," /* overhead tot, callee */ "%g,%g," /* #calls */ "%u,"
                            /* cross-context stats */ /* inclusive tot,avg */ "%g,%g," /* exclusive tot,avg */ "%g,%g," /* max */ "%g," /* overhead tot, callee */ "%g,%g," /* #calls */ "%u,"
                            "\n" ;

    // Find name part of full filename
    const char * filename = mId.mFilename;
    while (*filename) ++filename;
    while (filename != mId.mFilename && *(filename-1) != '\\') --filename;

    char logLine[ 512 ] ;
    sprintf( logLine , formatStr
        // Identifier
        , Indent( indent * 4 ) , mId.mLabel
        , filename , mId.mLine
        , mId.mLabel
        , indent

        // In-context stats
        , totDurIn_inCtx , avgDurIn_inCtx
        , totDurEx_inCtx , avgDurEx_inCtx
        , maxDurIn_inCtx
        , profOhTt_inCtx , profOhCl_inCtx
        , mInContextAggregate.mNumCalls

        // Cross-context stats
        , totDurIn_crCtx , avgDurIn_crCtx
        , totDurEx_crCtx , avgDurEx_crCtx
        , maxDurIn_crCtx
        , profOhTt_crCtx , profOhCl_crCtx
        , mCrossContextAggregate->mNumCalls

        ) ;

    if( logFunc != NULL )
    {   // Caller passed in a log function so use it.
        logFunc( logLine ) ;
    }
    else
    {   // Caller did not pass in a log function so print to a default stream.
        PERF_PRINTF( logLine ) ;
        fflush( stdout ) ;
        fflush( stderr ) ;
    }

    for( PerfTally * callee = mFirstCallee ; callee != 0 ; callee = callee->mNextSiblingCallee )
    {   // For each callee under this block...
        callee->Log( indent + 1, logFunc ) ;   // Tally stats for that callee.
    }
}
