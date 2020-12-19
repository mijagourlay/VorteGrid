/** \file perfTally.h

    \brief Tally of runtime performance measurements for a block being profiled.

    \author Written and copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PERF_TALLY_H
#define PERF_TALLY_H

#if defined( _XBOX )
    #include <xtl.h>        // For LONGLONG
#elif defined( WIN32 )
    #include <windows.h>    // For LONGLONG
    #ifdef min
        #undef min
    #endif
    #ifdef max
        #undef max
    #endif
#else
    #error "unknown platform"
#endif

#ifndef NULLPTR
#   define NULLPTR 0
#endif

// Macros ----------------------------------------------------------------------

#if defined( WIN32 )
    #define THREAD_LOCAL_STORAGE static _declspec( thread )
#else   // Fallback -- not really thread-local but it will compile and even run okay in single-threaded code.
    #define THREAD_LOCAL_STORAGE static volatile
#endif

// Types -----------------------------------------------------------------------

#if _MSC_VER && ( _MSC_VER < 1700 )    // Using older compiler that lacks std::atomic.
#   pragma message( "WARNING: Using non-thread-safe volatile for atomic bool" )
    class AtomicBool
    {   // Fake atomic bool for legacy compilers that lack std::atomic.  This is NOT thread safe but it will compile and run okay for single-threaded applications.
    public:
        AtomicBool( bool initialValue = false ) : mValue( initialValue ) {}
        bool exchange( bool newValue )
        {
            const bool oldValue = mValue ;
            mValue = newValue ;
            return oldValue ;
        }
    private:
        volatile bool mValue ;
    } ;
#else   // Presumed to be using modern compiler that has std::atomic.
    #include <atomic>
    typedef std::atomic<bool>   AtomicBool  ;
#endif




// Simple scoped spin lock based in atomic bool, for synchronizing on any architecture with atomic instructions, even if the OS does not provide locks.
class ScopedSpinLock
{
public:
    // Creating this object (e.g. on the stack as an automatic variable) obtains the lock.
    ScopedSpinLock( AtomicBool & lock )
        : _lock( lock )
    {
        while( _lock.exchange( true ) ) { } // Obtain lock.
    }

    // Destructing this object (e.g. leaving scope) releases the lock.
    ~ScopedSpinLock()
    {
        _lock = false ; // Release lock.
    }

private:
    ScopedSpinLock() ;                                      // Disallow default construction
    ScopedSpinLock( const ScopedSpinLock & ) ;              // Disallow copy
    ScopedSpinLock operator=( const ScopedSpinLock & ) ;    // Disallow assignment

    AtomicBool & _lock ;
} ;




typedef void (*PerfLogFunc)( const char * strFormat , ... ) ;




/** Unique identifier for a PerfBlock.
*/
struct PerfBlockIdentifier
{
    PerfBlockIdentifier()
        : mLabel( 0 )
        , mFilename( 0 )
        , mLine( 0 )
    {}

    PerfBlockIdentifier( const char * label , const char * filename , unsigned line )
        : mLabel( label )
        , mFilename( filename )
        , mLine( line )
    {}

    const char *    mLabel      ;   ///< Human-readable label for this block.
    const char *    mFilename   ;   ///< Module filename.
    unsigned        mLine       ;   ///< Line within file of this block.
} ;




/** Aggregated performance statistics.

    These are computed after accumulating raw performance measurements.

    These are computed for each label in two ways: per "context", i.e. across all calls within a single callstack,
    and "total", i.e. across all calls across all callstacks.

    So, for example, if a routine Foo is called from Bar 5 times and Gronk 7 times, then Foo shows up with 2 totals, one under Bar (with 5 calls) and one under Gronk (with 7 calls).
    The "total" indicates Foo was called 5+7 times.

*/
struct PerfAggregatedStats
{
    PerfAggregatedStats()
        : mTotalDurationWithOverhead( 0 )
        , mTotalDurationInclusive( 0 )
        , mMaxDurationInclusive( 0 )
        , mNumCalls( 0 )

        , mTotalDurationExclusive( 0 )
        , mCalleeProfileOverheadSum( 0 )
        , mTotalProfileOverheadSum( 0 )
        , mAverageDurationInclusive( 0.0 )
        , mAverageDurationExclusive( 0.0 )
        , mPerFrameDurationInclusive( 0.0 )
        , mPerFrameDurationExclusive( 0.0 )
    {}

    void Reset()
    {
        mTotalDurationWithOverhead = 0;
        mTotalDurationInclusive = 0;
        //mMaxDurationInclusive = 0 ;   // Do not reset max; this should record the max across all intervals.
        mNumCalls = 0;

        mTotalDurationExclusive = 0;
        mCalleeProfileOverheadSum = 0;
        mTotalProfileOverheadSum = 0;

        mAverageDurationInclusive = 0;
        mAverageDurationExclusive = 0;
        mPerFrameDurationInclusive = 0;
        mPerFrameDurationExclusive = 0;
    }

    // Primary tally quantities.  These are initially accumulated per call.
    // These are also used in the total cross-callstack aggregate.
    LONGLONG        mTotalDurationWithOverhead  ;   ///< Total run duration, in ticks, including profiling overhead.
    LONGLONG        mTotalDurationInclusive     ;   ///< Total run duration, in ticks, including all callee durations.
    LONGLONG        mMaxDurationInclusive       ;   ///< Longest run duration, in ticks, including all callee durations.
    unsigned        mNumCalls                   ;   ///< Number of calls of this block, within a single callstack context.

    // Quantities derived from primary tallies:
    LONGLONG        mTotalDurationExclusive     ;   ///< Total run duration, in ticks, excluding all callee durations.
    LONGLONG        mCalleeProfileOverheadSum   ;   ///< Profiling overhead, in ticks, of all callee durations.
    LONGLONG        mTotalProfileOverheadSum    ;   ///< Profiling overhead, in ticks, including self and all callees.

    // Summary statistical quantities:
    double          mAverageDurationInclusive   ;   ///< Average run duration per call including all callee durations, in milliseconds.
    double          mAverageDurationExclusive   ;   ///< Average run duration per call excluding all callee durations, in milliseconds.

    double          mPerFrameDurationInclusive  ;   ///< Average run duration per frame including all callee durations, in milliseconds.
    double          mPerFrameDurationExclusive  ;   ///< Average run duration per frame excluding all callee durations, in milliseconds.
} ;




/** Key,Value pair <Identifier,Stats> to make a dictionary of PerfAggregatedStats objects.
*/
struct PerfAggregatedStatsWithId
{
    static PerfAggregatedStatsWithId *  Find( const PerfBlockIdentifier * id ) ;
    static PerfAggregatedStatsWithId *  New() ;
    static PerfAggregatedStatsWithId *  FindOrNew( const PerfBlockIdentifier * id ) ;

    const PerfBlockIdentifier * mId     ;
    PerfAggregatedStats         mStats  ;
} ;




/** Persistent tally of performance information for each block.

    Each code block encompassed by a PerfBlock object (which is ephemeral) has
    at least one corresponding PerfTally object (which is persistent) which
    tallies all durations for that block.

    Furthermore, each different call path results in a different PerfTally object.
    For example if a block Foo is called from both Bar and Gronk, then Foo will
    have two PerfTally objects -- one under Bar and one under Gronk.

    Also, each thread will have a different tree of PerfTally objects.
    So if, for example, Foo gets called from threads A and B then Foo will have
    PerfTally objects under each of those trees.
*/
struct PerfTally
{
    PerfTally()
        : mFirstCallee( 0 )
        , mNextSiblingCallee( NULLPTR )
    {}

    PerfTally( const char * label , const char * filename , unsigned line )
        : mId( label , filename , line )
        , mFirstCallee( 0 )
        , mNextSiblingCallee( 0 )
        , mCrossContextAggregate( NULLPTR )
    {}

    static double       SecondsPerTick() ;
    static double       MilliSecondsPerTick() ;
    static PerfTally *  New( const char * label , const char * filename , unsigned line ) ;

    PerfTally *         FindOrAppendCallee( const char * label , const char * filename , unsigned line ) ;
    const PerfTally *   Find( const char * label ) const ;
    void                Tally( int frameCount ) ;
    void                Reset() ;
    void                Log( int indent , PerfLogFunc logFunc ) ;
    void                LogHeader( PerfLogFunc logFunc ) ;

    PerfBlockIdentifier     mId                         ;   ///< Block identifier.  Set once, upon first call to this block.
    PerfAggregatedStats     mInContextAggregate         ;   ///< Aggregated performance stats across all calls within a single callstack context.

    // Members used to track callees.  Set at most once, upon first call to each callee.
    PerfTally   *           mFirstCallee                ;   ///< First immediate callee block under this one.
    PerfTally   *           mNextSiblingCallee          ;   ///< Next sibling callee after this one.

    PerfAggregatedStats *   mCrossContextAggregate      ;   ///< Aggregated performance stats across all calls across all callstack contexts.

private:
    void                TallyCrossContext( int frameCount ) ;
} ;

// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

#endif
