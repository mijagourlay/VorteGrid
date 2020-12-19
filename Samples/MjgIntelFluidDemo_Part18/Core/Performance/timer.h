/** \file timer.h

    \brief Timer utility routines

    \author Copyright 2010 MJG; All rights reserved.
*/
#ifndef TIMER_H
#define TIMER_H

#if defined( _XBOX )
    #include <xtl.h>
#elif defined( WIN32 )
    #include <windows.h>
#else
    #error "unknown platform"
#endif

// Macros ----------------------------------------------------------------------


// Types -----------------------------------------------------------------------

/** Timer block.

    Create an object of this class at the beginning of any code to time, in
    its own scope.  When the scope ends, the block timing ends automatically.

*/
class Timer
{
    public:
        /** Initialize a timer.
        */
        Timer()
        {
            QueryPerformanceFrequency( & mQwTicksPerSec ) ;
            mSecondsPerTick = 1.0f / float( mQwTicksPerSec.QuadPart ) ;
            StartTimer() ;
        }


        ~Timer() {}


        /** Reset timer, that is, start it at zero.
        */
        void StartTimer()
        {
            QueryPerformanceCounter( & mTimeEnter ) ;
        }


        /** Get duration elapsed since timer started.
        */
        float GetElapsedTimeSeconds()
        {
            LARGE_INTEGER iTimeExit ;
            QueryPerformanceCounter( & iTimeExit ) ;
            LARGE_INTEGER iDuration ;
            iDuration.QuadPart = iTimeExit.QuadPart - mTimeEnter.QuadPart ;
            const float fDuration = mSecondsPerTick * static_cast< float >( iDuration.QuadPart ) ;
            return fDuration ;
        }


        /** Busy-wait for a given amount of time.
        */
        static void BusySleep( float sleepDurationSeconds )
        {
            Timer timer ;
            for(;;)
            {   // Loop unconditionally.
                const float elapsedTimeSeconds = timer.GetElapsedTimeSeconds() ;
                if( elapsedTimeSeconds >= sleepDurationSeconds )
                {   // Sleep time has elapsed.
                    break ;
                }
            }
        }


    private:
        LARGE_INTEGER   mQwTicksPerSec  ;   ///< Performance counter property; number of ticks per second.
        float           mSecondsPerTick ;   ///< Floating-point reciprocal of ticks-per-second
        LARGE_INTEGER   mTimeEnter      ;   ///< Performance counter value when timer started.
} ;

// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

#endif
