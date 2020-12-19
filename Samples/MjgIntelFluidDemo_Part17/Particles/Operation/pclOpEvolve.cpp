/** \file pclOpEvolve.cpp

    \brief Operation to evolve particle states -- position from velocity, orientation from angular velocity.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdlib.h>

#include "Core/SpatialPartition/uniformGridMath.h"

#include "Core/Performance/perf.h"

#include "../particle.h"

#include "pclOpEvolve.h"




/** Evolve (subset of) given particles.

    \param particles - dynamic array of particles to evolve

    \param timeStep     Amount of virtual time by which to advance simulation.

    \param itStart - index of first particle to evolve

    \param itEnd - index of last particle to evolve

    \see    Changex87FloatingPointToTruncate, Interpolate_AssumesFpcwSetToTruncate,
            IndicesOfPosition_AssumesFpcwSetToTruncate, StoreFloatAsInt.
*/
static void EvolveParticlesSlice( VECTOR< Particle > & particles , const float & timeStep , size_t itStart , size_t itEnd )
{
    ASSERT( itEnd <= particles.Size() ) ;

    //static bool evolve = true ;
    //if( evolve )
    {
        for( size_t offset = itStart ; offset < itEnd ; ++ offset )
        {   // For each particle in this slice...
            Particle & pcl = particles[ offset ] ;
            pcl.mPosition += pcl.mVelocity * timeStep ;
            //pcl.mOrientation += pcl.mAngularVelocity * timeStep ;
        #if ENABLE_PARTICLE_POSITION_HISTORY
            pcl.RecordPositionHistory() ;
        #endif
        }
    }
}




#if USE_TBB
/** Functor (function object) to evolve particles using Threading Building Blocks.
*/
class Particles_Evolve_TBB
{
        VECTOR< Particle >  &   mParticles    ;   ///< Array of particles to evolve
        const float         &   mTimeStep     ;   ///< Duration of this time step
    public:
        void operator() ( const tbb::blocked_range<size_t> & r ) const
        {   // Evolve subset of particles.
            SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
            SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
            EvolveParticlesSlice( mParticles , mTimeStep , r.begin() , r.end() ) ;
        }
        Particles_Evolve_TBB( VECTOR< Particle > & particles , const float & timeStep )
            : mParticles( particles )
            , mTimeStep( timeStep )
        {
            mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
            mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
        }
    private:
        WORD        mMasterThreadFloatingPointControlWord   ;
        unsigned    mMasterThreadMmxControlStatusRegister   ;
} ;
#endif




/** Evolve particle states.

    \param particles    Dynamic array of particles to evolve.

    \param timeStep     Amount of virtual time by which to advance simulation.

    \param uFrame       Frame counter.

    \see ComputeVelocityFromVorticity, EvolveParticlesSlice.

*/
static void Evolve( VECTOR< Particle > & particles , const float & timeStep )
{
    const size_t numParticles = particles.Size() ;

#if USE_TBB
    // Estimate grain size based on size of problem and number of processors.
    const size_t grainSize =  Max2( size_t( 1 ) , numParticles / gNumberOfProcessors ) ;
    // Evolve particles using multiple threads.
    parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , Particles_Evolve_TBB( particles , timeStep ) ) ;
#else
    EvolveParticlesSlice( particles , timeStep , 0 , numParticles ) ;
#endif
}




void PclOpEvolve::Operate(  VECTOR< Particle > & particles , float timeStep , unsigned /* uFrame */ )
{
    QUERY_PERFORMANCE_ENTER ;
    Evolve( particles , timeStep ) ;
    QUERY_PERFORMANCE_EXIT( PclOpEvolve_Operate ) ;
}
