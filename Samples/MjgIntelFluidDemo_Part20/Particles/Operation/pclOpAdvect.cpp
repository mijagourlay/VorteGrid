/** \file pclOpAdvect.cpp

    \brief Operation to assign particle velocity according to a velocity field.

    \see http://www.mijagourlay.com/

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "Particles/Operation/pclOpAdvect.h"

#include "Particles/particle.h"

#include "Core/SpatialPartition/uniformGridMath.h"

#include "Core/Performance/perfBlock.h"

#include <stdlib.h>





/** Assign velocity to a subset of given particles using given velocity field.

    \param particles Dynamic array of particles whose velocity to update.

    \param velocityGrid Uniform grid of velocity values.

        Particle velocity is updated with velocity interpolating from grid.

    \param gain Fraction of velocity from grid to use.
        Keep (1-gain) of original velocity.  Gain should be in [0,1).

    \param itStart Index of first particle to update.

    \param itEnd One past index of last particle to update.

    \see    Changex87FloatingPointToTruncate, Interpolate_AssumesFpcwSetToTruncate,
            IndicesOfPosition_AssumesFpcwSetToTruncate, StoreFloatAsInt.
*/
static void AssignParticleVelocityFromField_Slice( VECTOR< Particle > & particles , const UniformGrid< Vec3 > * velocityGrid , float gain , size_t itStart , size_t itEnd )
{
    ASSERT( itEnd <= particles.Size() ) ;
    ASSERT( velocityGrid && ! velocityGrid->HasZeroExtent() ) ;

    // Change floating-point control word to "truncate" for float-to-int conversion.
    // The default is "round to nearest", which is not what C++ normally does.
    // The compiler would normally emit a fldcw instruction per float-to-int operation,
    // which causes a pipeline flush and is insanely expensive.
    // This routine therefore uses a special version of Interpolate
    // which assumes the FPCW is set to "truncate" and uses inline assembly
    // to bypass the compiler emitting fldcw instructions.
    // See StoreFloatAsInt.
    const WORD OldCtrlWord = Changex87FloatingPointToTruncate() ;

    if( 1.0f == gain )
    {   // Gain is exactly 1 so ignore old velocity and use velocity from grid entirely.
        for( size_t offset = itStart ; offset < itEnd ; ++ offset )
        {   // For each particle in this slice...
            Particle & pcl = particles[ offset ] ;
            Vec3 velocityFromGrid ;
            velocityGrid->Interpolate_AssumesFpcwSetToTruncate( velocityFromGrid , pcl.mPosition ) ;
            pcl.mVelocity = velocityFromGrid ;
        }
    }
    else
    {
        for( size_t offset = itStart ; offset < itEnd ; ++ offset )
        {   // For each particle in this slice...
            Particle & pcl = particles[ offset ] ;
            Vec3 velocity ;
            velocityGrid->Interpolate_AssumesFpcwSetToTruncate( velocity , pcl.mPosition ) ;
            Vec3 velocityDelta = velocity - pcl.mVelocity ;
            pcl.mVelocity += gain * velocityDelta ;
        }
    }

    // Restore FPCW.  See comment above.
    SetFloatingPointControlWord( OldCtrlWord ) ;
}




#if USE_TBB
/** Functor (function object) to update particle velocities from a field, using Threading Building Blocks.
*/
class Particles_AssignVelocityFromField_TBB
{
        VECTOR< Particle >  &       mParticles    ;   ///< Array of particles whose velocities to update.
        const UniformGrid< Vec3 > * mVelocityGrid ;   ///< Grid of velocity values.
        const float                 mGain         ;   ///< Amount of velocity from grid to assign to particles, each frame.
    public:
        void operator() ( const tbb::blocked_range<size_t> & r ) const
        {   // Assign velocity from field for subset of particles.
            SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
            SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
            AssignParticleVelocityFromField_Slice( mParticles , mVelocityGrid , mGain , r.begin() , r.end() ) ;
        }
        Particles_AssignVelocityFromField_TBB( VECTOR< Particle > & particles , const UniformGrid< Vec3 > * velocityGrid , const float gain )
            : mParticles( particles )
            , mVelocityGrid( velocityGrid )
            , mGain( gain )
        {
            mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
            mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
        }
    private:
        WORD        mMasterThreadFloatingPointControlWord   ;
        unsigned    mMasterThreadMmxControlStatusRegister   ;
} ;
#endif




/** Assign particle velocity from to a velocity field.

    \param particles Dynamic array of particles whose velocity to update.

    \param velocityGrid Uniform grid of velocity values.

        Particle velocity is updated with velocity interpolating from grid.

    \param gain Fraction of velocity from grid to use.
        Keep 1-gain of original velocity.

    \see ComputeVelocityFromVorticity, AssignParticleVelocityFromField_Slice.

*/
void AssignVelocityFromField( VECTOR< Particle > & particles , const UniformGrid< Vec3 > * velocityGrid , const float gain )
{
    PERF_BLOCK( AssignVelocityFromField ) ;

    const size_t numParticles = particles.Size() ;

    if( velocityGrid && velocityGrid->HasZeroExtent() )
    {   // Caller provided a velocity grid with no data.
        // Leave particle velocity as-is.
        return ;
    }

#if USE_TBB
    // Estimate grain size based on size of problem and number of processors.
    const size_t grainSize =  Max2( size_t( 1 ) , numParticles / gNumberOfProcessors ) ;
    // Using multiple threads, update particle velocity from field.
    parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , Particles_AssignVelocityFromField_TBB( particles , velocityGrid , gain ) ) ;
#else
    AssignParticleVelocityFromField_Slice( particles , velocityGrid , gain , 0 , numParticles ) ;
#endif
}




void PclOpAssignVelocityFromField::Operate( VECTOR< Particle > & particles , float /* timeStep */, unsigned /* uFrame */ )
{
    if( mVelocityGrid )
    {
        AssignVelocityFromField( particles , mVelocityGrid , mGridWeight ) ;
    }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/** \file pclOpAccelerate.cpp

    \brief Operation to apply a uniform acceleration to particles.

    \see http://www.mijagourlay.com/

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "Particles/Operation/pclOpAdvect.h"

#include "Particles/particle.h"

#include "Core/SpatialPartition/uniformGridMath.h"

#include "Core/Performance/perfBlock.h"

#include <stdlib.h>





/** Accelerate a subset of given particles.

    \param particles Dynamic array of particles whose velocity to update.

    \param acceleration Uniform acceleration to apply to each particle.

    \param itStart Index of first particle to update.

    \param itEnd One past index of last particle to update.

    \see    Changex87FloatingPointToTruncate, Interpolate_AssumesFpcwSetToTruncate,
            IndicesOfPosition_AssumesFpcwSetToTruncate, StoreFloatAsInt.
*/
static void AccelerateParticles_Slice( VECTOR< Particle > & particles , const Vec3 & acceleration , const float timeStep , size_t itStart , size_t itEnd )
{
    ASSERT( itEnd <= particles.Size() ) ;

    ASSERT( ! IsNan( acceleration ) ) ;
    ASSERT( ! IsInf( acceleration ) ) ;

    ASSERT( ! IsNan( timeStep ) ) ;
    ASSERT( ! IsInf( timeStep ) ) ;

    const Vec3 deltaVelocity = acceleration * timeStep ;

    for( size_t offset = itStart ; offset < itEnd ; ++ offset )
    {   // For each particle in this slice...
        Particle & pcl = particles[ offset ] ;
        pcl.mVelocity += deltaVelocity ;
    }
}




#if USE_TBB
/** Functor (function object) to accelerate particles, using Threading Building Blocks.
*/
class Particles_Accelerate_TBB
{
        VECTOR< Particle >  &   mParticles      ;   ///< Array of particles whose velocities to update.
        const Vec3 &            mAcceleration   ;   ///< Uniform acceleration to apply to each particle.
        const float             mTimeStep       ;   ///< Amount of time passing in this update.
    public:
        void operator() ( const tbb::blocked_range<size_t> & r ) const
        {   // Accelerate subset of particles.
            SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
            SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
            AccelerateParticles_Slice( mParticles , mAcceleration , mTimeStep , r.begin() , r.end() ) ;
        }
        Particles_Accelerate_TBB( VECTOR< Particle > & particles , const Vec3 & acceleration , const float timeStep )
            : mParticles( particles )
            , mAcceleration( acceleration )
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




/** Accelerate particles.

    \param particles Dynamic array of particles whose velocity to update.

    \param acceleration Uniform acceleration to apply to each particle.

    \param timeStep Amount of time passing in this update.
*/
void AccelerateParticles( VECTOR< Particle > & particles , const Vec3 & acceleration , float timeStep )
{
    PERF_BLOCK( AccelerateParticles ) ;

    const size_t numParticles = particles.Size() ;

#if USE_TBB
    // Estimate grain size based on size of problem and number of processors.
    const size_t grainSize =  Max2( size_t( 1 ) , numParticles / gNumberOfProcessors ) ;
    // Using multiple threads, update particle velocity from field.
    parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , Particles_Accelerate_TBB( particles , acceleration , timeStep ) ) ;
#else
    AccelerateParticles_Slice( particles , acceleration , timeStep , 0 , numParticles ) ;
#endif
}




void PclOpAccelerate::Operate( VECTOR< Particle > & particles , float timeStep , unsigned /* uFrame */ )
{
    AccelerateParticles( particles , mAcceleration , timeStep ) ;
}
