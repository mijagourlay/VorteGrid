/** \file pclOpAssignScalarFromGrid.cpp

    \brief Operation to assign scalar member of particles from a grid.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdlib.h>

#include "Core/Performance/perf.h"
#include "Core/SpatialPartition/uniformGridMath.h"
#include "../particle.h"
#include "pclOpAssignScalarFromGrid.h"

/** Assign particle density based on density grid.

    \param particles - dynamic array of particles whose density to assign

    \param memberOffsetInBytes   Offset, in bytes, from head of particle object, to assign.

    \param scalarGrid - uniform grid of density values.

    \param iPclStart - index of first particle to process

    \param iPclEnd - index of last particle to process

    \note This routine assumes PopulateDensityAndMassFractionGrids has already run for this timestep.

    \note This routine should run before advecting tracers, otherwise the tracers
            could leave the velocity grid boundaries, and this routine needs to interpolate
            within those boundaries.

    \see PopulateDensityAndMassFractionGrids, GenerateBaroclinicVorticity
*/
static void AssignScalarFromGridSlice( VECTOR< Particle > & particles , size_t memberOffsetInBytes , const UniformGrid< float > & scalarGrid , size_t iPclStart , size_t iPclEnd )
{
    for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
    {   // For each particle in this slice...
        Particle &  rParticle = particles[ iPcl ] ;
        float value ;
        scalarGrid.Interpolate( value , rParticle.mPosition ) ;
        *(float*)(((char*)&rParticle) + memberOffsetInBytes) = value ;
        ASSERT( rParticle.mDensity > 0.0f ) ;
    }
}




#if USE_TBB
    /** Functor (function object) to assign scalar from particles from a grid.
    */
    class Particles_AssignScalarFromGrid_TBB
    {
            VECTOR< Particle >  &           mParticles              ;   ///< Array of particles whose scalar member to assign
            size_t                          mMemberOffsetInBytes    ;   ///< Offset, in bytes, of particle member to assign
            const UniformGrid< float > &    mScalarGrid             ;   ///< Grid of scalar information
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Assign scalar from grid to a subset of particles.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                AssignScalarFromGridSlice( mParticles , mMemberOffsetInBytes , mScalarGrid , r.begin() , r.end() ) ;
            }
            Particles_AssignScalarFromGrid_TBB( VECTOR< Particle > & particles , size_t memberOffsetInBytes , const UniformGrid< float > & scalarGrid )
                : mParticles( particles )
                , mMemberOffsetInBytes( memberOffsetInBytes )
                , mScalarGrid( scalarGrid )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;
#endif




/** Assign scalar field of a particle based on value from grid.

    \note
    This routine effectively treats the quantity being transferred from grid to particles
    as a "distribution" of that quantity, in other words, that the quantity does not
    "accumulate".

    For example, if a grid cell contains velocity values, then all particles inside
    that cell adopt that velocity.  That is what this routine does.

    But if a cell contains mass values, then if multiple particles reside inside that
    cell, they should (theoretically) divide that mass amongst them.  This routine
    does NOT handle that properly.

    If the caller wants to assign accumulating quantities (like mass) to particles,
    then it must divide the per-particle quantities assigned here by the number of
    particles receiving that quantity.

    \note This routine should run before advecting tracers, otherwise the tracers
            could leave the velocity grid boundaries, and this routine needs to interpolate
            within those boundaries.

    \see PopulateDensityAndMassFractionGrids, GenerateBaroclinicVorticity, AssignScalarFromGridSlice
*/
static void AssignScalarFromGrid( VECTOR< Particle > & particles , size_t memberOffsetInBytes , const UniformGrid< float > & scalarGrid )
{
#if ENABLE_FLUID_BODY_SIMULATION
    if( scalarGrid.HasZeroExtent() )
    {   // Scalar grid is empty.  Probably first iteration.
        return ; // Nothing to do.
    }
    const size_t    numParticles = particles.Size() ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , numParticles / gNumberOfProcessors ) ;
        // Assign scalar from grid using threading building blocks.
        parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , Particles_AssignScalarFromGrid_TBB( particles , memberOffsetInBytes , scalarGrid ) ) ;
    #else
        AssignScalarFromGridSlice( particles , memberOffsetInBytes , scalarGrid , 0 , numParticles ) ;
    #endif
#endif
}




void PclOpAssignScalarFromGrid::CountParticlesPerCell( const VECTOR< Particle > & particles , UniformGrid< unsigned > & ugParticleCount )
{
    ugParticleCount.Init( 0 ) ;
    const size_t numParticles = particles.Size() ;
    for( unsigned offset = 0 ; offset < numParticles ; ++ offset )
    {   // For each particle...
        const Particle & rParticle = particles[ offset ] ;
        // Tally the number of particles in its cell.
        ++ ugParticleCount[ rParticle.mPosition ] ;
    }
}




void PclOpAssignScalarFromGrid::DivideByNumParticlesPerCell( VECTOR< Particle > & particles , const UniformGrid< unsigned > & ugParticleCount )
{
    const size_t numParticles = particles.Size() ;
    for( unsigned offset = 0 ; offset < numParticles ; ++ offset )
    {   // For each particle...
        Particle &  rParticle           = particles[ offset ] ;
        float &     value               = *(float*) ( ( (char*) & rParticle ) + mMemberOffsetInBytes ) ;
        ASSERT( ugParticleCount[ rParticle.mPosition ] > 0 ) ;
        const float numParticlesPerCell = static_cast< float >( ugParticleCount[ rParticle.mPosition ] ) ;

        // Divide value by number of particles in the same cell as this one.
        value /= float( numParticlesPerCell ) ;
    }
}




void PclOpAssignScalarFromGrid::Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ )
{
    QUERY_PERFORMANCE_ENTER ;

    if( ( 0 == mScalarGrid ) || ( mScalarGrid->HasZeroExtent() ) )
    {   // Scalar grid is empty.  Probably first iteration.
        return ; // Nothing to do.
    }

    AssignScalarFromGrid( particles , mMemberOffsetInBytes , * mScalarGrid ) ;

    if( mDivideByParticleCount )
    {
        UniformGrid< unsigned > ugParticleCount( * mScalarGrid ) ;
        CountParticlesPerCell( particles , ugParticleCount ) ;
        DivideByNumParticlesPerCell( particles , ugParticleCount ) ;
    }

    QUERY_PERFORMANCE_EXIT( PclOpAssignScalarFromGrid_Operate ) ;
}
