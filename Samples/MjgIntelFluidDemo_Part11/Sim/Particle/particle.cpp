/*! \file particle.cpp

    \brief Basic particle for use in a visual effects particle system

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdlib.h>

#include "Core/Performance/perf.h"
#include "Space/uniformGridMath.h"
#include "Sim/Vorton/particle.h"


static const float  sOneMinusEpsilon        = 1.0f - FLT_EPSILON ;
const float Particle::sNaN = fsqrtf( -1.0f ) ;


/** Advect (subset of) given particles using given velocity field.

    \param particles - dynamic array of particles to advect

    \param velocityGrid - uniform grid of velocity values.

            Each particle velocity is INCREMENTED by the velocity obtained
            from interpolating values in this grid.
            Then the particle position is updated by that resulting velocity
            time the timeStep.

    \param timeStep - amount of time by which to advance simulation

    \param itStart - index of first particle to advect

    \param itEnd - index of last particle to advect
*/
static void AdvectParticlesSlice( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep , size_t itStart , size_t itEnd )
{
    const WORD OldCtrlWord = Changex87FloatingPointToTruncate() ;
    if( timeStep >= 0.0f )
    {   // Simulation clock is running forward.
        for( size_t offset = itStart ; offset < itEnd ; ++ offset )
        {   // For each particle in this slice...
            Particle & pcl = particles[ offset ] ;
            Vec3 velocity ;
            velocityGrid.Interpolate_AssumesFpcwSetToTruncate( velocity , pcl.mPosition ) ;
            pcl.mVelocity = velocity ; // Update for use in collisions
            pcl.mPosition += pcl.mVelocity * timeStep ;
        }
    }
    else
    {   // Simulation clock is running backwards.
        // Perform operations in reverse.
        for( size_t offset = itStart ; offset < itEnd ; ++ offset )
        {   // For each particle in this slice...
            Particle & pcl = particles[ offset ] ;
            Vec3 velocity ;
            pcl.mPosition += pcl.mVelocity * timeStep ;
            velocityGrid.Interpolate_AssumesFpcwSetToTruncate( velocity , pcl.mPosition ) ;
            pcl.mVelocity = velocity ; // Update for use in collisions
        }
    }
    Setx87ControlWord( OldCtrlWord ) ;
}




/** Assign particle density based on density grid.

    \param particles - dynamic array of particles whose density to assign

    \param memberOffsetInBytes   Offset, in bytes, from head of particle object, to assign.

    \param scalarGrid - uniform grid of density values.

    \param iPclStart - index of first particle to process

    \param iPclEnd - index of last particle to process

    \note This routine assumes PopulateDensityDevAndMassFractionGrids has already run for this timestep.

    \note This routine should run before advecting tracers, otherwise the tracers
            could leave the velocity grid boundaries, and this routine needs to interpolate
            within those boundaries.

    \see PopulateDensityDevAndMassFractionGrids, GenerateBaroclinicVorticity
*/
static void AssignScalarFromGridSlice( Vector< Particle > & particles , size_t memberOffsetInBytes , const UniformGrid< float > & scalarGrid , size_t iPclStart , size_t iPclEnd )
{
    for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
    {   // For each particle in this slice...
        Particle &  rParticle = particles[ iPcl ] ;
        float value ;
        scalarGrid.Interpolate( value , rParticle.mPosition ) ;
        *(float*)(((char*)&rParticle) + memberOffsetInBytes) = value ;
    }
}





#if USE_TBB
    unsigned gNumberOfProcessors = 8 ;  ///< Number of processors this machine has.  This will get reassigned later.

    /** Functor (function object) to assign scalar from particles from a grid.
    */
    class Particles_AssignScalarFromGrid_TBB
    {
            Vector< Particle >  &           mParticles              ;   ///< Array of particles whose scalar member to assign
            size_t                          mMemberOffsetInBytes    ;   ///< Offset, in bytes, of particle member to assign
            const UniformGrid< float > &    mScalarGrid             ;   ///< Grid of scalar information
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Assign scalar from grid to a subset of particles.
                AssignScalarFromGridSlice( mParticles , mMemberOffsetInBytes , mScalarGrid , r.begin() , r.end() ) ;
            }
            Particles_AssignScalarFromGrid_TBB( Vector< Particle > & particles , size_t memberOffsetInBytes , const UniformGrid< float > & scalarGrid )
                : mParticles( particles )
                , mMemberOffsetInBytes( memberOffsetInBytes )
                , mScalarGrid( scalarGrid )
            {}
    } ;




    /** Functor (function object) to advect particles using Threading Building Blocks.
    */
    class Particles_Advect_TBB
    {
            Vector< Particle >  &       mParticles    ;   ///< Array of particles to advect
            const UniformGrid< Vec3 > & mVelocityGrid ;   ///< Grid of velocity values
            const float         &       mTimeStep     ;   ///< Duration of this time step
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Advect subset of particles.
                AdvectParticlesSlice( mParticles , mVelocityGrid , mTimeStep , r.begin() , r.end() ) ;
            }
            Particles_Advect_TBB( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep )
                : mParticles( particles )
                , mVelocityGrid( velocityGrid )
                , mTimeStep( timeStep )
            {}
    } ;




    /** Functor (function object) to update axis-aligned bounding box for a dynamic array particles.
        
        This routine updates an existing bounding box, whose corners were found previously.
        This facilitates finding the bounding box enclosing multiple arrays of particles.

    */
    class Particles_FindBoundingBox_TBB
    {
        public:
            Particles_FindBoundingBox_TBB( const Vector< Particle > & particles , const Vec3 & vMinSoFar , const Vec3 & vMaxSoFar )
                : mParticles( particles )
                , mMin( vMinSoFar )
                , mMax( vMaxSoFar )
            {}

            // Special "map" copy constructor used by TBB
            Particles_FindBoundingBox_TBB( Particles_FindBoundingBox_TBB & that , tbb::split )
                : mParticles( that.mParticles )
                , mMin( that.mMin )
                , mMax( that.mMax )
            {}

            void operator() ( const tbb::blocked_range<size_t> & r )
            {   // Find bounding box for subset of particles
                const Particle * pParticles = & mParticles[0] ;
                for( size_t iPcl = r.begin() ; iPcl < r.end() ; ++ iPcl )
                {   // For each particle in this array...
                    const Particle & rPcl = pParticles[ iPcl ] ;
                    // Find corners of axis-aligned bounding box.
                    UpdateBoundingBox( rPcl.mPosition ) ;
                }
            }

            void join( const Particles_FindBoundingBox_TBB & other )
            {   // Reduce the results of 2 threads
                UpdateBoundingBox( other.mMin ) ;
                UpdateBoundingBox( other.mMax ) ;
            }

            Vec3                mMin        ; ///< Bounding box minimum corner for vortons visited by this thread
            Vec3                mMax        ; ///< Bounding box maximum corner for vortons visited by this thread

        private:
            void UpdateBoundingBox( const Vec3 & vPoint )
            {
                mMin.x = MIN2( mMin.x , vPoint.x ) ; // Note: TODO: Perhaps SSE/VMX have a SIMD/vector form of this operation.
                mMin.y = MIN2( mMin.y , vPoint.y ) ;
                mMin.z = MIN2( mMin.z , vPoint.z ) ;
                mMax.x = MAX2( mMax.x , vPoint.x ) ;
                mMax.y = MAX2( mMax.y , vPoint.y ) ;
                mMax.z = MAX2( mMax.z , vPoint.z ) ;
            }

            const Vector< Particle > & mParticles     ; ///< Array of particles
    } ;
#endif




/** Create particles within the given region.

    \param particles    (out) Dynamic array of particles to populate.

    \param uniformGrid  Region within which to emit particles.

    \param multiplier   Number of particles to emit along each dimension
                        within each cell of the given uniformGrid.
                        This is the cube root of the number of particles to emit
                        per cell.

    \param vortons      Optional address of Dynamic array of vortons.  If
                        provided then particles are only emitted where vortons exist.
                        This allows for placing tracers at the location of vortons.

*/
void Particles::Emit( Vector< Particle > & particles , const UniformGridGeometry & uniformGrid , unsigned multiplier , const Vector< Particle > * vortons )
{
    const Vec3      vSpacing        = uniformGrid.GetCellSpacing() ;
    // Must keep particles away from maximal boundary by at least cell.  Note the +vHalfSpacing in loop.
    //const unsigned  begin[3]        = { 1*uniformGrid.GetNumCells(0)/8 , 1*uniformGrid.GetNumCells(1)/8 , 1*uniformGrid.GetNumCells(2)/8 } ;
    //const unsigned  end[3]          = { 7*uniformGrid.GetNumCells(0)/8 , 7*uniformGrid.GetNumCells(1)/8 , 7*uniformGrid.GetNumCells(2)/8 } ;
    //const unsigned  begin[3]        = { 1,1,1 } ;
    //const unsigned  end[3]          = { uniformGrid.GetNumCells(0) - 1 , uniformGrid.GetNumCells(1) - 1 , uniformGrid.GetNumCells(2) - 1 } ;
    const unsigned  begin[3]        = { 0,0,0 } ;
    const unsigned  end[3]          = { uniformGrid.GetNumCells(0) , uniformGrid.GetNumCells(1) , uniformGrid.GetNumCells(2) } ;
    const float     pclSize         = 0.5f * powf( vSpacing.x * vSpacing.y * vSpacing.z , 1.0f / 3.0f ) / float( multiplier ) ;
    unsigned        idx[3]          ;

    const unsigned  nt[3]           = { multiplier , multiplier , multiplier } ;
    bool            emitParticle    = true ;

    UniformGrid< float > densityDeviationGrid  ; // used to mask out tracer particles
    float densityDeviationMin =  FLT_MAX ;
    float densityDeviationMax = -FLT_MAX ;

    if( vortons )
    {   // Caller provided vortons, to seed tracers.

        densityDeviationGrid.Clear() ;
        densityDeviationGrid.CopyShape( uniformGrid ) ;
        densityDeviationGrid.Init( 0.0f );

        // Populate density grid based on vortons.
        // Note that this is only a rough approximation of a proper density grid.
        // Here we only want to know whether a given cell has deviant density,
        // to get the overall shape of the density distribution.
        // This density grid will NOT have the same total mass as the particles.
        const size_t numVortons = vortons->Size() ;
        for( size_t uVorton = 0 ; uVorton < numVortons ; ++ uVorton )
        {   // For each vorton in the array...
            const Particle  &   rVorton     = (*vortons)[ uVorton ] ;
            const Vec3      &   rPosition   = rVorton.mPosition   ;
            
            densityDeviationGrid.Accumulate( rPosition , rVorton.GetMassDeviation() ) ;
        }
        // Aggregate density deviation min and max.
        for( idx[2] = begin[2] ; idx[2] <= end[2] ; ++ idx[2] )
        for( idx[1] = begin[1] ; idx[1] <= end[1] ; ++ idx[1] )
        for( idx[0] = begin[0] ; idx[0] <= end[0] ; ++ idx[0] )
        {   // For each grid cell...
            const unsigned offset = densityDeviationGrid.OffsetFromIndices( idx ) ;
            densityDeviationMin = MIN2( densityDeviationGrid[ offset ] , densityDeviationMin ) ;
            densityDeviationMax = MAX2( densityDeviationGrid[ offset ] , densityDeviationMax ) ;
        }
    }
    const float densityDeviationScale   = MAX2( fabsf( densityDeviationMax ) , fabsf( densityDeviationMin ) ) ;
    float       probMax                 = 0.0f ;

    // Shift each particle to center the distribution within each cell.
    const Vec3  vShift                  = 0.5f * vSpacing / float( multiplier ) ;

    // Jitter should distribute particles within their little subdomains,
    // and must not place particles outside grid, even by a tiny amount.
    const Vec3  jitterMagnitude         = 1.9999f * sOneMinusEpsilon * vShift ;

    for( idx[2] = begin[2] ; idx[2] < end[2] ; ++ idx[2] )
    for( idx[1] = begin[1] ; idx[1] < end[1] ; ++ idx[1] )
    for( idx[0] = begin[0] ; idx[0] < end[0] ; ++ idx[0] )
    {   // For each grid cell...
        Vec3 vPosMinCorner ;
        uniformGrid.PositionFromIndices( vPosMinCorner , idx ) ;
        Particle pcl ;
        pcl.mVelocity	            = Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mOrientation	        = Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mAngularVelocity	    = Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mDensityDeviation       = 0.0f ;
    #if ENABLE_FIRE
        pcl.mFuelFraction           = 0.0f ;
        pcl.mFlameFraction          = 0.0f ;
        pcl.mSmokeFraction          = 1.0f ;
    #endif
        pcl.mSize		            = pclSize ;
        pcl.mBirthTime              = 0 ;
        //const unsigned  offset      = densityDeviationGrid.OffsetFromIndices( idx ) ;

        if( ( ! vortons )                           // Either vorton array was not provided...
            //||  ( densityDeviationGrid[ offset ] != 0.0f)    // ...or it was provided and there is mass in this cell, (note, this does not work for negative density)
            ||  ( densityDeviationScale != 0.0f )            // ...or it was provided and there is mass variation in this domain.
            )
        {   // Want to place tracer particles here.
            unsigned it[3] ;
            for( it[2] = 0 ; it[2] < nt[2] ; ++ it[2] )
            for( it[1] = 0 ; it[1] < nt[1] ; ++ it[1] )
            for( it[0] = 0 ; it[0] < nt[0] ; ++ it[0] )
            {   // For each subdomain within each grid cell...
                const Vec3 vDisplacement(   float( it[0] ) / float( nt[0] ) * vSpacing.x ,
                                            float( it[1] ) / float( nt[1] ) * vSpacing.y ,
                                            float( it[2] ) / float( nt[2] ) * vSpacing.z ) ;

                const Vec3 jitter = RandomSpread( jitterMagnitude ) ;
                pcl.mPosition   = vPosMinCorner + vDisplacement + vShift + jitter ;
                if( vortons )
                {   // Vortons were passed in so assign density based on
                    float densityDeviationHere ;
                    densityDeviationGrid.Interpolate( densityDeviationHere , pcl.mPosition ) ;
                    pcl.mDensityDeviation = densityDeviationHere ;
                    const float probability = fabsf( densityDeviationHere / densityDeviationScale ) ;
                    probMax = MAX2( probMax , probability ) ;
                    //const float dieRoll     = float( rand() ) / float( RAND_MAX ) ;
                    //emitParticle = dieRoll <= probability ;
                    emitParticle = probability >= 0.5f ;
                }
                if( emitParticle )
                {
                    particles.PushBack( pcl ) ;
                }
            }
        }
    }
}




/** Find axis-aligned bounding box for a subset of all particles in this simulation.
*/
inline void Particles::FindBoundingBox( const Vector< Particle > & particles , Vec3 & minCorner , Vec3 & maxCorner )
{
    QUERY_PERFORMANCE_ENTER ;

    const size_t numParticles = particles.Size() ;
    #if USE_TBB
    {
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  MAX2( 1 , numParticles / gNumberOfProcessors ) ;
        // Find bounding box of vortons using multiple threads.
        Particles_FindBoundingBox_TBB fbbp( particles , minCorner , maxCorner ) ;
        parallel_reduce( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , fbbp ) ;
        minCorner = fbbp.mMin ;
        maxCorner = fbbp.mMax ;
    }
    #else   // Serial version.
    {
        Vec3 vMinCorner( minCorner ) ;
        Vec3 vMaxCorner( maxCorner ) ;
        for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
        {   // For each particle in the given array...
            const Particle & rPcl   = particles[ iPcl ] ;
            const Vec3     & vPoint = rPcl.mPosition ;
            // Update corners of axis-aligned bounding box.
            vMinCorner.x = MIN2( vMinCorner.x , vPoint.x ) ; // Note: TODO: Perhaps SSE/VMX have a SIMD/vector form of this operation.
            vMinCorner.y = MIN2( vMinCorner.y , vPoint.y ) ;
            vMinCorner.z = MIN2( vMinCorner.z , vPoint.z ) ;
            vMaxCorner.x = MAX2( vMaxCorner.x , vPoint.x ) ;
            vMaxCorner.y = MAX2( vMaxCorner.y , vPoint.y ) ;
            vMaxCorner.z = MAX2( vMaxCorner.z , vPoint.z ) ;
        }
        minCorner = vMinCorner ;
        maxCorner = vMaxCorner ;
    }
    #endif

    QUERY_PERFORMANCE_EXIT( Particles_FindBoundingBox ) ;
}




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

    \see PopulateDensityDevAndMassFractionGrids, GenerateBaroclinicVorticity, AssignScalarFromGridSlice
*/
void Particles::AssignScalarFromGrid( Vector< Particle > & particles , size_t memberOffsetInBytes , const UniformGrid< float > & scalarGrid )
{
#if ENABLE_FLUID_BODY_SIMULATION
    if( scalarGrid.HasZeroExtent() )
    {   // Scalar grid is empty.  Probably first iteration.
        return ; // Nothing to do.
    }

    const size_t    numParticles = particles.Size() ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  MAX2( 1 , numParticles / gNumberOfProcessors ) ;
        // Assign scalar from grid using threading building blocks.
        parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , Particles_AssignScalarFromGrid_TBB( particles , memberOffsetInBytes , scalarGrid ) ) ;
    #else
        AssignScalarFromGridSlice( particles , memberOffsetInBytes , scalarGrid , 0 , numParticles ) ;
    #endif
#endif
}




/** Advect particles using velocity field.

    \param particles - dynamic array of particles to advect

    \param velocityGrid - Uniform grid of velocity values used to advect particles

    \param timeStep - amount of time by which to advance simulation

    \param uFrame - update counter

    \see ComputeVelocityGrid

*/
void Particles::Advect( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep )
{
#if ENABLE_FLUID_BODY_SIMULATION

    if( velocityGrid.HasZeroExtent() )
    {   // Velocity grid is empty.  Probably first iteration.
        return ; // Nothing to do.
    }

    const size_t numParticles = particles.Size() ;

#if USE_TBB
    // Estimate grain size based on size of problem and number of processors.
    const size_t grainSize =  MAX2( 1 , numParticles / gNumberOfProcessors ) ;
    // Advect particles using multiple threads.
    parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , Particles_Advect_TBB( particles , velocityGrid , timeStep ) ) ;
#else
    AdvectParticlesSlice( particles , velocityGrid , timeStep , 0 , numParticles ) ;
#endif

#endif
}




/** Compute the geometric center of all given particles.

    The "geometric center" is simply the average position of all particles,
    in contrast to the center of mass in which each particle position is weighted
    by its mass.
*/
Vec3 Particles::ComputeGeometricCenter( const Vector< Particle > & particles )
{
    Vec3 vCenter( 0.0f , 0.0f , 0.0f ) ;
    const size_t numTracers = particles.Size() ;
    for( size_t iTracer = 0 ; iTracer < numTracers ; ++ iTracer )
    {
        const Particle & pcl = particles[ iTracer ] ;
        vCenter += pcl.mPosition ;
    }
    vCenter /= float( numTracers ) ;
    return vCenter ;
}




/** Compute the total angular momentum of all given particles.

    \note   Used to diagnose the simulation.  Angular momentum should be
            conserved, that is, constant over time.
*/
Vec3 Particles::ComputeAngularMomentum( const Vector< Particle > & particles , float ambientFluidDensity )
{
    Vec3 vAngMom( 0.0f , 0.0f , 0.0f ) ;
#if 1
    const size_t numParticles = particles.Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {
        const Particle & pcl = particles[ iPcl ] ;
        const float mass    = pcl.GetMass( ambientFluidDensity ) ;
        const float radius  = pcl.GetRadius() ;
        const float momentOfInertia = 0.4f * mass * radius * radius ;
        vAngMom += pcl.mAngularVelocity * momentOfInertia ;
    }
#endif
    return vAngMom ;
}




/** Kill particles marked for death.

    Some operations, such as "Merge", can effectively kill particles,
    but those operations can cause the particle array to become reordered.
    That is because Particles::Kill fills the slot previously occupied by
    the killed particle, with a particle at the end of the array, so
    that the array has no "holes".

    In order to make parallelizing this easier, some routines that want to
    kill particles instead simply "mark them for death" so that this routine
    can sweep through and reorder the particles array.

    \see MarkDead, IsAlive.
*/
extern void Particles::KillParticlesMarkedForDeath( Vector< Particle > & particles )
{
    size_t numParticles = particles.Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; )
    {
        const Particle & pcl = particles[ iPcl ] ;
        if( ! pcl.IsAlive() )
        {   // Particle was marked for death.
            Particles::Kill( particles , iPcl ) ;
            -- numParticles ;
        }
        else
        {   // Particle remains alive.
            ++ iPcl ;
        }
    }
}
