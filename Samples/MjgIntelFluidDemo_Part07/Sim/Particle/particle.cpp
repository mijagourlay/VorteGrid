/*! \file particle.cpp

    \brief Basic particle for use in a visual effects particle system

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include <stdlib.h>

#include "Core/Performance/perf.h"
#include "Space/uniformGridMath.h"
#include "Sim/Vorton/particle.h"




/*! \brief Advect (subset of) given particles using given velocity field

    \param particles - dynamic array of particles to advect

    \param velocityGrid - uniform grid of velocity values.

            Each particle velocity is INCREMENTED by the velocity obtained
            from interpolating values in this grid.
            Then the particle position is updated by that resulting velocity
            time the timeStep.

    \param timeStep - amount of time by which to advance simulation

    \param uFrame - frame counter

    \param itStart - index of first particle to advect

    \param itEnd - index of last particle to advect


*/
static void AdvectParticlesSlice( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep , const unsigned & uFrame , size_t itStart , size_t itEnd )
{
    for( size_t offset = itStart ; offset < itEnd ; ++ offset )
    {   // For each particle in this slice...
        Particle & pcl = particles[ offset ] ;
        Vec3 velocity ;
        velocityGrid.Interpolate( velocity , pcl.mPosition ) ;
        pcl.mVelocity += velocity ; // Update for use in collisions
        pcl.mPosition += pcl.mVelocity * timeStep ;
    }
}




#if USE_TBB
    unsigned gNumberOfProcessors = 8 ;  ///< Number of processors this machine has.  This will get reassigned later.

    /*! \brief Functor (function object) to advect particles using Threading Building Blocks
    */
    class Particles_Advect_TBB
    {
            Vector< Particle >  &       mParticles    ;   ///< Array of particles to advect
            const UniformGrid< Vec3 > & mVelocityGrid ;   ///< Grid of velocity values
            const float         &       mTimeStep     ;   ///< Duration of this time step
            const unsigned      &       mFrame        ;   ///< Update counter
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Advect subset of particles.
                AdvectParticlesSlice( mParticles , mVelocityGrid , mTimeStep , mFrame , r.begin() , r.end() ) ;
            }
            Particles_Advect_TBB( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep , const unsigned & uFrame )
                : mParticles( particles )
                , mVelocityGrid( velocityGrid )
                , mTimeStep( timeStep )
                , mFrame( uFrame )
            {}
    } ;




    /*! \brief Functor (function object) to update axis-aligned bounding box for a dynamic array particles
        
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




/*! \brief Assign mass to each particle

    \param particles - dynamic array of particles

    \param massPerParticle - mass to assign to each particle

*/
void Particles::SetMass( Vector< Particle > & particles , float massPerParticle )
{
    const size_t numParticles = particles.Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {   // For each particle in the given array...
        Particle & rPcl   = particles[ iPcl ] ;
        rPcl.mMass = massPerParticle ;
    }
}




/*! \brief Create particles within the given region

    \param particles - dynamic array of particles

    \param uniformGrid - uniformGrid within which to emit particles

    \param multiplier - number of particles to emit along each dimension within each cell of the given uniformGrid.
                        This is the cube root of the number of particles to emit per cell.

*/
void Particles::Emit( Vector< Particle > & particles , const UniformGridGeometry & uniformGrid , unsigned multiplier , float massPerParticle )
{
    const Vec3      vSpacing        = uniformGrid.GetCellSpacing() ;
    // Must keep particles away from maximal boundary by at least cell.  Note the +vHalfSpacing in loop.
    const unsigned  begin[3]        = { 1*uniformGrid.GetNumCells(0)/8 , 1*uniformGrid.GetNumCells(1)/8 , 1*uniformGrid.GetNumCells(2)/8 } ;
    const unsigned  end[3]          = { 7*uniformGrid.GetNumCells(0)/8 , 7*uniformGrid.GetNumCells(1)/8 , 7*uniformGrid.GetNumCells(2)/8 } ;
    const float     pclSize         = 2.0f * powf( vSpacing.x * vSpacing.y * vSpacing.z , 2.0f / 3.0f ) / float( multiplier ) ;
    const Vec3      noise           = vSpacing / float( multiplier ) ;
    unsigned        idx[3]          ;

    const unsigned  nt[3]           = { multiplier , multiplier , multiplier } ;

    for( idx[2] = begin[2] ; idx[2] <= end[2] ; ++ idx[2] )
    for( idx[1] = begin[1] ; idx[1] <= end[1] ; ++ idx[1] )
    for( idx[0] = begin[0] ; idx[0] <= end[0] ; ++ idx[0] )
    {   // For each interior grid cell...
        Vec3 vPosMinCorner ;
        uniformGrid.PositionFromIndices( vPosMinCorner , idx ) ;
        Particle pcl ;
        pcl.mVelocity	        = Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mOrientation	    = Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mAngularVelocity	= Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mMass               = massPerParticle ;
        pcl.mSize		        = pclSize ;
        pcl.mBirthTime          = 0 ;

        unsigned it[3] ;
        for( it[2] = 0 ; it[2] < nt[2] ; ++ it[2] )
        for( it[1] = 0 ; it[1] < nt[1] ; ++ it[1] )
        for( it[0] = 0 ; it[0] < nt[0] ; ++ it[0] )
        {
            Vec3 vShift( float( it[0] ) / float( nt[0] ) * vSpacing.x ,
                         float( it[1] ) / float( nt[1] ) * vSpacing.y ,
                         float( it[2] ) / float( nt[2] ) * vSpacing.z ) ;
            pcl.mPosition           = vPosMinCorner + vShift + RandomSpread( noise ) ;
            particles.PushBack( pcl ) ;
        }
    }
}




/*! \brief Find axis-aligned bounding box for a subset of all particles in this simulation.
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




/*! \brief Advect particles using velocity field

    \param particles - dynamic array of particles to advect

    \param velocityGrid - Uniform grid of velocity values used to advect particles

    \param timeStep - amount of time by which to advance simulation

    \param uFrame - update counter

    \see ComputeVelocityGrid

*/
void Particles::Advect( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep , const unsigned & uFrame )
{
    const size_t numParticles = particles.Size() ;

#if USE_TBB
    // Estimate grain size based on size of problem and number of processors.
    const size_t grainSize =  MAX2( 1 , numParticles / gNumberOfProcessors ) ;
    // Advect particles using multiple threads.
    parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , Particles_Advect_TBB( particles , velocityGrid , timeStep , uFrame ) ) ;
#else
    AdvectParticlesSlice( particles , velocityGrid , timeStep , uFrame , 0 , numParticles ) ;
#endif

}




float Particles::ComputeMassPerParticle( const UniformGridGeometry & uniformGrid , unsigned numParticlesPerCell , float density )
{
    float domainVolume = uniformGrid.GetExtent().x * uniformGrid.GetExtent().y * uniformGrid.GetExtent().z ;
    if( 0.0f == uniformGrid.GetExtent().z )
    {   // Domain is 2D in XY plane.
        domainVolume = uniformGrid.GetExtent().x * uniformGrid.GetExtent().y ;
    }
    const float     totalMass           = domainVolume * density ;
    const float     massPerParticle     = totalMass / float( uniformGrid.GetGridCapacity() * numParticlesPerCell ) ;
    return massPerParticle ;
}




//const Vec3 ComputeCenterOfMass( const Vector< Particle > & particles ) const
//{
//    Vec3 vCoM( 0.0f , 0.0f , 0.0f ) ;
//    const size_t & numTracers = GetTracers().Size() ;
//    for( size_t iTracer = 0 ; iTracer < numTracers ; ++ iTracer )
//    {
//        const Particle & pcl = GetTracers()[ iTracer ] ;
//        vCoM += pcl.mPosition ;
//    }
//    vCoM /= float( numTracers ) ;
//    return vCoM ;
//}
