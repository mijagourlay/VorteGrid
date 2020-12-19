/** \file pclOpFindBoundingBox.cpp

    \brief Operation to find the bounding box of a dynamic array of particles.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdlib.h>

#include "Core/Performance/perf.h"
#include "Core/SpatialPartition/uniformGridMath.h"
#include "../particle.h"
#include "pclOpFindBoundingBox.h"


static const float  sOneMinusEpsilon        = 1.0f - FLT_EPSILON ;




#if USE_TBB
    /** Functor (function object) to update axis-aligned bounding box for a dynamic array particles.
        
        This routine updates an existing bounding box, whose corners were found previously.
        This facilitates finding the bounding box enclosing multiple arrays of particles.

    */
    class Particles_FindBoundingBox_TBB
    {
        public:
            Particles_FindBoundingBox_TBB( const VECTOR< Particle > & particles , const Vec3 & vMinSoFar , const Vec3 & vMaxSoFar )
                : mParticles( particles )
                , mMin( vMinSoFar )
                , mMax( vMaxSoFar )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }

            // Special "map" copy constructor used by TBB
            Particles_FindBoundingBox_TBB( Particles_FindBoundingBox_TBB & that , tbb::split )
                : mParticles( that.mParticles )
                , mMin( that.mMin )
                , mMax( that.mMax )
                , mMasterThreadFloatingPointControlWord( that.mMasterThreadFloatingPointControlWord )
                , mMasterThreadMmxControlStatusRegister( that.mMasterThreadMmxControlStatusRegister )
            {
            }

            void operator() ( const tbb::blocked_range<size_t> & r )
            {   // Find bounding box for subset of particles
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
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
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                UpdateBoundingBox( other.mMin ) ;
                UpdateBoundingBox( other.mMax ) ;
            }

            Vec3                mMin        ; ///< Bounding box minimum corner for vortons visited by this thread
            Vec3                mMax        ; ///< Bounding box maximum corner for vortons visited by this thread

        private:
            void UpdateBoundingBox( const Vec3 & vPoint )
            {
                mMin.x = Min2( mMin.x , vPoint.x ) ; // Note: TODO: Perhaps SSE/VMX have a SIMD/vector form of this operation.
                mMin.y = Min2( mMin.y , vPoint.y ) ;
                mMin.z = Min2( mMin.z , vPoint.z ) ;
                mMax.x = Max2( mMax.x , vPoint.x ) ;
                mMax.y = Max2( mMax.y , vPoint.y ) ;
                mMax.z = Max2( mMax.z , vPoint.z ) ;
            }

            const VECTOR< Particle > & mParticles     ; ///< Array of particles
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;
#endif





/** Find axis-aligned bounding box for a subset of all particles in this simulation.
*/
void PclOpFindBoundingBox::FindBoundingBox( const VECTOR< Particle > & particles , Vec3 & minCorner , Vec3 & maxCorner )
{
    QUERY_PERFORMANCE_ENTER ;

    const size_t numParticles = particles.Size() ;
    #if USE_TBB
    {
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , numParticles / gNumberOfProcessors ) ;
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
            ASSERT( ! IsInf( rPcl.mPosition ) ) ;
            // Update corners of axis-aligned bounding box.
            vMinCorner.x = Min2( vMinCorner.x , vPoint.x ) ; // Note: TODO: Perhaps SSE/VMX have a SIMD/vector form of this operation.
            vMinCorner.y = Min2( vMinCorner.y , vPoint.y ) ;
            vMinCorner.z = Min2( vMinCorner.z , vPoint.z ) ;
            vMaxCorner.x = Max2( vMaxCorner.x , vPoint.x ) ;
            vMaxCorner.y = Max2( vMaxCorner.y , vPoint.y ) ;
            vMaxCorner.z = Max2( vMaxCorner.z , vPoint.z ) ;
        }
        minCorner = vMinCorner ;
        maxCorner = vMaxCorner ;
    }
    #endif

    QUERY_PERFORMANCE_EXIT( PclOpFindBoundingBox_FindBoundingBox ) ;
}




void PclOpFindBoundingBox::Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ )
{
    mMinCorner = Vec3 ( FLT_MAX , FLT_MAX , FLT_MAX );
    mMaxCorner = - mMinCorner ;
    FindBoundingBox( particles , mMinCorner , mMaxCorner ) ;
}
