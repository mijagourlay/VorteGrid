/*! \file PclOpFindBoundingBox.h

    \brief Particle operation to find the bounding box of a dynamic array of particles

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_OPERATION_FIND_BOUNDING_BOX_H
#define PARTICLE_OPERATION_FIND_BOUNDING_BOX_H

#include "particleOperation.h"

/*! \brief Find bounding box of a dynamic array of particles.
*/
class PclOpFindBoundingBox : public IParticleOperation
{
    public:
        PclOpFindBoundingBox( void )
            : mParticles( 0 )
            , mMinCorner( FLT_MAX , FLT_MAX , FLT_MAX )
            , mMaxCorner( - mMinCorner )
        {}

        void Operate( float timeStep , unsigned uFrame )
        {
            mMinCorner = Vec3 ( FLT_MAX , FLT_MAX , FLT_MAX );
            mMaxCorner = - mMinCorner ;
            Particles::FindBoundingBox( * mParticles , mMinCorner , mMaxCorner ) ;
        }

        const Vec3 & GetMinCorner( void ) const { return mMinCorner ; }
        const Vec3 & GetMaxCorner( void ) const { return mMaxCorner ; }

        Vector< Particle > *    mParticles ;    ///< Dynamic array of particles whose bounding box to find

    private:
        Vec3                    mMinCorner ;    ///< Minimal corner of bounding box containing particles
        Vec3                    mMaxCorner ;    ///< Maximal corner of bounding box containing particle
} ;

#endif