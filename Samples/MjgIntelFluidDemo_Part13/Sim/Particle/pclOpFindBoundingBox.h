/*! \file PclOpFindBoundingBox.h

    \brief Particle operation to find the bounding box of a dynamic array of particles

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
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_FIND_BOUNDING_BOX_H
#define PARTICLE_OPERATION_FIND_BOUNDING_BOX_H

#include "particleOperation.h"

/** Find bounding box of a dynamic array of particles.
*/
class PclOpFindBoundingBox : public IParticleOperation
{
    public:
        PclOpFindBoundingBox()
            : mMinCorner( FLT_MAX , FLT_MAX , FLT_MAX )
            , mMaxCorner( - mMinCorner )
        {}

        void Operate(  Vector< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ )
        {
            mMinCorner = Vec3 ( FLT_MAX , FLT_MAX , FLT_MAX );
            mMaxCorner = - mMinCorner ;
            Particles::FindBoundingBox( particles , mMinCorner , mMaxCorner ) ;
        }

        /// Return the minimal corner of axis-aligned bounding box that contains all particles.
        const Vec3 & GetMinCorner( void ) const { return mMinCorner ; }

        /// Return the maximal corner of axis-aligned bounding box that contains all particles.
        const Vec3 & GetMaxCorner( void ) const { return mMaxCorner ; }

    private:
        Vec3    mMinCorner ;    ///< Minimal corner of axis-aligned bounding box containing all particles.
        Vec3    mMaxCorner ;    ///< Maximal corner of axis-aligned bounding box containing all particles.
} ;

#endif
