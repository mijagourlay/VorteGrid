/*! \file PclOpAdvect.h

    \brief Particle operation to advect particles according to a velocity field.

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
#ifndef PARTICLE_OPERATION_ADVECT_H
#define PARTICLE_OPERATION_ADVECT_H

#include "particleOperation.h"

/** Particle operation to advect particles according to a velocity field.

    \see AdvectParticlesSlice, Particles::Advect
*/
class PclOpAdvect : public IParticleOperation
{
    public:
        PclOpAdvect()
            : mVelocityGrid( 0 )
            , mUseVelocityGrid( true )
        {}

        void Operate(  Vector< Particle > & particles , float timeStep , unsigned /* uFrame */ )
        {
            QUERY_PERFORMANCE_ENTER ;
            Particles::Advect( particles , mUseVelocityGrid ? mVelocityGrid : 0 , timeStep ) ;
            QUERY_PERFORMANCE_EXIT( Particles_Advect ) ;
        }

        const UniformGrid< Vec3  > *    mVelocityGrid       ;   ///< Grid of velocity values.
        bool                            mUseVelocityGrid    ;   ///< Whether to assign velocity values from grid, otherwise use velocity already in particle.
} ;

#endif
