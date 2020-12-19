/*! \file PclOpAdvect.h

    \brief Particle operation to advect particles according to a velocity field

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
#ifndef PARTICLE_OPERATION_ADVECT_H
#define PARTICLE_OPERATION_ADVECT_H

#include "particleOperation.h"

/*! \brief Particle operation to push particles up to a maximum speed
*/
class PclOpAdvect : public IParticleOperation
{
    public:
        PclOpAdvect( void )
            : mParticles( 0 )
            , mVelocityGrid( 0 )
        {}

        void Operate( float timeStep , unsigned uFrame )
        {
            QUERY_PERFORMANCE_ENTER ;
            Particles::Advect( * mParticles , * mVelocityGrid , timeStep , uFrame ) ;
            QUERY_PERFORMANCE_EXIT( Particles_Advect ) ;
        }

        Vector< Particle > *        mParticles      ;   ///< Dynamic array of particles to advect
        const UniformGrid< Vec3 > * mVelocityGrid   ;   ///< Grid of velocity values
} ;

#endif