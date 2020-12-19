/*! \file particleProcess.h

    \brief Group of particles and operations to perform on them.

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_PROCESS_H
#define PARTICLE_PROCESS_H

#include "particleOperation.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

/*! \brief Group of particles and operations to perform on them.
*/
class ParticleGroup
{
    public:
        ParticleGroup()
            : mParticles( 0 )
        {}

        ~ParticleGroup()
        {
            mParticles.Clear() ;
            mParticleOps.Clear() ;
        }

        void Update( float timeStep , unsigned uFrame )
        {
            const size_t numOps = mParticleOps.Size() ;
            for( size_t iOp = 0 ; iOp < numOps ; ++ iOp )
            {
                IParticleOperation * pOp = mParticleOps[ iOp ] ;
                pOp->Operate( mParticles , timeStep , uFrame ) ;
            }
        }

        Vector< Particle >              mParticles      ;
        Vector< IParticleOperation * >  mParticleOps    ;
} ;

// Public variables ------------------------------------------------------------

// Public functions ------------------------------------------------------------

#endif
