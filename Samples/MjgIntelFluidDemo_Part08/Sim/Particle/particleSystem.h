/*! \file particleSystem.h

    \brief Particle system

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include "particleProcess.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

class ParticleSystem
{
    public:
        void Update( float timeStep , unsigned uFrame )
        {
            const size_t numProcesses = mParticleProcesses.Size() ;
            for( size_t iProcess = 0 ; iProcess < numProcesses ; ++ iProcess )
            {
                ParticleProcess & process = mParticleProcesses[ iProcess ] ;
                process.Update( timeStep , uFrame ) ;
            }
        }

        Vector< ParticleProcess >    mParticleProcesses ;
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------

#endif
