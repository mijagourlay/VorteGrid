/*! \file particleSystem.h

    \brief Particle system

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include "particleProcess.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/** Container for particle system.

    A ParticleSystem contains multiple ParticleGroups where each group
    contains multiple ParticleOperations and a Particle array.
*/
class ParticleSystem
{
    public:

        /** Process all particle groups in this system.
            \see ParticleGroup
        */
        void Update( float timeStep , unsigned uFrame )
        {
            const size_t numParticleGroups = mParticleGroups.Size() ;
            if( timeStep >= 0.0f )
            {   // Simulation clock is running forward.
                for( size_t iPclGrp = 0 ; iPclGrp < numParticleGroups ; ++ iPclGrp )
                {   // Run through groups in order.
                    ParticleGroup * particleGroup = mParticleGroups[ iPclGrp ] ;
                    particleGroup->Update( timeStep , uFrame ) ;
                }
            }
            else
            {   // Simulation clock is running backward.
                for( size_t iPclGrp = numParticleGroups - 1 ; ( iPclGrp >= 0 ) && ( iPclGrp < numParticleGroups ) ; -- iPclGrp )
                {   // Run through groups in reverse.
                    ParticleGroup * particleGroup = mParticleGroups[ iPclGrp ] ;
                    particleGroup->Update( timeStep , uFrame ) ;
                }
            }
        }

        Vector< ParticleGroup * > mParticleGroups  ;    ///< Dynamic array of ParticleGroup objects that this ParticleSystem owns.
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------

#endif
