/*! \file particleProcess.h

    \brief Group of particles and operations to perform on them.

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

        /** Perform all particle operations in this group.
            \see IParticleOperation
        */
        void Update( float timeStep , unsigned uFrame )
        {
            const size_t numOps = mParticleOps.Size() ;
            if( timeStep >= 0.0f )
            {   // Simulation clock is running forward.
                for( size_t iOp = 0 ; iOp < numOps ; ++ iOp )
                {   // Run operations in order.
                    IParticleOperation * pOp = mParticleOps[ iOp ] ;
                    pOp->Operate( mParticles , timeStep , uFrame ) ;
                }
            }
            else
            {   // Simulation clock is running backward.
                for( size_t iOp = numOps - 1 ; ( iOp >= 0 ) && ( iOp < numOps ) ; -- iOp )
                {   // Run operations in reverse.
                    IParticleOperation * pOp = mParticleOps[ iOp ] ;
                    pOp->Operate( mParticles , timeStep , uFrame ) ;
                }
            }
        }

        Vector< Particle >              mParticles      ;   ///< Dynamic array of particles which this group owns and on which all ParticleOperations in this group act.
        Vector< IParticleOperation * >  mParticleOps    ;   ///< Dynamic array of particle operations which operate on the particles that this group owns.
} ;

// Public variables ------------------------------------------------------------

// Public functions ------------------------------------------------------------

#endif
