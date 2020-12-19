/*! \file particleProcess.h

    \brief Particle process

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
#ifndef PARTICLE_PROCESS_H
#define PARTICLE_PROCESS_H

#include "particleOperation.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Sequence of particle operations
*/
class ParticleProcess
{
    public:
        ParticleProcess( void ) { }

        ~ParticleProcess()
        {
            mParticleOps.Clear() ;
        }

        void Update( float timeStep , unsigned uFrame )
        {
            const size_t numOps = mParticleOps.Size() ;
            for( size_t iOp = 0 ; iOp < numOps ; ++ iOp )
            {
                IParticleOperation * pOp = mParticleOps[ iOp ] ;
                pOp->Operate( timeStep , uFrame ) ;
            }
        }

        Vector< IParticleOperation * >  mParticleOps ;
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------

#endif
