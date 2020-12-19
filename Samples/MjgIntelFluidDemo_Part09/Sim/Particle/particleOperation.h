/*! \file particleOperation.h

    \brief Particle operation interface

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

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_OPERATION_H
#define PARTICLE_OPERATION_H

#include "Sim/Vorton/particle.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Particle operation abstract base class
*/
class IParticleOperation
{
    public:
        /*! \brief Construct a particle
        */
        IParticleOperation() {}

        virtual ~IParticleOperation() {}

        virtual void Operate( Vector< Particle > & particles , float timeStep , unsigned uFrame ) = 0 ;
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
