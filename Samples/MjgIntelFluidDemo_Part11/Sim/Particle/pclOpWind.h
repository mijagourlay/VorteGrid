/*! \file PclOpWind.h

    \brief Particle operation to push particles up to a maximum speed

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_WIND_H
#define PARTICLE_OPERATION_WIND_H

#include "particleOperation.h"

/*! \brief Particle operation to push particles up to a maximum speed
*/
class PclOpWind : public IParticleOperation
{
    public:
        PclOpWind()
            : mWind( 1.0f , 0.0f , 0.0f )
        {}

        void Operate(  Vector< Particle > & particles , float timeStep , unsigned uFrame )
        {
            (void) timeStep ; // Avoid "unreferenced formal parameter" warning.
            (void) uFrame   ; // Avoid "unreferenced formal parameter" warning.
            if( mWind != Vec3( 0.0f , 0.0f , 0.0f ) )
            {
                const size_t numParticles = particles.Size() ;
                Particle * pPcls = & particles[ 0 ] ;
                for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
                {   // For each particle in the given array...
                    Particle & rPcl = pPcls[ iPcl ] ;
                    // Compute difference between wind and particle velocity.
                    const Vec3 velDiff      = mWind - rPcl.mVelocity ;
                    // Compute amount by which to change particle velocity.
                    const Vec3 velChange    = mGain * velDiff ;
                    // Update particle velocity.
                    rPcl.mVelocity += velChange ;
                }
            }
        }

        Vec3                    mWind       ;   ///< Wind velocity
        float                   mGain       ;   ///< Fraction of velocity difference to apply to particle, each frame
} ;

#endif
