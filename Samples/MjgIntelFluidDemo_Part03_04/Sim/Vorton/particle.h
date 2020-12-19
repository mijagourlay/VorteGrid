/*! \file particle.h

    \brief Basic particle for use in a visual effects particle system

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_H
#define PARTICLE_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "wrapperMacros.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Basic particle for use in a visual effects particle system
*/
class Particle
{
    public:
        /*! \brief Construct a particle
        */
        Particle()
            : mPosition( 0.0f , 0.0f , 0.0f )
            , mVelocity( 0.0f , 0.0f , 0.0f )
            , mOrientation( 0.0f , 0.0f , 0.0f )
            , mAngularVelocity( 0.0f , 0.0f , 0.0f )
            , mMass( 0.0f )
            , mSize( 0.0f )
            , mBirthTime( 0 )
        {
        }

        Vec3    mPosition	        ;   ///< Position (in world units) of center of particle
        Vec3    mVelocity	        ;   ///< Velocity of particle
        Vec3    mOrientation	    ;   ///< Orientation of particle, in axis-angle form where angle=|orientation|
        Vec3    mAngularVelocity	;   ///< Angular velocity of particle
        float	mMass               ;   ///< Mass of particle
        float	mSize		        ;   ///< Size of particle
        int     mBirthTime          ;   ///< Birth time of particle, in "ticks"
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------

#endif
