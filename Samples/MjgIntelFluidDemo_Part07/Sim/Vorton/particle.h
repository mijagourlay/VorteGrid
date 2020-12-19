/*! \file particle.h

    \brief Basic particle for use in a visual effects particle system

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
#ifndef PARTICLE_H
#define PARTICLE_H

#include <math.h>

#include "useTbb.h"

#include "Core/Math/vec3.h"
#include "wrapperMacros.h"

#include "Space/uniformGrid.h"

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

        Particle( const Vec3 & vPos , const Vec3 & vAngVel , float fSize = 0.0f )
            : mPosition( vPos )
            , mVelocity( 0.0f , 0.0f , 0.0f )
            , mOrientation( 0.0f , 0.0f , 0.0f )
            , mAngularVelocity( vAngVel )
            , mMass( 0.0f )
            , mSize( fSize )
            , mBirthTime( 0 )
        {
        }

        // Use the compiler-generated copy constructor

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

namespace Particles
{
    /*! \brief Kill the particle at the given index, replacing it with the last particle in the given dynamic array
    */
    inline void Kill( Vector< Particle > & particles , size_t iParticle )
    {
        particles[ iParticle ] = particles[ particles.Size() - 1 ] ;
        particles.PopBack() ;
    }

    extern void SetMass( Vector< Particle > & particles , float massPerParticle ) ;
    extern void Advect( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep , const unsigned & uFrame ) ;
    extern void FindBoundingBox( const Vector< Particle > & particles , Vec3 & minCorner , Vec3 & maxCorner ) ;
    extern void Emit( Vector< Particle > & particles , const UniformGridGeometry & uniformGrid , unsigned multiplier , float massPerParticle ) ;
    extern float ComputeMassPerParticle( const UniformGridGeometry & uniformGrid , unsigned numParticlesPerCell , float density ) ;
}

#endif
