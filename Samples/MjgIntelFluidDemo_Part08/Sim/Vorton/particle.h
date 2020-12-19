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
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/

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

static const float  Pi                  = 3.1415926535897932384626433832795f ;
static const float  PiOver6             = Pi / 6.0f ;

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
            , mDensity( 0.0f )
            , mSize( 0.0f )
            , mBirthTime( 0 )
        #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
            , mPositionSiblings( 0.0f , 0.0f , 0.0f )
            , mVorticitySiblings( 0.0f , 0.0f , 0.0f )
            //, mRadiusSiblings( 0.0f )
        #endif
        {
        }

        Particle( const Vec3 & vPos , const Vec3 & vAngVel , float fSize = 0.0f )
            : mPosition( vPos )
            , mVelocity( 0.0f , 0.0f , 0.0f )
            , mOrientation( 0.0f , 0.0f , 0.0f )
            , mAngularVelocity( vAngVel )
            , mDensity( 0.0f )
            , mSize( fSize )
            , mBirthTime( 0 )
        #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
            , mPositionSiblings( 0.0f , 0.0f , 0.0f )
            , mVorticitySiblings( 0.0f , 0.0f , 0.0f )
            //, mRadiusSiblings( 0.0f )
        #endif
        {
        }


        /*! \brief Return volume of this particle

            \return Volume of this particle

            \note Derivation:
                    Volume of a sphere is 4*pi*r^3/3.
                    r=size/2 so r^3=size/8.
                    V=4*pi*size^3/(3*8)=pi*size^3/6
        */
        float GetVolume( void ) const
        {
            return PiOver6 * mSize * mSize * mSize ;
        }



        /*! \brief Return mass of this particle

            \return Mass of this particle
        */
        float GetMass() const
        {
            const float fMass   = mDensity * GetVolume() ;
            return fMass ;
        }


        /*! \brief Return mass of this particle

            \param ambientDensity - density of fluid in the absence of particles.

            This form of GetMass is used when particle density
            represent a deviations from the ambient fluid density,
            instead of representing fluid density absolutely.
            Such a treatment allows for placing particles judiciously
            instead of ubiquitously.

            \return Mass of this particle
        */
        float GetMass( float ambientDensity ) const
        {
            const float fMass   = ( mDensity + ambientDensity ) * GetVolume() ;
            return fMass ;
        }


        // Use the compiler-generated copy constructor

        Vec3    mPosition	        ;   ///< Position (in world units) of center of particle
        Vec3    mVelocity	        ;   ///< Velocity (speed and direction) of particle
        Vec3    mOrientation	    ;   ///< Orientation of particle, in axis-angle form where angle=|orientation|
        Vec3    mAngularVelocity	;   ///< Angular velocity of particle.  Same as half the vorticity.
        float	mDensity            ;   ///< Either smoothed density or mass per particle, depending.
        float	mSize		        ;   ///< Diameter of the region of influence of a particle.
        int     mBirthTime          ;   ///< Birth time of particle, in "ticks"

        // HACK CODE: None of these members belong in Particle.
        // They're useful for debugging the vorton sim,
        // and we want to consolidate particle opertion code to
        // work on both Particle and Vorton.
#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
        Vec3    mPositionSiblings   ;   ///< Position (in world units) of center of vortex particle
        Vec3    mVorticitySiblings  ;   ///< Vorticity of vortex particle
        //float   mRadiusSiblings     ;   ///< Radius of sibling cluster
#endif

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

    extern void SetDensity( Vector< Particle > & particles , float density ) ;
    extern void Advect( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep , const unsigned & uFrame ) ;
    extern void FindBoundingBox( const Vector< Particle > & particles , Vec3 & minCorner , Vec3 & maxCorner ) ;
    extern void Emit( Vector< Particle > & particles , const UniformGridGeometry & uniformGrid , unsigned multiplier , float massPerParticle , const Vector< Particle > * vortons = 0 ) ;
    extern float ComputeMassPerParticle( const UniformGridGeometry & uniformGrid , unsigned numParticlesPerCell , float density ) ;
    extern Vec3 ComputeCenterOfMass( const Vector< Particle > & particles ) ;
}

#endif
