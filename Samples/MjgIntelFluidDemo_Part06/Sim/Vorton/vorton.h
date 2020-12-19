/*! \file vorton.h

    \brief Vortex particle

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef VORTON_H
#define VORTON_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "wrapperMacros.h"

// Macros --------------------------------------------------------------

/*! \brief Techniques for computing velocity from vorticity
*/
#define VELOCITY_TECHNIQUE_DIRECT               'VTBF'  ///< Slow (quadratic), accurate direct summation
#define VELOCITY_TECHNIQUE_TREE                 'VTTC'  ///< Faster (8N log N), moderately accurate treecode
#define VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL 'VTPG'  ///< Differential approach: Use a finite-difference Poisson solver

//#define VELOCITY_TECHNIQUE  VELOCITY_TECHNIQUE_DIRECT
//#define VELOCITY_TECHNIQUE  VELOCITY_TECHNIQUE_TREE
#define VELOCITY_TECHNIQUE  VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL




static const float  FourPi              = 4.0f * 3.1415926535897932384626433832795f ;
static const float  OneOverFourPi       = 1.0f / FourPi ;
static const float  FourPiOver3         = FourPi / 3.0f ;
// sAvoidSingularity is added to denominators, to avoid divide-by-zero.
// Ideally, sAvoidSingularity should be small enough not to measurably impact legitimate values,
// (i.e. should be smaller than epsilon times the value being divided)
// and should also be larger than a denormalized floating point value (which sometimes become zero).
//static const float  sAvoidSingularity   = FLT_MIN                         ;   // Too small
//static const float  sAvoidSingularity   = FLT_MIN * 4398046511104.0f      ;   // Just barely large enough
//static const float  sAvoidSingularity   = powf( FLT_MIN , 1.0f / 2.0f )   ;
static const float  sAvoidSingularity   = powf( FLT_MIN , 1.0f / 3.0f )     ;   // Probably larger than necessary




#define VORTON_ACCUMULATE_VELOCITY_private( vVelocity , vPosQuery , mPosition , mVorticity , mRadius )              \
{                                                                                                                   \
    const Vec3          vNeighborToSelf     = vPosQuery - mPosition ;                                               \
    const float         radius2             = mRadius * mRadius ;                                                   \
    const float         dist2               = vNeighborToSelf.Mag2() + sAvoidSingularity ;                          \
    const float         oneOverDist         = finvsqrtf( dist2 ) ;                                                  \
    const Vec3          vNeighborToSelfDir  = vNeighborToSelf * oneOverDist ;                                       \
    /* If the reciprocal law is used everywhere then when 2 vortices get close, they tend to jettison. */           \
    /* Mitigate this by using a linear law when 2 vortices get close to each other. */                              \
    const float         distLaw             = ( dist2 < radius2 )                                                   \
                                                ?   /* Inside vortex core */                                        \
                                                ( oneOverDist / radius2 )                                           \
                                                :   /* Outside vortex core */                                       \
                                                ( oneOverDist / dist2 ) ;                                           \
    vVelocity +=  OneOverFourPi * ( FourPiOver3 * radius2 * mRadius ) * mVorticity ^ vNeighborToSelf * distLaw ;    \
}

#define VORTON_ACCUMULATE_VELOCITY( vVelocity , vPosQuery , vorton ) VORTON_ACCUMULATE_VELOCITY_private( vVelocity , vPosQuery , vorton.mPosition , vorton.mVorticity , vorton.mRadius )


// Types --------------------------------------------------------------

/*! \brief Vortex particle
*/
class Vorton
{
    public:
        /*! \brief Construct a vortex particle
        */
        Vorton()
            : mPosition( 0.0f , 0.0f , 0.0f )
            , mVorticity( 0.0f , 0.0f , 0.0f )
            , mRadius( 0.0f )
            , mVelocity( 0.0f , 0.0f , 0.0f )
        {
        }

        Vorton( const Vec3 & vPos , const Vec3 & vVort , float fRadius = 0.0f )
            : mPosition( vPos )
            , mVorticity( vVort )
            , mRadius( fRadius )
            , mVelocity( 0.0f , 0.0f , 0.0f )
        {
        }


        /*! \brief Compute velocity induced by a tiny vortex element (a vorton)

            \param vVelocity - (in/out) variable in which to accumulate velocity

            \param vPosQuery - position where we want to know velocity

            \note mRadius currently serves double-duty for two things
                    which should probably be kept separate.
                    One is the radius of the finite-size vorton,
                    where the vorticity distribution inside the radius
                    is finite, to avoid evaluating a singularity.
                    The other is the volume of the "infinitesimal"
                    volume element, used to compute a contribution
                    to a velocity field.

        */
        void AccumulateVelocity( Vec3 & vVelocity , const Vec3 & vPosQuery ) const
        {
            VORTON_ACCUMULATE_VELOCITY_private( vVelocity , vPosQuery , mPosition , mVorticity , mRadius ) ;
        }
        

        /*! \brief Compute vorticity required to obtain a given velocity, due to a single vorton at a given position.

            This assigns the vorticity

                w = 4 Pi r^2 v / volumeElement

            where

                r is the distance from the vorton (which here is also the radius of the vorton)
                v is the velocity induced by the vorton
                volumeElement is the volume occupied by the vorton
                w_hat is r_hat cross v_hat.

            This assumes v and r are orthogonal, so this is a very special-purpose
            routine.

            This routine also assumes this vorton's position and radius are where they need to be.

        */
        void AssignByVelocity( const Vec3 & vQueryPosition , const Vec3 & velocity )
        {
            const Vec3  vPosRelative    = vQueryPosition - mPosition ;
            const float dist            = vPosRelative.Magnitude() ;
            mVorticity = FourPi * dist * vPosRelative ^ velocity / GetVolume() ;
        }


        /*! \brief Return volume of this vorton

            \return Volume of this vorton
        */
        float GetVolume( void ) const
        {
            return FourPiOver3 * mRadius * mRadius * mRadius ;
        }

        Vec3 GetAngularVelocity( void ) const
        {
            return mVorticity * 0.5f ;
        }


        Vec3    mPosition           ;   ///< Position (in world units) of center of vortex particle
        Vec3    mVorticity          ;   ///< Vorticity of vortex particle
        float   mRadius             ;   ///< Radius of vortex particle
        Vec3    mVelocity           ;   ///< Velocity of this vorton -- used to cache value obtained during advection, to optimize collision response.
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------


#endif
