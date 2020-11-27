/*! \file vorton.h

    \brief Vortex particle

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef VORTON_H
#define VORTON_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "wrapperMacros.h"

// Macros --------------------------------------------------------------

static const float  FourPi              = 4.0f * 3.1415926535897932384626433832795f ;
static const float  OneOverFourPi       = 1.0f / FourPi ;   
static const float  sAvoidSingularity   = powf( FLT_MIN , 1.0f / 3.0f ) ;   

#define VORTON_ACCUMULATE_VELOCITY_private( vVelocity , vPosQuery , mPosition , mVorticity , mRadius )      \
{                                                                                                           \
    const Vec3          vNeighborToSelf     = vPosQuery - mPosition ;                                       \
    const float         radius2             = mRadius * mRadius ;                                           \
    const float         dist2               = vNeighborToSelf.Mag2() + sAvoidSingularity ;                  \
    const float         oneOverDist         = finvsqrtf( dist2 ) ;                                          \
    const Vec3          vNeighborToSelfDir  = vNeighborToSelf * oneOverDist ;                               \
    /* If the reciprocal law is used everywhere then when 2 vortices get close, they tend to jettison. */   \
    /* Mitigate this by using a linear law when 2 vortices get close to each other. */                      \
    const float         distLaw             = ( dist2 < radius2 )                                           \
                                                ?   /* Inside vortex core */                                \
                                                ( oneOverDist / radius2 )                                   \
                                                :   /* Outside vortex core */                               \
                                                ( oneOverDist / dist2 ) ;                                   \
    vVelocity +=  OneOverFourPi * ( 8.0f * radius2 * mRadius ) * mVorticity ^ vNeighborToSelf * distLaw ;   \
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

        Vorton( const Vorton & that )
            : mPosition( that.mPosition )
            , mVorticity( that.mVorticity )
            , mRadius( that.mRadius )
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
            mVorticity = FourPi * dist * vPosRelative ^ velocity / ( 8.0f * mRadius * mRadius * mRadius ) ;
        }

        Vec3    mPosition	;   ///< Position (in world units) of center of vortex particle
        Vec3    mVorticity	;   ///< Vorticity of vortex particle
        float	mRadius		;	///< Radius of vortex particle
        Vec3    mVelocity   ;   ///< Velocity of this vorton -- used to cache value obtained during advection, to optimize collision response.
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------


#endif
