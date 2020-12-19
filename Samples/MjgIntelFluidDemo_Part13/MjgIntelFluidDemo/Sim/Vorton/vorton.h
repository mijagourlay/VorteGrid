/** \file vorton.h

    \brief Vortex particle

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
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef VORTON_H
#define VORTON_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "Core/wrapperMacros.h"

#include "Sim/Vorton/particle.h"

// Macros --------------------------------------------------------------

/** Techniques for computing velocity from vorticity.
*/
#define VELOCITY_TECHNIQUE_DIRECT               'VTBF'  ///< Slow O(N*N), accurate direct summation
#define VELOCITY_TECHNIQUE_TREE                 'VTTC'  ///< Faster O(N log N), moderately accurate treecode
#define VELOCITY_TECHNIQUE_MONOPOLES            'VTMP'  ///< Even faster O(N log N), less accurate monopole
#define VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL 'VTPG'  ///< Differential approach: Use a finite-difference Poisson solver.  O(N) when USE_MULTI_GRID is enabled, O(N^3/2) otherwise.

/// Which technique to use to compute velocity from vorticity.
//#define VELOCITY_TECHNIQUE  VELOCITY_TECHNIQUE_DIRECT
#define VELOCITY_TECHNIQUE  VELOCITY_TECHNIQUE_TREE
//#define VELOCITY_TECHNIQUE  VELOCITY_TECHNIQUE_MONOPOLES
//#define VELOCITY_TECHNIQUE  VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL

/** Variations on integral techniques for computing velocity from vorticity.

    Both of these variations can also alter how TREE and MONOPOLE techniques,
    but in those cases, the results are worse than with them disabled.

    When using the DIRECT method, enabling COMPUTE_VELOCITY_AT_VORTONS results
    in the most accurate motion, and also corresponds closest to the traditional
    direct method.  It is useful as a benchmark to compare other techniques,
    but it departs significantly from the TREE and MONOPOLE methods in how velocity
    is calculated.  If you want to compare overall accuracy, enable COMPUTE_VELOCITY_AT_VORTONS,
    but if you want to compare TREE and DIRECT to diagnose specific aspects of the
    velocity-from-vorticity algorithm, disable COMPUTE_VELOCITY_AT_VORTONS and USE_PARTICLE_IN_CELL.
*/
#if ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT )
    /// Calculate velocity at vortons instead of at gridpoints.
    #define COMPUTE_VELOCITY_AT_VORTONS 1

    /// Temporarily reposition vortons to gridpoints during the velocity-from-vorticity calculation.
    #define USE_PARTICLE_IN_CELL        0
#endif

#if ( ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) )
    #define COMPUTE_VELOCITY_AT_VORTONS 0

    #if COMPUTE_VELOCITY_AT_VORTONS
        /** Use original vortons rather than supervorton for base layer.
        */
        #define USE_ORIGINAL_VORTONS_IN_BASE_LAYER 1
    #else
        /** Automatically smooth velocity profile due to each vorton.
        */
        #define ENABLE_AUTO_MOLLIFICATION   1
    #endif

    /** Increase distance threshold for traversing tree.
        Results in slower but more accurate calcualation.  For testing only.
    */
    #define AVOID_CENTERS               0
#endif



static const float  FourPi      = 4.0f * Pi ;
static const float  FourPiOver3 = FourPi / 3.0f ;
static const float  FourPiOver6 = FourPi / 6.0f ;
static const float  TwoThirds   = 2.0f / 3.0f   ;

/** Compute velocity from vorticity using Biot-Savart law for a vortex particle with finite core size.

    \param vVelocity        (in/out) Variable into which to accumulate velocity induced by the given vorton.
                            The velocity contribution due to the given vorton is added to the input velocity value;
                            otherwise the input velocity value is not used.

    \param vPosQuery        (in) Position at which to compute velocity.

    \param mPosition        (in) Vorton position.

    \param mAngularVelocity (in) Vorton angular velocity.

    \param mSize            (in) Vorton diameter.

    \param mSpreadingRangeFactor (in) Amount by which to (effectively) scale vorton radius.

    \param mSpreadingCirculationFactor (in) Amount by which to (effectively) scale vorton vorticity.
                                            Must be 1/(mSpreadingRangeFactor^3).

*/
#define VORTON_ACCUMULATE_VELOCITY_private( vVelocity , vPosQuery , mPosition , mAngularVelocity , mSize , mSpreadingRangeFactor , mSpreadingCirculationFactor )    \
{                                                                                                                       \
    const Vec3          vNeighborToSelf = vPosQuery - mPosition ;                                                       \
    const float         radius          = mSize * 0.5f * mSpreadingRangeFactor ;                                        \
    const float         radius2         = radius * radius ;                                                             \
    const float         dist2           = vNeighborToSelf.Mag2() ;                                                      \
    /* If the reciprocal law is used everywhere then when 2 vortices get close, they tend to jettison. */               \
    /* Mitigate this by using a linear law when 2 vortices get close to each other.                    */               \
    /* This is called "regularizing" the vortex.                                                       */               \
    const float         distLaw         = ( dist2 < radius2 )                                                           \
                                            ?   /* Inside vortex core */                                                \
                                            ( 1.0f / ( radius2 * radius ) )                                             \
                                            :   /* Outside vortex core */                                               \
                                            ( finvsqrtf( dist2 ) / dist2 ) ;                                            \
    /* The formula for velocity is                                                          */                          \
    /*  OneOverFourPi * ( FourPiOver3 * radius^3 ) * vorticity ^ vNeighborToSelf * distLaw  */                          \
    /* but the FourPi's cancel, vorticity=mAngularVelocity*2.                               */                          \
    const Vec3  velocityContribution = TwoThirds * radius2 * radius * mAngularVelocity ^ vNeighborToSelf * distLaw ;    \
    vVelocity +=  velocityContribution * mSpreadingCirculationFactor ;                                                  \
}

/// Wrapper for VORTON_ACCUMULATE_VELOCITY_private.
#define VORTON_ACCUMULATE_VELOCITY( vVelocity , vPosQuery , vorton ) VORTON_ACCUMULATE_VELOCITY_private( vVelocity , vPosQuery , vorton.mPosition , vorton.mAngularVelocity , vorton.mSize , mSpreadingRangeFactor , mSpreadingCirculationFactor )


// Types --------------------------------------------------------------


/** Vortex particle, a particle that generates a circulating velocity field.

    \note   This class has no additional data beyond its base "Particle" class,
            but simply serves to add more functionality and document the intention
            of its use; a Vorton is a Particle which generates a velocity field.

*/
class Vorton : public Particle
{
    public:
        /// Construct a vortex particle.
        Vorton()
            : Particle()
        {
        }

        /** Construct a vortex particle.

            \param vPos     Position of vortex particle.

            \param vVort    Vorticity of vortex particle, which is twice its angular velocity.

            \param fRadius  Radius of vortex particle, which is half its diameter (i.e. half its "size").

            \see Particle::Particle
        */
        Vorton( const Vec3 & vPos , const Vec3 & vVort , float fRadius = 0.0f )
            : Particle( vPos , vVort * 0.5f , 2.0f * fRadius )
        {
            SetRadius( fRadius ) ;
        }


        // Use the compiler-generated copy constructor


        /** Compute velocity induced by a tiny vortex element (a vorton).

            \param vVelocity - (in/out) variable in which to accumulate velocity.

            \param vPosQuery - position where we want to know velocity.

        */
        void AccumulateVelocity( Vec3 & vVelocity , const Vec3 & vPosQuery ) const
        {
            VORTON_ACCUMULATE_VELOCITY_private( vVelocity , vPosQuery , mPosition , mAngularVelocity , mSize , 1.0f , 1.0f ) ;
        }
        

        /** Compute vorticity required to obtain a given velocity, due to a single vorton at a given position.

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
            SetVorticity( FourPi * dist * vPosRelative ^ velocity / GetVolume() ) ;
        }


        // For reference: Vec3 GetAngularVelocity( void ) const { return mVorticity * 0.5f ; }


        /** Get vorticity from angular velocity.
            \return Vorticity
            \see SetVorticity
        */
        Vec3 GetVorticity( void ) const
        {
            return mAngularVelocity * 2.0f ;
        }


        /** Assign vorticity as angular velocity.

            This class stores vorticity in the mAngularVelocity member of its parent class (Particle).
            The relationship between then is vorticity=2*angularVelocity.

            \see GetVorticity
        */
        void SetVorticity( Vec3 vort )
        {
            mAngularVelocity = vort * 0.5f ;
        }


        /// Compute sum of vorticity for all given particles.
        static Vec3 ComputeTotalVorticity( const Vector< Vorton > & particles )
        {
            Vec3 angVelSum( 0.0f , 0.0f , 0.0f ) ;
            const size_t & numParticles = particles.Size() ;
            for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
            {
                const Particle & pcl = particles[ iPcl ] ;
                angVelSum += pcl.mAngularVelocity ;
            }
            return angVelSum * 2.0f ;
        }


        /// Compute sum of kinetic energy for all given particles.
        static float ComputeTotalKineticEnergy( const Vector< Vorton > & particles )
        {
            float keSum = 0.0f ;
        #if 0
            const size_t & numParticles = particles.Size() ;
            for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
            {
                const Particle & pclI = particles[ iPcl ] ;
                if( pclI.IsAlive() )
                {   // This particle is valid.
                    const float size  = pclI.mSize ;
                    const float rad   = size * 0.5f ;
                    const float r2    = rad * rad ;
                    const float r2_15 = 1.5f * r2 ;
                    for( size_t jPcl = iPcl ; jPcl < numParticles ; ++ jPcl )
                    {
                        const Particle &    pclJ    = particles[ jPcl ] ;
                        if( pclJ.IsAlive() )
                        {   // This particle is valid.
                            const Vec3          dist    = pclI.mPosition - pclJ.mPosition ;
                            const float         dist2   = dist.Mag2() ;
                            const float         num     = dist2 + r2_15 ;
                            const float         den     = powf( dist2 + r2 , 1.5f ) ;
                            const float         coef    = pclI.mAngularVelocity * pclJ.mAngularVelocity ;
                            const float         ke      = num * coef / den ;
                            keSum += ke ;
                        }
                    }
                }
            }
        #else
            (void) particles ; // Avoid "unreferenced formal parameter" warning.
        #endif
            return keSum ;
        }

} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------


#endif
