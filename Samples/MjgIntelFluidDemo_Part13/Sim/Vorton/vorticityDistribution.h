/*! \file vorticityDistribution.h

    \brief Vorticity distributions for initializing fluid flow fields in diagnostic tests

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
#ifndef VORTICITY_DISTRIBUTION_H
#define VORTICITY_DISTRIBUTION_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "Core/wrapperMacros.h"
#include "vorton.h"

// Macros --------------------------------------------------------------

static const float  TwoPi                           = 2.0f * Pi ;
extern const float  sNegligibleEnstrophyThreshold ;

// Types --------------------------------------------------------------

/** Abstract base class (interface) for defining a vorticity distribution.
*/
class IVorticityDistribution
{
    public:
        /// Return size of domain required to fit this ball.
        virtual Vec3 GetDomainSize( void ) const = 0 ;

        /// Assign vorton properties consistent with this distribution.
        virtual void AssignVorton( Vec3 & vorticity , float & density , const Vec3 & position , const Vec3 & vCenter ) const = 0 ;
} ;




/** Specify vorticity in the shape of a vortex ring.

    The output of this routine is not guaranteed to be solenoidal.

    \see JetRing for a solenoidal formulation.

*/
class VortexRing : public IVorticityDistribution
{
    public:
        /** Initialize parameters for a vortex ring.

            The vorticity profile resulting from this is such that the vorticity is in [0,1].

            \param fRadius      Radius of vortex ring core.

            \param fThickness   Thickness of vortex ring, i.e. radius of annular core.

            \param vDirection   Vector of ring axis, also vector of propagation.
        */
        VortexRing( const float & fRadius , const float & fThickness , const Vec3 & vDirection )
            : mRadius( fRadius )
            , mThickness( fThickness )
            , mDirection( vDirection )
        {
        }

        /// Return size of domain required to fit this ball.
        virtual Vec3 GetDomainSize( void ) const
        {
            const float boxSideLength   = 2.f * ( mRadius + mThickness ) ;    // length of side of virtual cube
            return Vec3( 1.0f , 1.0f , 1.0f ) * boxSideLength ;
        }

        /// Assign vorton properties consistent with this distribution.
        virtual void AssignVorton( Vec3 & vorticity , float & density , const Vec3 & position , const Vec3 & vCenter ) const
        {
            const  Vec3     vFromCenter     = position - vCenter ;              // displacement from ring center to vorton position
            const  float    tween           = vFromCenter * mDirection ;        // projection of position onto axis
            const  Vec3     vPtOnLine       = vCenter + mDirection * tween ;    // closest point on axis to vorton position
            Vec3            vRho            = position - vPtOnLine ;            // direction radially outward from annulus core
            const  float    rho             = vRho.Magnitude() ;                // distance from axis
            const  float    distAlongDir    = mDirection * vFromCenter ;        // distance along axis of vorton position
            const  float    radCore         = sqrtf( POW2( rho - mRadius ) + POW2( distAlongDir ) ) ; // distance from annular core
            if( radCore < mThickness )
            {   // Probe position is inside vortex core.
                const  float    vortProfile     = radCore < mThickness ? 0.5f * ( cosf( Pi * radCore / mThickness ) + 1.0f ) : 0.0f ;
                const  float    vortPhi         = vortProfile ;
                Vec3            rhoHat          = vRho ;                        // direction radially away from annular core
                rhoHat.NormalizeFast() ;
                Vec3            phiHat          = mDirection ^ rhoHat ;         // direction along annular core
                vorticity                       = vortPhi * phiHat ;
            }
            else
            {
                vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
            }
            density = 0.0f ;
        }

        float   mRadius     ;   ///< Radius of vortex ring core.
        float   mThickness  ;   ///< Thickness of vortex ring, i.e. radius of annular core.
        Vec3    mDirection  ;   ///< vector of ring axis, also vector of propagation.
} ;




/** Specify vorticity in a specific shape of a vortex ring.

    The vorticity specified by this class derives from taking the curl of
    a localized jet.  The vorticity is therefore guaranteed to be solenoidal,
    to within the accuracy the discretization affords.

    \see VortexRing for a direct formulation

*/
class JetRing : public IVorticityDistribution
{
    public:
        /** Initialize parameters for a vortex ring (using a different formula from the other).

            The vorticity profile resulting from this is such that the induced velocity is in [0,1].

            \param fRadiusSlug - radius of central region where velocity is constant

            \param fThickness - thickness of vortex ring, i.e. radius of annular core

            \param vDirection - vector of ring axis, also vector of propagation

        */
        JetRing( const float & fRadiusSlug , const float & fThickness , const Vec3 & vDirection )
            : mRadiusSlug( fRadiusSlug )
            , mThickness( fThickness )
            , mRadiusOuter( mRadiusSlug + mThickness )
            , mDirection( vDirection )
        {
        }

        /// Return size of domain required to fit this ball.
        virtual Vec3 GetDomainSize( void ) const
        {
            const float boxSideLength   = 2.f * ( mRadiusOuter ) ;    // length of side of virtual cube
            return Vec3( 1.0f , 1.0f , 1.0f ) * boxSideLength ;
        }

        /// Assign vorton properties consistent with this distribution.
        virtual void AssignVorton( Vec3 & vorticity , float & density , const Vec3 & position , const Vec3 & vCenter ) const
        {
            const  Vec3     vFromCenter     = position - vCenter ;              // displacement from ring center to vorton position
            const  float    tween           = vFromCenter * mDirection ;        // projection of position onto axis
            const  Vec3     vPtOnLine       = vCenter + mDirection * tween ;    // closest point on axis to vorton position
            Vec3            vRho            = position - vPtOnLine ;            // direction radially outward from annulus core
            const  float    rho             = vRho.Magnitude() ;                // distance from axis
            const  float    distAlongDir    = mDirection * vFromCenter ;        // distance along axis of vorton position
            if( ( rho < mRadiusOuter ) && ( rho > mRadiusSlug ) )
            {   // Probe position is inside jet region.
                const  float    streamwiseProfile   = ( fabsf( distAlongDir ) < mRadiusSlug ) ? 0.5f * ( cos( Pi * distAlongDir / mRadiusSlug ) + 1.0f ) : 0.0f ;
                const  float    radialProfile       = sin( Pi * ( rho - mRadiusSlug ) / mThickness ) ;
                const  float    vortPhi             = streamwiseProfile * radialProfile * Pi / mThickness ;
                Vec3            rhoHat              = vRho ;                    // direction radially away from annular core
                rhoHat.NormalizeFast() ;
                Vec3            phiHat              = mDirection ^ rhoHat ;  // direction along annular core
                vorticity                           = vortPhi * phiHat ;
                density = 0.0001f ; // Just enough density to register for tracer placement
            }
            else
            {
                vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
                density = 0.0f ;
            }
        }

        float   mRadiusSlug     ;   ///< Radius of central region of jet, where velocity is uniform.
        float   mThickness      ;   ///< Thickness of region outside central jet, where velocity decays gradually
        float   mRadiusOuter    ;   ///< Radius of jet, including central region and gradial falloff.
        Vec3    mDirection      ;   ///< Direction of jet.
} ;




/** Specify vorticity in the shape of a vortex tube.

    The vorticity specified by this class abruptly terminates
    at the domain boundaries and therefore violates characteristics
    of true vorticity.

*/
class VortexTube : public IVorticityDistribution
{
    public:
        /** Initialize parameters for a vortex tube with spanwise variation.

            \param fDiameter    Characteristic thickness of tube.

            \param fVariation   Amplitude of variation of thickness.
                    Choose values in [0,0.2].

            \param fWidth       Spanwise width of domain.

            \param iPeriods     Number of spanwise periods in the variation.

            \param iLocation    Discrete location, used to create tubes in various orientations relative to each other, such as crossed tubes.

        */
        VortexTube( float fDiameter , float fVariation , float fWidth , int iPeriods , int iLocation )
            : mRadius( 0.5f * fDiameter )
            , mVariation( fVariation )
            , mWidth( fWidth )
            , mWavenumber( float( iPeriods ) )
            , mLocation( iLocation )
        {
        }

        /// Return size of domain required to fit this ball.
        virtual Vec3 GetDomainSize( void ) const
        {
            return Vec3( 8.0f * mRadius , mWidth , 8.0f * mRadius ) ;
        }

        /// Assign vorton properties consistent with this distribution.
        virtual void AssignVorton( Vec3 & vorticity , float & density , const Vec3 & position , const Vec3 & vCenter ) const
        {
            if( 0 == mLocation )
            {
                const Vec3  posRel      = position - vCenter ;
                const float rho         = sqrtf( POW2( posRel.x ) + POW2( posRel.z ) ) ;
                const float modulation  = 1.0f - mVariation * ( cosf( TwoPi * mWavenumber * posRel.y / mWidth ) - 1.0f ) ;
                const float radiusLocal = mRadius * modulation ;
                if( rho < radiusLocal )
                {   // Position is inside vortex tube.
                    const float vortY = 0.5f * ( cosf( Pi * rho / radiusLocal ) + 1 ) ;
                    vorticity = Vec3( 0.0f , vortY , 0.0f ) ;
                }
                else
                {   // Position is outside vortex tube.
                    vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
                }
            }
            else if( 1 == mLocation )
            {
                const Vec3  posRel      = position - vCenter - Vec3( 0.0f , 0.0f , 1.0f * mRadius ) ;
                const float rho         = sqrtf( POW2( posRel.x ) + POW2( posRel.z ) ) ;
                const float modulation  = 1.0f - mVariation * ( cosf( TwoPi * mWavenumber * posRel.y / mWidth ) - 1.0f ) ;
                const float radiusLocal = mRadius * modulation ;
                if( rho < radiusLocal )
                {   // Position is inside vortex tube.
                    const float vortY = 0.5f * ( cosf( Pi * rho / radiusLocal ) + 1 ) ;
                    vorticity = Vec3( 0.0f , vortY , 0.0f ) ;
                }
                else
                {   // Position is outside vortex tube.
                    vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
                }
            }
            else if( -1 == mLocation )
            {
                const Vec3  posRel      = position - vCenter - Vec3( 0.0f , 0.0f , -1.0f * mRadius ) ;
                const float rho         = sqrtf( POW2( posRel.y ) + POW2( posRel.z ) ) ;
                const float modulation  = 1.0f - mVariation * ( cosf( TwoPi * mWavenumber * posRel.x / mWidth ) - 1.0f ) ;
                const float radiusLocal = mRadius * modulation ;
                if( rho < radiusLocal )
                {   // Position is inside vortex tube.
                    const float vortX = 0.5f * ( cosf( Pi * rho / radiusLocal ) + 1 ) ;
                    vorticity = Vec3( vortX , 0.0f , 0.0f ) ;
                }
                else
                {   // Position is outside vortex tube.
                    vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
                }
            }
            density = FLT_EPSILON ; // Slightly positive so that thermal diffusion doesn't cause a sudden global jump.
        }

        float   mRadius     ;   ///< Maximum radius of vortex tube
        float   mVariation  ;   ///< Amplitude of radius variation
        float   mWidth      ;   ///< Spanwise width of domain
        float   mWavenumber ;   ///< Number of full periods of spanwise variation to fit in domain
        int     mLocation   ;   ///< HACK: one of a few hard-coded locations of tube
} ;




/** Specify a ball of fluid with a given density deviation.
*/
class DensityBall : public IVorticityDistribution
{
    public:
        /** Initialize ball of fluid with the given density deviation-about-ambient.
        */
        DensityBall( float fRadius , float fDensityDeviation )
            : mRadius( fRadius )
            , mDensityDeviation( fDensityDeviation )
        {
        }

        /// Return size of domain required to fit this ball.
        virtual Vec3 GetDomainSize( void ) const
        {
            const float boxSideLength   = 4.0f * mRadius ;    // length of side of virtual cube
            return Vec3( 1.0f , 1.0f , 1.0f ) * boxSideLength ;
        }

        /// Assign vorton properties consistent with this distribution.
        virtual void AssignVorton( Vec3 & vorticity , float & densityDeviation , const Vec3 & position , const Vec3 & vCenter ) const
        {
            const Vec3  posRel  = position - vCenter ;  // displacement from ball center to query position
            const float rho     = sqrtf( POW2( posRel.x ) + POW2( posRel.y ) + POW2( posRel.z ) ) ;

            if( rho < mRadius )
            {   // Probe position is inside ball.
                vorticity = RandomSpread( Vec3( 0.001f , 0.001f , 0.001f ) ) ;  // Enough vorticity to emit a vorton, not enough to influence motion
                // density = mDensityDeviation * ( mRadius - rho ) ;
                densityDeviation = mDensityDeviation ;
            }
            else
            {
                vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
                densityDeviation = 0.0f ;
            }
        }

        float   mRadius             ;   ///< Radius of ball of fluid
        float   mDensityDeviation   ;   ///< Density deviation-about-ambient of fluid in ball
} ;




/** Specify a box of fluid with a given density deviation.
*/
class DensityBox : public IVorticityDistribution
{
    public:
        /** Initialize box of fluid with a given density deviation-about-ambient.
        */
        DensityBox( const Vec3 & vDimensions , float fDensityDeviation )
            : mDimensions( vDimensions )
            , mDensityDeviation( fDensityDeviation )
        {
        }

        /// Return size of domain required to fit this distribution.
        virtual Vec3 GetDomainSize( void ) const
        {
            return mDimensions ;
        }

        /// Assign vorton properties consistent with this distribution.
        virtual void AssignVorton( Vec3 & vorticity , float & density , const Vec3 & position , const Vec3 & vCenter ) const
        {
            const Vec3  posRel  = position - vCenter ;  // displacement from box center to query position
            const bool inside =     ( fabsf( posRel.x ) <= fabsf( mDimensions.x ) )
                                &&  ( fabsf( posRel.y ) <= fabsf( mDimensions.y ) )
                                &&  ( fabsf( posRel.z ) <= fabsf( mDimensions.z ) ) ;

            if( inside )
            {   // Probe position is inside box.
                vorticity = RandomSpread( Vec3( 0.001f , 0.001f , 0.001f ) ) ;  // Enough vorticity to emit a vorton, not enough to influence motion
                density = mDensityDeviation ;
            }
            else
            {
                vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
                density = 0.0f ;
            }
        }

        Vec3    mDimensions         ;   ///< Dimensions of box of fluid region
        float   mDensityDeviation   ;   ///< Density deviation-about-ambient of fluid in ball
} ;




/** Specify vorticity in the shape of a vortex sheet, a.k.a. a shear layer.

    The vorticity specified by this class abruptly terminates
    at the domain boundaries and therefore violates characteristics
    of true vorticity.

*/
class VortexSheet : public IVorticityDistribution
{
    public:
        /** Initialize parameters for a vortex sheet with spanwise variation.

            \param fThickness - characteristic vertical thickness of shear layer

            \param fVariation - Amplitude of variation of thickness.
                    Choose values in [0,0.2].

            \param fWidth - spanwise width of shear layer

        */
        VortexSheet( const float & fThickness , const float & fVariation , const float & fWidth )
            : mThickness( fThickness )
            , mVariation( fVariation )
            , mWidth( fWidth )
        {
        }

        /// Return size of domain required to fit this distribution.
        virtual Vec3 GetDomainSize( void ) const
        {
            return Vec3( 14.0f * mThickness , mWidth , 14.0f * mThickness ) ;
        }

        /// Assign vorton properties consistent with this distribution.
        virtual void AssignVorton( Vec3 & vorticity , float & density , const Vec3 & position , const Vec3 & /* vCenter */ ) const
        {
            const float yOverWidth = position.y / mWidth ;
            const float d = 1.0f - 0.5f * mVariation * ( cosf( TwoPi * yOverWidth ) - 1.0f ) ;
            const float zOverD = position.z / d ;
            vorticity.x = 0.0f ;
            const float s = sechf( zOverD ) ;
            vorticity.y = s * s / d ;
            const float t = tanhf( zOverD ) ;
            vorticity.z = t * t * Pi * mVariation * zOverD / ( mWidth * d ) * sinf( TwoPi * yOverWidth ) ;
            if( vorticity.Mag2() < 0.01f )
            {   // When vorticity is small, force it to zero, to keep number of vortons down.
                vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
            }
            density = 0.0f ;
        }

        float   mThickness  ;   ///< Characteristic vertical thickness of shear layer.
        float   mVariation  ;   ///< Amplitude of variation of thickness.
        float   mWidth      ;   ///< Spanwise width of shear layer.
} ;




/** Specify a random field of vorticity.

    The vorticity specified by this class abruptly terminates
    at the domain boundaries and therefore violates characteristics
    of true vorticity.

*/
class VortexNoise : public IVorticityDistribution
{
    public:
        /** Initialize parameters for vortex noise.

            \param vBox Dimensions (in world units) of box with noisy vorticity

        */
        VortexNoise( const Vec3 & vBox )
            : mBox( vBox )
            , mAmplitude( 1.0f , 1.0f , 1.0f )
        {
            if( 0.0f == vBox.z )
            {   // Domain is 2D (in XY plane).
                // Make vorticity purely vertical.
                mAmplitude = Vec3(  0.0f , 0.0f , 1.0f ) ;
            }
        }

        /// Return size of domain required to fit this ball.
        virtual Vec3 GetDomainSize( void ) const
        {
            return mBox ;
        }

        /// Assign vorton properties consistent with this distribution.
        virtual void AssignVorton( Vec3 & vorticity , float & density , const Vec3 & /* position */ , const Vec3 & /* vCenter */ ) const
        {
            vorticity = RandomSpread( mAmplitude ) ;
            density = FLT_EPSILON ; // Slightly positive so that thermal diffusion doesn't cause a sudden global jump.
        }

        Vec3    mBox        ;   ///< Dimensions (in world units) of box to contain noise.
        Vec3    mAmplitude  ;   ///< Amplitude of noise.
} ;

// Public variables --------------------------------------------------------------


// Public functions --------------------------------------------------------------

extern void AddCornerVortons( Vector<Vorton> & vortons , const Vec3 & vMin , const Vec3 & vMax ) ;
extern void AssignVortons( Vector<Vorton> & vortons , float fMagnitude , unsigned numVortonsMax , const IVorticityDistribution & vorticityDistribution , Vec3 translation = Vec3( 0.0f , 0.0f , 0.0f ) ) ;

#endif
