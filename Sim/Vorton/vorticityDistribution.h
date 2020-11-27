/*! \file vorticityDistribution.h

    \brief Vorticity distributions for initializing fluid flow fields in diagnostic tests

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef VORTICITY_DISTRIBUTION_H
#define VORTICITY_DISTRIBUTION_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "wrapperMacros.h"
#include "vorton.h"

// Macros --------------------------------------------------------------

static const float  Pi              = 3.1415926535897932384626433832795f ;
static const float  TwoPi           = 2.0f * Pi ;

// Types --------------------------------------------------------------

/*! \brief Abstract base class (interface) for defining a vorticity distribution
*/
class IVorticityDistribution
{
    public:
        virtual Vec3 GetDomainSize( void ) const = 0 ;
        virtual void AssignVorticity( Vec3 & vorticity , const Vec3 & position , const Vec3 & vCenter ) const = 0 ;
} ;




/*! \brief Specify vorticity in the shape of a vortex ring.

    The output of this routine is not guaranteed to be solenoidal.

    \see JetRing for a solenoidal formulation.

*/
class VortexRing : public IVorticityDistribution
{
    public:
        /*! \brief Initialize parameters for a vortex ring

            The vorticity profile resulting from this is such that the vorticity is in [0,1].

            \param fRadius - radius of vortex ring core

            \param fThickness - thickness of vortex ring, i.e. radius of annular core

            \param vDirection - vector of ring axis, also vector of propagation
        */
        VortexRing( const float & fRadius , const float & fThickness , const Vec3 & vDirection )
            : mRadius( fRadius )
            , mThickness( fThickness )
            , mDirection( vDirection )
        {
        }

        virtual Vec3 GetDomainSize( void ) const
        {
            const float boxSideLength   = 2.f * ( mRadius + mThickness ) ;    // length of side of virtual cube
            return Vec3( 1.0f , 1.0f , 1.0f ) * boxSideLength ;
        }

        virtual void AssignVorticity( Vec3 & vorticity , const Vec3 & position , const Vec3 & vCenter ) const
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
                rhoHat.Normalize() ;
                Vec3            phiHat          = mDirection ^ rhoHat ;         // direction along annular core
                vorticity                       = vortPhi * phiHat ;
            }
            else
            {
                vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
            }
        }

        float   mRadius     ;
        float   mThickness  ;
        Vec3    mDirection  ;
} ;




/*! \brief Specify vorticity in the shape of a vortex ring.

    The vorticity specified by this class derives from taking the curl of
    a localized jet.  The vorticity is therefore guaranteed to be solenoidal,
    to within the accuracy the discretization affords.

    \see VortexRing

*/
class JetRing : public IVorticityDistribution
{
    public:
        /*! \brief Initialize parameters for a vortex ring (using a different formula from the other).

            The vorticity profile resulting from this is such that the induced velocity is in [0,1].

            \param fRadiusSlug - radius of central region where velocity is constant

            \param fThickness - thickness of vortex ring, i.e. radius of annular core

            \param vDirection - vector of ring axis, also vector of propagation

            \param fSpeed   - speed of slug

        */
        JetRing( const float & fRadiusSlug , const float & fThickness , const Vec3 & vDirection )
            : mRadiusSlug( fRadiusSlug )
            , mThickness( fThickness )
            , mRadiusOuter( mRadiusSlug + mThickness )
            , mDirection( vDirection )
        {
        }

        virtual Vec3 GetDomainSize( void ) const
        {
            const float boxSideLength   = 2.f * ( mRadiusOuter ) ;    // length of side of virtual cube
            return Vec3( 1.0f , 1.0f , 1.0f ) * boxSideLength ;
        }

        virtual void AssignVorticity( Vec3 & vorticity , const Vec3 & position , const Vec3 & vCenter ) const
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
                rhoHat.Normalize() ;
                Vec3            phiHat              = mDirection ^ rhoHat ;  // direction along annular core
                vorticity                           = vortPhi * phiHat ;
            }
            else
            {
                vorticity = Vec3( 0.0f , 0.0f , 0.0f ) ;
            }
        }

        float   mRadiusSlug     ;   ///< Radius of central region of jet, where velocity is uniform.
        float   mThickness      ;   ///< Thickness of region outside central jet, where velocity decays gradually
        float   mRadiusOuter    ;   ///< Radius of jet, including central region and gradial falloff.
        Vec3    mDirection      ;   ///< Direction of jet.
} ;




/*! \brief Specify vorticity in the shape of a vortex tube.

    The vorticity specified by this class abruptly terminates
    at the domain boundaries and therefore violates characteristics
    of true vorticity.

*/
class VortexTube : public IVorticityDistribution
{
    public:
        /*! \brief Initialize parameters for a vortex tube with spanwise variation

            \param fThickness - characteristic thickness of tube

            \param fVariation - Amplitude of variation of thickness.
                    Choose values in [0,0.2].

            \param fWidth - spanwise width of domain

            \param iPeriods - number of spanwise periods in the variation

        */
        VortexTube( const float & fDiameter , const float & fVariation , const float & fWidth , const int & iPeriods , const int & iLocation )
            : mRadius( 0.5f * fDiameter )
            , mVariation( fVariation )
            , mWidth( fWidth )
            , mWavenumber( float( iPeriods ) )
            , mLocation( iLocation )
        {
        }

        virtual Vec3 GetDomainSize( void ) const
        {
            return Vec3( 8.0f * mRadius , mWidth , 8.0f * mRadius ) ;
        }

        virtual void AssignVorticity( Vec3 & vorticity , const Vec3 & position , const Vec3 & vCenter ) const
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
        }

        float   mRadius     ;   ///< Maximum radius of vortex tube
        float   mVariation  ;   ///< Amplitude of radius variation
        float   mWidth      ;   ///< Spanwise width of domain
        float   mWavenumber ;   ///< Number of full periods of spanwise variation to fit in domain
        int     mLocation   ;   ///< HACK: one of a few hard-coded locations of tube
} ;




/*! \brief Specify vorticity in the shape of a vortex sheet, a.k.a. a shear layer.

    The vorticity specified by this class abruptly terminates
    at the domain boundaries and therefore violates characteristics
    of true vorticity.

*/
class VortexSheet : public IVorticityDistribution
{
    public:
        /*! \brief Initialize parameters for a vortex sheet with spanwise variation

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

        virtual Vec3 GetDomainSize( void ) const
        {
            return Vec3( 14.0f * mThickness , mWidth , 14.0f * mThickness ) ;
        }

        virtual void AssignVorticity( Vec3 & vorticity , const Vec3 & position , const Vec3 & vCenter ) const
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
        }

        float   mThickness  ;
        float   mVariation  ;
        float   mWidth      ;
} ;




/*! \brief Specify a random field of vorticity

    The vorticity specified by this class abruptly terminates
    at the domain boundaries and therefore violates characteristics
    of true vorticity.

*/
class VortexNoise : public IVorticityDistribution
{
    public:
        /*! \brief Initialize parameters for vortex noise

            \param shape - dimensions of box with noisy vorticity

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

        virtual Vec3 GetDomainSize( void ) const
        {
            return mBox ;
        }

        virtual void AssignVorticity( Vec3 & vorticity , const Vec3 & position , const Vec3 & vCenter ) const
        {
            vorticity = RandomSpread( mAmplitude ) ;
        }

        Vec3    mBox        ;
        Vec3    mAmplitude  ;
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------

extern void AddCornerVortons( Vector<Vorton> & vortons , const Vec3 & vMin , const Vec3 & vMax ) ;
extern void AssignVorticity( Vector<Vorton> & vortons , float fMagnitude , unsigned numVortonsMax , const IVorticityDistribution & vorticityDistribution ) ;

#endif
