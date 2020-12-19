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
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.
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
        static const float sNaN ;

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


        /*! \brief Get radius from size
            \return the radius of this vorton
        */
        float GetRadius( void ) const
        {
            return mSize * 0.5f ;
        }


        /*! \brief Set size from radius
        */
        void SetRadius( float radius )
        {
            mSize = 2.0f * radius ;
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



        /*! \brief Return mass deviation of this particle.

            \return Mass deviation of this particle

            Particle mDensity represents a deviation from the ambient fluid
            density, instead of representing fluid density absolutely.  Such a
            treatment allows for placing particles judiciously instead of
            ubiquitously.

        */
        float GetMassDeviation() const
        {
            const float fMass   = mDensity * GetVolume() ;
            return fMass ;
        }


        /*! \brief Return mass of this particle.

            \param ambientDensity - density of fluid in the absence of particles.

            \see GetMassDeviation

            \return Mass of this particle

            \todo This code improperly uses mDensity or GetMassDeviation
                    when it should use GetMass. It results in "negative mass" in some calculation.
                    Study populating the density grid, and using the density grid.
                    Study AssignDensityFromGridSlice.
        */
        float GetMass( float ambientDensity ) const
        {
            const float fMass   = ( mDensity + ambientDensity ) * GetVolume() ;
            return fMass ;
        }


        /*! \brief Assign mass for this particle.

            \param ambientDensity - density of fluid in the absence of particles.

            \see GetMassDeviation
        */
        void SetMass( float mass , float ambientDensity )
        {
            const float density = mass / GetVolume() ;
            mDensity = density - ambientDensity ;
        }


        /*! \brief Return temperature of this particle.

            This assumes a Boussinesq approxomation where temperature is
            related to density.

            \param ambientDensity - density of fluid in the absence of particles.

            \see GetMassDeviation

            \return Temperature of this particle.
            
            \note   The formula for temperature comes from the formula for density under
                    the Bousinesq approximation:

                    density = ambientDensity * ( 1 - thermalExpansionCoefficient * ( temperature - ambientTemperature ) )
                --> density / ambientDensity = 1 - thermalExpansionCoefficient * (temperature-ambientTemperature)
                --> density / ( ambientDensity * (1-thermalExpansionCoefficient) ) = temperature - ambientTemperature
                --> temperature = density / (ambientDensity * (1-thermalExpansionCoefficient) ) + ambientTemperature

            For an ideal gas, thermalExpansionCoefficient=1/temperature so the above formula becomes
                density = ambientDensity * ambientTemperature / temperature
                --> temperature = ambientTemperature * ambientDensity / density

        */
        float GetTemperature( float ambientDensity ) const
        {
        #if 0
            const float ambientTemperature = 1.0f ;
            const float thermalExpansionCoefficient = 1.0f ;
            const float density = mDensity + ambientDensity ; // mDensity is actual density deviation about ambient.
            const float temperature = density / ( ambientDensity * ( 1 - thermalExpansionCoefficient ) ) + ambientTemperature ;
        #else
            // Assume ambientTemperature is 1.
            const float temperature = /* ambientTemperature * */ ambientDensity / ( ambientDensity + mDensity ) ;
        #endif
            return temperature ;
        }


        void SetTemperature( float ambientDensity , float temperature )
        {
            float density = ambientDensity /* * ambientTemperature */ / temperature ;
            mDensity = density - ambientDensity ;
        }


        void MarkDead()
        {
            mPosition.x          = Particle::sNaN ;
            mPosition.y          = Particle::sNaN ;
            mPosition.z          = Particle::sNaN ;
            mVelocity.x          = Particle::sNaN ;
            mVelocity.y          = Particle::sNaN ;
            mVelocity.z          = Particle::sNaN ;
            mOrientation.x       = Particle::sNaN ;
            mOrientation.y       = Particle::sNaN ;
            mOrientation.z       = Particle::sNaN ;
            mAngularVelocity.x   = Particle::sNaN ;
            mAngularVelocity.y   = Particle::sNaN ;
            mAngularVelocity.z   = Particle::sNaN ;
            mDensity             = Particle::sNaN ;
            mSize                = Particle::sNaN ;
            mBirthTime           = -1 ;
        }


        bool IsAlive() const
        {
            return mBirthTime >= 0 ;
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

    /*! \brief Mark the particle at the given index as one to be killed.
    */
    inline void MarkForKill( Vector< Particle > & particles , size_t iParticle )
    {
        particles[ iParticle ].MarkDead() ;
    }

    inline void Merge( Vector< Particle > & particles , size_t iPcl1 , size_t iPcl2 , float ambientFluidDensity )
    {
        Particle &       pcl1           = particles[ iPcl1 ] ;
        const Particle & pcl2           = particles[ iPcl2 ] ;
        const float      mass1          = pcl1.GetMass( ambientFluidDensity ) ;
        const float      mass2          = pcl2.GetMass( ambientFluidDensity ) ;
        const float      radius1        = pcl1.mSize * 0.5f ;
        const float      radius2        = pcl2.mSize * 0.5f ;
        const float      radius         = radius1 ; // TODO: Set radius such that total volume is the same.
        const float      momIn1         = 0.4f * mass1 * radius1 * radius1 ;
        const float      momIn2         = 0.4f * mass2 * radius2 * radius2 ;
        const float      mass           = mass1 + mass2 ;
        const float      momIn          = 0.4f * mass * radius * radius ;
        const float      oneOverMass    = 1.0f / mass ;


        pcl1.mPosition        = ( pcl1.mPosition * mass1 + pcl2.mPosition * mass2 ) * oneOverMass ;
        pcl1.mVelocity        = ( pcl1.mVelocity * mass1 + pcl2.mVelocity * mass2 ) * oneOverMass ;
        pcl1.mOrientation     = ( pcl1.mOrientation + pcl2.mOrientation ) * 0.5f ;
        const Vec3 angMom     = ( pcl1.mAngularVelocity * momIn1 + pcl2.mAngularVelocity * momIn2 ) ;
        pcl1.mAngularVelocity = angMom / momIn ;
        pcl1.SetRadius( radius ) ;
        //pcl1.SetMass( mass1 /* Should be "mass" to conserve mass, but without also changing volume, "mass" would change density */ , ambientFluidDensity ) ;
        pcl1.mBirthTime = MIN2( pcl1.mBirthTime , pcl2.mBirthTime ) ;


        MarkForKill( particles , iPcl2 ) ;
    }

    //extern void SetDensity( Vector< Particle > & particles , float density ) ;
    extern void AssignDensityFromGrid( Vector< Particle > & particles , const UniformGrid< float > & densityGrid ) ;
    extern void Advect( Vector< Particle > & particles , const UniformGrid< Vec3 > & velocityGrid , const float & timeStep ) ;
    extern void FindBoundingBox( const Vector< Particle > & particles , Vec3 & minCorner , Vec3 & maxCorner ) ;
    extern void Emit( Vector< Particle > & particles , const UniformGridGeometry & uniformGrid , unsigned multiplier , const Vector< Particle > * vortons = 0 ) ;
    extern float ComputeMassPerParticle( const UniformGridGeometry & uniformGrid , unsigned numParticlesPerCell , float density ) ;
    extern Vec3 ComputeGeometricCenter( const Vector< Particle > & particles ) ;
    extern Vec3 ComputeAngularMomentum( const Vector< Particle > & particles , float ambientFluidDensity ) ;
    extern void KillParticlesMarkedForDeath( Vector< Particle > & particles ) ;
}

#endif
