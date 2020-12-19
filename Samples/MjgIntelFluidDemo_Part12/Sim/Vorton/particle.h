/** \file particle.h

    \brief Basic particle for use in a visual effects particle system

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_H
#define PARTICLE_H

#include <math.h>

#include "useTbb.h"

#include "Core/Math/vec3.h"
#include "wrapperMacros.h"

#include "Space/uniformGrid.h"

// Macros --------------------------------------------------------------

/// Whether to enable combustion aspects of fluid simulation.
#define ENABLE_FIRE 1


/** Whether to record a recent history of particle positions in a ring buffer, for rendering pathlines.

    This consumes a lot of memory and CPU time.
    It is meant for diagnosis, not for release.
*/
#define ENABLE_PARTICLE_POSITION_HISTORY 1


/** Whether to diagnose particle jerking.
*/
#define ENABLE_PARTICLE_JERK_RECORD 0


/** Whether to track a brief history of every particle member, for diagnosing determinism.
*/
#define ENABLE_PARTICLE_HISTORY 0


static const float  Pi      = 3.1415926535897932384626433832795f ;
static const float  PiOver6 = Pi / 6.0f ;


// Types --------------------------------------------------------------

/** Basic particle for use in a visual effects particle system.
*/
class Particle
{
    public:
        static const float sNaN ;   ///< Not a number.

        /** Construct a particle.
        */
        Particle()
            : mPosition( 0.0f , 0.0f , 0.0f )
            , mVelocity( 0.0f , 0.0f , 0.0f )
            , mOrientation( 0.0f , 0.0f , 0.0f )
            , mAngularVelocity( 0.0f , 0.0f , 0.0f )
            , mDensityDeviation( 0.0f )
            , mSize( 0.0f )
            , mBirthTime( 0 )
        #if ENABLE_FIRE
            , mFuelFraction( 0.0f )
            , mFlameFraction( 0.0f )
            , mSmokeFraction( 0.0f )
        #endif
        #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
            , mPositionSiblings( 0.0f , 0.0f , 0.0f )
            , mVorticitySiblings( 0.0f , 0.0f , 0.0f )
            //, mRadiusSiblings( 0.0f )
        #endif
        #if ENABLE_PARTICLE_POSITION_HISTORY
            , mPositionHistoryIndex( 0 )
        #endif
        #if ENABLE_PARTICLE_JERK_RECORD
            , mVelocityPrev( 0.0f , 0.0f , 0.0f )
            , mAcceleration( 0.0f , 0.0f , 0.0f )
            , mAccelerationPrev( 0.0f , 0.0f , 0.0f )
            , mJerk( 0.0f , 0.0f , 0.0f )
        #endif
        {
        #if ENABLE_PARTICLE_POSITION_HISTORY
            mPositionHistory[ mPositionHistoryIndex ] = mPosition ;
            for( size_t iHistory = 1 ; iHistory < NUM_HISTORICAL_POSITIONS ; ++ iHistory )
            {
                mPositionHistory[ iHistory ] = Vec3( Particle::sNaN , Particle::sNaN , Particle::sNaN ) ;
            }
        #endif
        }

        /** Construct a particle.

            \param vPos     Particle position.

            \param vAngVel  Angular velocity in radians per unit time.

            \param fSize    Particle diameter.
        */
        Particle( const Vec3 & vPos , const Vec3 & vAngVel , float fSize = 0.0f )
            : mPosition( vPos )
            , mVelocity( 0.0f , 0.0f , 0.0f )
            , mOrientation( 0.0f , 0.0f , 0.0f )
            , mAngularVelocity( vAngVel )
            , mDensityDeviation( 0.0f )
            , mSize( fSize )
            , mBirthTime( 0 )
        #if ENABLE_FIRE
            , mFuelFraction( 0.0f )
            , mFlameFraction( 0.0f )
            , mSmokeFraction( 1.0f )
        #endif
        #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
            , mPositionSiblings( 0.0f , 0.0f , 0.0f )
            , mVorticitySiblings( 0.0f , 0.0f , 0.0f )
            //, mRadiusSiblings( 0.0f )
        #endif
        #if ENABLE_PARTICLE_POSITION_HISTORY
            , mPositionHistoryIndex( 0 )
        #endif
        #if ENABLE_PARTICLE_JERK_RECORD
            , mVelocityPrev( sNaN , sNaN , sNaN )
            , mAcceleration( sNaN , sNaN , sNaN )
            , mAccelerationPrev( sNaN , sNaN , sNaN )
            , mJerk( sNaN , sNaN , sNaN )
        #endif
        {
        #if ENABLE_PARTICLE_POSITION_HISTORY
            mPositionHistory[ mPositionHistoryIndex ] = mPosition ;
            for( size_t iHistory = 1 ; iHistory < NUM_HISTORICAL_POSITIONS ; ++ iHistory )
            {
                mPositionHistory[ iHistory ] = Vec3( Particle::sNaN , Particle::sNaN , Particle::sNaN ) ;
            }
        #endif
        }


        /** Get radius from size.
            \return the radius of this vorton
        */
        float GetRadius( void ) const
        {
            return mSize * 0.5f ;
        }


        /** Set size from radius.
        */
        void SetRadius( float radius )
        {
            mSize = 2.0f * radius ;
        }


        /** Return volume of this particle.

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


        /// Return whole density of this particle.
        float GetDensity( float ambientDensity ) const
        {
            return mDensityDeviation + ambientDensity ;
        }

        /// Assign whole density of this particle.
        void SetDensity( float density , float ambientDensity )
        {
            mDensityDeviation = density - ambientDensity ;
        }

        /** Return mass deviation of this particle.

            \return Mass deviation of this particle

            Particle mDensityDeviation represents a deviation from the ambient fluid
            density, instead of representing fluid density absolutely.  Such a
            treatment allows for placing particles judiciously instead of
            ubiquitously.

        */
        float GetMassDeviation() const
        {
            const float fMass   = mDensityDeviation * GetVolume() ;
            return fMass ;
        }


        /** Return mass of this particle.

            \param ambientDensity - density of fluid in the absence of particles.

            \see GetMassDeviation

            \return Mass of this particle
        */
        float GetMass( float ambientDensity ) const
        {
            const float fMass   = GetDensity( ambientDensity ) * GetVolume() ;
            return fMass ;
        }


        /** Assign mass for this particle.

            \param mass             Mass to assign to this particle.

            \param ambientDensity   Density of fluid in the absence of particles.

            \see GetMassDeviation
        */
        void SetMass( float mass , float ambientDensity )
        {
            const float density = mass / GetVolume() ;
            SetDensity( density , ambientDensity ) ;
        }


        /** Return temperature of this particle.

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
            const float thermalExpansionCoefficient = 1.0f ;
            const float density = mDensityDeviation + ambientDensity ; // mDensityDeviation is actual density deviation about ambient.
            const float temperature = density / ( ambientDensity * ( 1 - thermalExpansionCoefficient ) ) + sAmbientTemperature ;
        #else
            const float temperature = sAmbientTemperature * ambientDensity / GetDensity( ambientDensity ) ;
        #endif
            return temperature ;
        }


        /** Set particle temperature given ambient density.

            \param ambientDensity   Density of fluid in absence of particles.

            \param temperature  Temperature of particle.

            Using the Bousinesq approximation and assuming the fluid is an ideal gas,
                temperature = ambientTemperature * ambientDensity / density

            In this simulation, particles store the density departure from ambient,
            i.e. mDensityDeviation = density - ambientDensity.

        */
        void SetTemperature( float ambientDensity , float temperature )
        {
            float density = ambientDensity * sAmbientTemperature / temperature ;
            SetDensity( density , ambientDensity ) ;
        }


        /** Mark this particle for deletion.
            \see Particles::KillParticlesMarkedForDeath, IsAlive.
        */
        void MarkDead()
        {
            mPosition.x         = Particle::sNaN ;
            mPosition.y         = Particle::sNaN ;
            mPosition.z         = Particle::sNaN ;
            mVelocity.x         = Particle::sNaN ;
            mVelocity.y         = Particle::sNaN ;
            mVelocity.z         = Particle::sNaN ;
            mOrientation.x      = Particle::sNaN ;
            mOrientation.y      = Particle::sNaN ;
            mOrientation.z      = Particle::sNaN ;
            mAngularVelocity.x  = Particle::sNaN ;
            mAngularVelocity.y  = Particle::sNaN ;
            mAngularVelocity.z  = Particle::sNaN ;
            mDensityDeviation   = Particle::sNaN ;
            mSize               = Particle::sNaN ;
            mBirthTime          = -1 ;
        #if ENABLE_FIRE
            mFuelFraction       = Particle::sNaN ;
            mFlameFraction      = Particle::sNaN ;
            mSmokeFraction      = Particle::sNaN ;
        #endif
        }


        /** Return whether this particle is alive (has not been marked dead).
            \see MarkDead
        */
        bool IsAlive() const
        {
            return mBirthTime >= 0 ;
        }


    #if ENABLE_PARTICLE_POSITION_HISTORY
        static size_t NextHistoryIndex( size_t positionHistoryIndex )
        {
            static const size_t lastHistoricalIndex = NUM_HISTORICAL_POSITIONS - 1 ;
            return positionHistoryIndex >= lastHistoricalIndex ? 0 : positionHistoryIndex + 1 ;
        }

        size_t HistoryNext() const
        {
            static const size_t lastHistoricalIndex = NUM_HISTORICAL_POSITIONS - 1 ;
            return NextHistoryIndex( mPositionHistoryIndex ) ;
        }

        /// Return index of first (oldest) element in position history.
        size_t HistoryBegin() const
        {
            return mPositionHistoryIndex ;
        }

        /// Return index of last (newest) element in position history.
        size_t HistoryEnd() const
        {
            static const size_t lastHistoricalIndex = NUM_HISTORICAL_POSITIONS - 1 ;
            return 0 == mPositionHistoryIndex ? lastHistoricalIndex : mPositionHistoryIndex - 1 ;
        }

        void RecordPositionHistory()
        {
            // Record current position in history.
            mPositionHistory[ mPositionHistoryIndex ] = mPosition ;
            // Increment ring buffer index.
            mPositionHistoryIndex = HistoryNext() ;
        }
    #endif

    #if ENABLE_PARTICLE_JERK_RECORD
        /** Update diagnostic quantities used to measure jerk.

            This must be called prior to updating particle velocity,
            but before computing jerk.

            \see ComputeJerk.
        */
        void UpdateJerkDiagnostics( float timeStep )
        {
            mJerk               = ( mAcceleration - mAccelerationPrev ) / timeStep ;
            mAccelerationPrev   = mAcceleration ;
            mAcceleration       = ( mVelocity - mVelocityPrev ) / timeStep ;
            mVelocityPrev       = mVelocity ;
        }
    #endif


        // Use the compiler-generated copy constructor

        Vec3        mPosition	        ;   ///< Position (in world units) of center of particle
        Vec3        mVelocity	        ;   ///< Velocity (speed and direction) of particle
        Vec3        mOrientation	    ;   ///< Orientation of particle, in axis-angle form where angle=|orientation|
        Vec3        mAngularVelocity	;   ///< Angular velocity of particle.  Same as half the vorticity.
        float	    mDensityDeviation   ;   ///< Either smoothed density or mass per particle, depending.  For fire simulations, this is the exhaust (smoke) density.
        float	    mSize		        ;   ///< Diameter of the region of influence of a particle.
        int         mBirthTime          ;   ///< Birth time of particle, in "ticks"

    #if ENABLE_FIRE
        // Each particle has 3 fractions: fuel, flame and exhaust (smoke).
        // Their sum must equal 1, and each must be non-negative.
        // Only 2 need be explicit; Smoke is computed as smoke=1-fuel-flame.
        // We model all 3 explicitly to simplify rendering code.
        float	    mFuelFraction       ;   ///< Fraction of mass which is fuel.
        float	    mFlameFraction      ;   ///< Fraction of mass which is flame.
        float	    mSmokeFraction      ;   ///< Fraction of mass which is smoke.
    #endif

        // HACK CODE: None of these members belong in Particle.
        // They're useful for debugging the vorton sim,
        // and we want to consolidate particle opertion code to
        // work on both Particle and Vorton.
    #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
        Vec3        mPositionSiblings   ;   ///< Position (in world units) of center of vortex particle
        Vec3        mVorticitySiblings  ;   ///< Vorticity of vortex particle
        //float   mRadiusSiblings     ;   ///< Radius of sibling cluster
    #endif


    #if ENABLE_PARTICLE_POSITION_HISTORY
        static const size_t NUM_HISTORICAL_POSITIONS = 32 ;
        Vec3    mPositionHistory[ NUM_HISTORICAL_POSITIONS ]    ;   ///< Ring buffer of recent history of positions of this particle.
        size_t  mPositionHistoryIndex                           ;   ///< Index into mPositionHistory of next position to record.
    #endif
    #if ENABLE_PARTICLE_JERK_RECORD
        Vec3    mVelocityPrev       ;   ///< Particle velocity from previous update.
        Vec3    mAcceleration       ;   ///< Particle acceleration for current state.
        Vec3    mAccelerationPrev   ;   ///< Particle acceleration from previous update.
        Vec3    mJerk               ;   ///< Particle jerk for current state.
    #endif
} ;

// Public functions --------------------------------------------------------------

namespace Particles
{
    /** Kill the particle at the given index, replacing it with the last particle in the given dynamic array.
    */
    inline void Kill( Vector< Particle > & particles , size_t iParticle )
    {
        particles[ iParticle ] = particles[ particles.Size() - 1 ] ;
        particles.PopBack() ;
    }

    /** Mark the particle at the given index as one to be killed.
    */
    inline void MarkForKill( Vector< Particle > & particles , size_t iParticle )
    {
        particles[ iParticle ].MarkDead() ;
    }

    /** Merge the given two particles.

        \param particles    Dynamic array of particles.

        \param iPcl1        Index of first particle, which will subsume the second particle.

        \param iPcl2        Index of second particle, which will be marked for deletion.

        \param ambientFluidDensity  Density of fluid in the absence of fluid particles.  Used to compute total particle mass.
    */
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

    extern void  AssignScalarFromGrid( Vector< Particle > & particles , size_t memberOffsetInBytes, const UniformGrid< float > & scalarGrid ) ;
    extern void  Advect( Vector< Particle > & particles , const UniformGrid< Vec3 > * velocityGrid , const float & timeStep ) ;
    extern void  FindBoundingBox( const Vector< Particle > & particles , Vec3 & minCorner , Vec3 & maxCorner ) ;
    extern void  Emit( Vector< Particle > & particles , const UniformGridGeometry & uniformGrid , unsigned multiplier , const Vector< Particle > * vortons = 0 ) ;
    extern Vec3  ComputeGeometricCenter( const Vector< Particle > & particles ) ;
    extern Vec3  ComputeAngularMomentum( const Vector< Particle > & particles , float ambientFluidDensity ) ;
    extern void  KillParticlesMarkedForDeath( Vector< Particle > & particles ) ;

#if ENABLE_PARTICLE_JERK_RECORD
    extern void  UpdateJerk( Vector< Particle > & particles , float timeStep ) ;
    extern void  ComputeJerkStatistics( const Vector< Particle > & particles , float & jerk2Avg , float & jerk2Dev , float & jerk2Min , float & jerk2Max ) ;
#endif

#if ENABLE_PARTICLE_HISTORY
    extern size_t gPclHistoryFrame ;
    extern bool gPclHistoryFirstRun ;
    extern void PclHistoryRecord( size_t offset , const Particle & pcl ) ;
#endif

}

#endif
