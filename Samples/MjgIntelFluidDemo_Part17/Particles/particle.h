/** \file particle.h

    \brief Basic particle for use in a visual effects particle system.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_H
#define PARTICLE_H

#include <math.h>

#include "Core/useTbb.h"

#include "Core/Math/vec3.h"
#include "Core/wrapperMacros.h"

#include "Core/SpatialPartition/uniformGrid.h"

// Macros --------------------------------------------------------------

/// Whether to enable combustion aspects of fluid simulation.
#define ENABLE_FIRE 1


/** Whether to record a recent history of particle positions in a ring buffer, for rendering pathlines.

    This consumes a lot of memory and CPU time.
    It is meant for diagnosis, not for release.
*/
#define ENABLE_PARTICLE_POSITION_HISTORY 0


/** Whether to diagnose particle jerking.
*/
#define ENABLE_PARTICLE_JERK_RECORD 0


/** Whether to track a brief history of every particle member, for diagnosing determinism.
*/
#define ENABLE_PARTICLE_HISTORY 0


/// Hacks to reassign density in or at walls.
#define POISON_DENSITY_BASED_ON_VORTONS_HITTING_WALLS 0
#define POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS 0

/// Hacks to remove component of density gradient parallel to walls.
#define POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS 0
#define POISON_DENSITY_GRADIENT_BASED_ON_GRIDPOINTS_INSIDE_WALLS 0
#define POISON_DENSITY_GRADIENT_BASED_ON_RIGID_SPHERE_GEOMETRY 0

/// Reduce vorton convergence using an ad-hoc SPH-like approach with VPM.
/// Do not enable this when USE_SMOOTHED_PARTICLE_HYDRODYNAMICS is 1.
/// It's mainly useful for using VPM with containers.
#define REDUCE_CONVERGENCE 0

/** Whether to compute pressure gradient explicitly.

    Alternative is to assume only vertical pressure gradient from hydrostatic balance.
*/
#define COMPUTE_PRESSURE_GRADIENT 0




static const float  Pi      = 3.1415926535897932384626433832795f ;
static const float  PiOver6 = Pi / 6.0f ;

static const float Particle_sAmbientTemperature = 300.0f ;


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
            , mDensity( 1.0f )
            , mSize( 0.0f )
            , mBirthTime( 0 )
        #if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
            , mHitBoundary( false )
            , mHitNormal( 0.0f , 0.0f , 0.0f )
        #endif
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
        #if defined( _DEBUG )
            , mNumVortonsIncorporated( 0 )
            , mTotalCirculation( 0.0f , 0.0f , 0.0f )
            #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
            , mNumSibVortonsIncorporated( 0 )
            #endif
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
        #if ENABLE_DISPLACEMENT_TRACKING
            , mDisplacement( 0.0f , 0.0f , 0.0f )
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

            \param position         Particle position.

            \param angularVelocity  Angular velocity in radians per unit time.

            \param size            Particle diameter.
        */
        Particle( const Vec3 & position , const Vec3 & angularVelocity , float size = 0.0f )
            : mPosition( position )
            , mVelocity( 0.0f , 0.0f , 0.0f )
            , mOrientation( 0.0f , 0.0f , 0.0f )
            , mAngularVelocity( angularVelocity )
            , mDensity( 1.0f )
            , mSize( size )
            , mBirthTime( 0 )
        #if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
            , mHitBoundary( false )
            , mHitNormal( 0.0f , 0.0f , 0.0f )
        #endif
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
        #if defined( _DEBUG )
            , mNumVortonsIncorporated( 0 )
            , mTotalCirculation( 0.0f , 0.0f , 0.0f )
            #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
            , mNumSibVortonsIncorporated( 0 )
            #endif
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
        #if ENABLE_DISPLACEMENT_TRACKING
            , mDisplacement( sNaN , sNaN , sNaN )
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


        // Use the compiler-generated copy constructor.


        /** Get radius from size.
            \return the radius of this vorton
        */
        float GetRadius() const
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

            \return Volume of this particle.

            \note Derivation:
                    Volume of a sphere is 4*pi*r^3/3.
                    r=size/2 so r^3=size/8.
                    V=4*pi*size^3/(3*8)=pi*size^3/6
        */
        float GetVolume() const
        {
            return PiOver6 * mSize * mSize * mSize ;
        }


        /** Return density of this particle.
        */
        float GetDensity() const
        {
            ASSERT( mDensity > 0.0f ) ;
            return mDensity ;
        }


        /** Assign density of this particle.

            \see GetDensity.
        */
        void SetDensity( float density )
        {
            mDensity = density ;
            ASSERT( mDensity > 0.0f ) ;
            ASSERT( ! IsNan( mDensity ) && ! IsInf( mDensity ) ) ;
            ASSERT( GetMass() >= 0.0f ) ;
        }


        /** Return mass of this particle.
        */
        float GetMass() const
        {
            const float fMass   = GetDensity() * GetVolume() ;
            ASSERT( fMass >= 0.0f ) ;
            return fMass ;
        }


        /** Assign mass for this particle.

            \param mass             Mass to assign to this particle.
        */
        void SetMass( float mass )
        {
            ASSERT( mass >= 0.0f ) ;
            const float density = mass / GetVolume() ;
            SetDensity( density ) ;
            ASSERT( GetMass() >= 0.0f ) ;
        }


        /** Return temperature of this particle.

            \return Temperature of this particle, in absolute degrees.

            \note   This assumes a Boussinesq approxomation where temperature is
                    related to density.

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
            const float temperature = GetDensity() / ( ambientDensity * ( 1 - thermalExpansionCoefficient ) ) + Particle_sAmbientTemperature ;
        #else
            const float temperature = Particle_sAmbientTemperature * ambientDensity / GetDensity() ;
        #endif
            ASSERT( temperature >= 0.0f ) ;
            return temperature ;
        }


        /** Set particle temperature.

            \param temperature  Temperature of particle in absolute degrees.

            Using the Bousinesq approximation and assuming the fluid is an ideal gas,
                temperature = ambientTemperature * ambientDensity / density

        */
        void SetTemperature( float ambientDensity , float temperature )
        {
            ASSERT( 1.0f == ambientDensity ) ;

            float density = ambientDensity * Particle_sAmbientTemperature / temperature ;
            ASSERT( ! IsNan( density ) && ! IsInf( density ) ) ;
            SetDensity( density ) ;
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
            mDensity            = Particle::sNaN ;
            mSize               = Particle::sNaN ;
            mBirthTime          = -1 ;

        #if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
            mHitBoundary        = false ;
            mHitNormal          = Vec3( Particle::sNaN , Particle::sNaN , Particle::sNaN ) ;
        #endif

        #if ENABLE_FIRE
            mFuelFraction       = Particle::sNaN ;
            mFlameFraction      = Particle::sNaN ;
            mSmokeFraction      = Particle::sNaN ;
        #endif
            ASSERT( ! IsAlive() ) ;
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
            ASSERT( mPositionHistoryIndex < NUM_HISTORICAL_POSITIONS ) ;
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


        Vec3        mPosition	        ;   ///< Position (in world units) of center of particle
        Vec3        mVelocity	        ;   ///< Velocity (speed and direction) of particle
        Vec3        mOrientation	    ;   ///< Orientation of particle, in axis-angle form where angle=|orientation|
        Vec3        mAngularVelocity	;   ///< Angular velocity of particle.  Same as half the vorticity.
        float	    mDensity            ;   ///< Either density or mass per particle, depending.  For fire simulations, this is the exhaust (smoke) density.
        float	    mSize		        ;   ///< Diameter of the region of influence of a particle.
        int         mBirthTime          ;   ///< Birth time of particle, in "ticks"

    #if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
        bool        mHitBoundary        ;   ///< Whether this particle hit a boundary this frame.
        Vec3        mHitNormal          ;   ///< Contact normal direction, when hit.
    #endif

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

#if defined( _DEBUG )
        unsigned    mNumVortonsIncorporated     ;   ///< Number of actual vortons encountered during influence calculations
        Vec3        mTotalCirculation           ;   ///< Total circulation encountered during influence calculation
    #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
        unsigned    mNumSibVortonsIncorporated  ;   ///< Number of sibling vortons encountered during influence calculations
        unsigned    mIndicesOfParent[3]         ;   ///< Indices of parent cell
    #endif
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
    #if ENABLE_DISPLACEMENT_TRACKING
        Vec3    mDisplacement       ;   ///< Displacement applied from projecting particles.
    #endif

} ;

// Public functions --------------------------------------------------------------

namespace Particles
{
    /** Kill the particle at the given index, replacing it with the last particle in the given dynamic array.
    */
    inline void Kill( VECTOR< Particle > & particles , size_t iParticle )
    {
        ASSERT( iParticle < particles.Size() ) ;
        particles[ iParticle ] = particles[ particles.Size() - 1 ] ;
        particles.PopBack() ;
    }

    /** Mark the particle at the given index as one to be killed.
    */
    inline void MarkForKill( VECTOR< Particle > & particles , size_t iParticle )
    {
        ASSERT( iParticle < particles.Size() ) ;
        particles[ iParticle ].MarkDead() ;
    }

    /** Merge the given two particles.

        \param particles    Dynamic array of particles.

        \param iPcl1        Index of first particle, which will subsume the second particle.

        \param iPcl2        Index of second particle, which will be marked for deletion.
    */
    inline void Merge( VECTOR< Particle > & particles , size_t iPcl1 , size_t iPcl2 )
    {
        Particle &       pcl1           = particles[ iPcl1 ] ;
        const Particle & pcl2           = particles[ iPcl2 ] ;
        const float      mass1          = pcl1.GetMass() ;
        const float      mass2          = pcl2.GetMass() ;
        const float      radius1        = pcl1.mSize * 0.5f ;
        const float      radius2        = pcl2.mSize * 0.5f ;
        ASSERT( radius1 == radius2 ) ; // TODO: FIXME: code elsewhere assumes all vortons have the same radius.
        const float      radius         = radius1 ; // TODO: Set radius such that total volume is the same.
        const float      momIn1         = 0.4f * mass1 * radius1 * radius1 ;
        const float      momIn2         = 0.4f * mass2 * radius2 * radius2 ;
        const float      mass           = mass1 + mass2 ;
        const float      momIn          = 0.4f * mass * radius * radius ;
        const float      oneOverMass    = 1.0f / mass ;

        ASSERT( mass > 0.0f ) ;
        ASSERT( ! IsNan( oneOverMass           ) && ! IsInf( oneOverMass           ) ) ;
        ASSERT( ! IsNan( pcl1.mPosition        ) && ! IsInf( pcl1.mPosition        ) ) ;
        ASSERT( ! IsNan( pcl1.mAngularVelocity ) && ! IsInf( pcl1.mAngularVelocity ) ) ;

        pcl1.mPosition        = ( pcl1.mPosition * mass1 + pcl2.mPosition * mass2 ) * oneOverMass ;
        pcl1.mVelocity        = ( pcl1.mVelocity * mass1 + pcl2.mVelocity * mass2 ) * oneOverMass ;
        pcl1.mOrientation     = ( pcl1.mOrientation + pcl2.mOrientation ) * 0.5f ;
        const Vec3 angMom     = ( pcl1.mAngularVelocity * momIn1 + pcl2.mAngularVelocity * momIn2 ) ;
        pcl1.mAngularVelocity = angMom / momIn ;
        pcl1.SetRadius( radius ) ;
        //pcl1.SetMass( mass1 /* Should be "mass" to conserve mass, but without also changing volume, "mass" would change density */ , ambientFluidDensity ) ;
        pcl1.mBirthTime = Min2( pcl1.mBirthTime , pcl2.mBirthTime ) ;

#if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
        pcl1.mHitBoundary = pcl1.mHitBoundary || pcl2.mHitBoundary ;
        pcl1.mHitNormal = pcl1.mHitBoundary ? pcl1.mHitNormal : pcl2.mHitNormal ;
#endif

        ASSERT( ! IsNan( pcl1.mPosition        ) && ! IsInf( pcl1.mPosition        ) ) ;
        ASSERT( ! IsNan( pcl1.mAngularVelocity ) && ! IsInf( pcl1.mAngularVelocity ) ) ;

        MarkForKill( particles , iPcl2 ) ;
    }

    void KillParticlesMarkedForDeath( VECTOR< Particle > & particles ) ;
    Vec3 ComputeGeometricCenter( const VECTOR< Particle > & particles ) ;
    void ComputeAngularVelocityStats( const VECTOR< Particle > & particles , float & min , float & max , float & mean , float & stddev , Vec3 & centerOfAngularVelocity ) ;
    Vec3 ComputeAngularMomentum( const VECTOR< Particle > & particles ) ;
    void ComputeVelocityStats( const VECTOR< Particle > & particles , float & min , float & max , float & mean , float & stddev , Vec3 & centerOfVelocity ) ;
    void ComputeTemperatureStats( const VECTOR< Particle > & particles , float & min , float & max , float & mean , float & stddev ) ;
    void ComputeDensityStats( const VECTOR< Particle > & particles , float & min , float & max , float & mean , float & stddev ) ;

    void PartitionParticles( const VECTOR< Particle > & particles ,
#if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION
                            SpatialPartition & particleGrid
#else
                            UniformGrid< VECTOR< unsigned > > & particleIndices
#endif
                        , float minCellSpacing ) ;

#if ENABLE_PARTICLE_JERK_RECORD
    extern void  UpdateJerk( VECTOR< Particle > & particles , float timeStep ) ;
    extern void  ComputeJerkStatistics( const VECTOR< Particle > & particles , float & jerk2Avg , float & jerk2Dev , float & jerk2Min , float & jerk2Max ) ;
#endif

#if ENABLE_PARTICLE_HISTORY
    extern size_t gPclHistoryFrame ;
    extern bool gPclHistoryFirstRun ;
    extern void PclHistoryRecord( size_t offset , const Particle & pcl ) ;
#endif

}

#endif
