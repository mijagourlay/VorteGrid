/** \file VortonSim.h

    \brief Dynamic simulation of a fluid, using tiny vortex elements.

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
#ifndef VORTON_SIM_H
#define VORTON_SIM_H

#include <math.h>

#include "Core/useTbb.h"

#include "Core/Space/nestedGrid.h"
#include "vorton.h"

// Macros --------------------------------------------------------------

/// Whether to use Multi-Grid technique for Poisson solver.
#define USE_MULTI_GRID 1

/// Whether to enable code to output density diagnostic data volumes each frame.
#define OUTPUT_DENSITY 0

// Types --------------------------------------------------------------

/** Dynamic simulation of a fluid, using tiny vortex elements.

    This implements a portion of a fluid simulation, and effectively
    neglects boundary conditions.  This module defers the enforcement
    of boundary conditions to another module.

    \see FluidBodySim

*/
class VortonSim
{
    public:

        /** Integrals that should be constant for a fluid.
        */
        struct Integrals
        {
            Integrals() ;

            Vec3                mTotalCirculation           ;   ///< Total circulation.
            Vec3                mLinearImpulseFromVorticity ;   ///< Linear impulse computed from vorticity.
            Vec3                mLinearImpulseFromVelocity  ;   ///< Linear impulse computed from velocity.
            Vec3                mAngularImpulse             ;   ///< Angular impulse.

            float ComputeMaxRelativeDifference( const Integrals & that ) const ;
        } ;


        /** Values of Integrals for various stages of computation.

            These should be the same as each other.  The degree to which they
            differ indicates the amount of numerical error.  Computing
            these at various stages help identify which stages introduce error
            and to what degree.
        */
        struct DiagnosticIntegrals
        {
            Integrals           mInitial            ;   ///< Integrals upon initialization.
            Integrals           mBefore             ;   ///< Integrals from last Update stage of previous frame.
            Integrals           mAfterAdvect        ;   ///< Integrals after advection (inferred from previous frame).
            Integrals           mAfterRegrid        ;   ///< Integrals after regridding vortons.
            Integrals           mAfterVelGrid       ;   ///< Integrals after computing velocity grid.
            Integrals           mAfterStretch       ;   ///< Integrals after stretching and tilting.
            Integrals           mAfterBaroclinic    ;   ///< Integrals after baroclinic generation.
            Integrals           mAfterDiffuse       ;   ///< Integrals after diffusing vorticity.
            Integrals           mAfterHeat          ;   ///< Integrals after heating fluid.
        } ;


        /** Which phase to run an algorithm that requires interleaving.

            \see GaussSeidelPortion
        */
        enum PhaseE
        {
            PHASE_ODD   ,   ///< Run on odd z slice index values.
            PHASE_EVEN  ,   ///< Run on even z slice index values.
            PHASE_BOTH  ,   ///< Run on all z slice index values.
        } ;


        VortonSim( float viscosity = 0.0f , float ambientFluidDensity = 1.0f ) ;
        void                        Initialize() ;
        void                        FindBoundingBox( void ) ;
        void                        UpdateBoundingBox( const Vec3 & minCorner , const Vec3 & maxCorner , bool bFinal ) ;
        void                        Update( float timeStep , unsigned uFrame ) ;
        void                        Clear( void ) ;
        void                        GatherVorticityStats( float & min , float & max , float & mean , float & stddev , Vec3 & centerOfVorticity ) const ;
        void                        GatherTemperatureStats( float & min , float & max , float & mean , float & stddev ) const ;

        /// Return reference to integrals used to diagnose fluid simulation accuracy.
        const DiagnosticIntegrals & GetDiagnosticIntegrals() const { return mDiagnosticIntegrals ; }

        /// Control whether diagnostic integrals are computed.
        void                        SetTallyDiagnosticIntegrals( bool bTallyDiagnosticIntegrals ) { mTallyDiagnosticIntegrals = bTallyDiagnosticIntegrals ; }
        const bool &                GetTallyDiagnosticIntegrals() const { return mTallyDiagnosticIntegrals ; }

        /// Set address of dynamic array used to store vortons.
        void                        SetVortons( Vector< Vorton > * vortons )                { mVortons = vortons ; }
              Vector< Vorton >  *   GetVortons()                                            { return mVortons ; }
        const Vector< Vorton >  *   GetVortons() const                                      { return mVortons ; }

        /// Return grid representing spatial distribution of velocity.
        const UniformGrid< Vec3 > & GetVelocityGrid() const                                 { return mVelGrid ; }

        /// Set viscous diffusion parameter for fluid simulation.
        /// This controls the rate at which momentum gradually diffuses throughout the fluid.
        void                        SetViscosity( float viscosity )                         { mViscosity = viscosity ; }
        const float &               GetViscosity() const                                    { return mViscosity ; }

        /// Set thermal diffusion parameter for fluid simulation.
        /// This controls the rate at which heat gradually diffuses throughout the fluid.
        void                        SetThermalDiffusivity( float thermalDiffusivity )       { mThermalDiffusivity = thermalDiffusivity ; }
        const float &               GetThermalDiffusivity() const                           { return mThermalDiffusivity ; }

        /// Set specific heat capacity for fluid simulation.
        /// This controls the amount of heat required to change the temperature of a unit of fluid.
        void                        SetSpecificHeatCapacity( float specificHeatCapacity )   { mSpecificHeatCapacity = specificHeatCapacity ; }
        const float &               GetSpecificHeatCapacity() const                         { return mSpecificHeatCapacity ; }

        /// Set ambient density of simulated fluid.
        /// This determines the mass per volume of simulated fluid, in the absence of particles.
        void                        SetAmbientDensity( float ambientFluidDensity )          { mAmbientDensity = ambientFluidDensity ; }
        const float &               GetAmbientDensity() const                               { return mAmbientDensity ; }

        /// Return reference to density deviation grid.
        /// This represents the spatial distribution of how density deviates from ambient.
        const UniformGrid< float >& GetDensityDeviationGrid() const                         { return mDensityDeviationGrid ; }

        /// Set the uniform gravitational acceleration.
        /// This affects buoyancy and the baroclinic generation of vorticity.
        void                        SetGravitationalAcceleration( const Vec3 & gravAccel )  { mGravAccel = gravAccel ; }
        const Vec3  &               GetGravitationalAcceleration()                          { return mGravAccel ; }

        /// Return the template for all grids used in this simulation, including density deviation, velocity and vorticity.
        /// \see GetVelocityGrid, GetDensityDeviationGrid, GetFuelGrid, GetFlameGrid, GetSmokeGrid.
        const UniformGridGeometry & GetGrid( void )                                         { return mGridTemplate ; }

        /// Get coordinate for minimal (lower left bottom) corner of grids used in this simulation.
        const Vec3 &                GetMinCorner( void ) const                              { return mMinCorner ; }

        /// Get coordinate for maximal (upper right top) corner of grids used in this simulation.
        const Vec3 &                GetMaxCorner( void ) const                              { return mMaxCorner ; }

        /// Get coordinate for center of grids used in this simulation.
        Vec3                        GetBoundingBoxCenter() const                            { return ( mMinCorner + mMaxCorner ) * 0.5f ; }

        /// Get size of grids used in this simulation.
        Vec3                        GetBoundingBoxSize() const                              { return mMaxCorner - mMinCorner ; }

        const Vec3 &                GetMinCornerEternal( void ) const                       { return mMinCornerEternal ; }
        const Vec3 &                GetMaxCornerEternal( void ) const                       { return mMaxCornerEternal ; }

    #if ENABLE_FIRE
        /// Set temperature above which fuel starts to become flame.
        void                        SetCombustionTemperature( float combustionTemperature ) { mCombustionTemperature = combustionTemperature ; }
        const float &               GetCombustionTemperature() const                        { return mCombustionTemperature ; }

        /// Set frequency factor controlling the rate at which heated fuel becomes flame.
        void                        SetCombustionRateFactor( float combustionRateFactor )   { mCombustionRateFactor = combustionRateFactor ; }
        const float &               GetCombustionRateFactor() const                         { return mCombustionRateFactor ; }

        /// Set temperature below which flame starts to become smoke.
        void                        SetSmokeTemperature( float smokeTemperature )           { mSmokeTemperature = smokeTemperature ; }
        const float &               GetSmokeTemperature() const                             { return mSmokeTemperature ; }

        /// Set frequency factor controlling rate at which cooled flame becomes smoke.
        void                        SetSmokeRateFactor( float smokeRateFactor )             { mSmokeRateFactor = smokeRateFactor ; }

        /// Set parameter controlling how much heat fuel releases when burned.
        void                        SetSpecificFreeEnergy( float specificFreeEnergy )       { mSpecificFreeEnergy = specificFreeEnergy ; }
        const float &               GetSpecificFreeEnergy() const                           { return mSpecificFreeEnergy ; }

        /// Return reference to spatial distribution of fuel mass fraction.
        const UniformGrid< float >& GetFuelGrid() const                                     { return mFuelFractionGrid ; }

        /// Return reference to spatial distribution of flame mass fraction.
        const UniformGrid< float >& GetFlameGrid() const                                    { return mFlameFractionGrid ; }

        /// Return reference to spatial distribution of smoke mass fraction.
        const UniformGrid< float >& GetSmokeGrid() const                                    { return mSmokeFractionGrid ; }
    #endif

    private:
        void        AssignVortonsFromVorticity( UniformGrid< Vec3 > & vortGrid ) ;
        void        TallyLinearImpulseFromVelocity( Vec3 & linearImpulse ) const ;
        void        TallyDiagnosticIntegrals( Vec3 & vCirculation , Vec3 & vLinearImpulseFromVorticity , Vec3 & vLinearImpulseFromVelocity , Vec3 & vAngularImpulse ) const ;
        void        ConditionallyTallyDiagnosticIntegrals( Integrals & integrals ) const ;

        // Integral-based velocity-from-vorticity routines
        void        MakeBaseVortonGrid() ;
        void        AggregateClusters( unsigned uParentLayer ) ;
        void        CreateInfluenceTree() ;
        Vec3        ComputeVelocityDirect( const Vec3 & vPosition ) ;
        Vec3        ComputeVelocityTree( const Vec3 & vPosition , const unsigned idxParent[3] , size_t iLayer ) ;
        Vec3        ComputeVelocityMonopoles( const unsigned indices[3] , const Vec3 & vPosition ) ;
        void        ComputeVelocityAtGridpointsSlice( size_t izStart , size_t izEnd ) ;
        void        ComputeVelocityAtVortonsSlice( size_t iPclStart , size_t iPclEnd ) ;
        void        ComputeVelocityGrid() ;

        // Differential-based velocity-from-vorticity routines
        void        PopulateVorticityGridFromVortons( UniformGrid< Vec3 > & vorticityGrid , float scale ) ;

        // Other vorticity equation terms
        void        StretchAndTiltVortons( const float & timeStep , const unsigned & uFrame ) ;
        void        PopulateVelocityGrid( UniformGrid< Vec3 > & velocityGrid , const Vector< Particle > & particles ) ;
        void        PopulateDensityDevAndMassFractionGrids( UniformGrid< float > & densityDevGrid , const Vector< Particle > & particles ) ;
        void        GenerateBaroclinicVorticitySlice( float timeStep , size_t izStart , size_t izEnd ) ;
        void        GenerateBaroclinicVorticity( const float & timeStep , const unsigned & uFrame ) ;

        void        PartitionVortons( const float & timeStep , const unsigned & uFrame , UniformGrid< Vector< unsigned > > & ugVortonIndices ) ;

        inline bool ExchangeVorticityOrMergeVortons( const unsigned & rVortIdxHere , Vorton & rVortonHere , Vec3 & rAngVelHere , const unsigned & ivThere , Vector< unsigned > & cell , const float & timeStep ) ;
        void        DiffuseVorticityGlobally( const float & timeStep , const unsigned & uFrame ) ;
        void        DiffuseVorticityPSESlice( const float & timeStep , UniformGrid< Vector< unsigned > > & ugVortRef , size_t izStart , size_t izEnd , PhaseE phase ) ;
        void        DiffuseVorticityPSE( const float & timeStep , const unsigned & uFrame , UniformGrid< Vector< unsigned > > & ugVortonIndices ) ;

        inline void ExchangeHeat( const unsigned & rVortIdxHere , Vorton & rVortonHere , float & rDensityHere , const unsigned & ivThere , const Vector< unsigned > & cell , const float & timeStep ) ;
        void        DiffuseHeatPSESlice( const float & timeStep , const UniformGrid< Vector< unsigned > > & ugVortRef , size_t izStart , size_t izEnd , PhaseE phase ) ;
        void        DiffuseHeatPSE( const float & timeStep , const unsigned & uFrame , const UniformGrid< Vector< unsigned > > & ugVortonIndices ) ;

        Vector< Vorton > *                  mVortons            ;   ///< Dynamic array of tiny vortex elements
        UniformGrid< Vector< unsigned > >   mVortonIndicesGrid  ;   ///< Spatial partition of indices into mVortons
        NestedGrid< Vorton >                mInfluenceTree      ;   ///< Influence tree
        UniformGrid< Vec3 >                 mVorticityGrid      ;   ///< Grid populated with vorticity from vortons
        NestedGrid< Vec3 >                  mVorticityMultiGrid ;   ///< Multi-resolution grid populated with vorticity from vortons
        UniformGrid< Vec3 >                 mVelGrid            ;   ///< Uniform grid of velocity values

        /** Effectively scale vorton radius and preserve total circulation.

            To avoid the high-velocity spike near the periphery of a vorton,
            effectively increase its radius and decrease its vorticity.
            This has no net effect outside the vorton radius, but inside,
            the speed that the vorton induces is lower and smoother than it
            would be otherwise.  This is similar to mollification.
        */
        float                   mSpreadingRangeFactor       ;   ///< Amount by which to scale vorton radius during integral velocity-from-vorticity calculation.
        float                   mSpreadingCirculationFactor ;   ///< Amount by which to scale vorton vorticity during velocity-from-vorticity calculation.  Must be 1/(mSpreadingRangeFactor^3).

        UniformGrid< float >    mDensityDeviationGrid       ;   ///< Uniform grid of density deviation-about-ambient values
        UniformGrid< Vec3 >     mDensityGradientGrid        ;   ///< Uniform grid of density gradient values

    #if ENABLE_FIRE
        UniformGrid< float >    mFuelFractionGrid           ;   ///< Uniform grid of fuel fraction values.
        UniformGrid< float >    mFlameFractionGrid          ;   ///< Uniform grid of flame fraction values.
        UniformGrid< float >    mSmokeFractionGrid          ;   ///< Uniform grid of flame fraction values.
    #endif

        UniformGridGeometry     mGridTemplate               ;   ///< Geometry of grid used to contain velocity, vorticity, etc.
        Vec3                    mMinCorner                  ;   ///< Minimal corner of axis-aligned bounding box
        Vec3                    mMaxCorner                  ;   ///< Maximal corner of axis-aligned bounding box
        Vec3                    mMinCornerEternal           ;   ///< Minimal corner of axis-aligned bounding box, across all time
        Vec3                    mMaxCornerEternal           ;   ///< Maximal corner of axis-aligned bounding box, across all time
        Vec3                    mCirculationInitial         ;   ///< Initial total circulation, which should be conserved when viscosity is zero.
        Vec3                    mLinearImpulseInitial       ;   ///< Initial linear impulse, which should be invariant when there are no external non-conservative forces.
        Vec3                    mAngularImpulseInitial      ;   ///< Initial angular impulse, which should be invariant when there are no external non-conservative forces.
        float                   mViscosity                  ;   ///< Viscosity.  Used to compute viscous diffusion.
        float                   mThermalDiffusivity         ;   ///< Thermal diffusivity.  Used to compute heat diffusion.
        float                   mSpecificHeatCapacity       ;   ///< Amount of heat required to change the temperature of one unit of mass by one unit of temperature.
        float                   mAmbientDensity             ;   ///< Ambient fluid density -- density assumed in the absence of particles.
        Vec3                    mGravAccel                  ;   ///< Acceleration due to gravity

        bool                    mTallyDiagnosticIntegrals   ;   ///< Whether to tally integrals
        DiagnosticIntegrals     mDiagnosticIntegrals        ;   ///< Integrals that should be invariant.

    #if ENABLE_FIRE
        float                   mCombustionTemperature      ;   ///< Activation temperature for combustion reaction.
        float                   mCombustionRateFactor       ;   ///< Coefficient throttling combustion rate.
        float                   mSmokeTemperature           ;   ///< Temperature below which smoke starts to form from flame, i.e. temperature of a glowing hot blackbody.
        float                   mSmokeRateFactor            ;   ///< Coefficient throttling smoke production rate.
        float                   mSpecificFreeEnergy         ;   ///< Amount of heat required to change temperature, per unit mass.
    #endif

    #if 1 || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) || defined( _DEBUG )
        float                   mVortonRadius           ;   ///< Radius of actual vortons (not of supervortons)
    #endif


    #if USE_TBB
        friend class VortonSim_ComputeVelocityAtGridpoints_TBB  ; ///< Multi-threading helper class for computing velocity at gridpoints.
        friend class VortonSim_ComputeVelocityAtVortons_TBB     ; ///< Multi-threading helper class for computing velocity at vortons.
        friend class VortonSim_GenerateBaroclinicVorticity_TBB  ; ///< Multi-threading helper class for computing fluid buoyancy.
        friend class VortonSim_DiffuseVorticityPSE_TBB          ; ///< Multi-threading helper class for computing vorticity diffusion.
        friend class VortonSim_DiffuseHeatPSE_TBB               ; ///< Multi-threading helper class for computing heat diffusion.
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
