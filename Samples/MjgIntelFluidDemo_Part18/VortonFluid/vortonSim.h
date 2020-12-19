/** \file VortonSim.h

    \brief Dynamic simulation of a fluid, using tiny vortex elements.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-18/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; all rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef VORTON_SIM_H
#define VORTON_SIM_H

#include <math.h>

#include "Core/useTbb.h"

#include "Core/SpatialPartition/nestedGrid.h"
#include "vorton.h"

// Macros --------------------------------------------------------------

/// Whether to use Multi-Grid technique for Poisson solver.
#define VORTON_SIM_USE_MULTI_GRID 1

/// Whether to enable code to output density diagnostic data volumes each frame.
#define VORTON_SIM_OUTPUT_DENSITY 0

/// Whether to enable code to output SDF diagnostic data volumes each frame.
#define VORTON_SIM_OUTPUT_SDF 0

/// Whether to gather statistics for vorticity equation terms.  This slows the computation and it used only for diagnostics
#if defined( _DEBUG )
    #define VORTON_SIM_GATHER_STATS 1
#else
    #define VORTON_SIM_GATHER_STATS 1
#endif

/// Use a linked list for spatial partition.  TODO: FIXME: Finish this implementation.
#define USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION 0

#define COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS 0
#define COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH 1

#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH && POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
    #error COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH is redundant with POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS.
#endif

/// Whether to enable code to perform smoothed-particle hydrodynamics (SPH).
#define USE_SMOOTHED_PARTICLE_HYDRODYNAMICS 1

/// Whether to use a UniformGrid to spatially partition SPH particles.
/// Alternative is to use direct, slower, simpler O(N^2) algorithm.
#define USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH 1

/// Disallow SPH with REDUCE_CONVERGENCE.  SPH should reduce divergence without that crutch.
#if USE_SMOOTHED_PARTICLE_HYDRODYNAMICS && REDUCE_CONVERGENCE
    #error REDUCE_CONVERGENCE should not be enabled for SPH.
#endif

extern const float sInvalidDensity ;

// Types --------------------------------------------------------------

#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || POISON_DENSITY_GRADIENT_BASED_ON_GRIDPOINTS_INSIDE_WALLS || POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS || POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS || COMPUTE_PRESSURE_GRADIENT
namespace Impulsion { class PhysicalObject ; } ;
#endif




struct SphFluidDensities
{
    SphFluidDensities( float numberDensity , float nearNumberDensity , float massDensity )
        : mNumberDensity( numberDensity )
        , mNearNumberDensity( nearNumberDensity )
        , mMassDensity( massDensity )
    {}

    float mNumberDensity        ;
    float mNearNumberDensity    ;
    float mMassDensity          ;
} ;




#if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION

/** Spatial partition of an array of items.
*/
struct SpatialPartition
{
    void Init( size_t numItems , const UniformGridGeometry & gridTemplate )
    {
        mGrid.Clear() ;                     // Clear any stale grid data.
        mGrid.CopyShape( gridTemplate ) ;   // Use given shape.
        mGrid.Init( INVALID_INDEX ) ;       // Initialize each cell to be empty.
        mLinks.Clear() ;                    // Clear any stale linked list nodes.
        mLinks.Reserve( numItems ) ;        // Allocate memory for linked list nodes.
    }

    void PushBack( size_t itemOffset , const Vec3 & itemPosition )
    {
        // Make (old) first item in grid cell point to given item.
        mLinks.PushBack( mGrid[ itemPosition ] ) ;
        // Make grid cell point to given item.
        mGrid[ itemPosition ] = itemOffset ;
    }

    static const unsigned INVALID_INDEX = ~0UL ;
    UniformGrid< size_t >   mGrid   ; ///< Uniform grid of indices into mItems, of first item in each cell.
    VECTOR< size_t >        mLinks  ; ///< Offset of subsequent item, or INVALID_INDEX if last item in cell.
} ;

#endif




/** Dynamic simulation of a fluid, using tiny vortex elements.

    This implements a portion of a fluid simulation, and effectively
    neglects boundary conditions.  This module defers the enforcement
    of boundary conditions to another module.

    \see FluidBodySim

*/
class VortonSim
{
    public:

        /** Fluid simulation technique.
        */
        enum FluidSimulationTechniqueE
        {
            FLUID_SIM_VORTEX_PARTICLE_METHOD            ,   ///< Use vortex particle method to simulate fluid.
            FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   ,   ///< Use smoothed particle hydrodynamics to simulate fluid.
            FLUID_SIM_VPM_SPH_HYBRID                    ,   ///< Use a hybrid of VPM and SPH.
            FLUID_SIM_NUM                                   ///<
        } ;

        /** Technique for obtaining velocity from vorticity.
        */
        //enum VelocityFromVorticityTechnique
        //{
        //    VELOCITY_FROM_VORTICITY_DIRECT  ,   ///< Direct summation
        //    VELOCITY_FROM_VORTICITY_TREE    ,   ///< Tree-code summation
        //    VELOCITY_FROM_VORTICITY_POISSON ,   ///< Solve Poisson equation
        //    VELOCITY_FROM_VORTICITY_NUM         
        //} ;


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


        /** Statistical quantities
        */
        template <class TypeT> struct Stats
        {
            TypeT   mMean   ;
            TypeT   mStdDev ;
            TypeT   mMin    ;
            TypeT   mMax    ;

            /** Accumulate sample values into statistics.
                \note   This uses mMean as a sum and mStdDev as a sum of squared values.
            */
            void Accumulate( const TypeT & sampleValue )
            {   // Accumulate sample values into statistics.
                mMean += sampleValue ;
                mStdDev += Pow2( sampleValue ) ;
                mMin = Min2( sampleValue , mMin ) ;
                mMax = Max2( sampleValue , mMax ) ;
            }

            void ConvertAccumulatedSamplesToStats( size_t numSamples )
            {   // Convert accumulated sample values into mean and stddev.
                mMean /= float( numSamples ) ;
                const float mean2 = mStdDev / float( numSamples ) ;
                const float var   = mean2 - Pow2( mMean ) ;
                mStdDev = fsqrtf( var ) ;
            }
        } ;


        typedef Stats<float> StatsFloat ;


        /** Statistics for vorticity equation terms.
        */
        struct VorticityTermsStatistics
        {
            static void Reset( StatsFloat & stat )
            {
                stat.mMean     = 0.0f ;
                stat.mStdDev   = 0.0f ;
                stat.mMin      =  FLT_MAX ;
                stat.mMax      = -FLT_MAX ;
            }

            void Reset()
            {
                Reset( mStretchTilt ) ;
                Reset( mBaroclinic ) ;
                Reset( mViscousDiffusion ) ;
            }

            StatsFloat  mStretchTilt        ;
            StatsFloat  mBaroclinic         ;
            StatsFloat  mViscousDiffusion   ;
        } ;


        enum InvestigationTermE
        {
            INVESTIGATE_ALL                 ,
            INVESTIGATE_STRETCHING_TILTING  ,
            INVESTIGATE_BAROCLINIC          ,
            INVESTIGATE_VISCOUS_DIFFUSION   ,
            INVESTIGATE_THERMAL_DIFFUSION   ,
            INVESTIGATE_NONE                ,
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

        VortonSim( const VortonSim & that )
        {
            this->operator=( that ) ;
        }

        VortonSim & operator=( const VortonSim & that )
        {
            if( this != & that )
            {
                // mVortons gets set before operating on it.  In fact, mVortons should
                // not be a member; should be passed in to routines that operate on it.
                mVortons = 0 ;
            }
            return * this ;
        }

        void                                Initialize( bool systemTracksFluidSurfaceUsingSdf ) ;
        void                                FindBoundingBox() ;
        void                                UpdateBoundingBox( const Vec3 & minCorner , const Vec3 & maxCorner , bool bFinal ) ;
        void                                Update( float timeStep , unsigned uFrame ) ;
        void                                Clear() ;
        void                                GatherVorticityStats( float & min , float & max , float & mean , float & stddev , Vec3 & centerOfVorticity ) const ;
        void                                GatherTemperatureStats( float & min , float & max , float & mean , float & stddev ) const ;
        void                                GatherDensityStats( float & min , float & max , float & mean , float & stddev ) const ;
        void                                GatherDensitySphStats( float & min , float & max , float & mean , float & stddev ) const ;
        void                                GatherNumberDensitySphStats( float & min , float & max , float & mean , float & stddev ) const ;
        void                                GatherDensityGradientStats( float & min , float & max , float & mean , float & stddev ) const ;
        void                                GatherSignedDistanceStats( float & min , float & max , float & mean , float & stddev ) const ;
        void                                GatherProximityStats( float & min , float & max , float & mean , float & stddev ) const ;

        /// Return reference to integrals used to diagnose fluid simulation accuracy.
        const DiagnosticIntegrals &         GetDiagnosticIntegrals() const                          { return mDiagnosticIntegrals ; }

        /// Return reference to statistics of vorticity equation terms.
        const VorticityTermsStatistics &    GetVorticityTermsStatistics() const                     { return mVorticityTermsStats ; }

        /// Control whether diagnostic integrals are computed.
        void                                SetTallyDiagnosticIntegrals( bool bTallyDiagnosticIntegrals ) { mTallyDiagnosticIntegrals = bTallyDiagnosticIntegrals ; }
        const bool &                        GetTallyDiagnosticIntegrals() const                     { return mTallyDiagnosticIntegrals ; }

        void                                SetFluidSimulationTechnique( FluidSimulationTechniqueE fluidSimTechnique ) { mFluidSimTechnique = fluidSimTechnique ; }
        const FluidSimulationTechniqueE &   GetFluidSimulationTechnique() const                     { return mFluidSimTechnique ; }

        /// Set address of dynamic array used to store vortons.
        void                                SetVortons( VECTOR< Vorton > * vortons )                { mVortons = vortons ; }
              VECTOR< Vorton >  *           GetVortons()                                            { return mVortons ; }
        const VECTOR< Vorton >  *           GetVortons() const                                      { return mVortons ; }

        /// Return grid representing spatial distribution of velocity.
        const UniformGrid< Vec3 > &         GetVelocityGrid() const                                 { return mVelGrid ; }
              UniformGrid< Vec3 > &         GetVelocityGrid()                                       { return mVelGrid ; }

        /// Set viscous diffusion parameter for fluid simulation.
        /// This controls the rate at which momentum gradually diffuses throughout the fluid.
        void                                SetViscosity( float viscosity )                         { mViscosity = viscosity ; }
        const float &                       GetViscosity() const                                    { return mViscosity ; }

        /// Set thermal diffusion parameter for fluid simulation.
        /// This controls the rate at which heat gradually diffuses throughout the fluid.
        void                                SetThermalDiffusivity( float thermalDiffusivity )       { mThermalDiffusivity = thermalDiffusivity ; }
        const float &                       GetThermalDiffusivity() const                           { return mThermalDiffusivity ; }

        /// Set specific heat capacity for fluid simulation.
        /// This controls the amount of heat required to change the temperature of a unit of fluid.
        void                                SetSpecificHeatCapacity( float specificHeatCapacity )   { mSpecificHeatCapacity = specificHeatCapacity ; }
        const float &                       GetSpecificHeatCapacity() const                         { return mSpecificHeatCapacity ; }

        /// Set ambient density of simulated fluid.
        /// This determines the mass per volume of simulated fluid, in the absence of particles.
        void                                SetAmbientDensity( float ambientFluidDensity )          { mAmbientDensity = ambientFluidDensity ; }
        const float &                       GetAmbientDensity() const                               { return mAmbientDensity ; }

        /// Return reference to density grid.
        /// This represents the spatial distribution of fluid density.
        const UniformGrid< float > &        GetDensityGrid() const                                  { return mDensityGrid ; }

        /// Return grid representing density gradient.
        const UniformGrid< Vec3 > &         GetDensityGradientGrid() const                          { return mDensityGradientGrid ; }

#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
        const VECTOR< SphFluidDensities > & GetFluidDensitiesAtPcls() const                         { return mFluidDensitiesAtPcls ; }
        const VECTOR< Vec3 > &              GetDensityGradientsAtPcls() const                       { return mDensityGradientsAtPcls ; }
#endif
        const VECTOR< float > &             GetProximities() const                                  { return mVortonBodyProximities ; }

#if COMPUTE_PRESSURE_GRADIENT
        /// Return grid representing pressure gradient.
        const UniformGrid< Vec3 > &         GetPressureGradientGrid() const                         { return mPressureGradientGrid ; }
#endif

        /// Return reference to signed distance function grid.
        /// This represents the distance to the nearest surface.
        const UniformGrid< float > &        GetSignedDistanceGrid() const                           { return mSignedDistanceGrid ; }

#if REDUCE_CONVERGENCE
        const UniformGrid< VECTOR< unsigned > > & GetVortonIndicesGrid() const { return mVortonIndicesGrid ; }
        static float ReduceDivergence( VECTOR< Vorton > * vortons , const UniformGrid< VECTOR< unsigned > > & ugVortonIndices ) ;
#endif

        /// Set the uniform gravitational acceleration.
        /// This affects buoyancy and the baroclinic generation of vorticity.
        void                        SetGravitationalAcceleration( const Vec3 & gravAccel )  { mGravAccel = gravAccel ; }
        const Vec3  &               GetGravitationalAcceleration()                          { return mGravAccel ; }

        /// Set whether to populate mass density grid from vortons and SDF grid from mass density grid.
        void SetPopulateSdfFromDensity( bool populateSdfFromDensity )
        {
            mPopulateSdfFromDensity = populateSdfFromDensity ;
        }

        /// Return the template for all grids used in this simulation, including density, velocity, vorticity and mass fractions.
        /// \see GetVelocityGrid, GetDensityGrid, GetFuelGrid, GetFlameGrid, GetSmokeGrid.
        const UniformGridGeometry & GetGrid()                                               { return mGridTemplate ; }

        /// Get coordinate for minimal (lower left bottom) corner of grids used in this simulation.
        const Vec3 &                GetMinCorner() const                                    { return mMinCorner ; }

        /// Get coordinate for maximal (upper right top) corner of grids used in this simulation.
        const Vec3 &                GetMaxCorner() const                                    { return mMaxCorner ; }

        /// Get coordinate for center of grids used in this simulation.
        Vec3                        GetBoundingBoxCenter() const                            { return ( mMinCorner + mMaxCorner ) * 0.5f ; }

        /// Get size of grids used in this simulation.
        Vec3                        GetBoundingBoxSize() const                              { return mMaxCorner - mMinCorner ; }

        const Vec3 &                GetMinCornerEternal() const                             { return mMinCornerEternal ; }
        const Vec3 &                GetMaxCornerEternal() const                             { return mMaxCornerEternal ; }

        InvestigationTermE &        GetInvestigationTerm()                                  { return mInvestigationTerm ; }

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

    #if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS || POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS || POISON_DENSITY_GRADIENT_BASED_ON_GRIDPOINTS_INSIDE_WALLS || COMPUTE_PRESSURE_GRADIENT
        void SetPhysicalObjects( VECTOR< Impulsion::PhysicalObject * > * physicalObjects ) { mPhysicalObjects = physicalObjects ; }
    #endif

    private:
        void        AssignVortonsFromVorticity( UniformGrid< Vec3 > & vortGrid ) ;
        void        TallyLinearImpulseFromVelocity( Vec3 & linearImpulse ) const ;
        void        TallyDiagnosticIntegrals( Vec3 & vCirculation , Vec3 & vLinearImpulseFromVorticity , Vec3 & vLinearImpulseFromVelocity , Vec3 & vAngularImpulse ) const ;
        void        ConditionallyTallyDiagnosticIntegrals( Integrals & integrals ) const ;

        // Integral-based velocity-from-vorticity routines
        void        MakeBaseVortonGrid( NestedGrid< Vorton > & influenceTree ) ;
        void        AggregateClusters( unsigned uParentLayer , NestedGrid< Vorton > & influenceTree ) ;
        void        CreateInfluenceTree( NestedGrid< Vorton > & influenceTree ) ;
        Vec3        ComputeVelocity_Direct( const Vec3 & vPosition ) ;
        Vec3        ComputeVelocity_Tree( const Vec3 & vPosition , const unsigned idxParent[3] , size_t iLayer , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree ) ;
        Vec3        ComputeVelocity_Monopoles( const unsigned indices[3] , const Vec3 & vPosition ) ;
        void        ComputeVelocityAtGridpoints_Slice( size_t izStart , size_t izEnd , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree ) ;
        void        ComputeVelocityAtVortons_Slice( size_t iPclStart , size_t iPclEnd , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree ) ;
        void        ComputeVelocityFromVorticity_Integral( const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree ) ;
        void        ComputeVelocityFromVorticity_Differential( const UniformGrid< Vec3 > & vorticityGrid ) ;
        void        ComputeVelocityFromVorticity( const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree , const UniformGrid< Vec3 > & vorticityGrid ) ;

        // Differential-based velocity-from-vorticity routines
        void        PopulateVorticityGridFromVortons( UniformGrid< Vec3 > & vorticityGrid , float scale ) ;

        // Other vorticity equation terms
        void        StretchAndTiltVortons( const float & timeStep , const unsigned & uFrame ) ;
        void        PopulateVelocityGrid( UniformGrid< Vec3 > & velocityGrid , const VECTOR< Particle > & particles ) ;
        void        PopulateDensityAndMassFractionGrids( UniformGrid< float > & densityGrid , const VECTOR< Particle > & particles , const unsigned uFrame ) ;
        void        PopulateSignedDistanceGridFromDensityGrid( UniformGrid< float > & signedDistanceGrid , const UniformGrid< float > & densityGrid , const unsigned uFrame) ;

        inline void CombustAndSetMassFractions( float timeStep , Vorton & rVorton ) ;

#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS
        void        GenerateBaroclinicVorticitySlice( float timeStep , const VECTOR< Vec3 > & densityGradients , size_t izStart , size_t izEnd ) ;
        void        GenerateBaroclinicVorticity( const float timeStep , const unsigned uFrame , const UniformGrid< VECTOR< unsigned > > & ugVortonIndices ) ;
#else
        void        GenerateBaroclinicVorticitySlice( float timeStep , size_t izStart , size_t izEnd ) ;
        void        GenerateBaroclinicVorticity( const float timeStep , const unsigned uFrame ) ;
#endif

        void        PartitionVortons( const float & /* timeStep */ , const unsigned & /* uFrame */ ,
#if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION
                                 SpatialPartition & vortonGrid
#else
                                 UniformGrid< VECTOR< unsigned > > & ugVortonIndices
#endif
                                 , float minCellSpacing = 0.0f ) ;


        inline bool ExchangeVorticityOrMergeVortons( const unsigned & rVortIdxHere , Vorton & rVortonHere , Vec3 & rAngVelHere , const unsigned & ivThere , VECTOR< unsigned > & cell , const float & timeStep ) ;
        void        DiffuseVorticityGlobally( const float & timeStep , const unsigned & uFrame ) ;
        void        DiffuseAndDissipateVorticityPSESlice( const float & timeStep , UniformGrid< VECTOR< unsigned > > & ugVortRef , size_t izStart , size_t izEnd , PhaseE phase ) ;
        void        DiffuseAndDissipateVorticityPSE( const float & timeStep , const unsigned & uFrame , UniformGrid< VECTOR< unsigned > > & ugVortonIndices ) ;

        void        UpdateVortexParticleMethod( float timeStep , unsigned uFrame ) ;
#if USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
        void        UpdateSmoothedParticleHydrodynamics( float timeStep , unsigned uFrame ) ;
#endif

        inline void ExchangeHeat( const unsigned & rVortIdxHere , Vorton & rVortonHere , float & rDensityHere , const unsigned & ivThere , const VECTOR< unsigned > & cell , const float & timeStep ) ;
        void        DiffuseAndDissipateHeatPSESlice( const float & timeStep , const UniformGrid< VECTOR< unsigned > > & ugVortRef , size_t izStart , size_t izEnd , PhaseE phase ) ;
        void        DiffuseAndDissipateHeatPSE( const float & timeStep , const unsigned & uFrame , const UniformGrid< VECTOR< unsigned > > & ugVortonIndices ) ;

        UniformGrid< Vec3 >             mVelGrid                    ;   ///< Uniform grid of velocity values

        UniformGrid< VECTOR< unsigned > >   mVortonIndicesGrid  ;   ///< Spatial partition of indices into mVortons. Cached for external use.

        /** Effectively scale vorton radius and preserve total circulation.

            To avoid the high-velocity spike near the periphery of a vorton,
            effectively increase its radius and decrease its vorticity.
            This has no net effect outside the vorton radius, but inside,
            the speed that the vorton induces is lower and smoother than it
            would be otherwise.  This is similar to mollification.
        */
        float                           mSpreadingRangeFactor       ;   ///< Amount by which to scale vorton radius during integral velocity-from-vorticity calculation.
        float                           mSpreadingCirculationFactor ;   ///< Amount by which to scale vorton vorticity during velocity-from-vorticity calculation.  Must be 1/(mSpreadingRangeFactor^3).

        UniformGrid< float >            mDensityGrid                ;   ///< Uniform grid of density.
        UniformGrid< float >            mSignedDistanceGrid         ;   ///< Uniform grid of signed distance values, for tracking fluid surfaces.
        bool                            mPopulateSdfFromDensity     ;   ///< Whether to populate SDF grid from density grid each timestep.  Useful when using SPH simulation technique and simulation is not tracking SDF from tracers.

    #if ENABLE_FIRE
        UniformGrid< float >            mFuelFractionGrid           ;   ///< Uniform grid of fuel fraction values.
        UniformGrid< float >            mFlameFractionGrid          ;   ///< Uniform grid of flame fraction values.
        UniformGrid< float >            mSmokeFractionGrid          ;   ///< Uniform grid of flame fraction values.
    #endif

        UniformGridGeometry             mGridTemplate               ;   ///< Geometry of grid used to contain velocity, vorticity, etc.
        Vec3                            mMinCorner                  ;   ///< Minimal corner of axis-aligned bounding box
        Vec3                            mMaxCorner                  ;   ///< Maximal corner of axis-aligned bounding box
        Vec3                            mMinCornerEternal           ;   ///< Minimal corner of axis-aligned bounding box, across all time
        Vec3                            mMaxCornerEternal           ;   ///< Maximal corner of axis-aligned bounding box, across all time
        Vec3                            mCirculationInitial         ;   ///< Initial total circulation, which should be conserved when viscosity is zero.
        Vec3                            mLinearImpulseInitial       ;   ///< Initial linear impulse, which should be invariant when there are no external non-conservative forces.
        Vec3                            mAngularImpulseInitial      ;   ///< Initial angular impulse, which should be invariant when there are no external non-conservative forces.
        float                           mViscosity                  ;   ///< Viscosity.  Used to compute viscous diffusion.
        float                           mThermalDiffusivity         ;   ///< Thermal diffusivity.  Used to compute heat diffusion.
        float                           mSpecificHeatCapacity       ;   ///< Amount of heat required to change the temperature of one unit of mass by one unit of temperature.
        float                           mAmbientDensity             ;   ///< Ambient fluid density -- density assumed in the absence of particles.
        Vec3                            mGravAccel                  ;   ///< Acceleration due to gravity

        FluidSimulationTechniqueE       mFluidSimTechnique          ;   ///< Fluid simulation technique.
        //VelocityFromVorticityTechnique  mVelFromVortTechnique       ;   ///< Which technique to obtain velocity from vorticity.

        bool                            mTallyDiagnosticIntegrals   ;   ///< Whether to tally integrals
        DiagnosticIntegrals             mDiagnosticIntegrals        ;   ///< Integrals that should be invariant.

        VorticityTermsStatistics        mVorticityTermsStats        ;   ///< Statistics of terms of the vorticity equation.
        InvestigationTermE              mInvestigationTerm          ;   ///< Which term of the vorticity equation to investigate.

    // Probably none of these exdented members should be members;
    // all should be ephemeral, existing only during an update.
    // In the case of mVortons, probably passed in from outside.
    // As members, they get copied during assignment, which is worse than useless.
    VECTOR< Vorton > *                  mVortons                    ;   ///< Dynamic array of tiny vortex elements
    NestedGrid< Vec3 >                  mVorticityMultiGrid         ;   ///< Multi-resolution grid populated with vorticity from vortons
    UniformGrid< Vec3 >                 mDensityGradientGrid        ;   ///< Uniform grid of density gradient values
    VECTOR< float >                     mVortonBodyProximities      ;   ///< Proximities (partially truncated signed distance) of vortons to body walls.

    #if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
        VECTOR< SphFluidDensities >     mFluidDensitiesAtPcls   ;
        VECTOR< Vec3 >                  mDensityGradientsAtPcls ;
    #endif

    #if COMPUTE_PRESSURE_GRADIENT
        UniformGrid< Vec3 >             mPressureGradientGrid       ;   ///< Uniform grid of pressure gradient values
    #endif

    #if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS || POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS || POISON_DENSITY_GRADIENT_BASED_ON_GRIDPOINTS_INSIDE_WALLS || COMPUTE_PRESSURE_GRADIENT
        VECTOR< Impulsion::PhysicalObject * > * mPhysicalObjects ; ///< Used to detect which vortons are inside walls.  Experimental / temporary code.
    #endif

    #if ENABLE_FIRE
        float                           mCombustionTemperature      ;   ///< Activation temperature for combustion reaction.
        float                           mCombustionRateFactor       ;   ///< Coefficient throttling combustion rate.
        float                           mSmokeTemperature           ;   ///< Temperature below which smoke starts to form from flame, i.e. temperature of a glowing hot blackbody.
        float                           mSmokeRateFactor            ;   ///< Coefficient throttling smoke production rate.
        float                           mSpecificFreeEnergy         ;   ///< Amount of heat required to change temperature, per unit mass.
    #endif

    #if defined( _DEBUG ) || OUTPUT_STRETCH_TILT || VORTON_SIM_OUTPUT_DENSITY
        bool                            mOutputDiagnostics      ;   ///< Whether to output diagnostic info
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

#if COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS
    extern void ComputeDensityGradientFromVortons( VECTOR< Vec3 > & densityGradient , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid , const VECTOR< Vorton > & vortons ) ;
#endif

#endif
