//#pragma optimize( "" , off )
/** \file vortonSim.cpp

    \brief Dynamic simulation of a fluid, using tiny vortex elements.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-18/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

    \author Written and copyright 2009-2016 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "vortonSim.h"

#include "vortonClusterAux.h"

#include "Particles/Operation/pclOpFindBoundingBox.h"
#include "Particles/Operation/pclOpPopulateVelocityGrid.h"
#include "Particles/Operation/pclOpEmit.h"

#include "Core/SpatialPartition/uniformGridMath.h"

#include "Core/Performance/perfBlock.h"

#include "Core/useTbb.h"

#include <limits>
#include <algorithm>
#include <stdlib.h>




#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH
// Routines defined in smoothedPclHydro.cpp.
void ComputeSphDensityAtParticles_Grid( VECTOR< SphFluidDensities > & fluidDensitiesAtPcls , const VECTOR< Vorton > & particles , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid ) ;
void ComputeSphMassDensityGradient_Grid( VECTOR< Vec3 > & massDensityGradients
                                        , const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                                        , const VECTOR< Vorton > & particles
                                        , const VECTOR< float > & proximities
                                        , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                                        , const float ambientDensity ) ;
#endif

#include "FluidBodySim/fluidBodySim.h"  // Included for experimental poisoning feature.




/** Whether to use Arrhenius equation to determine combustion rate.

    The alternative uses a simplified threshold model.
*/
#define USE_ARRHENIUS 1




const float sInvalidDensity = UNIFORM_GRID_INVALID_VALUE ;

static const unsigned INVALID_INDEX = ~0UL ;




#if USE_TBB
    /** Function object to compute vector potential at gridpoints using Threading Building Blocks.
    */
    class VortonSim_ComputeVectorPotentialAtGridpoints_TBB
    {
            VortonSim *                                 mVortonSim          ;    ///< Address of VortonSim object
            bool                                        mBoundariesOnly     ;
            const UniformGrid< VECTOR< unsigned > > &   mVortonIndicesGrid  ;
            const NestedGrid< Vorton > &                mInfluenceTree      ;
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of vector potential grid.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mVortonSim->ComputeVectorPotentialAtGridpoints_Slice( r.begin() , r.end() , mBoundariesOnly , mVortonIndicesGrid , mInfluenceTree ) ;
            }
            VortonSim_ComputeVectorPotentialAtGridpoints_TBB( VortonSim * pVortonSim , bool boundariesOnly , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
                : mVortonSim( pVortonSim )
                , mBoundariesOnly( boundariesOnly )
                , mVortonIndicesGrid( vortonIndicesGrid )
                , mInfluenceTree( influenceTree )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;


    /** Function object to compute velocity at gridpoints using Threading Building Blocks.
    */
    class VortonSim_ComputeVelocityAtGridpoints_TBB
    {
            VortonSim *                                 mVortonSim          ;    ///< Address of VortonSim object
            const UniformGrid< VECTOR< unsigned > > &   mVortonIndicesGrid  ;
            const NestedGrid< Vorton > &                mInfluenceTree      ;
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of velocity grid.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mVortonSim->ComputeVelocityAtGridpoints_Slice( r.begin() , r.end() , mVortonIndicesGrid , mInfluenceTree ) ;
            }
            VortonSim_ComputeVelocityAtGridpoints_TBB( VortonSim * pVortonSim , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
                : mVortonSim( pVortonSim )
                , mVortonIndicesGrid( vortonIndicesGrid )
                , mInfluenceTree( influenceTree )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;


    /** Function object to compute velocity at vortons using Threading Building Blocks.
    */
    class VortonSim_ComputeVelocityAtVortons_TBB
    {
            VortonSim *                                 mVortonSim          ;    ///< Address of VortonSim object
            const UniformGrid< VECTOR< unsigned > > &   mVortonIndicesGrid  ;
            const NestedGrid< Vorton > &                mInfluenceTree      ;
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of velocity grid.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mVortonSim->ComputeVelocityAtVortons_Slice( r.begin() , r.end() , mVortonIndicesGrid , mInfluenceTree ) ;
            }
            VortonSim_ComputeVelocityAtVortons_TBB( VortonSim * pVortonSim , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
                : mVortonSim( pVortonSim )
                , mVortonIndicesGrid( vortonIndicesGrid )
                , mInfluenceTree( influenceTree )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;


    /** Function object to compute fluid buoyancy using Threading Building Blocks.
    */
    class VortonSim_GenerateBaroclinicVorticity_TBB
    {
            float                   mTimeStep                   ;   /// Duration since last time step.
            VortonSim *             mVortonSim                  ;   /// Address of VortonSim object
            const VECTOR< Vec3 > *  mDensityGradientsPerVorton  ;   /// Density gradients per vortex particle.  Only used for COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH and COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS.
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of velocity grid.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
#           if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS
                mVortonSim->GenerateBaroclinicVorticitySlice( mTimeStep , mDensityGradientsPerVorton , r.begin() , r.end() ) ;
#           else
                mVortonSim->GenerateBaroclinicVorticitySlice( mTimeStep , r.begin() , r.end() ) ;
#           endif
            }
            VortonSim_GenerateBaroclinicVorticity_TBB( float timeStep , VortonSim * pVortonSim , const VECTOR< Vec3 > * densityGradientsPerVorton )
                : mTimeStep( timeStep )
                , mVortonSim( pVortonSim )
                , mDensityGradientsPerVorton( densityGradientsPerVorton )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;


    /** Function object to compute vorticity diffusion using Threading Building Blocks.
    */
    class VortonSim_DiffuseVorticityPSE_TBB
    {
            float                               mTimeStep       ;   ///< Duration since last time step.
            VortonSim *                         mVortonSim      ;   ///< Address of VortonSim object
            UniformGrid< VECTOR< unsigned > > & mUgVortonIndices;   ///< Reference to uniform grid of vorton indices
            VortonSim::PhaseE                   mPhase          ;   ///< Processing phase: whether to run on odd, even or both values for z slice.
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of vorticity diffusion.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mVortonSim->DiffuseAndDissipateVorticityPSESlice( mTimeStep , mUgVortonIndices , r.begin() , r.end() , mPhase ) ;
            }
            VortonSim_DiffuseVorticityPSE_TBB( float timeStep , VortonSim * pVortonSim , UniformGrid< VECTOR< unsigned > > & ugVortonIndices , VortonSim::PhaseE phase )
                : mTimeStep( timeStep )
                , mVortonSim( pVortonSim )
                , mUgVortonIndices( ugVortonIndices )
                , mPhase( phase )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;


    /** Function object to compute heat diffusion using Threading Building Blocks.
    */
    class VortonSim_DiffuseHeatPSE_TBB
    {
            float                                       mTimeStep       ;   ///< Duration since last time step.
            VortonSim *                                 mVortonSim      ;   ///< Address of VortonSim object
            const UniformGrid< VECTOR< unsigned > > &   mUgVortonIndices;   ///< Reference to uniform grid of vorton indices
            VortonSim::PhaseE                           mPhase          ;   ///< Processing phase.
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of heat diffusion.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mVortonSim->DiffuseAndDissipateHeatPSESlice( mTimeStep , mUgVortonIndices , r.begin() , r.end() , mPhase ) ;
            }
            VortonSim_DiffuseHeatPSE_TBB( float timeStep , VortonSim * pVortonSim , const UniformGrid< VECTOR< unsigned > > & ugVortonIndices , VortonSim::PhaseE phase )
                : mTimeStep( timeStep )
                , mVortonSim( pVortonSim )
                , mUgVortonIndices( ugVortonIndices )
                , mPhase( phase )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;


#if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
    static void PoisonDensityGradientSlice( UniformGrid< Vec3 > & densityGradientGrid , const VECTOR< Vorton > & particles , size_t iPclStart , size_t iPclEnd ) ;


    /** Function object to poison density gradient using Threading Building Blocks.
    */
    class VortonSim_PoisonDensityGradient_TBB
    {
            UniformGrid< Vec3 > &       mDensityGradientGrid;   ///< Reference to uniform grid of density gradient.
            const VECTOR< Vorton > &    mVortons            ;   ///< Dynamic array of vortex particles.
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Poison a subset of density gradient.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                PoisonDensityGradientSlice( mDensityGradientGrid , mVortons , r.begin() , r.end() ) ;
            }
            VortonSim_PoisonDensityGradient_TBB( UniformGrid< Vec3 > & densityGradientGrid , const VECTOR< Vorton > & vortons )
                : mDensityGradientGrid( densityGradientGrid )
                , mVortons( vortons )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;
#endif

#endif




float gVortonSim_DisplacementMax = - FLT_MAX ;
int gVortonSim_NumRelaxationIters = 0 ; // DO NOT SUBMIT -- Global for diagnostic display in another module.




DEBUG_ONLY( static unsigned sLeafHits = 0 ) ;       ///< Number of times an influence-tree leaf-node was hit while calculating velocity.
DEBUG_ONLY( static unsigned sDescents = 0 ) ;       ///< Descents while traversing influence tree, calculating velocity.




/** Return the relative difference between two floats.

    When the difference is smaller than 1, return the absolute difference.

*/
static inline float RelDiff( float absDiff , float basis )
{
    return ( basis < 1.0f ) ? absDiff : ( absDiff / basis ) ;
}




/** Initialize a set of vorticity invariant integrals.

    These are the set of integrals that should be invariant, that is,
    constant with respect to time.
*/
VortonSim::Integrals::Integrals()
    : mTotalCirculation( 0.0f , 0.0f , 0.0f )
    , mLinearImpulseFromVorticity( 0.0f , 0.0f , 0.0f )
    , mLinearImpulseFromVelocity( 0.0f , 0.0f , 0.0f )
    , mAngularImpulse( 0.0f , 0.0f , 0.0f )
{}




/** Compute the maximum relative difference between this vorticity integral set and that one.

    Vorticity has 3 fundamental invariant integrals:
    -   Total circulation
    -   Linear impulse (also known as hydrodynamic impulse)
    -   Angular impulse

    This routine compares them across two stages, determines which has the largest
    difference and returns that value.
    
    This is useful to diagnose problems in the numerical method for vortex-based fluid simulation.
*/
float VortonSim::Integrals::ComputeMaxRelativeDifference( const Integrals & that ) const
{
    float maxRelativeDifferenceMagnitude ;
    {
        const Vec3  totCircDiff         = mTotalCirculation - that.mTotalCirculation ;
        const float totCircDiffMag      = totCircDiff.Magnitude() ;
        maxRelativeDifferenceMagnitude  = RelDiff( totCircDiffMag , mTotalCirculation.Magnitude() ) ;
    }

    {
        const Vec3  linImpVortDiff      = mLinearImpulseFromVorticity - that.mLinearImpulseFromVorticity ;
        const float linImpVortDiffMag   = linImpVortDiff.Magnitude() ;
        const float linImpVortDiffMagRel= RelDiff( linImpVortDiffMag , mLinearImpulseFromVorticity.Magnitude() ) ;
        maxRelativeDifferenceMagnitude  = Max2( maxRelativeDifferenceMagnitude , linImpVortDiffMagRel ) ;
    }

    {
        const Vec3  linImpVelDiff       = mLinearImpulseFromVelocity - that.mLinearImpulseFromVelocity ;
        const float linImpVelDiffMag    = linImpVelDiff.Magnitude() ;
        const float linImpVelDiffMagRel = RelDiff( linImpVelDiffMag , mLinearImpulseFromVelocity.Magnitude() ) ;
        maxRelativeDifferenceMagnitude  = Max2( maxRelativeDifferenceMagnitude , linImpVelDiffMagRel ) ;
    }

    {
        const Vec3  angImpDiff          = mAngularImpulse - that.mAngularImpulse ;
        const float angImpDiffMag       = angImpDiff.Magnitude() ;
        const float angImpDiffMagRel    = RelDiff( angImpDiffMag , mAngularImpulse.Magnitude() ) ;
        maxRelativeDifferenceMagnitude  = Max2( maxRelativeDifferenceMagnitude , angImpDiffMagRel ) ;
    }
    return maxRelativeDifferenceMagnitude ;
}




/** Construct a vorton simulation
*/
VortonSim::VortonSim( float viscosity , float ambientFluidDensity )
    : mSpreadingRangeFactor( 1.0f )
    , mSpreadingCirculationFactor( 1.0f / Pow3( mSpreadingRangeFactor ) )
    , mPopulateSdfFromDensity( false )
    , mMinCorner( FLT_MAX , FLT_MAX , FLT_MAX )
    , mMaxCorner( - mMinCorner )
    , mMinCornerEternal( FLT_MAX , FLT_MAX , FLT_MAX )
    , mMaxCornerEternal( - mMinCorner )
    , mViscosity( viscosity )
    , mThermalDiffusivity( 0.0001f )
    , mSpecificHeatCapacity( 1.0f )
    , mCirculationInitial( 0.0f , 0.0f , 0.0f )
    , mLinearImpulseInitial( 0.0f , 0.0f , 0.0f )
    , mAmbientDensity( ambientFluidDensity )
    , mGravAccel( 0.0f , 0.0f , 0.0f )
#if 0 && USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
    , mFluidSimTechnique( FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS )
#elif 1
    , mFluidSimTechnique( FLUID_SIM_VPM_SPH_HYBRID )
#else
    , mFluidSimTechnique( FLUID_SIM_VORTEX_PARTICLE_METHOD )
#endif
    //, mVelFromVortTechnique( VELOCITY_FROM_VORTICITY_TREE )
    , mTallyDiagnosticIntegrals( false )
    , mInvestigationTerm( INVESTIGATE_ALL )

    , mVortons( 0 )

#if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS || POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS || POISON_DENSITY_GRADIENT_BASED_ON_GRIDPOINTS_INSIDE_WALLS || COMPUTE_PRESSURE_GRADIENT
    , mPhysicalObjects( 0 )
#endif

#if ENABLE_FIRE
    , mCombustionTemperature( Particle_sAmbientTemperature + 1.5f )
    , mCombustionRateFactor( 1.0f )
    , mSmokeTemperature( Particle_sAmbientTemperature + 50.0f )
    , mSmokeRateFactor( 5.0f )
    , mSpecificFreeEnergy( 10000.0f )
#endif
#if defined( _DEBUG ) || OUTPUT_STRETCH_TILT || VORTON_SIM_OUTPUT_DENSITY
    , mOutputDiagnostics( false )
#endif
{
    PERF_BLOCK( VortonSim__VortonSim ) ;

#if defined( _DEBUG ) && 0
    void OutputVelocityProfile( float ) ;
    OutputVelocityProfile( 0.5f ) ;
    OutputVelocityProfile( 1.0f ) ;
    OutputVelocityProfile( 2.0f ) ;
    OutputVelocityProfile( 5.0f ) ;
#endif
}




#if PRESERVE_VORTON_COUNT
/// Compare two vortons by the magnitude of their vorticity, for use with std::sort.
static bool VortonComparisonPredicate( const Vorton & v1 , const Vorton & v2 )
{
    return v1.GetVorticity().Mag2() > v2.GetVorticity().Mag2() ;
}
#endif




/** Assign vortons from a uniform grid of vorticity.

    \param vortGrid - uniform grid of vorticity values

    \note   This routine neglects density and species fractions.
            To properly transfer data from grid to particle would
            require also transfering those quantities.

*/
void VortonSim::AssignVortonsFromVorticity( UniformGrid< Vec3 > & vortGrid )
{
FAIL() ; // This routine has untested changes: mVortonRadius is no longer used.  Computation from cell spacing might be wrong.

#if PRESERVE_VORTON_COUNT
    const size_t numVortonsOrig = mVortons->size() ;
#endif

    mVortons->Clear() ; // Empty out any existing vortons.

    // Obtain characteristic size of each grid cell.
    const UniformGridGeometry & ug  = vortGrid ;
    const float     fVortonRadius   = powf( ug.GetCellSpacing().x * ug.GetCellSpacing().y  * ug.GetCellSpacing().z , 1.0f / 3.0f ) * 0.5f ;
    const Vec3      Nudge           ( ug.GetExtent() * FLT_EPSILON * 4.0f ) ;
    const Vec3      vMin            ( ug.GetMinCorner()   + Nudge ) ;
    const Vec3      vSpacing        ( ug.GetCellSpacing() * ( 1.0f - 8.0f * FLT_EPSILON ) ) ;
    const unsigned  numPoints[3]    = { ug.GetNumPoints(0) , ug.GetNumPoints(1) , ug.GetNumPoints(2) } ;
    const unsigned  numXY           = numPoints[0] * numPoints[1] ;
    const float cellVolume          = ug.GetCellVolume() ;
    const float vortonVolume        = FourPiOver3 * Pow3( fVortonRadius ) ;
    // The vortons resulting from this operation must preserve the
    // same total circulation that the grid has, so multiply
    // by the ratio of the vorton volume to gridcell volume.
    const float volumeCorrection    = cellVolume / vortonVolume ;

    DEBUG_ONLY( Vec3 vCirculationVortons( 0.0f , 0.0f , 0.0f ) ) ;
    DEBUG_ONLY( Vec3 vCirculationGrid( 0.0f , 0.0f , 0.0f ) ) ;

    unsigned idx[3] ;
    for( idx[2] = 0 ; idx[2] < numPoints[2] ; ++ idx[2] )
    {
        Vec3 vortonPosition ;
        vortonPosition.z = vMin.z + float( idx[2] ) * vSpacing.z ;
        const unsigned offsetZ = idx[2] * numXY ;
        for( idx[1] = 0 ; idx[1] < numPoints[1] ; ++ idx[1] )
        {
            vortonPosition.y = vMin.y + float( idx[1] ) * vSpacing.y ;
            const unsigned offsetYZ = idx[1] * vortGrid.GetNumPoints(0) + offsetZ ;
            for( idx[0] = 0 ; idx[0] < numPoints[0] ; ++ idx[0] )
            {
                vortonPosition.x = vMin.x + float( idx[0] ) * vSpacing.x ;
                const unsigned offsetXYZ = idx[0] + offsetYZ ;
                const Vec3 & rVort = vortGrid[ offsetXYZ ] ;
                Vec3 vCirculationCorrectedVorticity = rVort * volumeCorrection ;

                DEBUG_ONLY( vCirculationGrid    += rVort * cellVolume ) ;

                //if( vCirculationCorrectedVorticity.Mag2() > FLT_EPSILON )
                {   // This grid cell contains significant vorticity.
                    Vorton vorton( vortonPosition , vCirculationCorrectedVorticity , fVortonRadius ) ;
                    ASSERT( vorton.GetVorticity().Resembles( vCirculationCorrectedVorticity ) ) ;
                    mVortons->PushBack( vorton ) ;
                    DEBUG_ONLY( vCirculationVortons += vorton.GetVorticity() * vortonVolume ) ;
                }
                ASSERT( vCirculationVortons.Resembles( vCirculationGrid ) ) ;
            }
        }
    }

#if PRESERVE_VORTON_COUNT
    if( mVortons->size() > numVortonsOrig )
    {   // New vorton arrangement has too many vortons.
        // Sort vortons by their magnitude.
        std::sort( mVortons->begin() , mVortons->end() , VortonComparisonPredicate ) ;

    #if 1
        // Tally total circulation from vortons about to be omitted.
        // Note, since all vortons have the same size, angular velocity is a suitable proxy for circulation.
        Vec3 angVelRemoved( 0.0f , 0.0f , 0.0f ) ;
        const size_t numVortonsIntermediate = mVortons->size() ;
        for( size_t iVort = numVortonsOrig ; iVort < numVortonsIntermediate ; ++ iVort )
        {   // For each vorton about to be omitted...
            angVelRemoved += (*mVortons)[ iVort ].mAngularVelocity ;
        #if 0 && defined( _DEBUG )
            printf( "(%s) %5i %3.5g {%3.5g,%3.5g,%3.5g}\n"
                , iVort < numVortonsOrig ? "keep" : "omit"
                , iVort , (*mVortons)[ iVort ].GetVorticity().Magnitude()
                , (*mVortons)[ iVort ].GetVorticity().x , (*mVortons)[ iVort ].GetVorticity().y , (*mVortons)[ iVort ].GetVorticity().z ) ;
        #endif
        }

        const Vec3   angVelRemovedAvg   = angVelRemoved / float( numVortonsOrig ) ;

        // Adjust remaining vortons to keep total circulation unchanged.
        for( size_t iVort = 0 ; iVort < numVortonsOrig ; ++ iVort )
        {   // For each vorton to be kept...
            (*mVortons)[ iVort ].mAngularVelocity += angVelRemovedAvg ;
        }
    #endif

        // Keep the first N.
        // Note that std::sort is O(N log N).
        // To reduce time complexity, we could instead compute the mean & stddev of mVortons
        // and omit vortons below threshold (an O(N) operation).
        // The threshold could be computed as a factor * stddev, where factor could drift
        // from frame to frame using a control algorithm like MIAD.
        // Repeat (k times) until num vortons is what it was before, an O(k N) algorithm
        // where k would be, on average, ~1.
        VECTOR< Vorton >::iterator nth = mVortons->begin() + numVortonsOrig ;
        mVortons->erase( nth , mVortons->end() ) ;
    }
    ASSERT( mVortons->size() == numVortonsOrig ) ;
#endif
}




/** Assign vortons from a uniform grid of vorticity.

    \param linearImpulse - Volume integral of velocity.
        See Saffman section 3.2, "Hydrodynamic impulse", equation 14.

*/
void VortonSim::TallyLinearImpulseFromVelocity( Vec3 & linearImpulse ) const
{
    PERF_BLOCK( VortonSim__TallyLinearImpulseFromVelocity ) ;

    // Zero accumulator.
    linearImpulse = Vec3( 0.0f , 0.0f , 0.0f ) ;

    const UniformGridGeometry & ug  = GetVelocityGrid() ;
    const unsigned  numPoints[3]    = { ug.GetNumPoints(0) , ug.GetNumPoints(1) , ug.GetNumPoints(2) } ;
    const unsigned  numXY           = numPoints[0] * numPoints[1] ;

    unsigned idx[3] ;
    for( idx[2] = 0 ; idx[2] < numPoints[2] ; ++ idx[2] )
    {
        const unsigned offsetZ = idx[2] * numXY ;
        for( idx[1] = 0 ; idx[1] < numPoints[1] ; ++ idx[1] )
        {
            const unsigned offsetYZ = idx[1] * numPoints[0] + offsetZ ;
            for( idx[0] = 0 ; idx[0] < numPoints[0] ; ++ idx[0] )
            {
                const unsigned offsetXYZ = idx[0] + offsetYZ ;
                const Vec3 & velocity = GetVelocityGrid()[ offsetXYZ ] ;
                linearImpulse += velocity ;
            }
        }
    }
    // Apply various factors.
    // GetVolume : The tally code above should multiply velocity by the cell volume.
    // Since cell volume is uniform, we simply apply it here.
    // 1.5: See Saffman 3.2.14.
    linearImpulse *= 1.5f * ug.GetCellVolume() ;
}



/** Compute the total circulation, linear impulse and angular impulse of all vortons in this simulation.

    \param vCirculation (out) Total circulation, the volume integral of vorticity.
        See Saffman section 3.7, "Impulse of isolated vortices", equation 2.

    \param vLinearImpulseFromVorticity  (out) Volume integral of vorticity "weighted" by position.
        See Saffman section 3.2, "Hydrodynamic impulse", equation 8.

    \param vLinearImpulseFromVelocity   (out) Volume integral of velocity.
        See Saffman section 3.2, "Hydrodynamic impulse", equation 14.

    \param vAngularImpulse  (out) Volume integral of vorticity weighted by the second moment of position.
        See Saffman section 3.5, "Angular impulse", equation 2.

    When there are no external non-conservative forces or torques on the fluid, the
    impulses should remain constant over time, that is, they are invariants of the
    fluid motion. This is true even for viscous fluids.  (It is not true for compressible fluids.)
    (See Saffman section 3.6, "Effect of viscosity".)

    \see ConditionallyTallyDiagnosticIntegrals.
*/
void    VortonSim::TallyDiagnosticIntegrals( Vec3 & vCirculation , Vec3 & vLinearImpulseFromVorticity , Vec3 & vLinearImpulseFromVelocity , Vec3 & vAngularImpulse ) const
{
    PERF_BLOCK ( VortonSim__TallyDiagnosticIntegrals ) ;

    if( mVortons->empty() )
    {   // This sim has no vortons.
        return ; // There is nothing to do so leave immediately.
    }

#if defined( _DEBUG )
    const float vortonRadius = (*mVortons)[ 0 ].GetRadius() ;
#endif

    // Zero accumulators.
    vCirculation
        = vLinearImpulseFromVorticity
        = vLinearImpulseFromVelocity
        = vAngularImpulse
        = Vec3( 0.0f , 0.0f , 0.0f ) ;

    const size_t numVortons = mVortons->Size() ;
    for( unsigned iVorton = 0 ; iVorton < numVortons ; ++ iVorton )
    {   // For each vorton in this simulation...
        Vorton &        rVorton     = (*mVortons)[ iVorton ] ;
        const float     radiusCubed = Pow3( rVorton.GetRadius() ) ;
        ASSERT( vortonRadius == rVorton.GetRadius() ) ; // This is assumed elsewhere. Might as well check it here.
        // Accumulate total circulation.
        vCirculation    += rVorton.GetVorticity() * radiusCubed ;
        // Accumulate total linear impulse.
        vLinearImpulseFromVorticity  += rVorton.mPosition ^ rVorton.GetVorticity() * radiusCubed ;
        // Accumulate total angular impulse.
        vAngularImpulse  += POW2( rVorton.mPosition ) * rVorton.GetVorticity() * radiusCubed ;
    }
    // Apply various factors.
    // (4 pi / 3) : The tally code above applies r^3 but should be volume.  For sphere, V=4*pi*r^3/3
    // 0.5 and -0.5 are from the derivations of impulse.  See textbooks.
    vLinearImpulseFromVorticity *=  FourPiOver6 ;
    vAngularImpulse             *= -FourPiOver6 ;

    TallyLinearImpulseFromVelocity( vLinearImpulseFromVelocity ) ;
}




/** Tally diagnostic integrals, if their flag enables it.

    \see TallyDiagnosticIntegrals.
*/
void VortonSim::ConditionallyTallyDiagnosticIntegrals( Integrals & integrals ) const
{
    if( mTallyDiagnosticIntegrals )
    {
        TallyDiagnosticIntegrals( integrals.mTotalCirculation , integrals.mLinearImpulseFromVorticity , integrals.mLinearImpulseFromVelocity , integrals.mAngularImpulse ) ;
    }
}




/** Compute vorticity statistics for all vortex particles.
*/
void VortonSim::GatherVorticityStats( float & min , float & max , float & mean , float & stddev , Vec3 & centerOfVorticity ) const
{
    PERF_BLOCK( VortonSim__GatherVorticityStats ) ;

    Particles::ComputeAngularVelocityStats( reinterpret_cast< const VECTOR< Particle > & >( * mVortons ) , min , max , mean , stddev , centerOfVorticity ) ;
    // Vorticity is twice angular velocity, so double values:
    min     *= 2.0f ;
    max     *= 2.0f ;
    mean    *= 2.0f ;
    stddev  *= 2.0f ;
}




/** Compute temperature statistics for all vortex particles.

    \param min      Minimum temperature of all particles.

    \param max      Maximum temperature of all particles.

    \param mean     Average temperature of all particles.

    \param stddev   Standard deviation of temperature of all particles.
*/
void VortonSim::GatherTemperatureStats( float & min , float & max , float & mean , float & stddev ) const
{
    PERF_BLOCK( VortonSim__GatherTemperatureStats ) ;

    Particles::ComputeTemperatureStats( reinterpret_cast< const VECTOR< Particle > & >( * mVortons ) , min , max , mean , stddev ) ;
}




/** Compute density statistics for all vortex particles.

    \param min      Minimum density of all particles.

    \param max      Maximum density of all particles.

    \param mean     Average density of all particles.

    \param stddev   Standard deviation of density of all particles.
*/
void VortonSim::GatherDensityStats( float & min , float & max , float & mean , float & stddev ) const
{
    PERF_BLOCK( VortonSim__GatherDensityStats ) ;

    Particles::ComputeDensityStats( reinterpret_cast< const VECTOR< Particle > & >( * mVortons ) , min , max , mean , stddev ) ;
}




/** Compute mass density (from SPH) statistics for all vortex particles.

    \param min      Minimum SPH mass density of all particles.

    \param max      Maximum SPH mass density of all particles.

    \param mean     Average SPH mass density of all particles.

    \param stddev   Standard deviation of SPH mass density of all particles.
*/
void VortonSim::GatherDensitySphStats( float & min , float & max , float & mean , float & stddev ) const
{
    PERF_BLOCK( VortonSim__GatherDensitySphStats ) ;

    min = FLT_MAX ;
    max = - min ;
    float           sum     = 0.0f ;
    float           sum2    = 0.0f ;
    const size_t    numPcls = mFluidDensitiesAtPcls.Size() ;
    for( unsigned iPcl = 0 ; iPcl < numPcls ; ++ iPcl )
    {   // For each particle...
        const float & densitySph = GetFluidDensitiesAtPcls()[ iPcl ].mMassDensity ;
        sum += densitySph ;
        sum2 += densitySph * densitySph ;
        min = Min2( min , densitySph ) ;
        max = Max2( max , densitySph ) ;
    }
    mean = sum / float( numPcls ) ;
    const float mean2 = sum2 / float( numPcls ) ;
    stddev = fsqrtf( mean2 - mean * mean ) ;
}




/** Compute number density (from SPH) statistics for all vortex particles.

    \param min      Minimum SPH number density of all particles.

    \param max      Maximum SPH number density of all particles.

    \param mean     Average SPH number density of all particles.

    \param stddev   Standard deviation of SPH number density of all particles.
*/
void VortonSim::GatherNumberDensitySphStats( float & min , float & max , float & mean , float & stddev ) const
{
    PERF_BLOCK( VortonSim__GatherNumberDensitySphStats ) ;

    min = FLT_MAX ;
    max = - min ;
    float           sum     = 0.0f ;
    float           sum2    = 0.0f ;
    const size_t    numPcls = mFluidDensitiesAtPcls.Size() ;
    for( unsigned iPcl = 0 ; iPcl < numPcls ; ++ iPcl )
    {   // For each particle...
        const float & numberDensitySph = mFluidDensitiesAtPcls[ iPcl ].mNumberDensity ;
        sum += numberDensitySph ;
        sum2 += numberDensitySph * numberDensitySph ;
        min = Min2( min , numberDensitySph ) ;
        max = Max2( max , numberDensitySph ) ;
    }
    mean = sum / float( numPcls ) ;
    const float mean2 = sum2 / float( numPcls ) ;
    stddev = fsqrtf( mean2 - mean * mean ) ;
}




/** Compute density gradient statistics for all vortex particles.
*/
void VortonSim::GatherDensityGradientStats( float & min , float & max , float & mean , float & stddev ) const
{
    PERF_BLOCK( VortonSim__GatherDensityGradientStats ) ;

    min = FLT_MAX ;
    max = - min ;
    float           sum     = 0.0f ;
    float           sum2    = 0.0f ;
    const size_t    numPcls = GetDensityGradientsAtPcls().Size() ;
    for( unsigned iPcl = 0 ; iPcl < numPcls ; ++ iPcl )
    {   // For each vorton in this simulation...
        const Vec3 & densGrad = GetDensityGradientsAtPcls()[ iPcl ] ;
        const float densGradMag = densGrad.Magnitude() ;
        sum  += densGradMag ;
        sum2 += densGradMag * densGradMag ;
        min = Min2( min , densGradMag ) ;
        max = Max2( max , densGradMag ) ;
    }
    mean = sum / float( numPcls ) ;
    const float mean2 = sum2 / float( numPcls ) ;
    stddev = fsqrtf( mean2 - mean * mean ) ;
}




/** Compute signed distance statistics for SDF grid.

    \param min      Minimum SDF in grid.

    \param max      Maximum SDF in grid.

    \param mean     Average SDF in grid.

    \param stddev   Standard deviation of SDF in grid.
*/
void VortonSim::GatherSignedDistanceStats( float & min , float & max , float & mean , float & stddev ) const
{
    PERF_BLOCK( VortonSim__GatherSignedDistanceStats ) ;

    FindValueStats( mSignedDistanceGrid , min , max , mean , stddev ) ;
}




/** Compute particle-to-body-wall proximity statistics for all vortex particles.

    \param min      Minimum proximity of all particles.

    \param max      Maximum proximity of all particles.

    \param mean     Average proximity of all particles.

    \param stddev   Standard deviation of proximity of all particles.
*/
void VortonSim::GatherProximityStats( float & min , float & max , float & mean , float & stddev ) const
{
    PERF_BLOCK( VortonSim__GatherProximityStats ) ;

    min = FLT_MAX ;
    max = - min ;
    float           sum     = 0.0f ;
    float           sum2    = 0.0f ;
    const size_t    numPcls = GetProximities().Size() ;
    for( unsigned iPcl = 0 ; iPcl < numPcls ; ++ iPcl )
    {   // For each particle...
        const float & proximity = GetProximities()[ iPcl ] ;
        sum += proximity ;
        sum2 += proximity * proximity ;
        min = Min2( min , proximity ) ;
        max = Max2( max , proximity ) ;
    }
    mean = sum / float( numPcls ) ;
    const float mean2 = sum2 / float( numPcls ) ;
    stddev = fsqrtf( mean2 - mean * mean ) ;
}




#if 0 && defined( _DEBUG )
static void TestBiotSavart()
{
    PERF_BLOCK( TestBiotSavart ) ;

    // Exercise regularized Biot-Savart formula.
    FILE * velFromVortFile = fopen( "TestData/velFromVort.dat" , "w" ) ;
    if( velFromVortFile != NULL )
    {
        const float spreadingRangeFactor        = 1.0f ;
        const float spreadingCirculationFactor  = 1.0f / Pow3( spreadingRangeFactor ) ;
        const Vec3  vortonPosition( 0.0f , 0.0f , 0.0f ) ;
        const Vec3  vortonAngVel( 0.0f , 0.0f , 1.0f ) ;
        const float vortonDiameter           = 1.0f ;
        const float queryDistanceMax         = vortonDiameter * 10.0f ;
        const int   numQueryPositions        = 10000 ;
        const float oneOverNumQueryPositions = 1.0f / float( numQueryPositions ) ;
        fprintf( velFromVortFile , "# r vel\n" ) ;
        for( int iQueryPos = 0 ; iQueryPos < numQueryPositions ; ++ iQueryPos )
        {
            const float queryDist = queryDistanceMax * float( iQueryPos ) * oneOverNumQueryPositions ;
            Vec3        queryPositionAlongX( queryDist , 0.0f , 0.0f ) ;
            Vec3        velocityFromVorticity( 0.0f , 0.0f , 0.0f ) ;
            VORTON_ACCUMULATE_VELOCITY_private( velocityFromVorticity , queryPositionAlongX , vortonPosition , vortonAngVel , vortonDiameter , spreadingRangeFactor , spreadingCirculationFactor ) ;
            {
                Vec3        queryPositionAlongY( 0.0f , queryDist , 0.0f ) ;
                Vec3        velocityFromVorticityCheck( 0.0f , 0.0f , 0.0f ) ;
                VORTON_ACCUMULATE_VELOCITY_private( velocityFromVorticityCheck , queryPositionAlongX , vortonPosition , vortonAngVel , vortonDiameter , spreadingRangeFactor , spreadingCirculationFactor ) ;
                ASSERT( velocityFromVorticity.y == - velocityFromVorticityCheck.x ) ; // Make sure direction orthogonal to vorticity does not matter.
            }
            fprintf( velFromVortFile , "%g %g\n" , queryDist , velocityFromVorticity.y ) ;
        }
        fclose( velFromVortFile ) ;
    }
}
#endif




/** Initialize a vortex particle fluid simulation.
*/
void VortonSim::Initialize( bool systemTracksFluidSurfaceUsingSdf )
{
    PERF_BLOCK( VortonSim__Initialize ) ;

#if USE_TBB
    gNumberOfProcessors = GetNumberOfProcessors() ;
    #if PROFILE
        printf( "# CPU's: %u.  Built " __DATE__ " " __TIME__ "\n" , gNumberOfProcessors ) ;
    #endif
#endif

    if( mVortons->empty() )
    {   // There are no vortons initially.
        // None of the processing in this routine applies.
        return ;
    }

    const float vortonRadius = (*mVortons)[ 0 ].GetRadius() ;

    TallyDiagnosticIntegrals( mDiagnosticIntegrals.mInitial.mTotalCirculation , mDiagnosticIntegrals.mInitial.mLinearImpulseFromVorticity , mDiagnosticIntegrals.mInitial.mLinearImpulseFromVelocity , mDiagnosticIntegrals.mInitial.mAngularImpulse ) ;

    // Find grid geometry to seed passive tracer particles.
    FindBoundingBox() ; // Find axis-aligned bounding box that encloses all vortons.

    // Create a preliminary grid template, useful for initializing tracers.
    //  This grid define the shape of the velocity grid and the same shape gets
    //  reused for other quantities, which theoretically could have different geometries.
    UpdateBoundingBox( mMinCorner , mMaxCorner , /* finalize? See below. */ false ) ;

    // "Final" UpdateBoundingBox above did not "finalize", because finalization
    // takes into account the velocity solver technique, so the bounding box is
    // padded differently for different solvers.
    // At this phase, we only want to know the snug-fitting box, because we're
    // using that to compute particle sizes, which should not be influenced by
    // the velocity solver technique. So perform necessary finalizing operations here.
    {
        // Enlarge bounding box by 1 radius to include whole particle.
        // Further enlarge box by 1 diameter to include boundary so that outermost particles do not occupy outer band.
        const float margin = 3.0f * vortonRadius ;
        const Vec3 nudge( margin * Vec3( 1.0f , 1.0f , 1.0f ) ) ;
        mMinCorner -= nudge ;
        mMaxCorner += nudge ;
        mGridTemplate.DefineShape( mVortons->Size() , mMinCorner , mMaxCorner , true ) ;
    }

    // Compute initial density grid.
    ASSERT( sizeof( Particle ) == sizeof( Vorton ) ) ;
    PopulateDensityAndMassFractionGrids( mDensityGrid , reinterpret_cast< VECTOR< Particle > & >( * mVortons ) , 0 ) ;

    if( systemTracksFluidSurfaceUsingSdf )
    {
        // Compute initial signed distance field.
        // This low-res version is only used to initialize tracers and then subsequently not assigned from vortons.
        PopulateSignedDistanceGridFromDensityGrid( mSignedDistanceGrid , mDensityGrid , 0 ) ;
    }

#if 0 && defined( _DEBUG )
    TestBiotSavart() ;
#endif
}




/** Find axis-aligned bounding box for all vortons in this simulation.

    \see UpdateBoundingBox which is used to update the box to include tracers
*/
void VortonSim::FindBoundingBox()
{
    PERF_BLOCK( VortonSim__FindBoundingBox ) ;

    mMinCorner = Vec3( FLT_MAX , FLT_MAX , FLT_MAX ) ;
    mMaxCorner = - mMinCorner ;
    PclOpFindBoundingBox::FindBoundingBox( reinterpret_cast< const VECTOR<Particle> & >( * GetVortons() ) , mMinCorner , mMaxCorner ) ;
}




/** Update axis-aligned bounding box to include given region.

    \param minCorner - minimal corner of region to include

    \param maxCorner - maximal corner of region to include

    \param bFinal - whether this is the final update to the bounding box.
            On the final update, this routine performs finalization operations.
            They should only happen once, and only after all updates are done.

    \note that this operation does not shrink the box; it can enlarge it.

    \see FindBoundingBox
*/
void VortonSim::UpdateBoundingBox( const Vec3 & minCorner , const Vec3 & maxCorner , bool bFinal )
{
    PERF_BLOCK( VortonSim__UpdateBoundingBox ) ;

    mMinCorner.x = Min2( mMinCorner.x , minCorner.x ) ;
    mMinCorner.y = Min2( mMinCorner.y , minCorner.y ) ;
    mMinCorner.z = Min2( mMinCorner.z , minCorner.z ) ;
    mMaxCorner.x = Max2( mMaxCorner.x , maxCorner.x ) ;
    mMaxCorner.y = Max2( mMaxCorner.y , maxCorner.y ) ;
    mMaxCorner.z = Max2( mMaxCorner.z , maxCorner.z ) ;

    if( bFinal )
    {   // This is the final amendment to the bounding box.

        if( ! mVortons->empty() )
        {
            const float vortonRadius = (*mVortons)[ 0 ].GetRadius() ;

            // Enlarge bounding box to include entire vorton, not just its center. (+1)
            // 
            // Further enlarge it to include an entire vorton's width so that outer cells contain no vortons. (+2)
            // That is to make sure the SDF has a zero-crossing before the outer boundary of the domain.
            // Technically, expanding the domain in this way does not guarantee the outer layer of grid cells lacks vortons;
            // It only guarantees the domain is padded to keep vortons away from the outer layer of gridpoints.
            static const float margin = ( 1.0f + 2.0f ) * vortonRadius ;

        #define JITTER_BOX 0
        #if JITTER_BOX  // Randomly jitter bounding box
            static const float  sScaleOverRandMax   = 0.1f / float( RAND_MAX ) ;
            const float         jitter              =  float( rand() ) * sScaleOverRandMax ;
            const Vec3          domainSize          ( mMaxCorner - mMinCorner ) ;
            const Vec3          vJitter             = jitter * domainSize ;
            const Vec3          nudge               ( margin * Vec3( 1.0f , 1.0f , 1.0f ) + vJitter ) ;
        #else
            const Vec3          nudge               ( margin * Vec3( 1.0f , 1.0f , 1.0f )           ) ;
        #endif

        #if ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL )
            {   // Using differential velocity-from-vorticity technique.
                // Expand boundaries so they do not unduly affect flow.
                // This is a crude but effective approximation of a free fluid.
                //
                // OBSOLETE: Instead of expanding boundaries, VorteGrid now uses integral technique
                // (e.g. treecode or direct summation) to compute velocity-from-vorticity at domain
                // boundaries, then use those as boundary conditions for the Poisson solver.
                const Vec3          domainSize      ( mMaxCorner - mMinCorner ) ;
                static const float  expansionFactor = 0.0 ; // 0.5f ;
                mMinCorner -= expansionFactor * domainSize ;
                mMaxCorner += expansionFactor * domainSize ;
            }
        #endif

            mMinCorner -= nudge ;
            mMaxCorner += nudge ;

            mMinCornerEternal.x = Min2( mMinCorner.x , mMinCornerEternal.x ) ;
            mMinCornerEternal.y = Min2( mMinCorner.y , mMinCornerEternal.y ) ;
            mMinCornerEternal.z = Min2( mMinCorner.z , mMinCornerEternal.z ) ;
            mMaxCornerEternal.x = Max2( mMaxCorner.x , mMaxCornerEternal.x ) ;
            mMaxCornerEternal.y = Max2( mMaxCorner.y , mMaxCornerEternal.y ) ;
            mMaxCornerEternal.z = Max2( mMaxCorner.z , mMaxCornerEternal.z ) ;

            mGridTemplate.DefineShape( mVortons->Size() , mMinCorner , mMaxCorner , true ) ;
        }
        else
        {   // No vortons; define empty, zero-size grid.
            mGridTemplate.DefineShape( 0 , Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , true ) ;
        }
    }
}




/** Compute vector potential at a given point in space, due to influence of vortons, using direct summation.

    \param vPosition    Point in space at which to compute velocity.

    \return Vector potential at vPosition, due to influence of vortons.

    \note   This is a brute-force direct-summation algorithm with time
            complexity O(N) where N is the number of vortons.  This
            is too slow for regular use but it is useful for comparisons,
            since it makes no numerical approximations of consequence.

*/
Vec3 VortonSim::ComputeVectorPotential_Direct( const Vec3 & vPosition )
{
    PERF_BLOCK( VortonSim__ComputeVectorPotential_Direct ) ;

    const size_t    numVortons      = mVortons->Size() ;
    Vec3            vecPotAccumulator( 0.0f , 0.0f , 0.0f ) ;

    for( unsigned iVorton = 0 ; iVorton < numVortons ; ++ iVorton )
    {   // For each vorton...
        const Vorton &  rVorton = (*mVortons)[ iVorton ] ;
        VORTON_ACCUMULATE_VECTOR_POTENTIAL( vecPotAccumulator , vPosition , rVorton ) ;
    }

    return vecPotAccumulator ;
}




/** Compute velocity at a given point in space, due to influence of vortons, using direct summation.

    \param vPosition    Point in space at which to compute velocity.

    \return Velocity at vPosition, due to influence of vortons.

    \note   This is a brute-force direct-summation algorithm with time
            complexity O(N) where N is the number of vortons.  This
            is too slow for regular use but it is useful for comparisons,
            since it makes no numerical approximations of consequence.

*/
Vec3 VortonSim::ComputeVelocity_Direct( const Vec3 & vPosition )
{
    PERF_BLOCK( VortonSim__ComputeVelocity_Direct ) ;

    const size_t    numVortons          = mVortons->Size() ;
    Vec3            velocityAccumulator( 0.0f , 0.0f , 0.0f ) ;

    for( unsigned iVorton = 0 ; iVorton < numVortons ; ++ iVorton )
    {   // For each vorton...
        const Vorton &  rVorton = (*mVortons)[ iVorton ] ;
        VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , rVorton ) ;
    }

    return velocityAccumulator ;
}




/** Create base layer of vorton influence tree.

    This is the leaf layer, where each grid cell approximately corresponds (on
    average) to a single vorton.  Some cells might contain multiple vortons and
    some zero. Each cell effectively has a single "supervorton" which its parent
    layers in the influence tree will in turn aggregate.

    \note   This implementation of gridifying the base layer is NOT suitable
            for Eulerian operations like approximating spatial derivatives
            of vorticity or solving a vector Poisson equation, because this
            routine associates each vortex with a single corner point of the
            grid cell that contains it.  To create a grid for Eulerian calculations,
            each vorton would contribute to all 8 corner points of the grid
            cell that contains it.

            We could rewrite this to suit "Eulerian" operations, in which case
            we would want to omit "size" and "position" since the grid would
            implicitly represent that information.  That concern goes hand-in-hand
            with the method used to compute velocity from vorticity.
            Ultimately we need to make sure theoretically conserved quantities behave as expected.

    \note   This method assumes the influence tree skeleton has already been created,
            and the leaf layer initialized to all "zeros", meaning it contains no
            vortons.  This is notable since this routine accumulates values into
            each gridpoint.

    \see    PopulateVorticityGridFromVortons, which populates a grid of vorticity
            that could be used for Eulerian calculations.
*/
void VortonSim::MakeBaseVortonGrid( NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__MakeBaseVortonGrid ) ;

    const size_t numVortons = mVortons->Size() ;

    UniformGrid< VortonClusterAux > ugAux( influenceTree[0] ) ; // Temporary auxilliary information used during aggregation.
    ugAux.Init() ;

    DEBUG_ONLY( unsigned    numVortonsIncorporated = 0 ) ;

#if ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT )
    UniformGrid<Vorton> & baseGrid = influenceTree[0] ;
#else
    #error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
#endif

#if 0 && defined( _DEBUG )
    for( unsigned uVorton = 0 ; uVorton < numVortons ; ++ uVorton )
    {   // For each vorton in this simulation...
        const Vorton     &  rVorton     = (*mVortons)[ uVorton ] ;
        const Vec3       &  rPosition   = rVorton.mPosition   ;
        ASSERT( ! IsNan( rPosition ) && ! IsInf( rPosition ) ) ;
        const unsigned      uOffset     = baseGrid.OffsetOfPosition( rPosition ) ;
        ASSERT( uOffset < baseGrid.GetGridCapacity() ) ;
        Vorton           &  rVortonCell = baseGrid[ uOffset ] ;
        rVortonCell.mTotalCirculation = Vec3( 0.0f , 0.0f , 0.0f ) ;
    }
#endif

    // Compute preliminary vorticity grid.

#if defined( _DEBUG ) // && ( ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) )
    // So far in this simulation there has been no need to vary vorton size within a single simulation.
    // Due to paranoia about accidental reassignment, check to make sure each vorton does indeed have the same size.
    const float fUniformVortonRadius = numVortons ? (*mVortons)[ 0 ].GetRadius() : 0.0f ;
#endif

    for( unsigned uVorton = 0 ; uVorton < numVortons ; ++ uVorton )
    {   // For each vorton in this simulation...
        const Vorton     &  rVorton     = (*mVortons)[ uVorton ] ;

        const Vec3       &  rPosition   = rVorton.mPosition   ;
        ASSERT( ! IsNan( rPosition ) && ! IsInf( rPosition ) ) ;
        const unsigned      uOffset     = baseGrid.OffsetOfPosition( rPosition ) ;
        ASSERT( uOffset < baseGrid.GetGridCapacity() ) ;

        Vorton           &  rVortonCell = baseGrid[ uOffset ] ;
        VortonClusterAux &  rVortonAux  = ugAux[ uOffset ] ;
        const float         vortMag     = rVorton.GetVorticity().Magnitude() ;

        rVortonCell.mPosition  += rVorton.mPosition * vortMag ; // Compute weighted position -- to be normalized later.
        ASSERT( ! IsNan( rVortonCell.mPosition ) && ! IsInf( rVortonCell.mPosition ) ) ;

        ASSERT( rVorton.GetRadius() == fUniformVortonRadius ) ;   // Each vorton should have identical size.
        rVortonCell.mAngularVelocity += rVorton.mAngularVelocity    ; // Tally vorticity sum.
        rVortonCell.mSize             = rVorton.mSize               ; // Assign volume element size.
        DEBUG_ONLY( ++ rVortonCell.mNumVortonsIncorporated ) ;
        DEBUG_ONLY( ++ numVortonsIncorporated ) ;
        DEBUG_ONLY( rVortonCell.mTotalCirculation = rVortonCell.GetVorticity() * Pow3( rVortonCell.GetRadius() ) ) ;

        // OBSOLETE. See comments below: UpdateBoundingBox( rVortonAux.mMinCorner , rVortonAux.mMaxCorner , rVorton.mPosition ) ;
        rVortonAux.mVortNormSum += vortMag ;
    }

    ASSERT( numVortons == numVortonsIncorporated ) ;

#if defined( _DEBUG )
    DEBUG_ONLY( numVortonsIncorporated = 0 ) ;
    Vec3    vTotalCirculation( 0.0f , 0.0f , 0.0f ) ;
    for( unsigned offset = 0 ; offset < influenceTree[0].GetGridCapacity() ; ++ offset )
    {
        Vorton & rVortonCell = influenceTree[0][ offset ] ;
        DEBUG_ONLY( numVortonsIncorporated += rVortonCell.mNumVortonsIncorporated ) ;
        DEBUG_ONLY( const Vec3 vCellCirculation = rVortonCell.GetVorticity() * Pow3( rVortonCell.GetRadius() ) ) ;
        ASSERT( rVortonCell.mTotalCirculation.Resembles( vCellCirculation ) ) ;
        vTotalCirculation += rVortonCell.mTotalCirculation ;
    }
    ASSERT( numVortons == numVortonsIncorporated ) ;
    ASSERT( vTotalCirculation.Resembles( mDiagnosticIntegrals.mAfterAdvect.mTotalCirculation , 1.0e-3f ) ) ;
    DEBUG_ONLY( numVortonsIncorporated = 0 ) ;
#endif

    // Post-process preliminary grid; normalize center-of-vorticity and compute sizes, for each grid cell.
    DEBUG_ONLY( unsigned numGridPointsWithoutVortons = 0 ) ;

    const unsigned num[3] = {   influenceTree[0].GetNumPoints( 0 ) ,
                                influenceTree[0].GetNumPoints( 1 ) ,
                                influenceTree[0].GetNumPoints( 2 ) } ;
    const unsigned numXY = num[0] * num[1] ;
    unsigned idx[3] ;
    for( idx[2] = 0 ; idx[2] < num[2] ; ++ idx[2] )
    {
        const unsigned zShift = idx[2] * numXY ;
        for( idx[1] = 0 ; idx[1] < num[1] ; ++ idx[1] )
        {
            const unsigned yzShift = idx[1] * num[0] + zShift ;
            for( idx[0] = 0 ; idx[0] < num[0] ; ++ idx[0] )
            {
                const unsigned      offset      = idx[0] + yzShift ;
                VortonClusterAux &  rVortonAux  = ugAux[ offset ] ;
                if( rVortonAux.mVortNormSum != FLT_MIN )
                {   // This cell contains at least one vorton.
                    Vorton & rVortonCell = influenceTree[0][ offset ] ;
                    // Normalize weighted position sum to obtain center-of-vorticity.
                    rVortonCell.mPosition /= rVortonAux.mVortNormSum ;
                    ASSERT( ! IsNan( rVortonCell.mPosition ) && ! IsInf( rVortonCell.mPosition ) ) ;

                    DEBUG_ONLY( numVortonsIncorporated += rVortonCell.mNumVortonsIncorporated ) ;
                }
                else
                {
                    DEBUG_ONLY( ++ numGridPointsWithoutVortons ) ;
                }
            }
        }
    }

    // If this triggers but the above apparently identical assertions do not,
    // then the simulation probably has some zero-vorticity vortons.
    // That is okay, but there is logic elsewhere that does not include zero-vorticity
    // vortons into the influence tree, hence they do not get counted as "incorporated".
    // The solution to that is either to "incorporate" even zero-vorticity vortons
    // (which is obviously somewhat silly), ignore this assertion (which is dangerous
    // in situations where it catches legitimate errors), or assign some tiny,
    // otherwise negligible vorticity to all vortons, for example using FLT_EPSILON.
    // While epsilon-strength vortons might seem silly, in practice it helps with
    // diagnosing the algorithm.  For release code, zero-vorticity vortons work just fine.
    ASSERT( numVortons == numVortonsIncorporated ) ;
}




/** Aggregate vorton clusters from a child layer into a parent layer of the influence tree.

    This routine assumes the given parent layer is empty and its child layer (i.e. the layer
    with index uParentLayer-1) is populated.

    \param uParentLayer - index of parent layer into which aggregated influence information will be stored.
        This must be greater than 0 because the base layer, which has no children, has index 0.

    \see CreateInfluenceTree

*/
void VortonSim::AggregateClusters( unsigned uParentLayer , NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__AggregateClusters ) ;

    UniformGrid<Vorton> &   rParentLayer    = influenceTree[ uParentLayer     ] ;
    UniformGrid<Vorton> &   rChildLayer     = influenceTree[ uParentLayer - 1 ] ;
    const unsigned &        numXchild       = rChildLayer.GetNumPoints( 0 ) ;
    const unsigned          numXYchild      = numXchild * rChildLayer.GetNumPoints( 1 ) ;

#if defined( _DEBUG )
    Vec3 vTotalCirculation( 0.0f , 0.0f , 0.0f ) ;
    const float vortonRadius = (*mVortons)[ 0 ].GetRadius() ;
#endif

    // number of cells in each grid cluster
    const unsigned * const pClusterDims = influenceTree.GetDecimations( uParentLayer ) ;

    const unsigned  numCells[3]         = { rParentLayer.GetNumCells( 0 ) , rParentLayer.GetNumCells( 1 ) , rParentLayer.GetNumCells( 2 ) } ;
    const unsigned  numXY               = rParentLayer.GetNumPoints( 0 ) * rParentLayer.GetNumPoints( 1 ) ;
    // (Since this loop writes to each parent cell, it should readily parallelize without contention.)
    unsigned idxParent[3] ;
    for( idxParent[2] = 0 ; idxParent[2] < numCells[2] ; ++ idxParent[2] )
    {
        const unsigned offsetZ = idxParent[2] * numXY ;
        for( idxParent[1] = 0 ; idxParent[1] < numCells[1] ; ++ idxParent[1] )
        {
            const unsigned offsetYZ = idxParent[1] * rParentLayer.GetNumPoints( 0 ) + offsetZ ;
            for( idxParent[0] = 0 ; idxParent[0] < numCells[0] ; ++ idxParent[0] )
            {   // For each cell in the parent layer...
                const unsigned offsetXYZ = idxParent[0] + offsetYZ ;
                Vorton              & rVortonParent = rParentLayer[ offsetXYZ ] ;
                VortonClusterAux vortAux ;
                unsigned clusterMinIndices[ 3 ] ;
                influenceTree.GetChildClusterMinCornerIndex( clusterMinIndices , pClusterDims , idxParent ) ;
                unsigned increment[3] ;
                // For each cell of child layer in this grid cluster...
                for( increment[2] = 0 ; increment[2] < pClusterDims[2] ; ++ increment[2] )
                {
                    const unsigned offsetZ = ( clusterMinIndices[2] + increment[2] ) * numXYchild ;
                    for( increment[1] = 0 ; increment[1] < pClusterDims[1] ; ++ increment[1] )
                    {
                        const unsigned offsetYZ = ( clusterMinIndices[1] + increment[1] ) * numXchild + offsetZ ;
                        for( increment[0] = 0 ; increment[0] < pClusterDims[0] ; ++ increment[0] )
                        {
                            const unsigned  offsetXYZ       = ( clusterMinIndices[0] + increment[0] ) + offsetYZ ;
                            Vorton &        rVortonChild    = rChildLayer[ offsetXYZ ] ;
                            const float     vortMag         = rVortonChild.GetVorticity().Magnitude() ;

                            // Aggregate vorton cluster from child layer into parent layer:
                            rVortonParent.mPosition  += rVortonChild.mPosition * vortMag ;
                            ASSERT( ! IsNan( rVortonParent.mPosition ) && ! IsInf( rVortonParent.mPosition ) ) ;

                            rVortonParent.mAngularVelocity += rVortonChild.mAngularVelocity ;
                            DEBUG_ONLY( rVortonParent.mNumVortonsIncorporated += rVortonChild.mNumVortonsIncorporated ) ;
                            vortAux.mVortNormSum     += vortMag ;
                            if( rVortonChild.mSize != 0.0f )
                            {   // Child vorton exists
                            #if 0 && ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES )
                                // NOTE:    In order to preserve total circulation, changing supervorton radius would require changing its vorticity.
                                //          That would also unfortunately mean changing the long-distance influence of the supervorton, which would be undesirable.
                                //          Hence this code to change supervorton radius is disabled.
                                // Compute parent-to-child distance.
                                const Vec3  vParentToChild  = rVortonParent.mPosition - rVortonChild.mPosition ;
                                const float fParentToChild2 = vParentToChild.Mag2() ;
                                const float fParentToChild4 = POW2( fParentToChild2 ) ;
                                const float fParentToChild8 = POW2( fParentToChild4 ) ;
                                // Formula for parent radius shall be N-norm sum of parent-child distance:
                                // rVortonParent.mRadius += powf( parentChildDist , N ) where N is larger than 2, perhaps 8.
                                // (plus vortonRadius will be added at the very end, outside last loop below)
                                rVortonParent.SetRadius( rVortonParent.GetRadius() + fParentToChild8 ) ;
                                ASSERT( ! IsInf( rVortonParent.mSize ) ) ;
                            #else
                                rVortonParent.mSize = rVortonChild.mSize ;
                            #endif
                            }

                            ASSERT( ( rVortonChild.GetVorticity().Mag2()  < 1.0e-8f ) || ( rVortonChild.GetRadius() == vortonRadius ) ) ;

                        #if VELOCITY_TECHNIQUE != VELOCITY_TECHNIQUE_MONOPOLES
                            DEBUG_ONLY( vTotalCirculation += rVortonChild.GetVorticity() * Pow3( rVortonChild.GetRadius() ) ) ;
                        #endif
                        }
                    }
                }

                ASSERT( ( rVortonParent.GetVorticity().Magnitude() < 1.0e-8f ) || ( rVortonParent.GetRadius() == vortonRadius ) ) ;
                DEBUG_ONLY( rVortonParent.mTotalCirculation = rVortonParent.GetVorticity() * Pow3( rVortonParent.GetRadius() ) ) ;

            #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
                //// Correct vorticity to preserve total circulation.
                //const float vorticityReduction = vortonRadius / rVortonParent.GetRadius() ;
                //ASSERT( vorticityReduction <= 1.0f ) ;
                //rVortonParent.mAngularVelocity *= Pow3( vorticityReduction ) ;
                for( increment[2] = 0 ; increment[2] < pClusterDims[2] ; ++ increment[2] )
                {
                    const unsigned offsetZ = ( clusterMinIndices[2] + increment[2] ) * numXYchild ;
                    for( increment[1] = 0 ; increment[1] < pClusterDims[1] ; ++ increment[1] )
                    {
                        const unsigned offsetYZ = ( clusterMinIndices[1] + increment[1] ) * numXchild + offsetZ ;
                        for( increment[0] = 0 ; increment[0] < pClusterDims[0] ; ++ increment[0] )
                        {
                            const unsigned  offsetXYZ               = ( clusterMinIndices[0] + increment[0] ) + offsetYZ ;
                            Vorton &        rVortonChild            = rChildLayer[ offsetXYZ ] ;
                            const float     vortMag                 = rVortonChild.GetVorticity().Magnitude() ;
                            // If this parent cell contains only this child vorton then the vortNormSum will become zero.
                            // We need to avoid it becoming actually zero, because of the quotient below.
                            // When it would have become zero, the numerator would also be zero, so dividing by a small number is harmless.
                            // In contrast, doing a conditional branch instead costs more CPU time.
                            const float     vortNormSumMinusSelf    = vortAux.mVortNormSum - vortMag + sAvoidSingularity ;
                            ASSERT( vortNormSumMinusSelf > 0.0f ) ;

                            // Dissect parent supervorton to remove sibling information and store in this child.
                            rVortonChild.mPositionSiblings  = ( rVortonParent.mPosition - rVortonChild.mPosition * vortMag ) / vortNormSumMinusSelf ;
                            ASSERT( ! IsNan( rVortonChild.mPositionSiblings ) && ! IsInf( rVortonChild.mPositionSiblings ) ) ;

                            rVortonChild.SetRadius( vortonRadius ) ;

                            rVortonChild.mVorticitySiblings = rVortonParent.GetVorticity() - rVortonChild.GetVorticity() ;
                            DEBUG_ONLY( rVortonChild.mNumSibVortonsIncorporated = rVortonParent.mNumVortonsIncorporated - rVortonChild.mNumVortonsIncorporated ) ;
                            ASSERT( ( rVortonChild.mVorticitySiblings.Mag2() < 1.0e-8f ) || ( rVortonChild.GetRadius() == vortonRadius ) ) ;

                        #if 0
                            // NOTE:    In order to preserve total circulation, changing supervorton radius would require changing its vorticity.
                            //          That would also unfortunately mean changing the long-distance influence of the supervorton, which would be undesirable.
                            //          Hence this code to change supervorton radius is disabled.
                            // Correct radius by recomputing parent radius after removing this child.
                            // rVortonParent.mRadius is the sum of dist^N where N>2 (e.g. N=8, as defined above)
                            // so the intermediate corrected radius will be rVortonParent.mRadius - parentChildDist^N.
                            // The actual corrected radius will be the intermediate corrected radius + vortonRadius (the radius of each actual vorton),
                            // then taking the 1/N root.
                            // The N-norm approximates using the farthest parent-to-sibling distance without having to remember each individual distance.
                            // Compute parent-to-child distance.
                            const Vec3  vParentToChild  = rVortonParent.mPosition - rVortonChild.mPosition ;
                            const float fParentToChild2 = vParentToChild.Mag2() ;
                            const float fParentToChild4 = POW2( fParentToChild2 ) ;
                            const float fParentToChild8 = POW2( fParentToChild4 ) ;
                            rVortonChild.mRadiusSiblings = Max2( rVortonParent.GetRadius() - fParentToChild8 , 0.0f ) ;
                            rVortonChild.mRadiusSiblings = powf( rVortonChild.GetRadius() , 0.125f /* 1/8 */ ) + vortonRadius ;
                            ASSERT( ! IsInf( rVortonChild.mRadiusSiblings ) ) ;
                            ASSERT( rVortonChild.mRadiusSiblings >= vortonRadius ) ;

                            #if defined( _DEBUG )
                            {
                                const Vec3 vCirculationParent       = rVortonParent.GetVorticity() * Pow3( rVortonParent.GetRadius() ) ;
                                const Vec3 vCirculationSelf         = rVortonChild.GetVorticity()  * Pow3( rVortonChild.GetRadius() ) ;
                                const Vec3 vCirculationSiblings     = rVortonChild.mVorticitySiblings * Pow3( rVortonChild.mRadiusSiblings ) ;
                                const Vec3 vCirculationSelfAndSibs  = vCirculationSelf + vCirculationSiblings ;
                                ASSERT( vCirculationParent.Resembles( vCirculationSelfAndSibs ) ) ;
                            }
                            #endif

                            // Correct vorticity to preserve total circulation.
                            const float vorticityReduction = vortonRadius / rVortonChild.mRadiusSiblings ;
                            ASSERT( vorticityReduction <= 1.0f ) ;
                            rVortonChild.mVorticitySiblings *= Pow3( vorticityReduction ) ;
                            DEBUG_ONLY( vTotalCirculation += rVortonChild.mVorticitySiblings * Pow3( rVortonChild.mRadiusSiblings ) ) ;
                        #else
                            DEBUG_ONLY( vTotalCirculation += rVortonChild.mVorticitySiblings * Pow3( rVortonChild.GetRadius() ) ) ;
                        #endif

                        #if defined( _DEBUG )
                            rVortonChild.mIndicesOfParent[0] = idxParent[0] ;
                            rVortonChild.mIndicesOfParent[1] = idxParent[1] ;
                            rVortonChild.mIndicesOfParent[2] = idxParent[2] ;
                        #endif
                        }
                    }
                }
            #endif

                // Normalize weighted position sum to obtain center-of-vorticity.
                // (See analogous code in MakeBaseVortonGrid.)
                rVortonParent.mPosition /= vortAux.mVortNormSum ;
            }
        }
    }

#if defined( _DEBUG ) && ( VELOCITY_TECHNIQUE != VELOCITY_TECHNIQUE_MONOPOLES )
    unsigned numVortonsInParentLayer = 0 ;
    unsigned numVortonsInChildLayer  = 0 ;
    Vec3     vTotalCirculationDoubleCheck( 0.0f , 0.0f , 0.0f ) ;
    for( idxParent[2] = 0 ; idxParent[2] < numCells[2] ; ++ idxParent[2] )
    {
        const unsigned offsetZ = idxParent[2] * numXY ;
        for( idxParent[1] = 0 ; idxParent[1] < numCells[1] ; ++ idxParent[1] )
        {
            const unsigned offsetYZ = idxParent[1] * rParentLayer.GetNumPoints( 0 ) + offsetZ ;
            for( idxParent[0] = 0 ; idxParent[0] < numCells[0] ; ++ idxParent[0] )
            {   // For each cell in the parent layer...
                const unsigned offsetXYZ = idxParent[0] + offsetYZ ;
                UniformGrid<Vorton> & rChildLayer   = influenceTree[ uParentLayer - 1 ] ;
                Vorton              & rVortonParent = rParentLayer[ offsetXYZ ] ;

                numVortonsInParentLayer += rVortonParent.mNumVortonsIncorporated ;

                unsigned clusterMinIndices[ 3 ] ;
                influenceTree.GetChildClusterMinCornerIndex( clusterMinIndices , pClusterDims , idxParent ) ;
                unsigned increment[3] ;
                const unsigned & numXchild  = rChildLayer.GetNumPoints( 0 ) ;
                const unsigned   numXYchild = numXchild * rChildLayer.GetNumPoints( 1 ) ;
                // For each cell of child layer in this grid cluster...
                for( increment[2] = 0 ; increment[2] < pClusterDims[2] ; ++ increment[2] )
                {
                    const unsigned offsetZ = ( clusterMinIndices[2] + increment[2] ) * numXYchild ;
                    for( increment[1] = 0 ; increment[1] < pClusterDims[1] ; ++ increment[1] )
                    {
                        const unsigned offsetYZ = ( clusterMinIndices[1] + increment[1] ) * numXchild + offsetZ ;
                        for( increment[0] = 0 ; increment[0] < pClusterDims[0] ; ++ increment[0] )
                        {
                            const unsigned  offsetXYZ       = ( clusterMinIndices[0] + increment[0] ) + offsetYZ ;
                            Vorton &        rVortonChild    = rChildLayer[ offsetXYZ ] ;
                            //const float     vortMag         = rVortonChild.GetVorticity().Magnitude() ;

                            numVortonsInChildLayer += rVortonChild.mNumVortonsIncorporated ;
                        #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
                            //vTotalCirculationDoubleCheck += rVortonChild.mVorticitySiblings * Pow3( rVortonChild.mRadiusSiblings ) ;
                            vTotalCirculationDoubleCheck += rVortonChild.mVorticitySiblings   * Pow3( rVortonChild.GetRadius() ) ;
                            ASSERT( ! IsInf( vTotalCirculationDoubleCheck ) ) ;
                        #else
                            vTotalCirculationDoubleCheck += rVortonChild.GetVorticity() * Pow3( rVortonChild.GetRadius() ) ;
                        #endif
                        }
                    }
                }
            }
        }
    }
    ASSERT( numVortonsInChildLayer == numVortonsInParentLayer ) ;
    ASSERT( numVortonsInChildLayer == mVortons->Size() ) ;
    ASSERT( vTotalCirculation.Resembles( vTotalCirculationDoubleCheck ) ) ;
    ASSERT( vTotalCirculation.Resembles( mDiagnosticIntegrals.mAfterAdvect.mTotalCirculation , 1.0e-3f ) ) ;
#endif
}




/** Create nested grid vorticity influence tree.

    Each layer of this tree represents a simplified, aggregated version of
    all of the information in its "child" layer, where each
    "child" has higher resolution than its "parent".

    \see MakeBaseVortonGrid, AggregateClusters

    Derivation:

    Using conservation properties, I_0 = I_0' , I_1 = I_1' , I_2 = I_2'

    I_0 : wx d = w1x d1 + w2x d2
        : wy d = w1y d1 + w2y d2
        : wz d = w1z d1 + w2z d2

        These 3 are not linearly independent:
    I_1 : ( y wz - z wy ) d = ( y1 wz1 - z1 wy1 ) d1 + ( y2 wz2 - z2 wy2 ) d2
        : ( z wx - x wz ) d = ( z1 wx1 - x1 wz1 ) d1 + ( z2 wx2 - x2 wz2 ) d2
        : ( x wy - y wx ) d = ( x1 wy1 - y1 wx1 ) d1 + ( x2 wy2 - y2 wx2 ) d2

    I_2 : ( x^2 + y^2 + z^2 ) wx d = (x1^2 + y1^2 + z1^2 ) wx1 d1 + ( x2^2 + y2^2 + z2^2 ) wx2 d2
        : ( x^2 + y^2 + z^2 ) wy d = (x1^2 + y1^2 + z1^2 ) wy1 d1 + ( x2^2 + y2^2 + z2^2 ) wy2 d2
        : ( x^2 + y^2 + z^2 ) wz d = (x1^2 + y1^2 + z1^2 ) wz1 d1 + ( x2^2 + y2^2 + z2^2 ) wz2 d2

    Can replace I_2 with its magnitude:
              ( x^2  + y^2  + z^2  ) ( wx^2  + wy^2  + wz^2  )^(1/2) d
            = ( x1^2 + y1^2 + z1^2 ) ( wx1^2 + w1y^2 + w1z^2 )^(1/2) d1
            + ( x2^2 + y2^2 + z2^2 ) ( wx2^2 + w2y^2 + w2z^2 )^(1/2) d2

*/
void VortonSim::CreateInfluenceTree( NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__CreateInfluenceTree ) ;

    ASSERT( ! mVortons->Empty() ) ;

    // Create skeletal nested grid for influence tree.
    influenceTree.Initialize( mGridTemplate ) ; // Create skeleton of influence tree.

    MakeBaseVortonGrid( influenceTree ) ;

    {
        PERF_BLOCK( VortonSim__CreateInfluenceTree_AggregateClusters ) ;
        const size_t numLayers = influenceTree.GetDepth() ;
        for( unsigned int uParentLayer = 1 ; uParentLayer < numLayers ; ++ uParentLayer )
        {   // For each layer in the influence tree...
            AggregateClusters( uParentLayer , influenceTree ) ;
        }
    }
}




DEBUG_ONLY( static unsigned sNumVortonsEncounteredInTree = 0 ) ;
DEBUG_ONLY( static Vec3     sVorticityEncounteredInTree( 0.0f , 0.0f , 0.0f ) ) ;   ///< Vorticity encountered while traversing vorton tree.  Used to diagnose algorithm.
DEBUG_ONLY( static Vec3     sCirculationEncounteredInTree( 0.0f , 0.0f , 0.0f ) ) ; ///< Circulation encountered while traversing vorton tree.  Used to diagnose algorithm.




/** Compute vector potential at a given point in space, due to influence of vortons, using a treecode.

    \param vPosition - point in space whose vector potential to evaluate

    \param indices - indices of cell to visit in the given layer

    \param iLayer - which layer to process

    \return velocity at vPosition, due to influence of vortons

    \note This is a recursive algorithm with time complexity O(log(N)). 
            The outermost caller should pass in influenceTree.GetDepth().

*/
Vec3 VortonSim::ComputeVectorPotential_Tree( const Vec3 & vPosition , const unsigned indices[3] , size_t iLayer , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__ComputeVectorPotential_Tree ) ;

    ASSERT( iLayer > 0 ) ; // Child has index iLayer-1 so iLayer better be positive. Otherwise caller should use ComputeVectorPotential_Direct.
    const UniformGrid< Vorton > &   rChildLayer             = influenceTree[ iLayer - 1 ] ;
    unsigned                        clusterMinIndices[3] ;
    const unsigned *                pClusterDims            = influenceTree.GetDecimations( iLayer ) ;
    influenceTree.GetChildClusterMinCornerIndex( clusterMinIndices , pClusterDims , indices ) ;

    const Vec3 &            vGridMinCorner          = rChildLayer.GetMinCorner() ;
    const Vec3              vSpacing                = rChildLayer.GetCellSpacing() ;
    unsigned                increment[3]            ;
    const unsigned &        numXchild               = rChildLayer.GetNumPoints( 0 ) ;
    const unsigned          numXYchild              = numXchild * rChildLayer.GetNumPoints( 1 ) ;
    Vec3                    vecPotAccumulator( 0.0f , 0.0f , 0.0f ) ;

    ASSERT( ! mVortons->empty() ) ;

#if AVOID_CENTERS || USE_ORIGINAL_VORTONS_IN_BASE_LAYER || defined( _DEBUG )
    const float             vortonRadius = (*mVortons)[ 0 ].GetRadius() ;
    (void) vortonRadius ;
#endif

    // The larger this is, the more accurate (and slower) the evaluation.
    // Reasonable values lie in [0.00001,4.0].
    // Setting this to 0 leads to very bad numerical artifacts, but values greater than (tiny) lead to drastic improvements.
    // Changes in margin have a quantized effect since they effectively indicate how many additional
    // cluster subdivisions to visit.
#if AVOID_CENTERS
    static const float  marginFactor    = 2.0f * vortonRadius ;
#else
    static const float  marginFactor    = 0.0001f ; // 0.4f ; // ship with this number: 0.0001f ; test with 0.4
#endif
    // When domain is 2D in XY plane, min.z==max.z so vPos.z test below would fail unless margin.z!=0.
    const Vec3          margin          = marginFactor * vSpacing + ( 0.0f == vSpacing.z ? Vec3(0,0,FLT_MIN) : Vec3(0,0,0) );

    // For each cell of child layer in this grid cluster...
    for( increment[2] = 0 ; increment[2] < pClusterDims[2] ; ++ increment[2] )
    {
        unsigned idxChild[3] ;
        idxChild[2] = clusterMinIndices[2] + increment[2] ;
        Vec3 vCellMinCorner , vCellMaxCorner ;
        vCellMinCorner.z = vGridMinCorner.z + float( idxChild[2]     ) * vSpacing.z ;
        vCellMaxCorner.z = vGridMinCorner.z + float( idxChild[2] + 1 ) * vSpacing.z ;
        const unsigned offsetZ = idxChild[2] * numXYchild ;
        for( increment[1] = 0 ; increment[1] < pClusterDims[1] ; ++ increment[1] )
        {
            idxChild[1] = clusterMinIndices[1] + increment[1] ;
            vCellMinCorner.y = vGridMinCorner.y + float( idxChild[1]     ) * vSpacing.y ;
            vCellMaxCorner.y = vGridMinCorner.y + float( idxChild[1] + 1 ) * vSpacing.y ;
            const unsigned offsetYZ = idxChild[1] * numXchild + offsetZ ;
            for( increment[0] = 0 ; increment[0] < pClusterDims[0] ; ++ increment[0] )
            {
                idxChild[0] = clusterMinIndices[0] + increment[0] ;
                vCellMinCorner.x = vGridMinCorner.x + float( idxChild[0]     ) * vSpacing.x ;
                vCellMaxCorner.x = vGridMinCorner.x + float( idxChild[0] + 1 ) * vSpacing.x ;
                if(
                        ( iLayer > 1 )
                    &&  ( vPosition.x >= vCellMinCorner.x - margin.x )
                    &&  ( vPosition.y >= vCellMinCorner.y - margin.y )
                    &&  ( vPosition.z >= vCellMinCorner.z - margin.z )
                    &&  ( vPosition.x <  vCellMaxCorner.x + margin.x )
                    &&  ( vPosition.y <  vCellMaxCorner.y + margin.y )
                    &&  ( vPosition.z <  vCellMaxCorner.z + margin.z )
                  )
                {   // Test position is inside childCell and currentLayer > 0...
                    // Recurse child layer.
                    vecPotAccumulator += ComputeVectorPotential_Tree( vPosition , idxChild , iLayer - 1 , vortonIndicesGrid , influenceTree ) ;
                }
                else
                {   // Test position is outside childCell, or reached leaf node.
                    //    Compute velocity induced by cell at corner point x.
                    //    Accumulate influence, storing in velocityAccumulator.
                    const unsigned  offsetXYZ       = idxChild[0] + offsetYZ ;
                    const Vorton &  rVortonChild    = rChildLayer[ offsetXYZ ] ;

                #if USE_ORIGINAL_VORTONS_IN_BASE_LAYER
                    if( 1 == iLayer )
                    {   // Reached base layer.
                        // Instead of using supervorton, use direct summation of original vortons.
                        // This only makes a difference when a leaf-layer grid cell contains multiple vortons.
                        const unsigned numVortonsInCell = vortonIndicesGrid[ offsetXYZ ].Size() ;
                        DEBUG_ONLY( Vec3 vorticityEncounteredInCell( 0.0f , 0.0f , 0.0f ) ) ;
                        for( unsigned ivHere = 0 ; ivHere < numVortonsInCell ; ++ ivHere )
                        {   // For each vorton in this gridcell...
                            const unsigned &    rVortIdxHere    = vortonIndicesGrid[ offsetXYZ ][ ivHere ] ;
                            Vorton &            rVortonHere     = (*mVortons)[ rVortIdxHere ] ;
                            ASSERT( ( rVortonHere.GetVorticity().Magnitude() < 1.e-8f ) || ( rVortonHere.GetRadius() == vortonRadius ) ) ;
                            VORTON_ACCUMULATE_VECTOR_POTENTIAL( vecPotAccumulator , vPosition , rVortonHere ) ;
                            DEBUG_ONLY( vorticityEncounteredInCell += rVortonHere.GetVorticity() ) ;
                        }
                        ASSERT( numVortonsInCell == rVortonChild.mNumVortonsIncorporated ) ;
                        DEBUG_ONLY( sNumVortonsEncounteredInTree += numVortonsInCell ) ;
                        DEBUG_ONLY( sVorticityEncounteredInTree += vorticityEncounteredInCell ) ;
                        DEBUG_ONLY( const Vec3 vCellCirculation( vorticityEncounteredInCell * Pow3( rVortonChild.GetRadius() ) ) ) ;
                        ASSERT( rVortonChild.mTotalCirculation.Resembles( vCellCirculation ) ) ;
                        DEBUG_ONLY( sCirculationEncounteredInTree += vCellCirculation ) ;
                    }
                    else
                #endif
                    {   // Current layer is either...
                        // not a base layer, i.e. this layer is an "aggregation" layer of supervortons, ...or...
                        // this is tbe base layer and USE_ORIGINAL_VORTONS_IN_BASE_LAYER is disabled.
                        ASSERT( ( rVortonChild.GetVorticity().Magnitude() < 1.e-8f ) || ( rVortonChild.GetRadius() == vortonRadius ) ) ;
                        VORTON_ACCUMULATE_VECTOR_POTENTIAL( vecPotAccumulator , vPosition , rVortonChild ) ;
                        DEBUG_ONLY( sNumVortonsEncounteredInTree += rVortonChild.mNumVortonsIncorporated ) ;
                        DEBUG_ONLY( sVorticityEncounteredInTree += rVortonChild.GetVorticity() ) ;
                        DEBUG_ONLY( const Vec3 vCellCirculation( rVortonChild.GetVorticity() * Pow3( rVortonChild.GetRadius() ) ) ) ;
                        ASSERT( rVortonChild.mTotalCirculation.Resembles( vCellCirculation ) ) ;
                        DEBUG_ONLY( sCirculationEncounteredInTree += vCellCirculation ) ;
                    }
                }
            }
        }
    }

    return vecPotAccumulator ;
}




/** Compute velocity at a given point in space, due to influence of vortons, using a treecode.

    \param vPosition - point in space whose velocity to evaluate

    \param indices - indices of cell to visit in the given layer

    \param iLayer - which layer to process

    \return velocity at vPosition, due to influence of vortons

    \note This is a recursive algorithm with time complexity O(log(N)). 
            The outermost caller should pass in influenceTree.GetDepth().

*/
Vec3 VortonSim::ComputeVelocity_Tree( const Vec3 & vPosition , const unsigned indices[3] , size_t iLayer , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__ComputeVelocity_Tree ) ;

    ASSERT( iLayer > 0 ) ; // Child has index iLayer-1 so iLayer better be positive. Otherwise caller should use ComputeVelocity_Direct.
    const UniformGrid< Vorton > &   rChildLayer             = influenceTree[ iLayer - 1 ] ;
    unsigned                        clusterMinIndices[3] ;
    const unsigned *                pClusterDims            = influenceTree.GetDecimations( iLayer ) ;
    influenceTree.GetChildClusterMinCornerIndex( clusterMinIndices , pClusterDims , indices ) ;

    const Vec3 &            vGridMinCorner          = rChildLayer.GetMinCorner() ;
    const Vec3              vSpacing                = rChildLayer.GetCellSpacing() ;
    unsigned                increment[3]            ;
    const unsigned &        numXchild               = rChildLayer.GetNumPoints( 0 ) ;
    const unsigned          numXYchild              = numXchild * rChildLayer.GetNumPoints( 1 ) ;
    Vec3                    velocityAccumulator( 0.0f , 0.0f , 0.0f ) ;

    ASSERT( ! mVortons->empty() ) ;

#if AVOID_CENTERS || USE_ORIGINAL_VORTONS_IN_BASE_LAYER || defined( _DEBUG )
    const float             vortonRadius = (*mVortons)[ 0 ].GetRadius() ;
    (void) vortonRadius ;
#endif

    // The larger this is, the more accurate (and slower) the evaluation.
    // Reasonable values lie in [0.00001,4.0].
    // Setting this to 0 leads to very bad errors, but values greater than (tiny) lead to drastic improvements.
    // Changes in margin have a quantized effect since they effectively indicate how many additional
    // cluster subdivisions to visit.
#if AVOID_CENTERS
    static const float  marginFactor    = 2.0f * vortonRadius ;
#else
    static const float  marginFactor    = 0.0001f ; // 0.4f ; // ship with this number: 0.0001f ; test with 0.4
#endif
    // When domain is 2D in XY plane, min.z==max.z so vPos.z test below would fail unless margin.z!=0.
    const Vec3          margin          = marginFactor * vSpacing + ( 0.0f == vSpacing.z ? Vec3(0,0,FLT_MIN) : Vec3(0,0,0) );

    // For each cell of child layer in this grid cluster...
    for( increment[2] = 0 ; increment[2] < pClusterDims[2] ; ++ increment[2] )
    {
        unsigned idxChild[3] ;
        idxChild[2] = clusterMinIndices[2] + increment[2] ;
        Vec3 vCellMinCorner , vCellMaxCorner ;
        vCellMinCorner.z = vGridMinCorner.z + float( idxChild[2]     ) * vSpacing.z ;
        vCellMaxCorner.z = vGridMinCorner.z + float( idxChild[2] + 1 ) * vSpacing.z ;
        const unsigned offsetZ = idxChild[2] * numXYchild ;
        for( increment[1] = 0 ; increment[1] < pClusterDims[1] ; ++ increment[1] )
        {
            idxChild[1] = clusterMinIndices[1] + increment[1] ;
            vCellMinCorner.y = vGridMinCorner.y + float( idxChild[1]     ) * vSpacing.y ;
            vCellMaxCorner.y = vGridMinCorner.y + float( idxChild[1] + 1 ) * vSpacing.y ;
            const unsigned offsetYZ = idxChild[1] * numXchild + offsetZ ;
            for( increment[0] = 0 ; increment[0] < pClusterDims[0] ; ++ increment[0] )
            {
                idxChild[0] = clusterMinIndices[0] + increment[0] ;
                vCellMinCorner.x = vGridMinCorner.x + float( idxChild[0]     ) * vSpacing.x ;
                vCellMaxCorner.x = vGridMinCorner.x + float( idxChild[0] + 1 ) * vSpacing.x ;
                if(
                        ( iLayer > 1 )
                    &&  ( vPosition.x >= vCellMinCorner.x - margin.x )
                    &&  ( vPosition.y >= vCellMinCorner.y - margin.y )
                    &&  ( vPosition.z >= vCellMinCorner.z - margin.z )
                    &&  ( vPosition.x <  vCellMaxCorner.x + margin.x )
                    &&  ( vPosition.y <  vCellMaxCorner.y + margin.y )
                    &&  ( vPosition.z <  vCellMaxCorner.z + margin.z )
                  )
                {   // Test position is inside childCell and currentLayer > 0...
                    // Recurse child layer.
                    velocityAccumulator += ComputeVelocity_Tree( vPosition , idxChild , iLayer - 1 , vortonIndicesGrid , influenceTree ) ;
                }
                else
                {   // Test position is outside childCell, or reached leaf node.
                    //    Compute velocity induced by cell at corner point x.
                    //    Accumulate influence, storing in velocityAccumulator.
                    const unsigned  offsetXYZ       = idxChild[0] + offsetYZ ;
                    const Vorton &  rVortonChild    = rChildLayer[ offsetXYZ ] ;

                #if USE_ORIGINAL_VORTONS_IN_BASE_LAYER
                    if( 1 == iLayer )
                    {   // Reached base layer.
                        // Instead of using supervorton, use direct summation of original vortons.
                        // This only makes a difference when a leaf-layer grid cell contains multiple vortons.
                        const unsigned numVortonsInCell = vortonIndicesGrid[ offsetXYZ ].Size() ;
                        DEBUG_ONLY( Vec3 vorticityEncounteredInCell( 0.0f , 0.0f , 0.0f ) ) ;
                        for( unsigned ivHere = 0 ; ivHere < numVortonsInCell ; ++ ivHere )
                        {   // For each vorton in this gridcell...
                            const unsigned &    rVortIdxHere    = vortonIndicesGrid[ offsetXYZ ][ ivHere ] ;
                            Vorton &            rVortonHere     = (*mVortons)[ rVortIdxHere ] ;
                            ASSERT( ( rVortonHere.GetVorticity().Magnitude() < 1.e-8f ) || ( rVortonHere.GetRadius() == vortonRadius ) ) ;
                            VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , rVortonHere ) ;
                            DEBUG_ONLY( vorticityEncounteredInCell += rVortonHere.GetVorticity() ) ;
                        }
                        ASSERT( numVortonsInCell == rVortonChild.mNumVortonsIncorporated ) ;
                        DEBUG_ONLY( sNumVortonsEncounteredInTree += numVortonsInCell ) ;
                        DEBUG_ONLY( sVorticityEncounteredInTree += vorticityEncounteredInCell ) ;
                        DEBUG_ONLY( const Vec3 vCellCirculation( vorticityEncounteredInCell * Pow3( rVortonChild.GetRadius() ) ) ) ;
                        ASSERT( rVortonChild.mTotalCirculation.Resembles( vCellCirculation ) ) ;
                        DEBUG_ONLY( sCirculationEncounteredInTree += vCellCirculation ) ;
                    }
                    else
                #endif
                    {   // Current layer is either...
                        // not a base layer, i.e. this layer is an "aggregation" layer of supervortons, ...or...
                        // this is tbe base layer and USE_ORIGINAL_VORTONS_IN_BASE_LAYER is disabled.
                        ASSERT( ( rVortonChild.GetVorticity().Magnitude() < 1.e-8f ) || ( rVortonChild.GetRadius() == vortonRadius ) ) ;
                        VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , rVortonChild ) ;
                        DEBUG_ONLY( sNumVortonsEncounteredInTree += rVortonChild.mNumVortonsIncorporated ) ;
                        DEBUG_ONLY( sVorticityEncounteredInTree += rVortonChild.GetVorticity() ) ;
                        DEBUG_ONLY( const Vec3 vCellCirculation( rVortonChild.GetVorticity() * Pow3( rVortonChild.GetRadius() ) ) ) ;
                        ASSERT( rVortonChild.mTotalCirculation.Resembles( vCellCirculation ) ) ;
                        DEBUG_ONLY( sCirculationEncounteredInTree += vCellCirculation ) ;
                    }
                }
            }
        }
    }

    return velocityAccumulator ;
}




#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
/** Compute velocity at a given point in space, due to influence of vortons.

    \param indices - indices of grid cell of position whose velocity to evaluate

    \param vPosition - point in space whose velocity to evaluate

    \return velocity at vPosition, due to influence of vortons

    \note This is a recursive algorithm with time complexity O(log(N)). 
              The outermost caller should pass in influenceTree.GetDepth().

*/
Vec3 VortonSim::ComputeVelocity_Monopoles( const unsigned indices[3] , const Vec3 & vPosition , const NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__ComputeVelocity_Monopoles ) ;

    Vec3        velocityAccumulator( 0.0f , 0.0f , 0.0f ) ;
    unsigned    indicesVisited[4] = { indices[0] , indices[1] , indices[2] , 0 } ;

#if defined( _DEBUG )
    {
        unsigned    indicesTest[4] ;    // indices within base layer of query position
        influenceTree[ 0 ].IndicesOfPosition( indicesTest , vPosition ) ;
    }
#endif

    DEBUG_ONLY( unsigned    numVortonsEncountered = 0 ) ;
    DEBUG_ONLY( Vec3        vVorticityEncountered( 0.0f , 0.0f , 0.0f ) ) ;
    DEBUG_ONLY( Vec3        vCirculationEncountered( 0.0f , 0.0f , 0.0f ) ) ;

    const float vortonRadius = (*mVortons)[ 0 ].GetRadius() ;

    // Special case: Compute velocity due to vortons in same gridcell as query point.
    {
        UniformGrid< Vorton > & rLayer  = influenceTree[ 0 ] ;

        // The outermost layer of the grid contains no vortices.
        // Any queries that walk along that outer layer see no influence.
        // Make sure all queries start with indices that lie strictly within the region that contains influence data.
        indicesVisited[ 0 ] = Min2( indicesVisited[ 0 ] , rLayer.GetNumCells( 0 ) - 1 ) ;
        indicesVisited[ 1 ] = Min2( indicesVisited[ 1 ] , rLayer.GetNumCells( 1 ) - 1 ) ;
        indicesVisited[ 2 ] = Min2( indicesVisited[ 2 ] , rLayer.GetNumCells( 2 ) - 1 ) ;

        const unsigned          offset  = rLayer.OffsetFromIndices( indicesVisited ) ;
        const Vorton          & rVorton = rLayer[ offset ] ;

        VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , rVorton ) ;
ASSERT( FAbs( velocityAccumulator.x ) < 100.0f ) ; // DO NOT SUBMIT. heuristic, not always correct

        DEBUG_ONLY( numVortonsEncountered   += rVorton.mNumVortonsIncorporated ) ;
ASSERT( ( 0.0f == rVorton.GetRadius() ) || ( rVorton.GetRadius() == vortonRadius ) ) ; // This will probably end up changing.
        DEBUG_ONLY( const Vec3 vCellCirculation = rVorton.GetVorticity() * Pow3( rVorton.GetRadius() ) ) ;
        ASSERT( vCellCirculation.Resembles( rVorton.mTotalCirculation ) ) ;
        DEBUG_ONLY( vVorticityEncountered += rVorton.GetVorticity() ) ;
        DEBUG_ONLY( vCirculationEncountered += rVorton.mTotalCirculation ) ;
    }

    float clusterRadius = 1.0f * vortonRadius ;    // Experimental: estimate cluster radius based on level in influence tree.

    const size_t numLayers = influenceTree.GetDepth() ;
    for( size_t uLayer = 0 /* base */ ; uLayer < numLayers - 1 ; ++ uLayer )
    {   // For each layer in influence tree...
        UniformGrid< Vorton > & rLayer  = influenceTree[ uLayer ] ;

        // The outermost layer of the grid contains no vortices.
        // Any queries that walk along that outer layer see no influence.
        // Make sure all queries start with indices that lie strictly within the region that contains influence data.
        indicesVisited[ 0 ] = Min2( indicesVisited[ 0 ] , rLayer.GetNumCells( 0 ) - 1 ) ;
        indicesVisited[ 1 ] = Min2( indicesVisited[ 1 ] , rLayer.GetNumCells( 1 ) - 1 ) ;
        indicesVisited[ 2 ] = Min2( indicesVisited[ 2 ] , rLayer.GetNumCells( 2 ) - 1 ) ;

        const unsigned  offset  = rLayer.OffsetFromIndices( indicesVisited ) ;
        Vorton          vorton = rLayer[ offset ] ;
        // With this technique, we want to calculate the velocity due to siblings of this vorton
        // (not due to this vorton itself).
        vorton.mPosition  = vorton.mPositionSiblings ;
        ASSERT( ! IsNan( vorton.mPosition ) && ! IsInf( vorton.mPosition ) ) ;

        vorton.SetVorticity( vorton.mVorticitySiblings ) ;

ASSERT( ( 0.0f == vorton.GetRadius() ) || ( vorton.GetRadius() == vortonRadius ) ) ; // This will probably end up changing.
        //vorton.SetRadius( vorton.mRadiusSiblings ) ;

// Experimental: Estimate cluster radius based on level in influence tree, and adjust vorticity accordingly (to preserve total circulation)
vorton.SetRadius( clusterRadius ) ;
const float vorticityReduction = vortonRadius / clusterRadius ;
ASSERT( vorticityReduction <= 1.0f ) ;
vorton.SetVorticity( vorton.mVorticitySiblings * Pow3( vorticityReduction ) ) ;

        // Compute velocity induced by this supervorton.
        // Accumulate influence, storing in velocityAccumulator.
        VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , vorton ) ;
        ASSERT( ! IsInf( velocityAccumulator ) ) ;
ASSERT( FAbs( velocityAccumulator.x ) < 1000.0f ) ; // DO NOT SUBMIT. heuristic, not always correct

        DEBUG_ONLY( numVortonsEncountered   += vorton.mNumSibVortonsIncorporated ) ;
        DEBUG_ONLY( vVorticityEncountered   += vorton.GetVorticity() ) ;
        ASSERT( ( vorton.GetVorticity().Mag3() < 1.0e-8f ) || ( vorton.mSize > 0.0f ) ) ;
        DEBUG_ONLY( vCirculationEncountered += vorton.GetVorticity() * Pow3( vorton.GetRadius() ) ) ;

        // Compute indices into parent layer
        influenceTree.GetParentIndices( indicesVisited , indicesVisited , uLayer + 1 ) ;
        #if 0 && defined( _DEBUG )
        {
            unsigned indicesParentTest[4] ;
            influenceTree[ uLayer + 1 ].IndicesOfPosition( indicesParentTest , vPosition ) ;
            ASSERT( indicesParentTest[0] == indicesVisited[0] ) ;
            ASSERT( indicesParentTest[1] == indicesVisited[1] ) ;
            ASSERT( indicesParentTest[2] == indicesVisited[2] ) ;
        }
        #endif
    #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
        ASSERT( ( vorton.mIndicesOfParent[0] == 0x7fffffff ) || ( vorton.mIndicesOfParent[0] == indicesVisited[0] ) ) ;
        ASSERT( ( vorton.mIndicesOfParent[1] == 0x7fffffff ) || ( vorton.mIndicesOfParent[1] == indicesVisited[1] ) ) ;
        ASSERT( ( vorton.mIndicesOfParent[2] == 0x7fffffff ) || ( vorton.mIndicesOfParent[2] == indicesVisited[2] ) ) ;
    #endif

        clusterRadius *= 1.4142135623730950488016887242097f ; // Experimental: Estimate cluster radius based on level in influence tree
    }

    ASSERT( mVortons->Size() == numVortonsEncountered ) ;
    ASSERT( vCirculationEncountered.Resembles( mTotalCirculation ) ) ;
    //DEBUG_ONLY( printf( "nv[%3i,%3i,%3i]= %u\n" , indices[0] , indices[1], indices[2] , numVortonsEncountered ) ) ;

    return velocityAccumulator ;
}
#endif




/** Compute vector potential due to vortons, for a subset of points in a uniform grid.

    \param izStart  Starting value for z index

    \param izEnd    One past ending value for z index

    \param boundariesOnly   Compute vector potential only on domain boundaries.
                            If false, compute throughout domain interior.

    \note This routine assumes CreateInfluenceTree has already executed,
            and that the vector potential grid has been allocated.

*/
void VortonSim::ComputeVectorPotentialAtGridpoints_Slice( size_t izStart , size_t izEnd , bool boundariesOnly , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
#if VECTOR_POTENTIAL_TECHNIQUE == VECTOR_POTENTIAL_TECHNIQUE_TREE
    const size_t            numLayers               = influenceTree.GetDepth() ;
#endif

    UniformGrid< Vec3 > &   vectorPotentialGrid     = mVectorPotentialMultiGrid[ 0 ] ;

    const Vec3 &            vMinCorner              = mVelGrid.GetMinCorner() ;
    static const float      nudge                   = 1.0f - 2.0f * FLT_EPSILON ;   // Slightly shrink coordinate spacing to avoid roundoff from exceeding domain (and from having a conditional branch per iteration).
    const Vec3              vSpacing                = mVelGrid.GetCellSpacing() * nudge ;
    const unsigned          dims[3]                 =   { mVelGrid.GetNumPoints( 0 )
                                                        , mVelGrid.GetNumPoints( 1 )
                                                        , mVelGrid.GetNumPoints( 2 ) } ;
    const unsigned          numXY                   = dims[0] * dims[1] ;
    unsigned                idx[ 3 ] ;
    const unsigned          incrementXForInterior   = boundariesOnly ? ( dims[0] - 1 ) : 1 ;

    // Compute fluid flow vector potential at each boundary gridpoint, due to all vortons.
    for( idx[2] = static_cast< unsigned >( izStart ) ; idx[2] < izEnd ; ++ idx[2] )
    {   // For subset of z index values...
        Vec3 vPosition ;
        vPosition.z = vMinCorner.z + float( idx[2] ) * vSpacing.z ;                 // Compute z-coordinate of world-space position of current gridpoint.
        const unsigned  offsetZ     = idx[2] * numXY ;                              // Precompute z contribution to offset into grid.
        const bool      topOrBottom = ( 0 == idx[2] ) || ( dims[2]-1 == idx[2] ) ;  // Whether current gridpoint is on top (+Z) or bottom (-Z) plane
        for( idx[1] = 0 ; idx[1] < dims[1] ; ++ idx[1] )
        {   // For every gridpoint along the y-axis...
            vPosition.y = vMinCorner.y + float( idx[1] ) * vSpacing.y ;                                 // Compute y-coordinate of world-space position of current gridpoint.
            const unsigned  offsetYZ    = idx[1] * dims[0] + offsetZ ;                                  // Precompute y contribution to the offset into grid.
            const bool      frontOrBack = ( 0 == idx[1] ) || ( dims[1]-1 == idx[1] ) ;                  // Whether current gridpoint is on front (+Y) or back (-Y) plane
            const unsigned  incX        = ( topOrBottom || frontOrBack ) ? 1 : incrementXForInterior ;  // For points other than top|bottom|front|back, skip interior gridpoints.
            for( idx[0] = 0 ; idx[0] < dims[0] ; idx[0] += incX )
            {   // For every gridpoint along the x-axis...
                vPosition.x = vMinCorner.x + float( idx[0] ) * vSpacing.x ; // Compute x-coordinate of world-space position of current gridpoint.
                const unsigned offsetXYZ = idx[0] + offsetYZ ;              // Compute offset into grid for current point

#               if VECTOR_POTENTIAL_TECHNIQUE == VECTOR_POTENTIAL_TECHNIQUE_DIRECT

                    UNUSED_PARAM( influenceTree ) ;
                    UNUSED_PARAM( vortonIndicesGrid ) ;

                    vectorPotentialGrid[ offsetXYZ ] = ComputeVectorPotential_Direct( vPosition ) ;

#               elif VECTOR_POTENTIAL_TECHNIQUE == VECTOR_POTENTIAL_TECHNIQUE_TREE

#                   if ! USE_TBB
#                       define CHECK_BULK_VORTICITY_SELF_CONSISTENCY()                                                                              \
                        {                                                                                                                           \
                            ASSERT( mVortons->Size() == sNumVortonsEncounteredInTree ) ;                                                            \
                            ASSERT( sCirculationEncounteredInTree.Resembles( mDiagnosticIntegrals.mAfterAdvect.mTotalCirculation , 1.0e-3f ) ) ;    \
                        }
#                   else
#                       define CHECK_BULK_VORTICITY_SELF_CONSISTENCY() ;
#                   endif

                    if( 0 == ( idx[0] & 1 ) )
                    {   // Even x indices.  Compute value.
                        DEBUG_ONLY( sNumVortonsEncounteredInTree  = 0 ) ;
                        DEBUG_ONLY( sVorticityEncounteredInTree   = Vec3( 0.0f , 0.0f , 0.0f ) ) ;
                        DEBUG_ONLY( sCirculationEncounteredInTree = Vec3( 0.0f , 0.0f , 0.0f ) ) ;
                        static const unsigned zeros[3] = { 0 , 0 , 0 } ; /* Starter indices for recursive algorithm */
                        if( numLayers > 1 )
                        {
                            vectorPotentialGrid[ offsetXYZ ] = ComputeVectorPotential_Tree( vPosition , zeros , numLayers - 1 , vortonIndicesGrid , influenceTree ) ;
                            CHECK_BULK_VORTICITY_SELF_CONSISTENCY() ;
                        }
                        else
                        {
                            vectorPotentialGrid[ offsetXYZ ] = ComputeVectorPotential_Direct( vPosition ) ;
                        }
                    }
                    else
                    {   // Odd x indices. Copy value from preceding grid point.
                        vectorPotentialGrid[ offsetXYZ ] = vectorPotentialGrid[ offsetXYZ - 1 ] ;
                    }

#               else
#                   error Vector potential technique is invalid or undefined.  Assign VECTOR_POTENTIAL_TECHNIQUE in vorton.h or change this code.
#               endif

            }
        }
    }
}




/** Compute vector potential due to vortons, for every point in a uniform grid, using an integral technique (direct summation, treecode or multipole method).

    \param vortonIndicesGrid    Spatial partition of vortons, for fast lookup of vortons in a vicinity.

    \param influenceTree        Nested grid of vortons and "super-vortons", used to compute velocity-from-vorticity using treecode.

    \see CreateInfluenceTree

    \note This routine assumes CreateInfluenceTree has already executed.

*/
void VortonSim::ComputeVectorPotentialFromVorticity_Integral( bool boundariesOnly , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__ComputeVectorPotentialFromVorticity_Integral ) ;

    ASSERT( ! mVortons->empty() ) ;

    #if ENABLE_AUTO_MOLLIFICATION
    {
        // Smooth velocity induced by vorton, in its vicinity, so that its
        // length scale corresponds approximately to what the grid can resolve.
        // When too large, fluid motion lacks detail.
        // When too small, fluid motion jerks when vortons pass near query points.
        // Auto-mollification should be used only with integral techniques,
        // and usually only with VELOCITY_TECHNIQUE_TREE.
        // See comments where ENABLE_AUTO_MOLLIFICATION is defined.
        static const float  OneThird                 = 1.0f / 3.0f ;
        // Tune lengthScale based on comparing jerk statistics for the technique that uses this (TREE)
        // with DIRECT using COMPUTE_VELOCITY_AT_VORTONS (which is about as accurate as a vorton method can get).
        static const float  lengthScale              = 1.0f ;
        const float         characteristicGridLength = lengthScale * powf( mVelGrid.GetCellVolume() , OneThird ) ;
        ASSERT( ! mVortons->empty() ) ;
        const float         vortonRadius = (*mVortons)[ 0 ].GetRadius() ;
        mSpreadingRangeFactor       = Max2( characteristicGridLength / vortonRadius , 1.0f ) ;
        mSpreadingCirculationFactor = 1.0f / Pow3( mSpreadingRangeFactor ) ;
    }
    #endif

//#if COMPUTE_VELOCITY_AT_VORTONS
//    const size_t numVortons = mVortons->size() ;
//    // Compute velocity at vortons.
//    // This is only useful for diagnosing integral solvers.  Otherwise, compute velocity on the grid.
//    #if USE_TBB
//        // Estimate grain size based on size of problem and number of processors.
//        const size_t grainSize =  Max2( size_t( 1 ) , numVortons / gNumberOfProcessors ) ;
//        // Compute velocity at vortons using multiple threads.
//        parallel_for( tbb::blocked_range<size_t>( 0 , numVortons , grainSize ) , VortonSim_ComputeVelocityAtVortons_TBB( this , vortonIndicesGrid , influenceTree ) ) ;
//    #else
//        ComputeVectorPotentialAtVortons_Slice( 0 , numVortons , vortonIndicesGrid , influenceTree ) ;
//    #endif
//    // Transfer velocity from vortons to grid.
//    PopulateVectorPotentialGrid( mVelGrid , reinterpret_cast< VECTOR< Particle > & >( * mVortons ) ) ;
//#else   // Compute velocity at gridpoints.
    const unsigned numZ = mVelGrid.GetNumPoints( 2 ) ;
#   if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , numZ / gNumberOfProcessors ) ;
        // Compute velocity at gridpoints using multiple threads.
        parallel_for( tbb::blocked_range<size_t>( 0 , numZ , grainSize ) , VortonSim_ComputeVectorPotentialAtGridpoints_TBB( this , boundariesOnly , vortonIndicesGrid , influenceTree ) ) ;
#   else
        ComputeVectorPotentialAtGridpoints_Slice( 0 , numZ , boundariesOnly , vortonIndicesGrid , influenceTree ) ;
#   endif
//#endif
}




/** Compute velocity due to vortons, for a subset of points in a uniform grid.

    \param izStart - starting value for z index

    \param izEnd - one past ending value for z index

    \see CreateInfluenceTree, ComputeVelocityFromVorticity

    \note This routine assumes CreateInfluenceTree has already executed,
            and that the velocity grid has been allocated.

*/
void VortonSim::ComputeVelocityAtGridpoints_Slice( size_t izStart , size_t izEnd , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE
    const size_t        numLayers   = influenceTree.GetDepth() ;
#endif
    const Vec3 &        vMinCorner  = mVelGrid.GetMinCorner() ;
    static const float  nudge       = 1.0f - 2.0f * FLT_EPSILON ;
    const Vec3          vSpacing    = mVelGrid.GetCellSpacing() * nudge ;
    const unsigned      dims[3]     =   { mVelGrid.GetNumPoints( 0 )
                                        , mVelGrid.GetNumPoints( 1 )
                                        , mVelGrid.GetNumPoints( 2 ) } ;
    const unsigned      numXY       = dims[0] * dims[1] ;
    unsigned            idx[ 3 ] ;
    for( idx[2] = static_cast< unsigned >( izStart ) ; idx[2] < izEnd ; ++ idx[2] )
    {   // For subset of z index values...
        Vec3 vPosition ;
        // Compute the z-coordinate of the world-space position of this gridpoint.
        vPosition.z = vMinCorner.z + float( idx[2] ) * vSpacing.z ;
        // Precompute the z contribution to the offset into the velocity grid.
        const unsigned offsetZ = idx[2] * numXY ;
        for( idx[1] = 0 ; idx[1] < dims[1] ; ++ idx[1] )
        {   // For every gridpoint along the y-axis...
            // Compute the y-coordinate of the world-space position of this gridpoint.
            vPosition.y = vMinCorner.y + float( idx[1] ) * vSpacing.y ;
            // Precompute the y contribution to the offset into the velocity grid.
            const unsigned offsetYZ = idx[1] * dims[0] + offsetZ ;
            for( idx[0] = 0 ; idx[0] < dims[0] ; ++ idx[0] )
            {   // For every gridpoint along the x-axis...
                // Compute the x-coordinate of the world-space position of this gridpoint.
                vPosition.x = vMinCorner.x + float( idx[0] ) * vSpacing.x ;
                // Compute the offset into the velocity grid.
                const unsigned offsetXYZ = idx[0] + offsetYZ ;

                // Compute the fluid flow velocity at this gridpoint, due to all vortons.
            #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT
                mVelGrid[ offsetXYZ ] = ComputeVelocity_Direct( vPosition ) ;
                (void) influenceTree , vortonIndicesGrid ; // Avoid "unreferenced formal parameter" compiler warning.
            #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE
                DEBUG_ONLY( sNumVortonsEncounteredInTree  = 0 ) ;
                DEBUG_ONLY( sVorticityEncounteredInTree   = Vec3( 0.0f , 0.0f , 0.0f ) ) ;
                DEBUG_ONLY( sCirculationEncounteredInTree = Vec3( 0.0f , 0.0f , 0.0f ) ) ;
                static const unsigned zeros[3] = { 0 , 0 , 0 } ; // Starter indices for recursive algorithm
                if( numLayers > 1 )
                {
                    mVelGrid[ offsetXYZ ] = ComputeVelocity_Tree( vPosition , zeros , numLayers - 1  , vortonIndicesGrid , influenceTree ) ;
                #if ! USE_TBB
                    ASSERT( mVortons->Size() == sNumVortonsEncounteredInTree ) ;
                    ASSERT( sCirculationEncounteredInTree.Resembles( mDiagnosticIntegrals.mAfterAdvect.mTotalCirculation , 1.0e-3f ) ) ;
                #endif
                }
                else
                {
                    mVelGrid[ offsetXYZ ] = ComputeVelocity_Direct( vPosition ) ;
                }
            #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
                mVelGrid[ offsetXYZ ] = ComputeVelocity_Monopoles( idx , vPosition ) ;
                ASSERT( ! IsInf( mVelGrid[ offsetXYZ ] ) ) ;
            #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL
                FAIL() ; // Differential solvers do not use this routine.
                (void) offsetXYZ ;
                (void) influenceTree , vortonIndicesGrid ; // Avoid "unreferenced formal parameter" compiler warning.
            #else
                #error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
            #endif
            }
        }
    }
#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
    DEBUG_ONLY( printf( "\n" ) ) ;
#endif
}




/** Compute velocity due to vortons, at vorton locations, for a subset of vortons.

    \param iPclStart - starting value for vorton index

    \param iPclEnd - one past ending value for vorton index

    \see ComputeVelocityFromVorticity, ComputeVelocityAtGridpoints_Slice

    \note This routine is an alternative to ComputeVelocityAtGridpoints_Slice,
            which computes velocity at gridpoints
            (which are at grid cell corners).

    \note This routine assumes CreateInfluenceTree has already executed.

*/
void VortonSim::ComputeVelocityAtVortons_Slice( unsigned iPclStart , unsigned iPclEnd , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE
    const size_t        numLayers   = influenceTree.GetDepth() ;
#endif
    for( unsigned iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
    {   // For subset of vortex particle index values...
        Vorton & vorton = (*mVortons)[ iPcl ] ;
        Vec3 & vPosition = vorton.mPosition ;
        Vec3 & vVelocity = vorton.mVelocity ;
        // Compute the fluid flow velocity at this gridpoint, due to all vortons.
    #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT
        vVelocity = ComputeVelocity_Direct( vPosition ) ;
        #if ENABLE_PARTICLE_HISTORY
            PclHistoryRecord( iPcl , vorton ) ; // For diagnosing determinism.
        #endif
        (void) influenceTree , vortonIndicesGrid ; // Avoid "unreferenced formal parameter" compiler warning.
    #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE
        DEBUG_ONLY( sNumVortonsEncounteredInTree  = 0 ) ;
        DEBUG_ONLY( sVorticityEncounteredInTree   = Vec3( 0.0f , 0.0f , 0.0f ) ) ;
        DEBUG_ONLY( sCirculationEncounteredInTree = Vec3( 0.0f , 0.0f , 0.0f ) ) ;
        static const unsigned zeros[3] = { 0 , 0 , 0 } ; // Starter indices for recursive algorithm
        vVelocity = ComputeVelocity_Tree( vPosition , zeros , numLayers - 1  , vortonIndicesGrid , influenceTree ) ;
        #if ! USE_TBB
            ASSERT( mVortons->Size() == sNumVortonsEncounteredInTree ) ;
            ASSERT( sCirculationEncounteredInTree.Resembles( mDiagnosticIntegrals.mAfterAdvect.mTotalCirculation ) ) ;
        #endif
    #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
        FAIL() ;  // Monopole method does not use this routine.
        (void) vPosition , vVelocity ; // Avoid "local variable is initialized but not referenced" warning.
    #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL
        FAIL() ; // Differential solvers do not use this routine.
        (void) vPosition , vVelocity ; // Avoid "local variable is initialized but not referenced" warning.
        (void) influenceTree , vortonIndicesGrid  ; // Avoid "unreferenced formal parameter" warning.
    #else
        #error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
    #endif
    }
#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
    DEBUG_ONLY( printf( "\n" ) ) ;
#endif
}




/** Populate a UniformGrid with vorticity values from vortons.

    \param vorticityGrid    (out) Reference to grid representing spatial distribution of vorticity.

    \param scale    Scale by which to multiply vorticity values.
                    Normally scale should be either 1.0f or -1.0f.

    \note   The Poisson equation we want to solve has the form
        \verbatim
                Laplacian vectorPotential = - vorticity
        \endverbatim
            in which case scale should be -1.0f.
*/
void VortonSim::PopulateVorticityGridFromVortons( UniformGrid< Vec3 > & vorticityGrid , float scale )
{
    PERF_BLOCK( VortonSim__PopulateVorticityGridFromVortons ) ;

    vorticityGrid.Clear() ;                            // Clear any stale vorticity information
    vorticityGrid.CopyShape( mGridTemplate ) ;         // Use same shape as base vorticity grid. (Note: could differ if you want.)
    vorticityGrid.Init( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Reserve memory for velocity grid and initialize all values to zero.

    if( mVortons->Empty() )
    {
        return ;
    }

    DEBUG_ONLY( Vec3 vCirculationVortons( 0.0f , 0.0f , 0.0f ) ) ;
    DEBUG_ONLY( Vec3 vCirculationToGrid( 0.0f , 0.0f , 0.0f ) ) ;

    // In order for the Poisson and Tree/Monopole velocity-from-vorticity 
    // techniques to produce consistent results, this formula for the volume of
    // a vorton must match that implicitly used in VORTON_ACCUMULATE_VELOCITY_private.
    ASSERT( ! mVortons->empty() ) ;
    const float vortonRadius        = (*mVortons)[ 0 ].GetRadius() ;
    const float cellVolume          = vorticityGrid.GetCellVolume() ;
    const float vortonVolume        = FourPiOver3 * Pow3( vortonRadius ) ;
    // The grid resulting from this operation must preserve the
    // same total circulation that the vortons have, so multiply
    // by the ratio of the vorton volume to gridcell volume.
    const float volumeCorrection    = scale * vortonVolume / cellVolume ;

    // Make sure vorton radius has not changed.  This simulation code assumes vorton radius is both uniform and constant.
    ASSERT( mVortons->empty() || Math::Resembles( vortonVolume , (*mVortons)[ 0 ].GetVolume() ) ) ;

    // Populate vorticity grid.
    const size_t numVortons = mVortons->Size() ;
    for( size_t uVorton = 0 ; uVorton < numVortons ; ++ uVorton )
    {   // For each vorton in this simulation...
        const Vorton     &  rVorton     = (*mVortons)[ uVorton ] ;
        const Vec3       &  rPosition   = rVorton.mPosition   ;
        ASSERT( ! IsNan( rPosition ) && ! IsInf( rPosition ) ) ;
        DEBUG_ONLY( const unsigned      uOffset     = vorticityGrid.OffsetOfPosition( rPosition ) ) ;
        ASSERT( uOffset < vorticityGrid.GetGridCapacity() ) ;

        // Note that the use of a uniform volumeCorrection here
        // assumes that all vortons have the same volume.
        Vec3 vCirculationCorrectedVorticity = rVorton.GetVorticity() * volumeCorrection ;
        vorticityGrid.Accumulate( rPosition , vCirculationCorrectedVorticity ) ;

        DEBUG_ONLY( vCirculationToGrid  += vCirculationCorrectedVorticity * cellVolume ) ;
        DEBUG_ONLY( vCirculationVortons += rVorton.GetVorticity() * vortonVolume ) ;

        // Make sure vorton radius is uniform and constant.
        ASSERT( Math::Resembles( vortonVolume , rVorton.GetVolume() ) ) ;
    }

#if defined( _DEBUG )
    {
        Vec3 vCirculationGrid( 0.0f , 0.0f , 0.0f ) ;
        for( unsigned offset = 0 ; offset < vorticityGrid.GetGridCapacity() ; ++ offset )
        {   // For each cell in vorticity grid...
            vCirculationGrid += vorticityGrid[ offset ] * cellVolume ;
        }
// DO NOT SUBMIT commented out
        //ASSERT( vCirculationVortons.Resembles( scale * vCirculationGrid , 0.01f ) ) ;
    }
#endif
}




/** Compute velocity due to vortons, for every point in a uniform grid, using an integral technique (direct summation, treecode or multipole method).

    \param vortonIndicesGrid    Spatial partition of vortons, for fast lookup of vortons in a vicinity.

    \param influenceTree        Nested grid of vortons and "super-vortons", used to compute velocity-from-vorticity using treecode.

    \see CreateInfluenceTree

    \note This routine assumes CreateInfluenceTree has already executed.

*/
void VortonSim::ComputeVelocityFromVorticity_Integral( const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__ComputeVelocityFromVorticity_Integral ) ;

    ASSERT( ! mVortons->empty() ) ;

    #if ENABLE_AUTO_MOLLIFICATION
    {
        // Smooth velocity induced by vorton, in its vicinity, so that its
        // length scale corresponds approximately to what the grid can resolve.
        // When too large, fluid motion lacks detail.
        // When too small, fluid motion jerks when vortons pass near query points.
        // Auto-mollification should be used only with integral techniques,
        // and usually only with VELOCITY_TECHNIQUE_TREE.
        // See comments where ENABLE_AUTO_MOLLIFICATION is defined.
        static const float  OneThird                 = 1.0f / 3.0f ;
        // Tune lengthScale based on comparing jerk statistics for the technique that uses this (TREE)
        // with DIRECT using COMPUTE_VELOCITY_AT_VORTONS (which is about as accurate as a vorton method can get).
        static const float  lengthScale              = 1.0f ;
        const float         characteristicGridLength = lengthScale * powf( mVelGrid.GetCellVolume() , OneThird ) ;
        ASSERT( ! mVortons->empty() ) ;
        const float         vortonRadius = (*mVortons)[ 0 ].GetRadius() ;
        mSpreadingRangeFactor       = Max2( characteristicGridLength / vortonRadius , 1.0f ) ;
        mSpreadingCirculationFactor = 1.0f / Pow3( mSpreadingRangeFactor ) ;
    }
    #endif

#if COMPUTE_VELOCITY_AT_VORTONS
    const size_t numVortons = mVortons->size() ;
    // Compute velocity at vortons.
    // This is only useful for diagnosing integral solvers.  Otherwise, compute velocity on the grid.
    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , numVortons / gNumberOfProcessors ) ;
        // Compute velocity at vortons using multiple threads.
        parallel_for( tbb::blocked_range<size_t>( 0 , numVortons , grainSize ) , VortonSim_ComputeVelocityAtVortons_TBB( this , vortonIndicesGrid , influenceTree ) ) ;
    #else
        ComputeVelocityAtVortons_Slice( 0 , numVortons , vortonIndicesGrid , influenceTree ) ;
    #endif
    // Transfer velocity from vortons to grid.
    PopulateVelocityGrid( mVelGrid , reinterpret_cast< VECTOR< Particle > & >( * mVortons ) ) ;
#else   // Compute velocity at gridpoints.
    const unsigned numZ = mVelGrid.GetNumPoints( 2 ) ;
    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , numZ / gNumberOfProcessors ) ;
        // Compute velocity at gridpoints using multiple threads.
        parallel_for( tbb::blocked_range<size_t>( 0 , numZ , grainSize ) , VortonSim_ComputeVelocityAtGridpoints_TBB( this , vortonIndicesGrid , influenceTree ) ) ;
    #else
        ComputeVelocityAtGridpoints_Slice( 0 , numZ , vortonIndicesGrid , influenceTree ) ;
    #endif
#endif
}




/** Compute vector potential due to vorticity, for every point in a uniform grid, by solving a vector Poisson partial differential equation.

    \param vectorPotentialMultiGrid  (out) Nested grid of vector potential values, computed from vorticityGrid.

    \param vorticityGrid        (in) Uniform grid of vorticity values, used to compute velocity-from-vorticity using Poisson solver.

    \todo   This routine calls SolveVectorPoisson which (depending on compiler settings inside that routine) uses "natural" boundary
            conditions.  Instead, what we want to use "radiation" or "open" boundary conditions, where the boundary has "no influence",
            that is, we want this solver to yield results consistent with using direct summation (or the other integral approaches).
            To accomplish that, this routine could first use an integral technique (e.g. direct summation, treecode or multipole)
            to compute vector potential at the boundaries, then use SolveVectorPoisson with "essential" boundary
            conditions to populate the domain interior.  Incidentally that approach would also allow the domain to be closer
            to the interior, rather than inflating the domain, as is done now to diminish the problematic influence of the boundary.
*/
void VortonSim::ComputeVectorPotential( NestedGrid< Vec3 > & vectorPotentialMultiGrid , NestedGrid< Vec3 > & negativeVorticityMultiGrid , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__ComputeVectorPotential ) ;

    ASSERT( ! mVortons->empty() ) ;

    static const BoundaryConditionE boundaryCondition = BC_DIRICHLET ;

    vectorPotentialMultiGrid[0].Init( Vec3( 0.0f , 0.0f , 0.0f ) ) ;

    // Assign vector potential on domain boundary points.
    // Note: For Part 19, assign vector potential everywhere in domain, not just on boundaries.
    //       For Part 20, assign only on domain boundaries.
    static const sBoundariesOnly = true ;
    ComputeVectorPotentialFromVorticity_Integral( sBoundariesOnly , vortonIndicesGrid , influenceTree ) ;

#   if VORTON_SIM_USE_MULTI_GRID

        static const size_t numSolverSteps = 16 ; // Tuned by comparing residuals using numSolverSteps versus numStepsAuto computed inside SolveVectorPoisson.

        // Initialize finest grid with first pass of solution, to propagate boundary conditions inward from boundary.
        SolveVectorPoisson( vectorPotentialMultiGrid[ 0 ] , negativeVorticityMultiGrid[ 0 ] , numSolverSteps , boundaryCondition , mPoissonResidualStats ) ;

        unsigned maxValidDepth = 0 ;

        // Fine-to-coarse stage of V-cycle: downsample from fine to coarse, running some iterations of the Poisson solver for each downsampled grid.
        for( unsigned iLayer = 1 ; iLayer < negativeVorticityMultiGrid.GetDepth() ; ++ iLayer )
        {
            const unsigned minDim = MIN3( negativeVorticityMultiGrid[ iLayer ].GetNumPoints( 0 ) , negativeVorticityMultiGrid[ iLayer ].GetNumPoints( 1 ) , negativeVorticityMultiGrid[ iLayer ].GetNumPoints( 2 ) ) ;
            if( minDim > 2 )
            {
                negativeVorticityMultiGrid.DownSampleInto( iLayer , UniformGridGeometry::FASTER_LESS_ACCURATE ) ;
                vectorPotentialMultiGrid.DownSampleInto( iLayer , UniformGridGeometry::FASTER_LESS_ACCURATE ) ;
                SolveVectorPoisson( vectorPotentialMultiGrid[ iLayer ] , negativeVorticityMultiGrid[ iLayer ] , numSolverSteps , boundaryCondition , mPoissonResidualStats ) ;
            }
            else
            {
                maxValidDepth = iLayer - 1 ;
                break ;
            }
        }

        // Coarse-to-fine stage of V-cycle: Upsample from coarse to fine, running some iterations of the Poisson solver for each upsampled grid.
        for( unsigned iLayer = maxValidDepth ; iLayer >= 1 ; -- iLayer )
        {
            // Retain boundary values as they were computed initially (above) in finer grids.
            vectorPotentialMultiGrid.UpSampleFrom( iLayer , UniformGridGeometry::INTERIOR_ONLY ) ;
            SolveVectorPoisson( vectorPotentialMultiGrid[ iLayer - 1 ] , negativeVorticityMultiGrid[ iLayer - 1 ] , numSolverSteps , boundaryCondition , mPoissonResidualStats ) ;
        }

        // Omit calling ComputeVectorPotentialFromVorticity_Integral again; it redundantly recomputes boundary values which were computed above.
        //ComputeVectorPotentialFromVorticity_Integral( sBoundariesOnly , vortonIndicesGrid , influenceTree ) ; // Assign vector potential on boundary points.  This would be redundant with call above.
        //SolveVectorPoisson( vectorPotentialMultiGrid[ 0 ] , mNegativeVorticityMultiGrid[ 0 ] , numSolverSteps , boundaryCondition , mPoissonResidualStats ) ;

#   else

#       pragma warning(push)
#       pragma warning(disable: 4127) // conditional expression is constant
        if( sBoundariesOnly )
#       pragma warning(pop)
        {
            SolveVectorPoisson( vectorPotentialMultiGrid[0] , negativeVorticityMultiGrid[ 0 ] , /* numSteps; 0 means auto-choose */ 0 , boundaryCondition , mPoissonResidualStats ) ;
        }
        else
        {
            // For Part 19: Temporarily disable solving Poisson, to demonstrate using integral approach technique.
            //              Notice call to ComputeVectorPotentialFromVorticity_Integral above.
            UNUSED_PARAM( negativeVorticityMultiGrid ) ; // For Part 19, avoid unreferenced formal parameter compiler warning.  For Part 20, disable this line.
        }

#       if 0 // Test data flow / repeatability
        {
            UniformGrid< Vec3 > vectorPotentialDummy( mGridTemplate ) ;
            vectorPotentialDummy.Init( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
        ComputeVectorPotentialFromVorticity_Integral( sBoundariesOnly , vortonIndicesGrid , influenceTree ) ; // Note: This clobbers mVectorPotentialMultiGrid, which is likely the reason the assert below fails
            SolveVectorPoisson( vectorPotentialDummy , negativeVorticityMultiGrid[0] , 0 , boundaryCondition , mPoissonResidualStats ) ;
            ASSERT( vectorPotentialDummy == vectorPotentialMultiGrid[0] ) ;
        }
#       endif
#   endif

    mPoissonResidualStats_AcrossTime.mMean = ( mPoissonResidualStats.mMean + mPoissonResidualStats_AcrossTime.mMean ) * 0.5f ; // Incorrect aggregate of mean
    mPoissonResidualStats_AcrossTime.mStdDev = ( mPoissonResidualStats.mStdDev + mPoissonResidualStats_AcrossTime.mStdDev ) * 0.5f ; // Incorrect aggregate of stdDev
    mPoissonResidualStats_AcrossTime.mMin = Min2( mPoissonResidualStats.mMin , mPoissonResidualStats_AcrossTime.mMin ) ;
    mPoissonResidualStats_AcrossTime.mMax = Max2( mPoissonResidualStats.mMax , mPoissonResidualStats_AcrossTime.mMax ) ;
}




/** Compute velocity due to vorticity, for every point in a uniform grid, by solving a vector Poisson partial differential equation.

    \param vorticityGrid        Uniform grid of vorticity values, used to compute velocity-from-vorticity using Poisson solver.

    \todo   This routine calls SolveVectorPoisson which (depending on compiler settings inside that routine) uses "natural" boundary
            conditions.  Instead, what we want to use "radiation" or "open" boundary conditions, where the boundary has "no influence",
            that is, we want this solver to yield results consistent with using direct summation (or the other integral approaches).
            To accomplish that, this routine could first use an integral technique (e.g. direct summation, treecode or multipole)
            to compute velocity (or the vector potential) at the boundaries, then use SolveVectorPoisson with "essential" boundary
            conditions to populate the domain interior.  Incidentally that approach would also allow the domain to be closer
            to the interior, rather than inflating the domain, as is done now to diminish the problematic influence of the boundary.
*/
void VortonSim::ComputeVelocityFromVorticity_Differential( NestedGrid< Vec3 > & negativeVorticityMultiGrid , const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree )
{
    PERF_BLOCK( VortonSim__ComputeVelocityFromVorticity_Differential ) ;

    ASSERT( ! mVortons->empty() ) ;

    mVectorPotentialMultiGrid.Initialize( mGridTemplate ) ;   // Use same shape as base vorticity grid. (Note: could differ if you want.)

    ComputeVectorPotential( mVectorPotentialMultiGrid , negativeVorticityMultiGrid , vortonIndicesGrid , influenceTree ) ;

    UniformGrid< Vec3 > &   vectorPotentialGrid = mVectorPotentialMultiGrid[ 0 ] ;

    UniformGrid< Mat33 > jacobianGrid( mVelGrid ) ;

    jacobianGrid.Init() ;
    // TODO: It would be expedient here to have a routine to compute curl directly from vectorPotential, skipping Jacobian.
    ComputeJacobian( jacobianGrid , vectorPotentialGrid ) ;

    ComputeCurlFromJacobian( mVelGrid , jacobianGrid ) ;
}




/** Compute velocity due to vortons, for every point in a uniform grid.

    \param vortonIndicesGrid    Spatial partition of vortons, for fast lookup of vortons in a vicinity.
                                This is mutually exclusive with vorticityGrid.

    \param influenceTree        Nested grid of vortons and "super-vortons", used to compute velocity-from-vorticity using treecode.
                                This is mutually exclusive with vorticityGrid.

    \param vorticityGrid        Uniform grid of vorticity values, used to compute velocity-from-vorticity using Poisson solver.
                                This is mutually exclusive with influenceTree.

    \see CreateInfluenceTree

    \note This routine assumes CreateInfluenceTree has already executed.
*/
void VortonSim::ComputeVelocityFromVorticity( const UniformGrid< VECTOR< unsigned > > & vortonIndicesGrid , const NestedGrid< Vorton > & influenceTree , NestedGrid< Vec3 > & negativeVorticityMultiGrid )
{
    PERF_BLOCK( VortonSim__ComputeVelocityFromVorticity ) ;

    mVelGrid.Clear() ;                      // Clear any stale velocity information
    mVelGrid.CopyShape( mGridTemplate ) ;   // Use same shape as base vorticity grid. (Note: could differ if you want.)
    mVelGrid.Init() ;                       // Reserve memory for velocity grid.

    if( mVortons->empty() )
    {   // No vortons; no velocity.
        return ;
    }

#if ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES )
    ComputeVelocityFromVorticity_Integral( vortonIndicesGrid , influenceTree ) ;
    UNUSED_PARAM( negativeVorticityMultiGrid ) ; // Avoid "unused formal parameter" compiler warning.
#elif ( ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL ) )
    ComputeVelocityFromVorticity_Differential( negativeVorticityMultiGrid , vortonIndicesGrid , influenceTree ) ;
#else
    #error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
#endif
}




#if defined( _DEBUG )

static void TrackVorticityAndKineticEnergyChange(
      const VECTOR< Vorton > & vortons
    , const Vec3 & totalVorticityBefore
    , const float totalKineticEnergyBefore
    , float totalVorticityChangeMagMax
    , float totalVorticityChangeMagRelMax
    , float totalKineticEnergyChangeMax
    , float totalKineticEnergyChangeRelMax )
{
    const Vec3   totalVorticityAfter            = Vorton::ComputeTotalVorticity( vortons ) ;
    const Vec3   totalVorticityChange           = totalVorticityAfter - totalVorticityBefore ;
    const float  totalVorticityChangeMag        = totalVorticityChange.Magnitude() ;
    const float  totalVorticityChangeMagRel     = totalVorticityChangeMag / totalVorticityBefore.Magnitude() ;
    if( totalVorticityChangeMag > totalVorticityChangeMagMax )
    {
        totalVorticityChangeMagMax = totalVorticityChangeMag ;
    }
    if( totalVorticityChangeMagRel > totalVorticityChangeMagRelMax )
    {
        totalVorticityChangeMagRelMax = totalVorticityChangeMagRel ;
    }

    const float  totalKineticEnergyAfter        = Vorton::ComputeTotalKineticEnergy( vortons ) ;
    const float  totalKineticEnergyChange       = totalKineticEnergyAfter - totalKineticEnergyBefore ;
    const float  totalKineticEnergyChangeRel    = totalKineticEnergyChange / totalKineticEnergyBefore ;
    if( fabsf( totalKineticEnergyChange ) > fabsf( totalKineticEnergyChangeMax ) )
    {
        totalKineticEnergyChangeMax = totalKineticEnergyChange ;
    }
    if( fabsf( totalKineticEnergyChangeRel ) > fabsf( totalKineticEnergyChangeRelMax ) )
    {
        totalKineticEnergyChangeRelMax = totalKineticEnergyChangeRel ;
    }
}

#endif



/** Stretch and tilt vortons using velocity field.

    \param timeStep     Amount of time by which to advance simulation.

    \param uFrame       Frame counter, used to generate diagnostic files.

    \see GenerateBaroclinicVorticity, DiffuseAndDissipateVorticityPSE

    \see J. T. Beale, A convergent three-dimensional vortex method with
            grid-free stretching, Math. Comp. 46 (1986), 401-24, April.

    \note This routine assumes CreateInfluenceTree has already executed.

    \note This routine should run before advecting vortons, otherwise the vortons
            could leave the velocity grid boundaries.

*/
void VortonSim::StretchAndTiltVortons( const float & timeStep , const unsigned & uFrame )
{
#if ENABLE_FLUID_BODY_SIMULATION

    PERF_BLOCK( VortonSim__StretchAndTiltVortons ) ;

    if(     ( 0.0f == mVelGrid.GetExtent().x )
        ||  ( 0.0f == mVelGrid.GetExtent().y )
        ||  ( 0.0f == mVelGrid.GetExtent().z ) )
    {   // Domain is 2D, so stretching & tilting does not occur.
        return ;
    }

    // Compute all gradients of all components of velocity.
    UniformGrid< Mat33 > velocityJacobianGrid( mVelGrid ) ;
    velocityJacobianGrid.Init() ;
    ComputeJacobian( velocityJacobianGrid , mVelGrid ) ;

#if defined( _DEBUG )
    if( mOutputDiagnostics )
    {   // Compute curl from velocity, as a test.  This should approximately match the vorticity in mInfluenceTree[0].
        UniformGrid< Vec3 > curl( /* Copy geometry only, not contents */ (UniformGridGeometry&) mVelGrid ) ;
        curl.Init() ;
        ComputeCurlFromJacobian( curl , velocityJacobianGrid ) ;
        curl.GenerateBrickOfBytes( "curl" , uFrame ) ;
        #if TEST_RESAMPLE_VORTONS && USE_GIVEN_VORT_GRID
            vortonSim.AssignVortonsFromVorticity( curl ) ;
        #endif
    }
    const Vec3  totalVorticityBefore     = Vorton::ComputeTotalVorticity( * mVortons ) ;
    const float totalKineticEnergyBefore = Vorton::ComputeTotalKineticEnergy( * mVortons ) ;
#endif
    (void) uFrame ; // Avoid "unreferenced formal parameter" warning.

#if VORTON_SIM_GATHER_STATS
    mVorticityTermsStats.Reset( mVorticityTermsStats.mStretchTilt ) ;
#endif

    const size_t numVortons = mVortons->Size() ;

    for( unsigned offset = 0 ; offset < numVortons ; ++ offset )
    {   // For each vorton...
        Vorton &    rVorton     = (*mVortons)[ offset ] ;
        Mat33       velJac      ;
        velocityJacobianGrid.Interpolate( velJac , rVorton.mPosition ) ;
        // Compute stretching & tilting:
    #if 1
        const Vec3  stretchTilt = rVorton.mAngularVelocity * velJac ;    // ...using transpose formulation.
    #else
        const Vec3  stretchTilt = velJac * rVorton.mAngularVelocity ;    // ...using "classical" formulation.
    #endif
    #if VORTON_SIM_GATHER_STATS
        mVorticityTermsStats.mStretchTilt.Accumulate( stretchTilt.Magnitude() ) ;
    #endif
        rVorton.mAngularVelocity += 0.5f * stretchTilt * timeStep ;
    }

#if VORTON_SIM_GATHER_STATS
    mVorticityTermsStats.mStretchTilt.ConvertAccumulatedSamplesToStats( numVortons ) ;
#endif

#if defined( _DEBUG )
    {   // In principle, stretching and tilting should not change bulk vorticity or kinetic energy.
        static float totalVorticityChangeMagMax     = 0.0f ;
        static float totalVorticityChangeMagRelMax  = 0.0f ;
        static float totalKineticEnergyChangeMax    = 0.0f ;
        static float totalKineticEnergyChangeRelMax = 0.0f ;
        TrackVorticityAndKineticEnergyChange( * mVortons , totalVorticityBefore , totalKineticEnergyBefore , totalVorticityChangeMagMax , totalVorticityChangeMagRelMax , totalKineticEnergyChangeMax , totalKineticEnergyChangeRelMax ) ;
    }
#endif

#endif
}




/** Populate a UniformGrid with velocity values from vortons.

    \param velocityGrid (out) Grid into which to transfer velocity values.

    \param particles    Dynamic array of particles whose velocity information to
                        accumulate into velocityGrid.

*/
void VortonSim::PopulateVelocityGrid( UniformGrid< Vec3 > & velocityGrid , const VECTOR< Particle > & particles )
{
    PERF_BLOCK( VortonSim__PopulateVelocityGrid ) ;

    velocityGrid.Clear() ;                              // Clear any stale density information.
    velocityGrid.CopyShape( mGridTemplate ) ;           // Use same shape as base vorticity grid. (Note: could differ if you want.)
    Particles::PopulateVelocityGrid( velocityGrid , particles ) ;
}




#if defined( _DEBUG )

static const float tolerance = 0.001f ;

/** Diagnose density grid.

    This routine can be called per particle, to isolate problems populating the
    density grid.
*/
static void VerifyDensityGrid( const UniformGrid< float > & densityGrid , const UniformGrid< float > ugParticleContribution , float volumeCorrection , float massSumVortons )
{
    PERF_BLOCK( VortonSim__VerifyDensityGrid ) ;

#if VERIFY_NEUTRALLY_BUOYANT_DENSITY    // For diagnosing density grid population. Works only for neutrally buoyant configurations.
    for( unsigned offset = 0 ; offset < densityGrid.GetGridCapacity() ; ++ offset )
    {   // For each point in grid...
        const float pclsContrib = ugParticleContribution[ offset ] ;

        if( pclsContrib != 0.0f )
        {
            const float densFromGrid = densityGrid[ offset ] ;
            const float densOfPcls   = densFromGrid / pclsContrib ;
            if( ! Math::Resembles( densOfPcls , volumeCorrection , tolerance ) )
            {   // Density has an unexpected value.
                int i ; i = 0 ; ++ i ;  // Set breakpoint here.
            }
        }

        const float clampedVolCorrectedPclsContrib  = Min2( pclsContrib * volumeCorrection , 1.0f ) ;
        const float ambientDensityContrib           = 1.0f - clampedVolCorrectedPclsContrib  ;
        const float totalContribution               = clampedVolCorrectedPclsContrib + ambientDensityContrib ;

        if( ! Math::Resembles( totalContribution , 1.0f , tolerance ) )
        {   // Corrected contribution tally is not exactly full.
            // That could legitimately happen if particles "over" contribute,
            // that is, if ambient does not contribute.  That probably means
            // multiple particles contributed to a single gridpoint.
            if( ambientDensityContrib > 0.0f )
            {   // Clamping did not kick in so ambient did contribute.
                // Corrected contribution tally has an unexpected value.
                int i ; i = 0 ; ++ i ;  // Set breakpoint here.
            }
        }
    }
#else
    UNUSED_PARAM( volumeCorrection ) ;
#endif

    {   // Verify mass is the same for particles and grid.
        const float densityGridSum  = densityGrid.Sum() * densityGrid.GetCellVolume() ;
        const float massSumGrid     = densityGridSum ;
        ASSERT( Math::Resembles( massSumVortons , massSumGrid ) ) ;
    }
}
#endif




/** Populate a UniformGrid with density and mass fraction values from vortons.

    \param densityGrid (out)    Grid into which to transfer density values.

    \param particles    Dynamic array of particles whose density information to
                        accumulate into densityGrid.
*/
void VortonSim::PopulateDensityAndMassFractionGrids( UniformGrid< float > & densityGrid , const VECTOR< Particle > & particles , const unsigned uFrame )
{
    PERF_BLOCK( VortonSim__PopulateDensityAndMassFractionGrids ) ;

    densityGrid.Clear() ;                       // Clear any stale density information.
    densityGrid.Expand( mGridTemplate , 1 ) ;   // Use same shape as base vorticity grid. (Note: could differ if you want; just change expansion factor from 1 to whatever, though ideally a power of 2.)
    densityGrid.Init( 0.0f )                ;   // Reserve memory for density grid and initialize all values to zero.

    const UniformGridGeometry & gridGeometry = static_cast< UniformGridGeometry >( densityGrid ) ;

#if ENABLE_FIRE
    mFuelFractionGrid.Clear() ;
    mFuelFractionGrid.CopyShape( gridGeometry ) ;
    mFuelFractionGrid.Init( 0.0f ) ;

    mFlameFractionGrid.Clear() ;
    mFlameFractionGrid.CopyShape( gridGeometry ) ;
    mFlameFractionGrid.Init( 0.0f ) ;

    mSmokeFractionGrid.Clear() ;
    mSmokeFractionGrid.CopyShape( gridGeometry ) ;
    mSmokeFractionGrid.Init( 0.0f ) ;
#endif

    DEBUG_ONLY( float massSumVortons = 0.0f ) ;

    // The grid resulting from this operation must preserve the same total mass 
    // that the particles represent, so multiply by the ratio of the particle 
    // volume to gridcell volume.
    // Assume vortons all have same volume.  If they do not, the formula inside
    // the loop becomes more complicated.
    const float oneOverCellVolume   = 1.0f / densityGrid.GetCellVolume() ;
    const float vortonVolume        = particles[ 0 ].GetVolume() ;
    const float volumeRatio         = vortonVolume * oneOverCellVolume ;

    // Amount of contribution to each grid point.
    UniformGrid< float > ugParticleContribution( gridGeometry ) ;
    ugParticleContribution.Init( 0.0f ) ;

    // Populate density and mass-fraction grids.
    const size_t numParticles = particles.Size() ;
    for( size_t uParticle = 0 ; uParticle < numParticles ; ++ uParticle )
    {   // For each particle in the array...
        const Particle  &   rParticle   = particles[ uParticle ] ;
        const Vec3      &   rPosition   = rParticle.mPosition   ;
        ASSERT( ! IsNan( rPosition ) && ! IsInf( rPosition ) ) ;

        DEBUG_ONLY( const unsigned      uOffset     = densityGrid.OffsetOfPosition( rPosition ) ) ;
        ASSERT( uOffset < densityGrid.GetGridCapacity() ) ;

        ASSERT( rParticle.mDensity > 0.0f ) ;
        ASSERT( fabsf( rParticle.GetVolume() - vortonVolume ) < FLT_EPSILON ) ; // Assuming vorton volume is uniform.  Otherwise formula changes.
        const float volumeCorrectedDensity = rParticle.mDensity * volumeRatio ;

        densityGrid.Accumulate( rPosition , volumeCorrectedDensity ) ;

        DEBUG_ONLY( massSumVortons += rParticle.GetMass() ) ;

        // Tally particle contribution to each gridpoint.
        // Note that technically 1.0 here should be volume-corrected, but we
        // assume volume correction is uniform and apply it later.
        ugParticleContribution.Accumulate( rPosition , 1.0f ) ;

    #if ENABLE_FIRE
        // Since fractions should average (not add), need to divide by contribution later.
        DEBUG_ONLY( const float totalFraction = rParticle.mFuelFraction + rParticle.mFlameFraction + rParticle.mSmokeFraction ) ;
        mFuelFractionGrid.Accumulate ( rPosition , rParticle.mFuelFraction  ) ;
        mFlameFractionGrid.Accumulate( rPosition , rParticle.mFlameFraction ) ;
        mSmokeFractionGrid.Accumulate( rPosition , rParticle.mSmokeFraction ) ;
        ASSERT( ( 0.0f == totalFraction ) || Math::Resembles( totalFraction , 1.0f ) ) ;
    #endif
    }

    DEBUG_ONLY( VerifyDensityGrid( densityGrid , ugParticleContribution , volumeRatio , massSumVortons ) ) ;

    // Apply per-gridpoint corrections.
    const unsigned densityGridCapacity = densityGrid.GetGridCapacity() ;
    for( unsigned offset = 0 ; offset < densityGridCapacity ; ++ offset )
    {   // For each point in grid...

        // Add ambient density contribution.
        // Fluid pervades the region even in the absence of particles.
        // Particles provide information (like density) where they reside
        // but when absent, there is still fluid.  Multiple particles can
        // contribute to each grid cell.  Particles contribute to each gridpoint
        // inversely proportational to their distance to it. Whether N particles
        // each contribute 1/N of their mass, or a single particle contributes
        // all of its mass, the result is the same.  That makes it possible for
        // N particles to have the cummulative contribution to a particular gridpoint equal or exceed 1.0.  In
        // that case, ambient contributes nothing.  If particle contribution
        // is zero at a gridpoint then that gridpoint has only ambient density.
        // If the contribution is in (0,1) then ambient contributes a complementary amount.
        //
        // Details:
        //    Each particle i contributes phi_i fraction of its rho_i density.
        //    Each gridpoint then has a tuple < SUM_i rho_i phi_i , SUM_i phi_i >.
        //    Ambient contributes PHI_0 = max( 1 - SUM_i phi_i , 0 )
        //    so the density rho at a gridpoint is
        //        rho = SUM_i rho_i phi_i + PHI_0 * rho_0.
        const float pclsContrib             = ugParticleContribution[ offset ] * volumeRatio ; // Technically volumeRatio should be in ugParticleContribution already. This assumes it is uniform.
        const float clampedPclsContrib      = Min2( pclsContrib , 1.0f ) ;
        const float ambientDensityContrib   = 1.0f - clampedPclsContrib ;

        densityGrid[ offset ] += ambientDensityContrib * mAmbientDensity ;

    #if VERIFY_NEUTRALLY_BUOYANT_DENSITY    // For diagnosing density grid population. Works only for neutrally buoyant configurations
        if( ! Math::Resembles( densityGrid[ offset ] , 1.0f , tolerance ) )
        {   // Corrected density does not match ambient.
            // That could legitimately happen if particles "over" contribute,
            // that is, if ambient does not contribute.  That probably means
            // multiple particles contributed to a single gridpoint.
            const float ddg = densityGrid[ offset ] ; (void) ddg ;
            if( ambientDensityContrib > 0.0f )
            {   // Clamping did not kick in so ambient did contribute.
                // Corrected density has an unexpected value.
                int i ; i = 0 ; ++ i ;  // Set breakpoint here.
            }
        }
    #endif

    #if ENABLE_FIRE
        // Turn mass-fraction sums into averages.
        // This assumes "1" was accumulated instead of volumeRatio.
        // Technically volumeRatio should have been accumulated for contribution,
        // and particle number would have to be tallied separately.
        // But here we exploit the fact that vorton volume is uniform.
        if( ugParticleContribution[ offset ] > 0.0f )
        {
            const float oneOverContribution = 1.0f / ugParticleContribution[ offset ] ;
            mFuelFractionGrid [ offset ] *= oneOverContribution ;
            mFlameFractionGrid[ offset ] *= oneOverContribution ;
            mSmokeFractionGrid[ offset ] *= oneOverContribution ;
            DEBUG_ONLY( const float totalFraction = mFuelFractionGrid[ offset ] + mFlameFractionGrid[ offset ] + mSmokeFractionGrid[ offset ] ) ;
            DEBUG_ONLY( (void) totalFraction ) ;
        }
    #endif
    }

    ASSERT( ! densityGrid.HasZeroExtent() ) ;

    #if defined( _DEBUG ) || VORTON_SIM_OUTPUT_DENSITY
    if( mOutputDiagnostics )
    {   // Output density for visualization.
        mDensityGrid.GenerateBrickOfBytes( "density" , uFrame ) ;
    }
    #else
        UNUSED_PARAM( uFrame ) ;
    #endif
}




#if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS

static void PoisonDensityGradientSlice( UniformGrid< Vec3 > & densityGradientGrid , const VECTOR< Vorton > & particles , size_t iPclStart , size_t iPclEnd )
{
    // Poison density gradient grid based on vortons in contact with container boundaries.
    const Vec3 zero( 0.0f , 0.0f , 0.0f ) ;
    for( size_t iParticle = iPclStart ; iParticle < iPclEnd ; ++ iParticle )
    {   // For each particle in the array...
        const Particle  &   rParticle   = particles[ iParticle ] ;
        if( rParticle.mHitBoundary )
        {   // This particle hit a boundary.
            const Vec3      &   rPosition   = rParticle.mPosition   ;
            ASSERT( ! IsNan( rPosition ) && ! IsInf( rPosition ) ) ;
            // Zero out any gradients along contact normal.
            // Remove normal component at each surrounding gridpoint.
        #if USE_TBB && 0
            densityGradientGrid.RemoveComponent_ThreadSafe( rPosition , rParticle.mHitNormal ) ;
        #else
            densityGradientGrid.RemoveComponent( rPosition , rParticle.mHitNormal ) ;
        #endif
        }
    }
}




/** Remove component of density gradient parallel to surface normals, where vortons contacted with container boundaries.

    This uses hit info from previous frame, and the latency leaves spurious
    torque especially notable on the first frame.

    Resetting the whole grid cell means particles near, but not at, boundaries
    do not get the torque they need to flatten surface.

*/
static void PoisonDensityGradient( UniformGrid< Vec3 > & densityGradientGrid , const VECTOR< Vorton > & particles )
{
    PERF_BLOCK( PoisonDensityGradient_VortonHits ) ;

    const size_t numVortons = particles.Size() ;

#if USE_TBB && 0 // Thread-safe multi-threaded version runs slower than serial version, so this is disabled.
    // Estimate grain size based on size of problem and number of processors.
    const size_t grainSize =  Max2( 1 , numVortons / gNumberOfProcessors ) ;
    // Compute velocity at gridpoints using multiple threads.
    parallel_for( tbb::blocked_range<size_t>( 0 , numVortons , grainSize ) , VortonSim_PoisonDensityGradient_TBB( densityGradientGrid , particles ) ) ;
#else
    PoisonDensityGradientSlice( densityGradientGrid , particles , 0 , numVortons ) ;
#endif
}

#endif




#if POISON_DENSITY_GRADIENT_BASED_ON_RIGID_SPHERE_GEOMETRY
/** Remove spurious density gradients that could arise due to geometric arrangement of vortons.

    Any gradient due to the arrangement of packed hard spheres is a spurious artifact of geometry.
    Closest packing leads to an average density of pi/sqrt(18)~=0.74048.
    Farthest packing leads to an average density of 4pi/24~=0.52350.
    Their ratio is 1/sqrt(2)~=0.70711.  So a variation smaller than (1-1/sqrt(2))~=0.29289
    is unreliable.  Note that applying this ratio requires knowing the density
    at each point where this is applied.  So the algorithm would need density as well as its gradient.
*/
static void PoisonDensityGradient( UniformGrid< Vec3 > & densityGradientGrid , UniformGrid< float > & densityGrid )
{
    PERF_BLOCK( PoisonDensityGradient_SphereGeometry ) ;

    const Vec3 zero( 0.0f , 0.0f , 0.0f ) ;
    const unsigned densityGradientGridCapacity = densityGradientGrid.GetGridCapacity() ;
    for( unsigned offset = 0 ; offset < densityGradientGridCapacity ; ++ offset )
    {   // For each point in grid...
        Vec3 gridPointPosition ;
        densityGradientGrid.PositionFromOffset( gridPointPosition , offset ) ;
        // Test whether grid point is inside rigid body.
        // If so, set gradient to zero there.
        Vec3 &          densGrad        = densityGradientGrid[ offset ] ;
        const float     densGradMag2    = densGrad.Mag2() ;
        const float &   density         = densityGrid[ offset ] ;
        const float     densGrad2Thresh = 0.29289f * density ;

        if( densGradMag2 < densGrad2Thresh )
        {
            densGrad = zero ;
        }
    }
}
#endif




/** Process combustion for a vorton, and assign mass fractions.

    \param timeStep     Amount of time by which to advance simulation.

    \param rVorton Reference to vorton to process.

    \note This routine assumes PopulateDensityAndMassFractionGrids has
            already executed, specifically the mass fraction grid is populated.

    \note This routine should run before advecting vortons, otherwise the vortons
            could leave the velocity grid boundaries, and this routine needs to interpolate
            within those boundaries.

*/
inline void VortonSim::CombustAndSetMassFractions( float timeStep , Vorton & rVorton )
{
#if ENABLE_FIRE

    //PERF_BLOCK( VortonSim__CombustAndSetMassFractions ) ;

    const float vortonTemperature = rVorton.GetTemperature( mAmbientDensity ) ;
    ASSERT( ! IsNan( vortonTemperature ) && ! IsInf( vortonTemperature ) ) ;

    ASSERT( ! IsNan( rVorton.mFlameFraction ) && ! IsInf( rVorton.mFlameFraction ) ) ;
    if( rVorton.mFlameFraction > 0.0f )
    {   // This particle is on fire.
        ASSERT( rVorton.mFuelFraction  >= 0.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFuelFraction  <= 1.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFlameFraction >= 0.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFlameFraction <= 1.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFuelFraction + rVorton.mFlameFraction <= 1.0f ) ; // Paranoid sanity check.
    #if 0 // Stam & Fiume (1995) -- Akin to an inverse Arrhenius reaction rate.
        const float temperatureDependence   = exp( - vortonTemperature / mSmokeTemperature ) ;
    #elif 1 // MJG "gradual step" function (Gaussian)
        const float arg1 = vortonTemperature / mSmokeTemperature ;
        const float arg2 = arg1 * arg1 ;
        const float temperatureDependence   = exp( - arg2 ) ;
    #else // Step function
        const float temperatureDependence   = vortonTemperature < mSmokeTemperature ? 1.0f : 0.0f ;
    #endif
        const float smokeRate               = mSmokeRateFactor * temperatureDependence * rVorton.mFlameFraction ;
        const float flameToSmokeChange      = smokeRate * timeStep ;
        // Decrease amount of flame.  This automatically increases the amount of smoke,
        // since (smokeFraction) is defined as 1-mFlameFraction-mFuelFraction.
        // Note that flame-to-smoke conversion happens first.  This
        // allows for flames to appear for at least a frame even if the burn
        // rate is extremely high.  If the order of operations was
        // fuel-to-flame and flame-to-smoke, then fuel could turn to smoke
        // in a single frame without flame ever getting a change to render.
        rVorton.mFlameFraction -= flameToSmokeChange ;

        ASSERT( rVorton.mFuelFraction  >= 0.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFuelFraction  <= 1.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFlameFraction >= 0.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFlameFraction <= 1.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFuelFraction + rVorton.mFlameFraction <= 1.0f ) ; // Paranoid sanity check.
    }

    ASSERT( ! IsNan( rVorton.mFuelFraction ) && ! IsInf( rVorton.mFuelFraction ) ) ;
    if( rVorton.mFuelFraction > 0.0f )
    {   // This particle has some fuel.
    #if USE_ARRHENIUS
        const float temperatureDependence   = exp( - mCombustionTemperature / vortonTemperature ) ;
    #else
        const float temperatureDependence   = vortonTemperature > mCombustionTemperature ? 1.0f : 0.0f ;
    #endif
        const float combustionRate  = mCombustionRateFactor * temperatureDependence * rVorton.mFuelFraction ;
        const float fuelToFlameChange = combustionRate * timeStep ;
        // Decrease amount of fuel and increase amount of flame.
        rVorton.mFuelFraction  -= fuelToFlameChange ;
        rVorton.mFlameFraction += fuelToFlameChange ;
        ASSERT( rVorton.mFuelFraction  >= 0.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFuelFraction  <= 1.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFlameFraction >= 0.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFlameFraction <= 1.0f ) ; // Paranoid sanity check.
        ASSERT( rVorton.mFuelFraction + rVorton.mFlameFraction <= 1.0f ) ; // Paranoid sanity check.

        // Change temperature due to heat released by combusting fuel:
        const float temperatureChange = mSpecificFreeEnergy * fuelToFlameChange * rVorton.GetDensity() * mSpecificHeatCapacity ;
        rVorton.SetTemperature( mAmbientDensity , vortonTemperature + temperatureChange ) ;
    }

    // For simplicity, assign the balance of the species fraction to smoke.
    // In principle, smoke should only result from combustion, but here
    // we just make everything in the fluid that isn't fuel or flame, smoke.
    rVorton.mSmokeFraction = 1.0f - ( rVorton.mFuelFraction + rVorton.mFlameFraction ) ;

    ASSERT( rVorton.mFuelFraction  >= 0.0f ) ; // Paranoid sanity check.
    ASSERT( rVorton.mFuelFraction  <= 1.0f ) ; // Paranoid sanity check.
    ASSERT( rVorton.mFlameFraction >= 0.0f ) ; // Paranoid sanity check.
    ASSERT( rVorton.mFlameFraction <= 1.0f ) ; // Paranoid sanity check.
    ASSERT( rVorton.mSmokeFraction >= 0.0f ) ; // Paranoid sanity check.
    ASSERT( rVorton.mSmokeFraction <= 1.0f ) ; // Paranoid sanity check.

#endif
}




/** Populate signed distance grid from density grid.

    \param signedDistanceGrid   Grid of signed distance values, where negative means inside,
                                positive means outside and 0 means at the surface of a fluid.

    \param densityGrid          Grid of fluid density values.
*/
void VortonSim::PopulateSignedDistanceGridFromDensityGrid( UniformGrid< float > & signedDistanceGrid , const UniformGrid< float > & densityGrid , const unsigned uFrame )
{
    PERF_BLOCK( VortonSim__PopulateSignedDistanceGridFromParticleDensity ) ;

    signedDistanceGrid.Clear() ;
    signedDistanceGrid.CopyShape( mDensityGrid ) ;
    signedDistanceGrid.Init( FLT_MAX ) ; // Reserve memory for SDF grid and initialize all values to FLT_MAX.

    {
        UniformGrid< int > sdfPinnedGrid ;
        sdfPinnedGrid.Clear() ;
        sdfPinnedGrid.CopyShape( densityGrid ) ;
        sdfPinnedGrid.Init( 0 ) ; // Reserve memory for grid and initialize all values to false.
        ComputeImmediateSDFFromDensity_Nearest( signedDistanceGrid , sdfPinnedGrid , densityGrid , mAmbientDensity ) ;
        ComputeRemainingSDFFromImmediateSDF_Nearest( signedDistanceGrid , sdfPinnedGrid ) ;
//ComputeSDF_Sphere( signedDistanceGrid ) ; // DO NOT SUBMIT. Diagnosing surface tracer placment
    }

#   if defined( _DEBUG ) || VORTON_SIM_OUTPUT_SDF
    if( mOutputDiagnostics )
    {   // Output density for visualization.
        signedDistanceGrid.GenerateBrickOfBytes( "sdf" , uFrame ) ;
    }
#   else
        UNUSED_PARAM( uFrame ) ;
#   endif
}




/** Compute baroclinic generation of vorticity due to buoyancy.

    \param timeStep     Amount of time by which to advance simulation.

    \param iPclStart    Index of first particle to process.

    \param iPclEnd      One past index of last particle to process.

    \see DiffuseAndDissipateVorticityPSE, StretchAndTiltVortons

    \note This routine assumes CreateInfluenceTree has already executed,
            specifically that the velocity grid shape has already been calculated.

    \note This routine assumes PopulateDensityAndMassFractionGrids has
            already executed, specifically the mass fraction grid is populated.

    \note This routine should run before advecting vortons, otherwise the vortons
            could leave the velocity grid boundaries, and this routine needs to
            interpolate within those boundaries.

*/
void VortonSim::GenerateBaroclinicVorticitySlice( float timeStep
#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS
                                                 , const VECTOR< Vec3 > * densityGradients
#endif

                                                 , size_t iPclStart , size_t iPclEnd )
{
    PERF_BLOCK( VortonSim__GenerateBaroclinicVorticitySlice ) ;

    const float oneOverDensity = 1.0f / GetAmbientDensity() ;

    for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
    {   // For each particle in this slice...
        Vorton &    rVorton         = (*mVortons)[ iPcl ] ;

        // Technically the baroclinic term should have density in the
        // denominator, but it's relatively expensive to calculate the local
        // value, plus the Boussinesq approximation assumes density variations
        // are small compared to the ambient.  So instead, we use the ambient
        // density in the denominator, which yields the correct units and gets
        // results in the right ballpark.
        // This is yet another example of where, for visual effects,
        // we take drastic liberties in the name of speed over accuracy.

        //float       density ;
        //mDensityGrid.Interpolate( density , rVorton.mPosition ) ;
        //ASSERT( density > 0.0f ) ;

    #if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS
        const Vec3 & densityGradient = (*densityGradients)[ iPcl ] ;
        ASSERT( ! IsNan( densityGradient ) && ! IsInf( densityGradient ) ) ;
    #elif POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS
        Vec3    densityGradient( 0.0f , 0.0f , 0.0f ) ;
        mDensityGradientGrid.InterpolateConditionally( densityGradient , rVorton.mPosition ) ;
    #else   // Use density gradient computed on grid.
        Vec3    densityGradient ;
        mDensityGradientGrid.Interpolate( densityGradient , rVorton.mPosition ) ;
    #endif

        if( ! IsNan( densityGradient ) && ! IsInf( densityGradient ) )
        {

        #if COMPUTE_PRESSURE_GRADIENT
            #error Do not use this branch.  It was a half-baked idea that did not work out.
            Vec3        pressureGradient ;
            mPressureGradientGrid.Interpolate( pressureGradient , rVorton.mPosition ) ;
            ASSERT( ! IsNan( pressureGradient ) && ! IsInf( pressureGradient ) ) ;
            pressureGradient += mGravAccel ;
        #else
            const Vec3  pressureGradient = mGravAccel ;
        #endif

            const Vec3  baroclinicGeneration = densityGradient ^ pressureGradient * oneOverDensity ;
            // The formula for the line below is meant to update vorticity,
            // but since particles store vorticity as angular velocity, the formula has an extra factor of 0.5.
            ASSERT( ! IsNan( baroclinicGeneration ) && ! IsInf( baroclinicGeneration ) ) ;
            ASSERT( ! IsNan( rVorton.mAngularVelocity ) && ! IsInf( rVorton.mAngularVelocity ) ) ;
            rVorton.mAngularVelocity += 0.5f * baroclinicGeneration * timeStep ;

        #if VORTON_SIM_GATHER_STATS
            mVorticityTermsStats.mBaroclinic.Accumulate( baroclinicGeneration.Magnitude() ) ; // Note: This is not thread safe.
        #endif
        }

        CombustAndSetMassFractions( timeStep , rVorton ) ;
    }
}




/** Compute baroclinic generation of vorticity due to buoyancy.

    \param timeStep     Amount of time by which to advance simulation.

    \param uFrame       Frame counter, used to generate diagnostic files.

    \see DiffuseAndDissipateVorticityPSE, StretchAndTiltVortons

    \note This routine assumes CreateInfluenceTree has already executed,
            specifically that the velocity grid has already been calculated.

    \note This routine assumes PopulateDensityAndMassFractionGrids has
            already executed, specifically the mass fraction grid is populated.

    \note This routine should run before advecting vortons, otherwise the vortons
            could leave the velocity grid boundaries.

*/
void VortonSim::GenerateBaroclinicVorticity( const float timeStep , const unsigned uFrame
#if COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS || COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH
                                            , const UniformGrid< VECTOR< unsigned > > & ugVortonIndices
#endif
                                            )
{
    PERF_BLOCK( VortonSim__GenerateBaroclinicVorticity ) ;

    UNUSED_PARAM( uFrame ) ; // Avoid "unreferenced formal parameter" warning.

    if( fabsf( mGravAccel * mVelGrid.GetExtent() ) < FLT_EPSILON )
    {   // Domain is 2D and has no significant component along the gravity direction,
        // so baroclinic generation cannot occur in this Boussinesq approximation.
        return ;
    }

#if VORTON_SIM_GATHER_STATS
    mVorticityTermsStats.Reset( mVorticityTermsStats.mBaroclinic ) ;
#endif

#if COMPUTE_PRESSURE_GRADIENT
    if( mPhysicalObjects )
    {
        const float vortonRadius = (*mVortons)[ 0 ].GetRadius() * 1.2f ;
        ComputePressureGradient( mGridTemplate , mPressureGradientGrid , * mPhysicalObjects , vortonRadius ) ;
    }
#endif

    // Populate density grid.
    //
    // Passing mVortons to PopulateDensityAndMassFractionGrids assumes Particle and Vorton layout are identical.
    // This is a weird, fragile use of inheritance but I desperately want to avoid virtual calls, while for
    // pedagogic purposes I want to distinguish between a regular tracer particle and a vortex particle, and
    // still treat arrays of those things uniformly, for economy of code.

    ASSERT( sizeof( Particle ) == sizeof( Vorton ) ) ;
    PopulateDensityAndMassFractionGrids( mDensityGrid , reinterpret_cast< VECTOR< Particle > & >( * mVortons ) , uFrame ) ;

    const size_t    numVortons      = mVortons->Size() ;

    // Compute density gradient.

#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH
    {
        // Compute density gradient using SPH, then use that to compute baroclinic term.
        // This uses an SPH scheme that computes gradients at the boundaries differently depending on whether particles touch a wall or just particle-free zones.
        // Note that for this purpose it would be better to have a smaller inflRad.  1 is probably too small.  2 to 3.5 might be okay.  4 is probably too big.

    #if USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH
        extern float influenceRadiusScale ; // Defined in smoothedPclHydro.cpp
        const float pclRad  = ( * mVortons )[ 0 ].GetRadius() ;
        const float inflRad = influenceRadiusScale * pclRad ;   // Make grid cell big enough to include all possible neighbors.

        UniformGrid< VECTOR< unsigned > >   vortonIndicesGridSph  ;   ///< Spatial partition of indices into mVortons.
        PartitionVortons( timeStep , uFrame , vortonIndicesGridSph , inflRad ) ;
    #endif

        (void) ugVortonIndices ; // SPH and VPM have different requirements on the spatial partition.  Might be okay to use SPH requirement for both, though that would be slower.

        ComputeSphDensityAtParticles_Grid( mFluidDensitiesAtPcls , * mVortons , vortonIndicesGridSph ) ;
        FluidBodySim::ComputeParticleProximityToWalls( mVortonBodyProximities , reinterpret_cast< VECTOR< Particle > & >( * mVortons ) , * mPhysicalObjects , inflRad ) ;
        ComputeSphMassDensityGradient_Grid( mDensityGradientsAtPcls , mFluidDensitiesAtPcls , * mVortons , mVortonBodyProximities , vortonIndicesGridSph , mAmbientDensity ) ;

    #if ENABLE_FLUID_BODY_SIMULATION
        #if USE_TBB
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize =  Max2( size_t( 1 ) , numVortons / gNumberOfProcessors ) ;
            // Compute baroclinic generation of vorticity using threading building blocks
            parallel_for( tbb::blocked_range<size_t>( 0 , numVortons , grainSize ) , VortonSim_GenerateBaroclinicVorticity_TBB( timeStep , this , & mDensityGradientsAtPcls ) ) ;
        #else
            GenerateBaroclinicVorticitySlice( timeStep , & mDensityGradientsAtPcls , 0 , numVortons ) ;
        #endif
    #endif
    }
#elif COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS // Older experiment -- did not work out.
    {
        VECTOR< Vec3 > densityGradients ; // One density gradient per vorton.
        ComputeDensityGradientFromVortons( densityGradients , ugVortonIndices , * mVortons ) ;

    #if ENABLE_FLUID_BODY_SIMULATION
        #if USE_TBB
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize =  Max2( size_t( 1 ) , numVortons / gNumberOfProcessors ) ;
            // Compute baroclinic generation of vorticity using threading building blocks
            parallel_for( tbb::blocked_range<size_t>( 0 , numVortons , grainSize ) , VortonSim_GenerateBaroclinicVorticity_TBB( timeStep , this , densityGradients ) ) ;
        #else
            GenerateBaroclinicVorticitySlice( timeStep , & densityGradients , 0 , numVortons ) ;
        #endif
    #endif
    }
#else // Transfer density from vortons to grid and compute density gradient from grid.
    #if POISON_DENSITY_BASED_ON_VORTONS_HITTING_WALLS
        PoisonDensity( mDensityGrid , * mVortons ) ;
    #endif

    #if POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS
        if( mPhysicalObjects )
        {
            const float vortonRadius = (*mVortons)[ 0 ].GetRadius() * 1.2f ;
            FluidBodySim::PoisonDensity( mDensityGrid , * mPhysicalObjects , vortonRadius ) ;
        }
    #endif

        mDensityGradientGrid.Clear() ;                      // Clear any stale density gradient information
        mDensityGradientGrid.CopyShape( mGridTemplate ) ;   // Use same shape as base velocity grid. (Note: could differ if you want.)
        mDensityGradientGrid.Init() ;                       // Reserve memory for density gradient grid.
    #if POISON_DENSITY_BASED_ON_VORTONS_HITTING_WALLS || POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS
        // Compute density gradient using conditional difference.
        ComputeGradientConditionally( mDensityGradientGrid , mDensityGrid ) ;
    #else
        // Compute density gradient using centered difference in interior and forward/backward difference on boundaries.
        ComputeGradient( mDensityGradientGrid , mDensityGrid ) ;
    #endif

    #if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
        PoisonDensityGradient( mDensityGradientGrid , * mVortons ) ;
    #endif

    #if POISON_DENSITY_GRADIENT_BASED_ON_RIGID_SPHERE_GEOMETRY
        PoisonDensityGradient( mDensityGradientGrid , mDensityGrid , mAmbientDensity ) ;
    #endif

    #if POISON_DENSITY_GRADIENT_BASED_ON_GRIDPOINTS_INSIDE_WALLS
        if( mPhysicalObjects )
        {
            const float vortonRadius = (*mVortons)[ 0 ].GetRadius() * 1.2f ;
            FluidBodySim::PoisonDensityGradient( mDensityGradientGrid , * mPhysicalObjects , vortonRadius ) ;
        }
    #endif

    #if ENABLE_FLUID_BODY_SIMULATION
        #if USE_TBB
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize =  Max2( size_t( 1 ) , numVortons / gNumberOfProcessors ) ;
            // Compute baroclinic generation of vorticity using threading building blocks
            parallel_for( tbb::blocked_range<size_t>( 0 , numVortons , grainSize ) , VortonSim_GenerateBaroclinicVorticity_TBB( timeStep , this , NULL_PTR ) ) ;
        #else
            GenerateBaroclinicVorticitySlice( timeStep , 0 , numVortons ) ;
        #endif
    #endif

#endif


#if VORTON_SIM_GATHER_STATS
    mVorticityTermsStats.mBaroclinic.ConvertAccumulatedSamplesToStats( numVortons ) ;
#endif
}




/** Spatially partition vortons.

    This routine partitions space into cells using the same grid geometry as the
    "base vorton" grid, possibly modified to respect the given minCellSpacing.  Each
    vorton gets assigned to the cell that contains it.

    \param timeStep     Amount of time by which to advance simulation.

    \param uFrame       Frame counter, used to generate diagnostic files.

    \param ugVortonIndices - Uniform grid of vorton indices.  Assign each cell
        to contain a vector of the indices of vortons that reside inside that cell.
        The index values refer to elements in mVortons.

    \see StretchAndTiltVortons, GenerateBaroclinicVorticity

    \note This routine assumes mGridTemplate has already been established for this frame.

*/
void VortonSim::PartitionVortons( const float & /* timeStep */ , const unsigned & /* uFrame */ ,
#if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION
                                 SpatialPartition & vortonGrid
#else
                                 UniformGrid< VECTOR< unsigned > > & ugVortonIndices
#endif
                                 , float minCellSpacing )
{
#if ENABLE_FLUID_BODY_SIMULATION

    PERF_BLOCK( VortonSim__PartitionVortons ) ;

#if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION
    //vortonGrid.Init( numVortons , mGridTemplate ) ;                 // Initialize spatial partition.
#else
    ugVortonIndices.Clear() ;                                       // Clear any stale index information.
    ugVortonIndices.FitShape( mGridTemplate , minCellSpacing ) ;    // Use same shape as base vorticity grid.
    //ugVortonIndices.Init() ;                                        // Reserve memory for grid and initialize all values to empty.
#endif

    Particles::PartitionParticles( reinterpret_cast< const VECTOR< Particle > & >( *mVortons ) ,
#   if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION
                            vortonGrid
#   else
                            ugVortonIndices
#   endif
                        , minCellSpacing ) ;

#endif
}




/** Exchange vorticity or merge vortons.

    \return true if particle was kept, false if deleted
*/
inline bool VortonSim::ExchangeVorticityOrMergeVortons( const unsigned & rVortIdxHere , Vorton & rVortonHere , Vec3 & rAngVelHere , const unsigned & ivThere , VECTOR< unsigned > & cell , const float & timeStep )
{
#if ! ENABLE_MERGING_VORTONS
    (void) rVortIdxHere , rVortonHere ; // Avoid "unreferenced formal parameter" warning.
#endif

    ASSERT( rVortonHere.IsAlive() ) ;
    const unsigned &    rVortIdxThere   = cell[ ivThere ] ;
    Vorton &            rVortonThere    = (*mVortons)[ rVortIdxThere ] ;
    const float         diffusionRange  = 4.0f * rVortonHere.mSize ;
    const float         diffusionRange2 = POW2( diffusionRange ) ;
    //if( rVortonThere.IsAlive() )
    {   // Particle is alive.
        const Vec3      separation      = rVortonHere.mPosition - rVortonThere.mPosition ;
        const float     dist2           = separation.Mag2() ;

    #if ENABLE_MERGING_VORTONS
        const float     radiusSum       = ( rVortonHere.mSize + rVortonThere.mSize ) * 0.5f ; // 1/2 because size=2*radius
        const float     radiusSum2      = Pow2( radiusSum ) ;
        static float    mergeThreshold  = 0.25f ; // A value of 0.25 means the center of 1 vorton is just barely inside the core of another.
        if( dist2 < radiusSum2 * mergeThreshold )
        {   // Vortices are close enough to merge.
            Particles::Merge( reinterpret_cast< VECTOR< Particle > & >( * mVortons ) , rVortIdxHere , rVortIdxThere , mAmbientDensity ) ;
            // Remove particle from cell.
            ASSERT( ivThere < cell.Size() ) ;
            cell[ ivThere ] = cell[ cell.Size() - 1 ] ;
            cell.PopBack() ;
            return false ;   // Tell caller a particle was deleted.
        }
        else
    #endif
        {   // Vortices are far enough to remain separate.
            const float distLaw         = exp( - dist2 / diffusionRange2 ) ;
            Vec3 &      rAngVelThere    = rVortonThere.mAngularVelocity ;
            const Vec3  vortDiff        = rAngVelHere - rAngVelThere ;
            const float exchangeFraction= Clamp( 2.0f * mViscosity * timeStep * distLaw , -0.5f , 0.5f ) ;    // Fraction of vorticity to exchange between particles.
            ASSERT( fabsf( exchangeFraction ) < 0.5f ) ; // If this triggers, Clamp above triggered.
            const Vec3  exchange        = exchangeFraction * vortDiff ;    // Amount of vorticity to exchange between particles.
            ASSERT( ! IsNan( rAngVelHere  ) && ! IsInf( rAngVelHere ) ) ;
            ASSERT( ! IsNan( rAngVelThere ) && ! IsInf( rAngVelThere ) ) ;
            rAngVelHere  -= exchange ;   // Make "here" vorticity a little closer to "there".
            rAngVelThere += exchange ;   // Make "there" vorticity a little closer to "here".
            ASSERT( ! IsNan( rAngVelHere  ) && ! IsInf( rAngVelHere ) ) ;
            ASSERT( ! IsNan( rAngVelThere ) && ! IsInf( rAngVelThere ) ) ;

        #if VORTON_SIM_GATHER_STATS
            mVorticityTermsStats.mViscousDiffusion.Accumulate( exchange.Magnitude() ) ; // Note: This is not thread safe.
        #endif

            return true ;  // Tell caller particles were kept.
        }
    }
    //return true ;  // Tell caller particles were kept.
}




/** Diffuse vorticity using a particle strength exchange (PSE) method, for a slice of the domain.

    \see DiffuseAndDissipateVorticityPSE
*/
void VortonSim::DiffuseAndDissipateVorticityPSESlice( const float & timeStep , UniformGrid< VECTOR< unsigned > > & ugVortonIndices , size_t izStart , size_t izEnd , PhaseE phase )
{
    if( ugVortonIndices.Empty() )
    {   // No vortons.
        return ;
    }

    ASSERT( mViscosity * timeStep <= 1.0f ) ; // ...otherwise dissipation will cause vorticity to flip direction.

    const float viscousVorticityDissipationFactor = mViscosity * timeStep ;

    // Exchange vorticity with nearest neighbors

    const size_t   & nx     = ugVortonIndices.GetNumPoints( 0 ) ;
    const size_t     nxm1   = nx - 1 ;
    const size_t   & ny     = ugVortonIndices.GetNumPoints( 1 ) ;
    const size_t     nym1   = ny - 1 ;
    const size_t     nxy    = nx * ny ;

    // Set up increment and offset for alternating odd-even access.
    const size_t    zInc    =  PHASE_BOTH == phase   ? 1 : 2 ;  ///< Amount by which to increment z index.
    const size_t    flipper = ( PHASE_ODD == phase ) ? 1 : 0 ;
    const size_t    izShift = ( izStart & 1 ) ^ flipper ; // Flip lowest bit on odd phase.

    DEBUG_ONLY( const size_t   & nz     = ugVortonIndices.GetNumPoints( 2 ) ) ;
    DEBUG_ONLY( const size_t     nzm1   = nz - 1 ) ;
    ASSERT( nz   > 0  ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( nzm1 < nz ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( izStart <  izEnd ) ;
    ASSERT( izEnd   <= nzm1  ) ;

    // Note that the cell-neighborhood loop visits only some of the neighboring
    // cells. That is because each particle visitation is symmetric so there is
    // no need to visit all neighboring cells.
    const size_t neighborCellOffsets[] =
    {   // Offsets to neighboring cells whose indices exceed this one:
           1            // + , 0 , 0 ( 1)
      //, -1 + nx       // - , + , 0 ( 2)
        ,    + nx       // 0 , + , 0 ( 3)
        ,  1 + nx       // + , + , 0 ( 4)
      //, -1 - nx + nxy // - , - , + ( 5)
      //,    - nx + nxy // 0 , - , + ( 6)
      //,  1 - nx + nxy // + , - , + ( 7)
      //, -1      + nxy // - , 0 , + ( 8)
        ,         + nxy // 0 , 0 , + ( 9)
        ,  1      + nxy // + , 0 , + (10)
      //, -1 + nx + nxy // - , 0 , + (11)
        ,      nx + nxy // 0 , + , + (12)
        ,  1 + nx + nxy // + , + , + (13)
    } ;
    static const size_t numNeighborCells = sizeof( neighborCellOffsets ) / sizeof( neighborCellOffsets[ 0 ] ) ;

    size_t idx[3] ;
    for( idx[2] = izStart + izShift ; idx[2] < izEnd ; idx[2] += zInc )
    {   // For all grid cells along z...
        const size_t offsetZ0 = idx[2] * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all grid cells along y...
            const size_t offsetY0Z0 = idx[1] * nx + offsetZ0 ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all grid cells along x...
                const size_t offsetX0Y0Z0 = idx[0]     + offsetY0Z0 ;
                const size_t numInCurrentCell = ugVortonIndices[ offsetX0Y0Z0 ].Size() ;
                for( unsigned ivHere = 0 ; ivHere < numInCurrentCell ; ++ ivHere )
                {   // For each vorton in this gridcell...
                    const unsigned &    rVortIdxHere    = ugVortonIndices[ offsetX0Y0Z0 ][ ivHere ] ;
                    Vorton &            rVortonHere     = (*mVortons)[ rVortIdxHere ] ;
                    // Note: This algorithm assumes calls to ExchangeVorticityOrMergeVortons never
                    // delete rVortonHere.
                    if( rVortonHere.IsAlive() )
                    {
                        Vec3 &              rAngVelHere     = rVortonHere.mAngularVelocity ;

                        // Diffuse vorticity with other vortons in same cell.
                        // Notice that the loop only visits vortons with indices
                        // following the "here" vorton.  That is because each
                        // visitation symmetrically modifies both vortons in the
                        // pair.  If "here" and "there" is visited, then "there" and
                        // "here" should not also be visited, because it would be
                        // redundant.
                        for( unsigned ivThere = ivHere + 1 ; ivThere < ugVortonIndices[ offsetX0Y0Z0 ].Size() ; /* Conditionally increment in body. */ )
                        {   // For each OTHER vorton within this same cell...
                            if( ExchangeVorticityOrMergeVortons( rVortIdxHere , rVortonHere , rAngVelHere , ivThere , ugVortonIndices[ offsetX0Y0Z0 ] , timeStep ) )
                            {   // Both particles remain.
                                ++ ivThere ;
                            }
                        }

                        // Diffuse vorticity with other vortons in adjacent cells.
                        for( size_t idxNeighborCell = 0 ; idxNeighborCell < numNeighborCells ; ++ idxNeighborCell )
                        {   // For each cell in neighborhood...
                            const size_t cellOffset = offsetX0Y0Z0 + neighborCellOffsets[ idxNeighborCell ] ; // offset of adjacent cell
                            VECTOR< unsigned > & cell = ugVortonIndices[ cellOffset ] ;
                            for( unsigned ivThere = 0 ; ivThere < cell.Size() ; /* Conditionally increment in body. */ )
                            {   // For each vorton in the visited cell...
                                if( ExchangeVorticityOrMergeVortons( rVortIdxHere , rVortonHere , rAngVelHere , ivThere , cell , timeStep ) )
                                {   // Both particles remain.
                                    ++ ivThere ;
                                }
                            }
                        }

                        // Dissipate vorticity.  See notes in header comment for DiffuseAndDissipateVorticityPSE,
                        // related to dissipation at Kolmogorov scales.  Technically, that should
                        // get converted into heat, but the amounts are usually negligible.
                        // This also simulates diffusing vorticity into regions of fluid that
                        // have no vortons.  A more accurate approach would involve creating vortons
                        // in those regions, so that vorticity could be tracked better, but that would
                        // dramatically slow the simulation, yielding results qualitatively similar to
                        // what this formula does.
                        rAngVelHere  -= viscousVorticityDissipationFactor * rAngVelHere ;   // Reduce vorticity here.
                    }
                }
            }
        }
    }
}




/** Diffuse vorticity using a particle strength exchange (PSE) method.

    Each vorton exchanges some of its vorticity with its neighbors in its own
    and adjacent cells.

    This routine makes some simplifying assumptions to speed execution:

        -   Only vortons within a certain region of each other exchange
            vorticity.

        -   This simulation reduces the vorticity of each vorton, alleging that
            this vorticity is dissipated analogously to how energy dissipates at
            Kolmogorov microscales.  This treatment is not realistic but it
            retains qualitative characteristics that we want, e.g. that the flow
            dissipates at a rate related to viscosity. Dissipation in real flows
            is a more complicated phenomenon.

    Theoretically, if an adjacent cell contains no vortons then this simulation
    should generate vorticity within that cell, e.g. by creating a new vorton in
    the adjacent cell.  But this diffusion implementation does not create new
    vortons.

    \see    Degond & Mas-Gallic (1989): The weighted particle method for
            convection-diffusion equations, part 1: the case of anisotropic viscosity.
            Math. Comput., v. 53, n. 188, pp. 485-507, October.

    \param timeStep     Amount of time by which to advance simulation.

    \param uFrame       Frame counter, used to generate diagnostic files.

    \param ugVortonIndices - UniformGrid of Vorton indices.  \see ParititionVortons.

    \see StretchAndTiltVortons, GenerateBaroclinicVorticity, DiffuseAndDissipateHeatPSE

*/
void VortonSim::DiffuseAndDissipateVorticityPSE( const float & timeStep , const unsigned & /* uFrame */ , UniformGrid< VECTOR< unsigned > > & ugVortonIndices )
{
#if ENABLE_FLUID_BODY_SIMULATION
    PERF_BLOCK( VortonSim__DiffuseAndDissipateVorticityPSE ) ;

#if defined( _DEBUG )
    const Vec3  totalVorticityBefore     = Vorton::ComputeTotalVorticity( * mVortons ) ; // TODO: Note that TallyDiagnosticIntegrals effectively gathers this info.  Consolidate?
    const float totalKineticEnergyBefore = Vorton::ComputeTotalKineticEnergy( * mVortons ) ;
#endif

#if VORTON_SIM_GATHER_STATS
    mVorticityTermsStats.Reset( mVorticityTermsStats.mViscousDiffusion ) ;
#endif

    // Exchange vorticity with nearest neighbors

    const unsigned & nz     = ugVortonIndices.GetNumPoints( 2 ) ;
    const unsigned   nzm1   = nz - 1 ;

#   if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , nzm1 / gNumberOfProcessors ) ;
        // Compute vorticity diffusion using threading building blocks.
        // Alternate between even and odd z-slices to avoid multiple threads accessing the same vortons simultaneously.
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , VortonSim_DiffuseVorticityPSE_TBB( timeStep , this , ugVortonIndices , VortonSim::PHASE_EVEN ) ) ;
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , VortonSim_DiffuseVorticityPSE_TBB( timeStep , this , ugVortonIndices , VortonSim::PHASE_ODD  ) ) ;
#   else
        DiffuseAndDissipateVorticityPSESlice( timeStep , ugVortonIndices , 0 , nzm1 , VortonSim::PHASE_BOTH ) ;
#   endif

#   if defined( _DEBUG )
    {   // In principle, diffusion should not change bulk vorticity or kinetic energy. But note that 
        static float totalVorticityChangeMagMax     = 0.0f ;
        static float totalVorticityChangeMagRelMax  = 0.0f ;
        static float totalKineticEnergyChangeMax    = 0.0f ;
        static float totalKineticEnergyChangeRelMax = 0.0f ;
        TrackVorticityAndKineticEnergyChange( * mVortons , totalVorticityBefore , totalKineticEnergyBefore , totalVorticityChangeMagMax , totalVorticityChangeMagRelMax , totalKineticEnergyChangeMax , totalKineticEnergyChangeRelMax ) ;
    }
#   endif

#if VORTON_SIM_GATHER_STATS
    mVorticityTermsStats.mViscousDiffusion.ConvertAccumulatedSamplesToStats( mVortons->Size() ) ;
#endif

#endif
}




/** Exchange heat between 2 vortons.

    \param rVortIdxHere Index within mVortons of vorton at "here" location.

    \param rVortonHere  Reference to mVortons[ rVortIdxHere ]

    \param rDensityHere Reference to mVortons[ rVortIdxHere ].mDensity

    \param ivThere      Index within mVortons of vorton at "there" location.

    \param cell         Reference to cell containing "here" vorton.

    \param timeStep     Amount of virtual time by which to advance simulation.

    This routine uses the particle strength exchange method to evolve temperature, T,
    according to the heat diffusion equation:

        DT/Dt = kappa laplacian T

    where D/Dt represents the convective derivative (where t is time),
    kappa represents thermal diffusivity of the fluid and "laplacian"
    is the laplacian differential operator (del^2, a.k.a. nabla^2).

    \note   Under the Boussinesq approximation,

                    density = ambientDensity * ( 1 - thermalExpansionCoefficient * ( temperature - ambientTemperature ) )
                --> temperature = density / ( ambientDensity * ( 1 - thermalExpansionCoefficient ) ) + ambientTemperature

            If the thermalExpansionCoefficient is constant with respect to density then
            density and temperature are linearly related (but with opposite sign).

            For an ideal gas, thermalExpansionCoefficient=1/temperature so the above formula becomes

                density = ambientDensity * ambientTemperature / temperature
                --> temperature = ambientTemperature * ambientDensity / density

            In this case, density and temperature are inversely proportional.

            Either way, as temperature increases, density decreases.
*/
inline void VortonSim::ExchangeHeat( const unsigned & /* rVortIdxHere */ , Vorton & rVortonHere , float & rDensityHere , const unsigned & ivThere , const VECTOR< unsigned > & cell , const float & timeStep )
{
    const unsigned &    rVortIdxThere   = cell[ ivThere ] ;
    Vorton &            rVortonThere    = (*mVortons)[ rVortIdxThere ] ;
    const float         diffusionRange  = 4.0f * rVortonHere.mSize ;
    const float         diffusionRange2 = POW2( diffusionRange ) ;
    ASSERT( rVortonThere.IsAlive() ) ;
    //if( rVortonThere.IsAlive() )
    {   // Particle is alive.
        const Vec3  separation          = rVortonHere.mPosition - rVortonThere.mPosition ;
        const float dist2               = separation.Mag2() ;
        const float distLaw             = exp( - dist2 / diffusionRange2 ) ;
        const float exchangeRatio       = Clamp( 2.0f * mThermalDiffusivity * timeStep * distLaw , -0.5f , 0.5f ) ; // Portion of heat to exchange between particles.
        float &     rDensityThere       = rVortonThere.mDensity ;
        const float densityDiff         = rDensityHere - rDensityThere ;
        const float exchange            = exchangeRatio * densityDiff ; // Amount of heat to exchange between particles.
        rDensityHere  -=   exchange ;   // Make "here"  temperature a little closer to "there".
        rDensityThere +=   exchange ;   // Make "there" temperature a little closer to "here".
        ASSERT( rVortonHere .GetMass() >= 0.0f ) ;
        ASSERT( rVortonThere.GetMass() >= 0.0f ) ;
    }
}




/** Diffuse heat using a particle strength exchange (PSE) method, for a slice of the domain

    \see DiffuseAndDissipateVorticityPSE, DiffuseAndDissipateHeatPSE, PartitionVortons
*/
void VortonSim::DiffuseAndDissipateHeatPSESlice( const float & timeStep , const UniformGrid< VECTOR< unsigned > > & ugVortonIndices , size_t izStart , size_t izEnd , PhaseE phase )
{
    if( ugVortonIndices.Empty() )
    {   // No vortons.
        return ;
    }

    ASSERT( mThermalDiffusivity * timeStep <= 1.0f ) ; // ...otherwise dissipation will cause temperature to flip sign.

    const float thermalDissipationGain = Clamp( mThermalDiffusivity * timeStep , -0.5f , 0.5f ) ;

    // Exchange heat with nearest neighbors

    const size_t   & nx     = ugVortonIndices.GetNumPoints( 0 ) ;
    const size_t     nxm1   = nx - 1 ;
    const size_t   & ny     = ugVortonIndices.GetNumPoints( 1 ) ;
    const size_t     nym1   = ny - 1 ;
    const size_t     nxy    = nx * ny ;

    // Set up increment and offset for alternating odd-even access.
    const size_t    zInc    =  PHASE_BOTH == phase   ? 1 : 2 ;  ///< Amount by which to increment z index.
    const size_t    flipper = ( PHASE_ODD == phase ) ? 1 : 0 ;
    const size_t    izShift = ( izStart & 1 ) ^ flipper ; // Flip lowest bit on odd phase.

    DEBUG_ONLY( const size_t   & nz     = ugVortonIndices.GetNumPoints( 2 ) ) ;
    DEBUG_ONLY( const size_t     nzm1   = nz - 1 ) ;
    ASSERT( nz   > 0  ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( nzm1 < nz ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( izStart <  izEnd ) ;
    ASSERT( izEnd   <= nzm1  ) ;

    // Note that the cell-neighborhood loop visits only some of the neighboring
    // cells. That is because each particle visitation is symmetric so there is
    // no need to visit all neighboring cells.
    const size_t neighborCellOffsets[] =
    {   // Offsets to neighboring cells whose indices exceed this one:
           1            // + , 0 , 0 ( 1)
      //, -1 + nx       // - , + , 0 ( 2)
        ,    + nx       // 0 , + , 0 ( 3)
      //,  1 + nx       // + , + , 0 ( 4)
      //, -1 - nx + nxy // - , - , + ( 5)
      //,    - nx + nxy // 0 , - , + ( 6)
      //,  1 - nx + nxy // + , - , + ( 7)
      //, -1      + nxy // - , 0 , + ( 8)
        ,         + nxy // 0 , 0 , + ( 9)
      //,  1      + nxy // + , 0 , + (10)
      //, -1 + nx + nxy // - , 0 , + (11)
      //,      nx + nxy // 0 , + , + (12)
      //,  1 + nx + nxy // + , + , + (13)
    } ;
    static const size_t numNeighborCells = sizeof( neighborCellOffsets ) / sizeof( neighborCellOffsets[ 0 ] ) ;

    size_t idx[3] ;
    for( idx[2] = izStart + izShift ; idx[2] < izEnd ; idx[2] += zInc )
    {   // For all grid cells along z...
        const size_t offsetZ0 = idx[2] * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all grid cells along y...
            const size_t offsetY0Z0 = idx[1] * nx + offsetZ0 ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all grid cells along x...
                const size_t offsetX0Y0Z0 = idx[0]     + offsetY0Z0 ;
                const size_t numInCurrentCell = ugVortonIndices[ offsetX0Y0Z0 ].Size() ;
                for( unsigned ivHere = 0 ; ivHere < numInCurrentCell ; ++ ivHere )
                {   // For each vorton in this gridcell...
                    const unsigned &    rVortIdxHere    = ugVortonIndices[ offsetX0Y0Z0 ][ ivHere ] ;
                    Vorton &            rVortonHere     = (*mVortons)[ rVortIdxHere ] ;
                    ASSERT( rVortonHere.IsAlive() ) ;
                    float &             rDensityHere    = rVortonHere.mDensity ;
                    ASSERT( rVortonHere.mDensity > 0.0f ) ;

                    // Diffuse heat with other vortons in this same cell.
                    // Notice that the loop only visits vortons with indices
                    // following the "here" vorton.  That is because each
                    // visitation symmetrically modifies both vortons in the
                    // pair.  If "here" and "there" is visited, then "there" and
                    // "here" should not also be visited, because it would be
                    // redundant.
                    for( unsigned ivThere = ivHere + 1 ; ivThere < ugVortonIndices[ offsetX0Y0Z0 ].Size() ; ++ ivThere )
                    {   // For each OTHER vorton within this same cell...
                        ExchangeHeat( rVortIdxHere , rVortonHere , rDensityHere , ivThere , ugVortonIndices[ offsetX0Y0Z0 ] , timeStep ) ;
                    }

                    // Diffuse heat with other vortons in adjacent cells.
                    for( size_t idxNeighborCell = 0 ; idxNeighborCell < numNeighborCells ; ++ idxNeighborCell )
                    {   // For each cell in neighborhood...
                        const size_t cellOffset = offsetX0Y0Z0 + neighborCellOffsets[ idxNeighborCell ] ; // offset of adjacent cell
                        const VECTOR< unsigned > & cell = ugVortonIndices[ cellOffset ] ;
                        for( unsigned ivThere = 0 ; ivThere < cell.Size() ; ++ ivThere )
                        {   // For each vorton in the visited cell...
                            ExchangeHeat( rVortIdxHere , rVortonHere , rDensityHere , ivThere , cell , timeStep ) ;
                        }
                    }

                    // "Dissipate" heat.
                    // This roughly simulates diffusing heat into regions of fluid that have no explicit particles.
                    // The proper solution to this would involve either radiating heat into and out of the environment,
                    // and/or adding fluid particles to regions that lack them, but that would consume computational resources,
                    // and yield qualitatively very similar results.  Remember, this is for a video game eye candy, not
                    // science and engineering.
                    {
                        const float densityDelta = mAmbientDensity - rDensityHere ; // density departure from ambient
                        rDensityHere  += thermalDissipationGain * densityDelta ;   // Reduce temperature departure from ambient, of this vorton.
                        ASSERT( rVortonHere.GetMass() >= 0.0f ) ;
                    }
                }
            }
        }
    }
}




/** Diffuse heat using a particle strength exchange (PSE) method.

    \see DiffuseAndDissipateVorticityPSE

*/
void VortonSim::DiffuseAndDissipateHeatPSE( const float & timeStep , const unsigned & /* uFrame */ , const UniformGrid< VECTOR< unsigned > > & ugVortonIndices )
{
#if ENABLE_FLUID_BODY_SIMULATION

    PERF_BLOCK( VortonSim__DiffuseAndDissipateHeatPSE ) ;

    // Exchange heat with nearest neighbors

    const unsigned & nz     = ugVortonIndices.GetNumPoints( 2 ) ;
    const unsigned   nzm1   = nz - 1 ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , nzm1 / gNumberOfProcessors ) ;
        // Compute heat diffusion using threading building blocks
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , VortonSim_DiffuseHeatPSE_TBB( timeStep , this , ugVortonIndices , VortonSim::PHASE_EVEN ) ) ;
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , VortonSim_DiffuseHeatPSE_TBB( timeStep , this , ugVortonIndices , VortonSim::PHASE_ODD  ) ) ;
    #else
        DiffuseAndDissipateHeatPSESlice( timeStep , ugVortonIndices , 0 , nzm1 , VortonSim::PHASE_BOTH ) ;
    #endif
#endif
}




/** Update vortex particle fluid simulation to next time.

    \param timeStep     Amount of virtual time by which to advance simulation.

    \param uFrame       Frame counter, used to generate diagnostic files.

    \note FindBoundingBox and UpdateBoundingBox must have been called
            before this routine starts, for each timestep.

    Effectively this routine generates the velocity field and prepares the
    simulation for the next step.  It does NOT, however, actually advect
    vortons.  The Evolve particle operation does that, and it is up to the
    caller of this routine to invoke that operation.  This separation of
    processes facilitates adding other motion to the vortons which are not due
    to the fluid simulation.  It also facilitates using the same velocity field
    to advect passive tracer particles.

*/
void VortonSim::UpdateVortexParticleMethod( float timeStep , unsigned uFrame )
{
    PERF_BLOCK( VortonSim__UpdateVortexParticleMethod ) ;

    ASSERT( ( FLUID_SIM_VORTEX_PARTICLE_METHOD == mFluidSimTechnique ) || ( FLUID_SIM_VPM_SPH_HYBRID == mFluidSimTechnique ) ) ;

    if( mVortons->Empty() )
    {
        return ;
    }

    if( mTallyDiagnosticIntegrals )
    {
        // Advection happens after Update, so store off last stage ("after heat") from the previous frame.
        mDiagnosticIntegrals.mBefore = mDiagnosticIntegrals.mAfterHeat ;
        ConditionallyTallyDiagnosticIntegrals( mDiagnosticIntegrals.mAfterAdvect ) ;
    #if ENABLE_PARTICLE_JERK_RECORD
        VECTOR< Particle > &    vortonsAsParticles  = reinterpret_cast< VECTOR< Particle > & >( * mVortons ) ;
        Particles::UpdateJerk( vortonsAsParticles , timeStep ) ;
    #endif
    }
    else
    {   // Debug builds must always calculate mDiagnosticIntegrals.mAfterAdvect for use in asserts elsewhere, even when mTallyDiagnosticIntegrals is disabled.
    #if defined( _DEBUG )
        TallyDiagnosticIntegrals( mDiagnosticIntegrals.mAfterAdvect.mTotalCirculation , mDiagnosticIntegrals.mAfterAdvect.mLinearImpulseFromVorticity , mDiagnosticIntegrals.mAfterAdvect.mLinearImpulseFromVelocity , mDiagnosticIntegrals.mAfterAdvect.mAngularImpulse ) ;
    #endif
    }

#if USE_PARTICLE_IN_CELL
    //#if ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL )
    //    #error USE_PARTICLE_IN_CELL cannot work with VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL.
    //#endif
    mNegativeVorticityMultiGrid.Initialize( mGridTemplate ) ;
    PopulateVorticityGridFromVortons( mNegativeVorticityMultiGrid[ 0 ] , 1.0f ) ; // Note that in this case, mNegativeVorticityMultiGrid actually contains positive vorticity, not negative.
    VECTOR< Vorton > originalVortons ;  // Place for original vortons to live while the temporary ones take their place temporarily.
    mVortons->swap( originalVortons ) ;
    AssignVortonsFromVorticity( mNegativeVorticityMultiGrid[ 0 ] ) ;
#endif

    ConditionallyTallyDiagnosticIntegrals( mDiagnosticIntegrals.mAfterRegrid ) ;

    NestedGrid< Vorton > influenceTree ;

//#error TODO: Use mVelFromVortTechnique to change VELOCITY_TECHNIQUE to a runtime decision.

#if ( ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) || ( VECTOR_POTENTIAL_TECHNIQUE == VECTOR_POTENTIAL_TECHNIQUE_TREE ) )
    CreateInfluenceTree( influenceTree ) ;
#endif

#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL
    mNegativeVorticityMultiGrid.Initialize( mGridTemplate ) ;
    PopulateVorticityGridFromVortons( mNegativeVorticityMultiGrid[ 0 ] , -1.0f ) ;
#endif

#if ( ( VELOCITY_TECHNIQUE != VELOCITY_TECHNIQUE_TREE ) && ( VELOCITY_TECHNIQUE != VELOCITY_TECHNIQUE_MONOPOLES ) && ( VELOCITY_TECHNIQUE != VELOCITY_TECHNIQUE_DIRECT ) && ( VELOCITY_TECHNIQUE != VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL ) )
#   error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
#endif

#if ! USE_PARTICLE_IN_CELL
    // When NOT using PIC, vortons can be partitioned any time before
    // they advect (which happens outside this Update), and one
    // velocity-from-vorticity technique (namely USE_ORIGINAL_VORTONS_IN_BASE_LAYER)
    // uses the vorton partition.
    // When using PIC, ComputeVelocityFromVorticity uses the ghost vortons
    // generated in the call to AssignVortonsFromVorticity (above),
    // so partitioning those vortons here would be useless for PSE below.
    PartitionVortons( timeStep , uFrame , mVortonIndicesGrid ) ;
#endif

    // Use vorticity to compute velocity.
    ComputeVelocityFromVorticity( mVortonIndicesGrid , influenceTree , mNegativeVorticityMultiGrid ) ;

    #if defined( _DEBUG )
    if( mOutputDiagnostics )
    {
        mVelGrid.GenerateBrickOfBytes( "vel" , uFrame ) ;
    }
    #endif

#if USE_PARTICLE_IN_CELL
    // Restore original vortons after computing velocity grid, but before any other operations on vortons.
    mVortons->swap( originalVortons ) ;

    // Vorton partition must be for the original vortons, not for PIC ghost vortons.
    // See other comments above.
    PartitionVortons( timeStep , uFrame , mVortonIndicesGrid ) ;
#endif

    ConditionallyTallyDiagnosticIntegrals( mDiagnosticIntegrals.mAfterVelGrid ) ;

    mVorticityTermsStats.Reset() ;

    if( ( FLUID_SIM_VORTEX_PARTICLE_METHOD == mFluidSimTechnique )      // Disable stretching for VPM-SPH hybrid, because it causes numerical instability.
        &&  (   ( INVESTIGATE_ALL                == mInvestigationTerm )
            ||  ( INVESTIGATE_STRETCHING_TILTING == mInvestigationTerm )
            )
        )
    {
        StretchAndTiltVortons( timeStep , uFrame ) ;
    }

    ConditionallyTallyDiagnosticIntegrals( mDiagnosticIntegrals.mAfterStretch ) ;

    if( ( INVESTIGATE_ALL == mInvestigationTerm ) || ( INVESTIGATE_BAROCLINIC == mInvestigationTerm ) )
    {
        GenerateBaroclinicVorticity( timeStep , uFrame
    #if COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS || COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH
                                    , mVortonIndicesGrid
    #endif
                                    ) ;
    }

    ConditionallyTallyDiagnosticIntegrals( mDiagnosticIntegrals.mAfterBaroclinic ) ;

    if( ( INVESTIGATE_ALL == mInvestigationTerm ) || ( INVESTIGATE_THERMAL_DIFFUSION == mInvestigationTerm ) )
    {
        DiffuseAndDissipateHeatPSE( timeStep , uFrame , mVortonIndicesGrid ) ;
    }

    ConditionallyTallyDiagnosticIntegrals( mDiagnosticIntegrals.mAfterHeat ) ;

    if( ( INVESTIGATE_ALL == mInvestigationTerm ) || ( INVESTIGATE_VISCOUS_DIFFUSION == mInvestigationTerm ) )
    {
        DiffuseAndDissipateVorticityPSE( timeStep , uFrame , mVortonIndicesGrid ) ;
    }

    ConditionallyTallyDiagnosticIntegrals( mDiagnosticIntegrals.mAfterDiffuse ) ;

#if 0
    // Kill particles merged during DiffuseAndDissipateVorticityPSE.
    // NOTE: TODO: FIXME: Perhaps this should move into block with DiffuseAndDissipateVorticityPSE.
    Particles::KillParticlesMarkedForDeath( reinterpret_cast< VECTOR< Particle > & >( * mVortons ) ) ;
#endif

#if ! COMPUTE_VELOCITY_AT_VORTONS
    // Update vorton velocity from field.
    extern void AssignVelocityFromField( VECTOR< Particle > & particles , const UniformGrid< Vec3 > * velocityGrid , const float gain ) ;
    AssignVelocityFromField( reinterpret_cast< VECTOR< Particle > & >( * mVortons ) , & mVelGrid , 1.0f ) ;
#endif
}




/** Update particle fluid simulation to next time.

    \param timeStep     Amount of virtual time by which to advance simulation.

    \param uFrame       Frame counter, used to generate diagnostic files.

    \note FindBoundingBox and UpdateBoundingBox must have been called
            before this routine starts, for each timestep.

    Effectively this routine generates the velocity field and prepares the
    simulation for the next step.  It does NOT, however, actually advect
    vortons.  The Evolve particle operation does that, and it is up to the
    caller of this routine to invoke that operation.  This separation of
    processes facilitates adding other motion to the vortons which are not due
    to the fluid simulation.  It also facilitates using the same velocity field
    to advect passive tracer particles.

*/
void VortonSim::Update( float timeStep , unsigned uFrame )
{
    PERF_BLOCK( VortonSim__Update ) ;

#if USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
    if( FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS == mFluidSimTechnique )
    {   // Use smoothed particle hydrodynamics.
        UpdateSmoothedParticleHydrodynamics( timeStep , uFrame ) ;

        if( mPopulateSdfFromDensity )
        {
            // Populate density & mass fraction grids from vortons, for diagnostic and surface rendering.
            // Note that SPH uses number density, not mass density, so it does not use the results of this operation.
            // In that sense, it is superfluous or optional, but it does facilitate some data visualization.
            PopulateDensityAndMassFractionGrids( mDensityGrid , reinterpret_cast< VECTOR< Particle > & >( * mVortons ) , 0 ) ;

            // Populate SDF from density grid, for visualization.  This is in contrast to tracking SDF from tracer particles.
            PopulateSignedDistanceGridFromDensityGrid( mSignedDistanceGrid , mDensityGrid , 0 ) ;
        }

    }
    else if( FLUID_SIM_VPM_SPH_HYBRID == mFluidSimTechnique )
    {   // Use vortex particle method.
        UpdateVortexParticleMethod( timeStep , uFrame ) ;
//#error READ ALL COMMENTS before proceeding.
//#error This call to UpdateSmoothedParticleHydrodynamics doesn't do what I want, but it also does more than I meant.
        // I just want to see what happens if we compute grav accel this way (linearly) instead of baroclinic.
        // BTW baro is (was) disabled.
        //
        // UPDATE: Solved many of the issues associated with the initial comment.
        // Also, Baro is enabled again.
//#error TODO: Comment out everything in UpdateSmoothedParticleHydrodynamics except ApplyBodyForce, and make that routine not use accelerations (or make accelerations all zero).
        // See what that does.  Problem is, it will let particles pile up.  So we'll need to "reduce divergence".
        // SPH pressure keeps vortons from collapsing (desirable).  It also keeps them together (undesirable unless we want surface tension).
        // I meant for UpdateVortexParticleMethod to assign vel from vort, and UPdateSph to reduce divergence and accel grav.
        //
        // UPDATE: mixing VPM and SPH works better now that vorton velocity is set from field.
        //  But they still have a weird interaction -- motion seems too slow.  Might be because simulation runs slower due to having 2 updates.
        //  But also just seems slower.
        // (NOTE: See the UPDATE below regarding the problem of particle-grid velocity transfer.  It might explain the slowness problem.)
//#error Still have a problem: SPH pressure pushes unbalanced at corners which leads to velocity there.
        // Then boundary solver causes a coherent vorticity in the corners.  This is totally spurious:
        // Velocity there should never reach non-zero.
        // Proper solution would be to treat velocity only after boundary conditions have been applied,
        // i.e. apply wall-bounce impulses to get relaxed velocity, then use that to compute Brinkman vorticity flux.
        // Another option: Don't apply SPH pressure during VPM -- just use REDUCE_CONVERGENCE (which adjusts positions directly, not velocity).
        // Ideally the former would also be solved; they're not mutually exclusive.  Try both!
        //
        // UPDATE: Moving impulse assignment to above Brinkman vorticity assignment had the desired effect.
//#error TODO: Need to try the selective Baro/linear idea I described in Evernote.
        //
        // NOTE: Mixing VPM with SPH in a single frame creates a weird problem concerning the velocity field:
        // For VPM it makes sense to solve velocity on the grid, then transfer velocity from grid to vortons.
        // For SPH it makes sense to sovle velocity on the "vortons", then transfer velocity from vortons to grid.
        // That particle-grid transfer happens outside this routine, in the particle group operations list, which is problematic.
        // The solution should entail either moving that transfer into UpdateVortexParticleMethod & UpdateSmoothedParticleHydrodynamics, or alternating VPM and SPH phases,
        // meanwhile also alternating the particle-grid transfer in the particle group operation list (which would be preposterously complicated).
        UpdateSmoothedParticleHydrodynamics( timeStep , uFrame ) ;
    }
    else
#endif
    {   // Use vortex particle method.
        ASSERT( FLUID_SIM_VORTEX_PARTICLE_METHOD == mFluidSimTechnique ) ;
        UpdateVortexParticleMethod( timeStep , uFrame ) ;
    }
}




/** Reset the simulation
*/
void VortonSim::Clear()
{
    PERF_BLOCK( VortonSim__Clear ) ;

    mVortons->Clear() ;
    mNegativeVorticityMultiGrid.Clear() ;
    mVectorPotentialMultiGrid.Clear() ;
    mVelGrid.Clear() ;
    mDensityGrid.Clear() ;
    mDensityGradientGrid.Clear() ;

#if ENABLE_FIRE
    mFuelFractionGrid.Clear() ;
    mFlameFractionGrid.Clear() ;
    mSmokeFractionGrid.Clear() ;
#endif
}




#if defined( _DEBUG )
/** Output the velocity profile of the vorton model used by this simulation.

    Use this to input into a graphing program to visualize the velocity-versus-radius function.
*/
void OutputVelocityProfile( float spreadingRangeFactor )
{
    PERF_BLOCK( OutputVelocityProfile ) ;

    const float spreadingCirculationFactor = 1.0f / Pow3( spreadingRangeFactor ) ;

    char velocityProfileFilename[ 256 ] ;
    sprintf( velocityProfileFilename , "TestData/velocityProfile-%.1f.dat" , spreadingRangeFactor ) ;

    FILE * velProfFile = fopen( velocityProfileFilename , "w" ) ;
    if( velProfFile )
    {
        static const Vec3       vortonPosition( 0.0f , 0.0f , 0.0f ) ;
        static const Vec3       vortonAngVel( 0.0f , 0.0f , 1.0f ) ;
        static const float      vortonSize      = 0.2f ;
        static const float      vortonRadius    = vortonSize * 0.5f ;
        static const unsigned   numSamples      = 2000 ;
        static const float      range           = vortonSize * 5.0f ;
        static const float      sampleSpacing   = range / float( numSamples ) ;
        for( unsigned iSample = 0 ; iSample < numSamples ; ++ iSample )
        {   // For each vorton...
            const float distanceFromVortonCenter = sampleSpacing * float( iSample ) ;
            Vec3 vPosQuery( distanceFromVortonCenter , 0.0f , 0.0f ) ;
            Vec3    velocityAccumulator( 0.0f , 0.0f , 0.0f ) ;
            VORTON_ACCUMULATE_VELOCITY_private( velocityAccumulator , vPosQuery , vortonPosition , vortonAngVel , vortonSize , spreadingRangeFactor , spreadingCirculationFactor ) ;
            const float distanceInVortonRadii = distanceFromVortonCenter / vortonRadius ;
            fprintf( velProfFile , "%g %g\n" , distanceInVortonRadii , velocityAccumulator.y ) ;
        }
        fclose( velProfFile ) ;
    }
}
#endif
