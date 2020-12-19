/** \file smoothedPclHydro.cpp

    \brief Dynamic simulation of a fluid, using smoothed particle hydrodynamics.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-17/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <limits>
#include <stdlib.h>
#include <algorithm>

#include "Core/Math/Vec2.h"

#include "Core/useTbb.h"

#include "Core/Performance/perfBlock.h"

#include "Core/SpatialPartition/uniformGridMath.h"

#include "Particles/Operation/pclOpFindBoundingBox.h"

#include "VortonFluid/vortonSim.h"

#include "FluidBodySim/fluidBodySim.h"  // Included for experimental poisoning feature.


// Whether to use the normalized form of SPH smoothing kernel when computing number density.
#define USE_NORMALIZED_SPH_FORM_FOR_NUMBER_DENSITY 0


/** SPH fluid parameters.

    Optimal values are are sensitive to timestep, gravity, fluid density
    and to each other.

    Any change in any of these values might require changes in the others, so
    after changing any, tune all of them.  Change each incrementally, for
    example, do not change any by more than half or double, in any given pass.

    - Tune target number density.
        # Use a spherical ball (case 26) of particles suspended away from boundaries.
        # Disable gravity or any other force that would cause the ball to move.
        # Increase targetNumberDensity if the ball expands.
        # Decrease targetNumberDensity if the ball shrinks.
    - Tune stiffness.
        # Use a vertical column of heavy fluid that nearly fills the container (case 39).
        # Try increasing stiffness if the fluid compresses.  Ideally it should
            not compress even while falling on itself but that might be
            computationally impractical. It should also not compress after it
            falls and settles, which is a weaker requirement.  It should occupy
            the same volume even if you increase stiffness further.
        # Try decreasing stiffness if the fluid sticks to itself, but also
          try increasing nearToFar.
        # Try decreasing stiffness if the fluid becomes unstable.  If you cannot
          find a value where the fluid does not compress, and is stable, then
          the timestep is too large.  If you can afford to decrease the timestep,
          do so and start over.  Otherwise choose the largest stiffness that
          yields a stable simulation.  Compressibility is better than instability.
    - Tune nearToFar.
        # Increase nearToFar if the fluid sticks to itself.
        # Try increasing nearToFar if the fluid compressess, but also try increasing stiffness.
        # Try decreasing nearToFar if the simulation is unstable.  But also try decreasing stiffness.
        # Try decreasing nearToFar if increasing its value has no visible effect.
        # nearToFar should be above 1.0 because otherwise the "far" curve might have no influence.
    - Tune influenceRadiusScale.
        # influenceRadiusScale should be at least 2, assuming you do not want
            particle to overlap much.
        # Increase influenceRadiusScale if tuning the other parameters does not
            yield a lack of compression.
        # Try decreasing influenceRadiusScale if the fluid sticks to itself or
            seems to have too much global coherence.  If the fluid pulls itself
            together from too far away, for example.
        # Use the smallest value (>=2) of influenceRadiusScale that achieves a
            lack of compression given the best tunings of other parameters.

*/
#if 0 // Use with timeStep=1/30
    static float targetNumberDensity    =  4.0f ;
    static float stiffness              =  5.0f ;
    static float nearToFar              =  2.0f ;
           float influenceRadiusScale   =  4.0f ;
#elif 0 // Use with timeStep=1/30.
    static float targetNumberDensity    =  2.9f ;
    static float stiffness              =  5.0f ;
    static float nearToFar              =  2.0f ;
           float influenceRadiusScale   =  3.5f ;
#elif 1 // Use with timeStep=1/30.
    static float targetNumberDensity    =  2.2f ;
    static float stiffness              =  5.0f ;
    static float nearToFar              =  2.0f ;
           float influenceRadiusScale   =  3.0f ;
#elif 1 // Use with timeStep=1/30.  Note that this never settles down.  influenceRadiusScale=2.5 seems to be too small to achieve steady state.
    static float targetNumberDensity    =  1.75f ;
    static float stiffness              =  5.0f ;
    static float nearToFar              =  2.0f ;
           float influenceRadiusScale   =  2.5f ;
#elif 1 // Use with timeStep=1/30.  Note that this never settles down.  influenceRadiusScale=2 seems to be too small to achieve steady state.
    static float targetNumberDensity    =  1.23f ;
    static float stiffness              =  5.0f ;
    static float nearToFar              =  2.0f ;
           float influenceRadiusScale   =  2.0f ;
#else
    static float targetNumberDensity    =  2.2f ;
    static float stiffness              = 20.0f ;
    static float nearToFar              =  2.0f ;
           float influenceRadiusScale   =  3.0f ;
#endif




static const float sSphereToCubeVolumeRatio     = PI / 6.0f ; // 4*pi*r^3/3 where r=0.5
static const float sSqrt18                      = sqrtf( 18.0f ) ;
static const float sClosestPackedSphereDensity  = PI / sSqrt18 ;    // Average density of closest-packed spheres.

static const float sHardCoreRadius               = 0.001f ;
static const float sHardCoreRadius2              = Pow2( sHardCoreRadius ) ;
static const Vec3  sTinyJiggle( sHardCoreRadius , sHardCoreRadius , sHardCoreRadius ) ;

#if USE_TBB && USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH

    static void ComputeSphDensityAtParticles_Grid_Slice( VECTOR< SphFluidDensities > & fluidDensitiesAtPcls , const VECTOR< Vorton > & particles , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid , size_t izStart , size_t izEnd , VortonSim::PhaseE phase ) ;

    /** Function object to compute particle number density using Threading Building Blocks.
    */
    class SphSim_ComputeSphNumberDensityAtParticles_TBB
    {
            VECTOR< SphFluidDensities > &               mFluidDensitiesAtPcls   ; ///< Array of particle density.  Elements map one-to-one with mParticles.
            const VECTOR< Vorton > &                    mParticles              ; ///< Array of particles whose number density to accumulate.
            const UniformGrid< VECTOR< unsigned > > &   mPclIndicesGrid         ; ///< Reference to uniform grid of particle indices
            VortonSim::PhaseE                           mPhase                  ; ///< Processing phase: whether to run on odd, even or both values for z slice.

        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute number density for subset of domain.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                ComputeSphDensityAtParticles_Grid_Slice( mFluidDensitiesAtPcls , mParticles , mPclIndicesGrid , r.begin() , r.end() , mPhase ) ;
            }

            SphSim_ComputeSphNumberDensityAtParticles_TBB(
                  VECTOR< SphFluidDensities > &             fluidDensitiesAtPcls
                , const VECTOR< Vorton > &                  particles
                , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                , VortonSim::PhaseE                         phase
                )
                : mFluidDensitiesAtPcls( fluidDensitiesAtPcls )
                , mParticles( particles )
                , mPclIndicesGrid( pclIndicesGrid )
                , mPhase( phase )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;




    static void ComputeSphPressureGradientAcceleration_Grid_Slice( VECTOR< Vec3 > & accelerations
        , const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
        , const VECTOR< Vorton > & particles
        , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
        , size_t izStart
        , size_t izEnd
        , VortonSim::PhaseE phase ) ;

    /** Function object to compute pressure gradient acceleration using Threading Building Blocks.
    */
    class SphSim_ComputeSphPressureGradientAcceleration_TBB
    {
            VECTOR< Vec3 > &                            mAccelerations          ; ///< Array of particle accelerations.  Elements map one-to-one with mParticles.
            const VECTOR< SphFluidDensities > &         mFluidDensitiesAtPcls   ; ///< Array of particle number density.  Elements map one-to-one with mParticles.
            const VECTOR< Vorton > &                    mParticles              ; ///< Array of particles whose accelerations to calculate.
            const UniformGrid< VECTOR< unsigned > > &   mPclIndicesGrid         ; ///< Reference to uniform grid of particle indices
            VortonSim::PhaseE                           mPhase                  ; ///< Processing phase: whether to run on odd, even or both values for z slice.

        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute particle acceleration due to pressure gradients for subset of domain.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                ComputeSphPressureGradientAcceleration_Grid_Slice( mAccelerations , mFluidDensitiesAtPcls , mParticles , mPclIndicesGrid , r.begin() , r.end() , mPhase ) ;
            }

            SphSim_ComputeSphPressureGradientAcceleration_TBB(
                  VECTOR< Vec3 > &                          accelerations
                , const VECTOR< SphFluidDensities > &       fluidDensitiesAtPcls
                , const VECTOR< Vorton > &                  particles
                , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                , VortonSim::PhaseE                         phase
                )
                : mAccelerations( accelerations )
                , mFluidDensitiesAtPcls( fluidDensitiesAtPcls )
                , mParticles( particles )
                , mPclIndicesGrid( pclIndicesGrid )
                , mPhase( phase )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;





    static void ComputeSphMassDensityGradient_Grid_Slice( VECTOR< Vec3 > & massDensityGradients
        , const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
        , const VECTOR< Vorton > & particles
        , const VECTOR< float > & proximities
        , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
        , const float ambientDensity
        , size_t izStart
        , size_t izEnd
        , VortonSim::PhaseE phase ) ;

    /** Function object to compute mass density gradient using Threading Building Blocks.
    */
    class SphSim_ComputeSphMassDensityGradient_TBB
    {
            VECTOR< Vec3 > &                            mMassDensityGradients   ; ///< Array of mass density gradients.  Elements map one-to-one with mParticles.
            const VECTOR< SphFluidDensities > &         mFluidDensitiesAtPcls   ; ///< Array of particle densities.  Elements map one-to-one with mParticles.
            const VECTOR< Vorton > &                    mParticles              ; ///< Array of particles whose accelerations to calculate.
            const VECTOR< float > &                     mProximities            ; ///< Array of particle-to-wall partially truncated signed distances.
            const UniformGrid< VECTOR< unsigned > > &   mPclIndicesGrid         ; ///< Reference to uniform grid of particle indices
            const float                                 mAmbientDensity         ; ///< Density of fluid in the absence of fluid particles
            VortonSim::PhaseE                           mPhase                  ; ///< Processing phase: whether to run on odd, even or both values for z slice.

        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute particle acceleration due to pressure gradients for subset of domain.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                ComputeSphMassDensityGradient_Grid_Slice( mMassDensityGradients , mFluidDensitiesAtPcls , mParticles , mProximities , mPclIndicesGrid , mAmbientDensity , r.begin() , r.end() , mPhase ) ;
            }

            SphSim_ComputeSphMassDensityGradient_TBB(
                  VECTOR< Vec3 > &                          massDensityGradients
                , const VECTOR< SphFluidDensities > &       fluidDensitiesAtPcls
                , const VECTOR< Vorton > &                  particles
                , const VECTOR< float > &                   proximities
                , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                , const float                               ambientDensity
                , VortonSim::PhaseE                         phase
                )
                : mMassDensityGradients( massDensityGradients )
                , mFluidDensitiesAtPcls( fluidDensitiesAtPcls )
                , mParticles( particles )
                , mProximities( proximities )
                , mPclIndicesGrid( pclIndicesGrid )
                , mAmbientDensity( ambientDensity )
                , mPhase( phase )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;





#if USE_SMOOTHED_PARTICLE_HYDRODYNAMICS

    static void DiffuseAndDissipateVelocitySph_Grid_Slice( VECTOR< Vorton > & particles , const float timeStep , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid , size_t izStart , size_t izEnd , VortonSim::PhaseE phase ) ;

    /** Function object to diffuse and dissipate particle velocity using Threading Building Blocks.
    */
    class SphSim_DiffuseAndDissipateVelocity_TBB
    {
            VECTOR< Vorton > &                          mParticles      ; ///< Array of particles whose velocity to diffuse and dissipate.
            const float                                 mTimeStep       ; ///< Amount of time by which to advance simulation.
            const UniformGrid< VECTOR< unsigned > > &   mPclIndicesGrid ; ///< Reference to uniform grid of particle indices.
            VortonSim::PhaseE                           mPhase          ; ///< Processing phase: whether to run on odd, even or both values for z slice.

        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute particle velocity diffusion and dissipation for subset of domain.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                DiffuseAndDissipateVelocitySph_Grid_Slice( mParticles , mTimeStep , mPclIndicesGrid , r.begin() , r.end() , mPhase ) ;
            }

            SphSim_DiffuseAndDissipateVelocity_TBB(
                  VECTOR< Vorton > &                        particles
                , const float                               timeStep
                , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                , VortonSim::PhaseE                         phase
                )
                : mParticles( particles )
                , mTimeStep( timeStep )
                , mPclIndicesGrid( pclIndicesGrid )
                , mPhase( phase )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;

#endif // USE_SMOOTHED_PARTICLE_HYDRODYNAMICS

#endif // USE_TBB && USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH




/** Functor to accumulate density between two smoothed particles.
*/
class DensityAccumulator
{
    public:
        /** Construct a functor to accumulate density between two smoothed particles.

            \param fluidDensitiesAtPcls (out) Fluid particle densities.

            \param particles                Fluid particles whose density to accumulate.

            \param influenceRadius          Cutoff range of smoothing function.
                                            Particles inside this range contribute
                                            to each others' densities.
        */
        DensityAccumulator( 
                    VECTOR< SphFluidDensities > &   fluidDensitiesAtPcls
                ,   const VECTOR< Vorton > &        particles
                ,   const float                     influenceRadius
            )
            : mPclDensities( fluidDensitiesAtPcls )
            , mParticles( particles )
            , mInfluenceRadius( influenceRadius )
            , mInflRad2( influenceRadius * influenceRadius )
            , mNormFactor( 15.0f / ( PI * mInflRad2 * mInfluenceRadius ) )
        {
            ASSERT( mPclDensities.Size() == mParticles.Size() ) ;
        }

        void operator()( size_t idxA , size_t idxB )
        {
            const Vec3  sep     = mParticles[ idxA ].mPosition - mParticles[ idxB ].mPosition ;
            const float dist2   = sep.Mag2() ;
            if( dist2 < mInflRad2 )
            {   // Particles are close enough to contribute density to each other.
                const float dist = fsqrtf( dist2 ) ;
                const float q    = 1.0f - dist / mInfluenceRadius ;
                const float q2   = Pow2( q ) ;
                const float q3   = q2 * q  ;
                const float q4   = q2 * q2 ;
                ASSERT( ( 0.0f <= q ) && ( q <= 1.0f ) ) ;
#if USE_NORMALIZED_SPH_FORM_FOR_NUMBER_DENSITY
                mPclDensities[ idxA ].mNumberDensity      += q3 * mNormFactor * mParticles[ idxB ].GetVolume() ;
                mPclDensities[ idxA ].mNearNumberDensity  += q4 * mNormFactor * mParticles[ idxB ].GetVolume() ;
                mPclDensities[ idxA ].mMassDensity        += q3 * mNormFactor * mParticles[ idxB ].GetMass() ;

                mPclDensities[ idxB ].mNumberDensity      += q3 * mNormFactor * mParticles[ idxA ].GetVolume() ;
                mPclDensities[ idxB ].mNearNumberDensity  += q4 * mNormFactor * mParticles[ idxB ].GetVolume() ;
                mPclDensities[ idxB ].mMassDensity        += q3 * mNormFactor * mParticles[ idxA ].GetMass() ;
#else           // Non-normalized SPH kernel for number density
                // NOTE: mNumberDensity is not the actual number density, since it is not normalized.
                //      To compute actual number density, it would need to be multiplied by mNormFactor * mParticles[ idxB ].GetVolume().
                //      Note that the fact that this is not actual number density has implications when
                //      computing other quantities using SPH formalism, namely that mNormFactor gets omitted from density derivative calculations.
                mPclDensities[ idxA ].mNumberDensity      += q3 ;
                mPclDensities[ idxA ].mNearNumberDensity  += q4 ;
                mPclDensities[ idxA ].mMassDensity        += q3 * mNormFactor * mParticles[ idxB ].GetMass() ;

                mPclDensities[ idxB ].mNumberDensity      += q3 ;
                mPclDensities[ idxB ].mNearNumberDensity  += q4 ;
                mPclDensities[ idxB ].mMassDensity        += q3 * mNormFactor * mParticles[ idxA ].GetMass() ;
#endif
            }
        }

    private:
        DensityAccumulator & operator=( const DensityAccumulator & ) ; // Prevent assignment

        VECTOR< SphFluidDensities > &   mPclDensities       ; ///< Fluid particle density at each particle.
        const VECTOR< Vorton > &        mParticles          ; ///< Fluid particles.
        const float                     mInfluenceRadius    ; ///< Range of influence each particle has on others.
        const float                     mInflRad2           ; ///< Square of mInfluenceRadius.
        const float                     mNormFactor         ; ///< Normalization factor
} ;




#if USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH

/** Compute fluid particle number density at each SPH particle using a uniform grid spatial partition, for a subset of the domain.
*/
static void ComputeSphDensityAtParticles_Grid_Slice( VECTOR< SphFluidDensities > & fluidDensitiesAtPcls , const VECTOR< Vorton > & particles , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid , size_t izStart , size_t izEnd , VortonSim::PhaseE phase )
{
    if( 0 == particles.Size() )
    {
        return ;
    }

    ASSERT( fluidDensitiesAtPcls.Size() == particles.Size() ) ;
    ASSERT( ! pclIndicesGrid.Empty() ) ; // Spatial partition must be populated.

    const float pclRad          = particles[ 0 ].GetRadius() ;
    const float influenceRadius = influenceRadiusScale * pclRad ;

    // Make sure grid cell spans maximum possible neighbor distance, otherwise search below will not find all neighbors.
    ASSERT( influenceRadius <= MIN3( pclIndicesGrid.GetCellSpacing().x , pclIndicesGrid.GetCellSpacing().y , pclIndicesGrid.GetCellSpacing().z ) ) ;

    DensityAccumulator accumulateDensity( fluidDensitiesAtPcls , particles , influenceRadius ) ;

    const int &     nx      = pclIndicesGrid.GetNumPoints( 0 ) ;
    const size_t    nxm1    = nx - 1 ;
    const int &     ny      = pclIndicesGrid.GetNumPoints( 1 ) ;
    const size_t    nym1    = ny - 1 ;
    const int       nxy     = nx * ny ;

    // Set up increment and offset for alternating odd-even access.
    const size_t   zInc     =   VortonSim::PHASE_BOTH == phase   ? 1 : 2 ;  ///< Amount by which to increment z index.
    const size_t   flipper  = ( VortonSim::PHASE_ODD  == phase ) ? 1 : 0 ;
    const size_t   izShift  = ( izStart & 1 ) ^ flipper ; // Flip lowest bit on odd phase.

    DEBUG_ONLY( const size_t & nz      = pclIndicesGrid.GetNumPoints( 2 ) ) ;
    DEBUG_ONLY( const size_t   nzm1    = nz - 1 ) ;
    ASSERT( nz   > 0  ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( nzm1 < nz ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( izStart <  izEnd ) ;
    ASSERT( izEnd   <= nzm1  ) ;

    // Note that the cell-neighborhood loop visits only half the neighboring
    // cells. That is because each particle visitation is symmetric so there is
    // no need to visit all neighboring cells.
    const int neighborCellOffsets[] =
    {   // Offsets to neighboring cells whose indices exceed this one:
           1            // + , 0 , 0 ( 1)
        , -1 + nx       // - , + , 0 ( 2)
        ,    + nx       // 0 , + , 0 ( 3)
        ,  1 + nx       // + , + , 0 ( 4)
        , -1 - nx + nxy // - , - , + ( 5)
        ,    - nx + nxy // 0 , - , + ( 6)
        ,  1 - nx + nxy // + , - , + ( 7)
        , -1      + nxy // - , 0 , + ( 8)
        ,         + nxy // 0 , 0 , + ( 9)
        ,  1      + nxy // + , 0 , + (10)
        , -1 + nx + nxy // - , 0 , + (11)
        ,      nx + nxy // 0 , + , + (12)
        ,  1 + nx + nxy // + , + , + (13)
    } ;
    static const size_t numNeighborCells = sizeof( neighborCellOffsets ) / sizeof( neighborCellOffsets[ 0 ] ) ;

    const size_t gridCapacity = pclIndicesGrid.GetGridCapacity() ;

    size_t idx[3] ;
    for( idx[2] = izStart + izShift ; idx[2] < izEnd ; idx[2] += zInc )
    {   // For all grid cells along z...
        const size_t offsetZ0 =   idx[2]       * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all grid cells along y...
            const size_t offsetY0Z0 = idx[1] * nx + offsetZ0 ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all grid cells along x...
                const size_t offsetX0Y0Z0 = idx[0] + offsetY0Z0 ;

                const size_t numInCurrentCell = pclIndicesGrid[ offsetX0Y0Z0 ].Size() ;
                for( unsigned ivHere = 0 ; ivHere < numInCurrentCell ; ++ ivHere )
                {   // For each particle in this gridcell...
                    const unsigned & rVortIdxHere = pclIndicesGrid[ offsetX0Y0Z0 ][ ivHere ] ;

                    ASSERT( pclRad == particles[ rVortIdxHere ].GetRadius() ) ; // This implementation assumes all SPH particles have same radius.

                    // Aggregate density between particles that share a cell.
                    // Notice that the loop only visits particles with indices
                    // following the "here" particle.  That is because each
                    // visitation symmetrically modifies both particles in the
                    // pair.  If "here" and "there" is visited, then "there"
                    // and "here" should not also be visited, because it
                    // would be redundant.
                    for( unsigned ivThere = ivHere + 1 ; ivThere < numInCurrentCell ; ++ ivThere )
                    {   // For each OTHER particle within this same cell...
                        const unsigned & rVortIdxThere = pclIndicesGrid[ offsetX0Y0Z0 ][ ivThere ] ;
                        accumulateDensity( rVortIdxHere , rVortIdxThere ) ;
                    }

                    // Aggregate density between particles in adjacent cells.
                    for( size_t idxNeighborCell = 0 ; idxNeighborCell < numNeighborCells ; ++ idxNeighborCell )
                    {   // For each cell in neighborhood...
                        const size_t cellOffset = offsetX0Y0Z0 + neighborCellOffsets[ idxNeighborCell ] ; // offset of adjacent cell
                        if( cellOffset >= gridCapacity ) break ; // Would-be neighbor is out-of-bounds.
                        const VECTOR< unsigned > & cell = pclIndicesGrid[ cellOffset ] ;
                        for( unsigned ivThere = 0 ; ivThere < cell.Size() ; ++ ivThere )
                        {   // For each particle in the visited cell...
                            const unsigned & rVortIdxThere = cell[ ivThere ] ;
                            accumulateDensity( rVortIdxHere , rVortIdxThere ) ;
                        }
                    }
                }
            }
        }
    }
}




/** Include "self influence" of each particle's mass to its SPH density.
*/
static void InitializeDensitySelfInfluence(     VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                                            ,   const VECTOR< Vorton > & particles )
{
    PERF_BLOCK( InitializeDensitySelfInfluence ) ;

    const float pclRad          = particles[ 0 ].GetRadius() ;
    const float influenceRadius = influenceRadiusScale * pclRad ;
    const float normFactor      = 15.0f / ( PI * influenceRadius * influenceRadius * influenceRadius ) ;
    const size_t numPcls = particles.Size() ;
    for( size_t idx = 0 ; idx < numPcls ; ++ idx )
    {
        // Calculate self-influence.
#if USE_NORMALIZED_SPH_FORM_FOR_NUMBER_DENSITY
        fluidDensitiesAtPcls[ idx ].mNumberDensity      = normFactor * particles[ idx ].GetVolume() ;
        fluidDensitiesAtPcls[ idx ].mNearNumberDensity  = normFactor * particles[ idx ].GetVolume() ;
        fluidDensitiesAtPcls[ idx ].mMassDensity        = normFactor * particles[ idx ].GetMass() ;
#else   // Non-normalized SPH kernel for number density
        ASSERT( 1.0f == fluidDensitiesAtPcls[ idx ].mNumberDensity     ) ;
        ASSERT( 1.0f == fluidDensitiesAtPcls[ idx ].mNearNumberDensity ) ;
        fluidDensitiesAtPcls[ idx ].mMassDensity        = normFactor * particles[ idx ].GetMass() ;
#endif
    }
}




/** Compute fluid particle number density at each SPH particle using a uniform grid spatial partition.

    This is an O(N*k) operation where N is the number of particles and k is the
    average number of particles in the neighborhood of one of the N particles.
*/
void ComputeSphDensityAtParticles_Grid( VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                                      , const VECTOR< Vorton > & particles
                                      , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid )
{
#if ENABLE_FLUID_BODY_SIMULATION
    PERF_BLOCK( ComputeSphDensityAtParticles_Grid ) ;

    fluidDensitiesAtPcls.clear() ;
    fluidDensitiesAtPcls.resize( particles.Size() , SphFluidDensities( 1.0f , 1.0f , 0.0f ) ) ;
    InitializeDensitySelfInfluence( fluidDensitiesAtPcls , particles ) ;

    const unsigned & nz     = pclIndicesGrid.GetNumPoints( 2 ) ;
    const unsigned   nzm1   = nz - 1 ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , nzm1 / gNumberOfProcessors ) ;
        // Compute particle number density using threading building blocks.
        // Alternate between even and odd z-slices to avoid multiple threads accessing the same particles simultaneously.
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , 
            SphSim_ComputeSphNumberDensityAtParticles_TBB( fluidDensitiesAtPcls , particles , pclIndicesGrid , VortonSim::PHASE_EVEN ) ) ;
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) ,
            SphSim_ComputeSphNumberDensityAtParticles_TBB( fluidDensitiesAtPcls , particles , pclIndicesGrid , VortonSim::PHASE_ODD  ) ) ;
    #else
        ComputeSphDensityAtParticles_Grid_Slice( fluidDensitiesAtPcls , particles , pclIndicesGrid , 0 , nzm1 , VortonSim::PHASE_BOTH ) ;
    #endif
#endif
}




#else

/** Compute fluid particle number density at each SPH particle using direct summation.

    This is an O(N^2) operation where N is the number of particles.
*/
void ComputeSphDensityAtParticles_Direct( VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                                        , const VECTOR< Vorton > & particles )
{
    PERF_BLOCK( ComputeSphDensityAtParticles_Direct ) ;

    const size_t numPcls = particles.size() ;
    if( 0 == numPcls )
    {
        return ;
    }

    ASSERT( fluidDensitiesAtPcls.empty() ) ;
    fluidDensitiesAtPcls.resize( numPcls , SphFluidDensities( 1.0f , 1.0f , 0.0f ) ) ;
    InitializeDensitySelfInfluence( fluidDensitiesAtPcls , particles ) ;

    const float pclRad          = particles[ 0 ].GetRadius() ;
    const float influenceRadius = influenceRadiusScale * pclRad ;

    DensityAccumulator accumulateDensity( fluidDensitiesAtPcls , particles , influenceRadius ) ;

    for( size_t idxA = 0 ; idxA < numPcls ; ++ idxA )
    {   // For each particle, A...
        ASSERT( pclRad == particles[ idxA ].GetRadius() ) ; // This implementation assumes all SPH particles have same radius.
        for( size_t idxB = idxA + 1 ; idxB < numPcls ; ++ idxB )
        {   // For each OTHER particle following A...
            accumulateDensity( idxA , idxB ) ;
        }
    }
}

#endif




/** Functor to accumulate pressure gradient between two smoothed particles.
*/
class PressureGradientAccumulator
{
    public:
        /** Construct a functor to accumulate pressure gradient between two smoothed particles.

            \param accelerations      (out) Change in velocity, per unit time, for each particle.

            \param fluidDensitiesAtPcls     Fluid particle densities.

            \param particles                Fluid particles whose accelerations to compute.

            \param influenceRadius          Cutoff range of smoothing function.
                                            Particles inside this range contribute
                                            to each others' densities.
        */
        PressureGradientAccumulator( 
                    VECTOR< Vec3 > &                    accelerations
                ,   const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                ,   const VECTOR< Vorton > &            particles
                ,   const float                         influenceRadius
            )
            : mAccelerations( accelerations )
            , mPclDensities( fluidDensitiesAtPcls )
            , mParticles( particles )
            , mInfluenceRadius( influenceRadius )
            , mInflRad2( influenceRadius * influenceRadius )
            , mInvInflRad( 1.0f / influenceRadius )
            , mTargetNumberDensity( targetNumberDensity )
            , mWaveSpeed2( stiffness )
            , mWaveSpeed2Near( nearToFar * stiffness )
            , mNormFactor( 15.0f / ( PI * mInflRad2 * mInfluenceRadius ) )
        {
            ASSERT( mAccelerations.Size() == mPclDensities.Size() ) ;
            ASSERT( mAccelerations.Size() == mParticles.Size() ) ;
            // NormFactor is reciprocal of integral of smoothing function over a spherical volume:
            // Integrate[ ( 1 - r / h )^3 * r^2 * 4 * pi , { r , 0 , h } ]
        }

        void operator()( size_t idxA , size_t idxB )
        {
            Vec3  sep     = mParticles[ idxA ].mPosition - mParticles[ idxB ].mPosition ;
            float dist2   = sep.Mag2() ;

            if(     ( dist2 < sHardCoreRadius2 )
                ||  ( fabsf( sep.x ) < sHardCoreRadius )
                ||  ( fabsf( sep.y ) < sHardCoreRadius )
                ||  ( fabsf( sep.z ) < sHardCoreRadius ) )
            {   // Particles too close. Pressure gradient would be zero.
                // Impose a random separation.
                sep    += RandomSpread( sTinyJiggle ) ;
                dist2   = sep.Mag2() ;
            }

            if( dist2 < mInflRad2 )
            {   // Particles are close enough to apply pressure on each other.
                const float dist    = fsqrtf( dist2 ) ;
                const float q       = 1.0f - dist / mInfluenceRadius ;
                const float q2      = Pow2( q ) ;

                ASSERT( ( 0.0f <= q ) && ( q <= 1.0f ) ) ;

                const Vec3      dir         = sep / ( dist + FLT_EPSILON ) ;

                const float &   numDensA    = mPclDensities[ idxA ].mNumberDensity ;
                const float &   numDensB    = mPclDensities[ idxB ].mNumberDensity ;

                {   // Compute acceleration due to pressure gradient.
                    const float q3          = q2 * q  ;
                    const float & nearDensA = mPclDensities[ idxA ].mNearNumberDensity ;
                    const float & nearDensB = mPclDensities[ idxB ].mNearNumberDensity ;
                    //const float presA       = numDensA - targetNumberDensity ;
                    //const float presB       = numDensB - targetNumberDensity ;
                    const float pressure    = mWaveSpeed2     * ( numDensA + numDensB - 2.0f * mTargetNumberDensity ) ;
                    const float pressNear   = mWaveSpeed2Near * ( nearDensA + nearDensB ) ;
                    const float dReg        = pressure  * q2 ;
                    const float dNear       = pressNear * q3 ;
                    const Vec3  accel       = ( dReg + dNear ) * dir ;
                    mAccelerations[ idxA ] += accel ;
                    mAccelerations[ idxB ] -= accel ;
                    ASSERT( ! IsNan( mAccelerations[ idxA ] ) && ! IsInf( mAccelerations[ idxA ] ) ) ;
                    ASSERT( ! IsNan( mAccelerations[ idxB ] ) && ! IsInf( mAccelerations[ idxB ] ) ) ;
                }
            }
        }

    private:
        PressureGradientAccumulator & operator=( const PressureGradientAccumulator & ) ; // Prevent assignment

        VECTOR< Vec3 > &                    mAccelerations      ; ///< Change in velocity, per unit time, for each particle.
        const VECTOR< SphFluidDensities > & mPclDensities       ; ///< Fluid particle density at each particle.
        const VECTOR< Vorton > &            mParticles          ; ///< Fluid particles.
        const float                         mInfluenceRadius    ; ///< Range of influence each particle has on others.
        const float                         mInflRad2           ; ///< Square of mInfluenceRadius.
        const float                         mInvInflRad         ; ///< Reciprocal of mInfluenceRadius.
        const float                         mNormFactor         ; ///< Normalization factor
        const float                         mTargetNumberDensity; ///< Target fluid particle number density.  Tuned for equilibrium test cases.
        const float                         mWaveSpeed2         ; ///< Long-range  pressure stiffness.  Tuned for compromise between stickiness and stability.
        const float                         mWaveSpeed2Near     ; ///< Short-range pressure stiffness.  Tuned for compromise between compression and stability.
} ;




/** Functor to accumulate mass density gradient between two smoothed particles.
*/
class MassDensityGradientAccumulator
{
    public:
        /** Construct a functor to accumulate mass density gradient between two smoothed particles.

            \param fluidMassDensitiesAtPcls Fluid particle mass densities.

            \param particles                Fluid particles whose accelerations to compute.

            \param proximities              Partially truncated signed distance between particle and nearest wall.

            \param influenceRadius          Cutoff range of smoothing function.
                                            Particles inside this range contribute
                                            to each others' densities.
        */
        MassDensityGradientAccumulator(
                    VECTOR< Vec3 > &                        massDensityGradients
                    ,   const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                ,   const VECTOR< Vorton > &                particles
                ,   const VECTOR< float > &                 proximities
                ,   const float                             influenceRadius
                ,   const float                             ambientDensity
            )
            : mMassDensityGradients( massDensityGradients )
            , mPclDensities( fluidDensitiesAtPcls )
            , mParticles( particles )
            , mProximities( proximities )
            , mInfluenceRadius( influenceRadius )
            , mAmbientDensity( ambientDensity )
            , mInflRad2( influenceRadius * influenceRadius )
            , mInvInflRad( 1.0f / influenceRadius )
            , mNormFactor( 15.0f / ( PI * mInflRad2 * mInfluenceRadius ) )

//, influenceSum( 0.0f )
//, influenceSum2( 0.0f )
//, influenceMin( FLT_MAX )
//, influenceMax( - FLT_MAX )

        {
            ASSERT( mMassDensityGradients.Size() == mPclDensities.Size() ) ;
            ASSERT( mMassDensityGradients.Size() == mParticles.Size() ) ;
            ASSERT( mMassDensityGradients.Size() == mProximities.Size() ) ;
            // NormFactor is reciprocal of integral of smoothing function over a spherical volume:
            // Integrate[ ( 1 - r / h )^3 * r^2 * 4 * pi , { r , 0 , h } ]
        }

        void operator()( size_t idxA , size_t idxB )
        {
            Vec3  sep     = mParticles[ idxA ].mPosition - mParticles[ idxB ].mPosition ;
            float dist2   = sep.Mag2() ;

            if(     ( dist2 < sHardCoreRadius2 )
                ||  ( fabsf( sep.x ) < sHardCoreRadius )
                ||  ( fabsf( sep.y ) < sHardCoreRadius )
                ||  ( fabsf( sep.z ) < sHardCoreRadius ) )
            {   // Particles too close. Pressure gradient would be zero.
                // Impose a random separation.
                sep    += RandomSpread( sTinyJiggle ) ;
                dist2   = sep.Mag2() ;
            }

            if( dist2 < mInflRad2 )
            {   // Particles are close enough to contribute density gradient to each other.
                const float dist    = fsqrtf( dist2 ) ;
                const float q       = 1.0f - dist / mInfluenceRadius ;
                const float q2      = Pow2( q ) ;

                ASSERT( ( 0.0f <= q ) && ( q <= 1.0f ) ) ;

                const Vec3      dir         = sep / ( dist + FLT_EPSILON ) ;

                const float &   numDensA    = mPclDensities[ idxA ].mNumberDensity ;
                const float &   numDensB    = mPclDensities[ idxB ].mNumberDensity ;

                {   // Compute mass density gradient.
                    // Form of smoothGrad here depends on form of smoothing function used to accumulate mass density.
                    // This formula is based on the "spikey" q3 formula.
                    // This omits mNormFactor because mNumberDensity also omits it and it is in the denominator, so they cancel.
#if USE_NORMALIZED_SPH_FORM_FOR_NUMBER_DENSITY
                    const float smoothGrad  = -3.0f * q2 * mInvInflRad * mNormFactor * mParticles[ idxA ].GetVolume() ;
#else
                    const float smoothGrad  = -3.0f * q2 * mInvInflRad ;
#endif

// Use per-particle mass density instead of SPH mass density.
#define USE_PER_PARTICLE_MASS_DENSITY 1

                    // Take into account ambient density by subtracting it from massDensX below.
                    // Note that when using the "difference" gradient formula, that cancels out.
#if USE_PER_PARTICLE_MASS_DENSITY
                    // Using this, difference gradient is zero at boundaries
                    // between fluid and empty space.
                    const float massDensA = mParticles[ idxA ].mDensity - mAmbientDensity ;
                    const float massDensB = mParticles[ idxB ].mDensity - mAmbientDensity ;
#else   // Use SPH particle mass density
                    const float massDensA = mPclDensities[ idxA ].mMassDensity - mAmbientDensity ;
                    const float massDensB = mPclDensities[ idxB ].mMassDensity - mAmbientDensity ;
#endif

                    // Canonical SPH gradient formula.  Simple, but has issues
                    // that other forms solve.
                    const float densGradA_can = massDensB * smoothGrad / numDensB ;
                    const float densGradB_can = massDensA * smoothGrad / numDensA ;

                    // This form is symmetric, meaning the contribution due to
                    // each other particle is the opposite as the reverse.
                    const float common_sym      = smoothGrad * (  massDensA / ( numDensA * numDensA )
                                                               +  massDensB / ( numDensB * numDensB ) ) ;
                    const float densGradA_sym   =   common_sym * numDensA ;
                    const float densGradB_sym   =   common_sym * numDensB ;
                    (void) densGradA_sym , densGradB_sym ; // Unused currently, but useful for debugging, and might use later.

                    // Difference (Monaghan 2005, section 2.2, Phi=1)
                    // This form has some gradient at boundaries between fluid
                    // and empty space, due to the fact that the SPH mass
                    // density varies there, due to its spatial smoothing.
                    // It also yields results comparable with the above formulae
                    // in regions far from fluid-empty edges.
                    const float common_dif      = ( massDensB - massDensA ) * smoothGrad ;
                    const float densGradA_dif   =   common_dif / numDensA ;
                    const float densGradB_dif   = - common_dif / numDensB ;

                    // Use difference formula at body boundaries, symmetric formula elsewhere.

                    if( mProximities[ idxA ] >= mInfluenceRadius )
                    {   // Far from wall.
                        mMassDensityGradients[ idxA ] += densGradA_can * dir ;
                    }
                    else
                    {   // Near or in wall.
                        mMassDensityGradients[ idxA ] += densGradA_dif * dir ;
                    }

                    if( mProximities[ idxB ] >= mInfluenceRadius )
                    {   // Far from wall.
                        mMassDensityGradients[ idxB ] -= densGradB_can * dir ;
                    }
                    else
                    {   // Near or in wall.
                        mMassDensityGradients[ idxB ] -= densGradB_dif * dir ;
                    }

//#error Some density gradients are huge and I don't know why.
//#error TODO: Set a breakpoint in here for large density gradients and track that down.
//#error NOTE: I already tried detecting and responding to particles "in isolation".  That didn't help. See below.
//#error The case where this is noticeable: vertical column of water on side of tank. Alt-F2 or something close to that.

                }
            }
        }

    private:
        MassDensityGradientAccumulator & operator=( const MassDensityGradientAccumulator & ) ; // Prevent assignment

        VECTOR< Vec3 > &                    mMassDensityGradients   ; ///< Mass density gradients for each particle.
        const VECTOR< SphFluidDensities > & mPclDensities           ; ///< Fluid particle densities at each particle.
        const VECTOR< Vorton > &            mParticles              ; ///< Fluid particles.
        const VECTOR< float > &             mProximities            ; ///< Partially truncated signed distance between particle and wall.
        const float                         mInfluenceRadius        ; ///< Range of influence each particle has on others.
        const float                         mAmbientDensity         ; ///< Density of fluid in the absence of fluid particles.
        const float                         mInflRad2               ; ///< Square of mInfluenceRadius.
        const float                         mInvInflRad             ; ///< Reciprocal of mInfluenceRadius.
        const float                         mNormFactor             ; ///< Normalization factor

    //public:
    //  // "Influence" statistics: Want to know which particles have too few neighbors to compute a reliable density gradient estimate,
    //  // in order to disable baroclinic generation of vorticity in those regions.
    //  // The idea was to use linear gravity acceleration (instead of baroclinic vorticity)
    //  // in those regions.  So far, automatically detecting those regions has not worked out very well.
    //    float influenceSum    ;   ///< Sum of "influence" (like 1/d) across all particles.
    //    float influenceSum2   ;   ///< Sum of influence^2 (like 1/d) across all particles.
    //    float influenceMin    ;   ///< Minumum influence across all particles.
    //    float influenceMax    ;   ///< Maximum influence across all particles.
} ;




#if USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH

/** Compute fluid particle acceleration due to pressure gradient at each SPH particle using a uniform grid spatial partition, for a subset of the domain.
*/
static void ComputeSphPressureGradientAcceleration_Grid_Slice( VECTOR< Vec3 > & accelerations
                                                             , const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                                                             , const VECTOR< Vorton > & particles
                                                             , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                                                             , size_t izStart
                                                             , size_t izEnd
                                                             , VortonSim::PhaseE phase )
{
    if( 0 == particles.Size() )
    {
        return ;
    }

    ASSERT( accelerations.Size()    == particles.Size() ) ;

    ASSERT( ! pclIndicesGrid.Empty() ) ; // Spatial partition must be populated.

    const float pclRad          = particles[ 0 ].GetRadius() ;
    const float influenceRadius = influenceRadiusScale * pclRad ;

    ASSERT( influenceRadius <= MIN3( pclIndicesGrid.GetCellSpacing().x , pclIndicesGrid.GetCellSpacing().y , pclIndicesGrid.GetCellSpacing().z ) ) ;

    PressureGradientAccumulator accumulatePressureGradient( accelerations , fluidDensitiesAtPcls , particles , influenceRadius ) ;

    const int &     nx      = pclIndicesGrid.GetNumPoints( 0 ) ;
    const size_t    nxm1    = nx - 1 ;
    const int &     ny      = pclIndicesGrid.GetNumPoints( 1 ) ;
    const size_t    nym1    = ny - 1 ;
    const int       nxy     = nx * ny ;

    // Set up increment and offset for alternating odd-even access.
    const size_t   zInc     =   VortonSim::PHASE_BOTH == phase   ? 1 : 2 ;  ///< Amount by which to increment z index.
    const size_t   flipper  = ( VortonSim::PHASE_ODD  == phase ) ? 1 : 0 ;
    const size_t   izShift  = ( izStart & 1 ) ^ flipper ; // Flip lowest bit on odd phase.

    DEBUG_ONLY( const size_t & nz      = pclIndicesGrid.GetNumPoints( 2 ) ) ;
    DEBUG_ONLY( const size_t   nzm1    = nz - 1 ) ;
    ASSERT( nz   > 0  ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( nzm1 < nz ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( izStart <  izEnd ) ;
    ASSERT( izEnd   <= nzm1  ) ;

    // Note that the cell-neighborhood loop visits only half the neighboring
    // cells. That is because each particle visitation is symmetric so there is
    // no need to visit all neighboring cells.
    const int neighborCellOffsets[] =
    {   // Offsets to neighboring cells whose indices exceed this one:
           1            // + , 0 , 0 ( 1)
        , -1 + nx       // - , + , 0 ( 2)
        ,    + nx       // 0 , + , 0 ( 3)
        ,  1 + nx       // + , + , 0 ( 4)
        , -1 - nx + nxy // - , - , + ( 5)
        ,    - nx + nxy // 0 , - , + ( 6)
        ,  1 - nx + nxy // + , - , + ( 7)
        , -1      + nxy // - , 0 , + ( 8)
        ,         + nxy // 0 , 0 , + ( 9)
        ,  1      + nxy // + , 0 , + (10)
        , -1 + nx + nxy // - , 0 , + (11)
        ,      nx + nxy // 0 , + , + (12)
        ,  1 + nx + nxy // + , + , + (13)
    } ;
    static const size_t numNeighborCells = sizeof( neighborCellOffsets ) / sizeof( neighborCellOffsets[ 0 ] ) ;

    const size_t gridCapacity = pclIndicesGrid.GetGridCapacity() ;

    size_t idx[3] ;
    for( idx[2] = izStart + izShift ; idx[2] < izEnd ; idx[2] += zInc )
    {   // For all grid cells along z...
        const size_t offsetZ0 = idx[2] * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all grid cells along y...
            const size_t offsetY0Z0 = idx[1] * nx + offsetZ0 ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all grid cells along x...
                const size_t                offsetX0Y0Z0       = idx[0] + offsetY0Z0 ;
                const VECTOR< unsigned > &  currentCell        = pclIndicesGrid[ offsetX0Y0Z0 ] ;
                const size_t                numInCurrentCell   = currentCell.Size() ;

                for( unsigned ivHere = 0 ; ivHere < numInCurrentCell ; ++ ivHere )
                {   // For each particle in this gridcell...
                    const unsigned & rVortIdxHere = currentCell[ ivHere ] ;

                    ASSERT( pclRad == particles[ rVortIdxHere ].GetRadius() ) ; // This implementation assumes all SPH particles have same radius.

                    // Aggregate accelerations between particles that share a cell.
                    // Notice that the loop only visits particles with indices
                    // following the "here" particle.  That is because each
                    // visitation symmetrically modifies both particles in the
                    // pair.  If "here" and "there" is visited, then "there"
                    // and "here" should not also be visited, because it
                    // would be redundant.
                    for( unsigned ivThere = ivHere + 1 ; ivThere < numInCurrentCell ; ++ ivThere )
                    {   // For each OTHER particle within this same cell...
                        const unsigned & rVortIdxThere = currentCell[ ivThere ] ;
                        accumulatePressureGradient( rVortIdxHere , rVortIdxThere ) ;
                    }

                    // Aggregate accelerations between particles in neighboring cells.
                    for( size_t idxNeighborCell = 0 ; idxNeighborCell < numNeighborCells ; ++ idxNeighborCell )
                    {   // For each cell in neighborhood...
                        const size_t cellOffset = offsetX0Y0Z0 + neighborCellOffsets[ idxNeighborCell ] ; // offset of neighbor cell
                        if( cellOffset >= gridCapacity ) break ; // Would-be neighbor is out-of-bounds.

                        const VECTOR< unsigned > &  neighborCell        = pclIndicesGrid[ cellOffset ] ;
                        const size_t                numInNeighborCell   = neighborCell.Size() ;
                        for( unsigned ivThere = 0 ; ivThere < numInNeighborCell ; ++ ivThere )
                        {   // For each particle in neighbor cell...
                            const unsigned & rVortIdxThere = neighborCell[ ivThere ] ;
                            accumulatePressureGradient( rVortIdxHere , rVortIdxThere ) ;
                        }
                    }
                }
            }
        }
    }
}




/** Compute fluid mass density gradient at each SPH particle using a uniform grid spatial partition, for a subset of the domain.
*/
static void ComputeSphMassDensityGradient_Grid_Slice( VECTOR< Vec3 > & massDensityGradients
                                                    , const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                                                    , const VECTOR< Vorton > & particles
                                                    , const VECTOR< float > & proximities
                                                    , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                                                    , const float ambientDensity
                                                    , size_t izStart
                                                    , size_t izEnd
                                                    , VortonSim::PhaseE phase )
{
    if( 0 == particles.Size() )
    {
        return ;
    }

    ASSERT( massDensityGradients.Size() == particles.Size() ) ;

    ASSERT( ! pclIndicesGrid.Empty() ) ; // Spatial partition must be populated.

    const float pclRad          = particles[ 0 ].GetRadius() ;
    const float influenceRadius = influenceRadiusScale * pclRad ;

    ASSERT( influenceRadius <= MIN3( pclIndicesGrid.GetCellSpacing().x , pclIndicesGrid.GetCellSpacing().y , pclIndicesGrid.GetCellSpacing().z ) ) ;

    MassDensityGradientAccumulator accumulateMassDensityGradient( massDensityGradients , fluidDensitiesAtPcls , particles , proximities , influenceRadius , ambientDensity ) ;

    const int &     nx      = pclIndicesGrid.GetNumPoints( 0 ) ;
    const size_t    nxm1    = nx - 1 ;
    const int &     ny      = pclIndicesGrid.GetNumPoints( 1 ) ;
    const size_t    nym1    = ny - 1 ;
    const int       nxy     = nx * ny ;

    // Set up increment and offset for alternating odd-even access.
    const size_t   zInc     =   VortonSim::PHASE_BOTH == phase   ? 1 : 2 ;  ///< Amount by which to increment z index.
    const size_t   flipper  = ( VortonSim::PHASE_ODD  == phase ) ? 1 : 0 ;
    const size_t   izShift  = ( izStart & 1 ) ^ flipper ; // Flip lowest bit on odd phase.

    DEBUG_ONLY( const size_t & nz      = pclIndicesGrid.GetNumPoints( 2 ) ) ;
    DEBUG_ONLY( const size_t   nzm1    = nz - 1 ) ;
    ASSERT( nz   > 0  ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( nzm1 < nz ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( izStart <  izEnd ) ;
    ASSERT( izEnd   <= nzm1  ) ;

    // Note that the cell-neighborhood loop visits only half the neighboring
    // cells. That is because each particle visitation is symmetric so there is
    // no need to visit all neighboring cells.
    const int neighborCellOffsets[] =
    {   // Offsets to neighboring cells whose indices exceed this one:
           1            // + , 0 , 0 ( 1)
        , -1 + nx       // - , + , 0 ( 2)
        ,    + nx       // 0 , + , 0 ( 3)
        ,  1 + nx       // + , + , 0 ( 4)
        , -1 - nx + nxy // - , - , + ( 5)
        ,    - nx + nxy // 0 , - , + ( 6)
        ,  1 - nx + nxy // + , - , + ( 7)
        , -1      + nxy // - , 0 , + ( 8)
        ,         + nxy // 0 , 0 , + ( 9)
        ,  1      + nxy // + , 0 , + (10)
        , -1 + nx + nxy // - , 0 , + (11)
        ,      nx + nxy // 0 , + , + (12)
        ,  1 + nx + nxy // + , + , + (13)
    } ;
    static const size_t numNeighborCells = sizeof( neighborCellOffsets ) / sizeof( neighborCellOffsets[ 0 ] ) ;

    const size_t gridCapacity = pclIndicesGrid.GetGridCapacity() ;

    size_t idx[3] ;
    for( idx[2] = izStart + izShift ; idx[2] < izEnd ; idx[2] += zInc )
    {   // For all grid cells along z...
        const size_t offsetZ0 = idx[2] * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all grid cells along y...
            const size_t offsetY0Z0 = idx[1] * nx + offsetZ0 ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all grid cells along x...
                const size_t                offsetX0Y0Z0       = idx[0] + offsetY0Z0 ;
                const VECTOR< unsigned > &  currentCell        = pclIndicesGrid[ offsetX0Y0Z0 ] ;
                const size_t                numInCurrentCell   = currentCell.Size() ;

                for( unsigned ivHere = 0 ; ivHere < numInCurrentCell ; ++ ivHere )
                {   // For each particle in this gridcell...
                    const unsigned & rVortIdxHere = currentCell[ ivHere ] ;

                    ASSERT( pclRad == particles[ rVortIdxHere ].GetRadius() ) ; // This implementation assumes all SPH particles have same radius.

                    // Aggregate mass density gradient between particles that share a cell.
                    // Notice that the loop only visits particles with indices
                    // following the "here" particle.  That is because each
                    // visitation symmetrically modifies both particles in the
                    // pair.  If "here" and "there" is visited, then "there"
                    // and "here" should not also be visited, because it
                    // would be redundant.
                    for( unsigned ivThere = ivHere + 1 ; ivThere < numInCurrentCell ; ++ ivThere )
                    {   // For each OTHER particle within this same cell...
                        const unsigned & rVortIdxThere = currentCell[ ivThere ] ;
                        accumulateMassDensityGradient( rVortIdxHere , rVortIdxThere ) ;
                    }

                    // Aggregate mass density gradient between particles in neighboring cells.
                    for( size_t idxNeighborCell = 0 ; idxNeighborCell < numNeighborCells ; ++ idxNeighborCell )
                    {   // For each cell in neighborhood...
                        const size_t cellOffset = offsetX0Y0Z0 + neighborCellOffsets[ idxNeighborCell ] ; // offset of neighbor cell
                        if( cellOffset >= gridCapacity ) break ; // Would-be neighbor is out-of-bounds.

                        const VECTOR< unsigned > &  neighborCell        = pclIndicesGrid[ cellOffset ] ;
                        const size_t                numInNeighborCell   = neighborCell.Size() ;
                        for( unsigned ivThere = 0 ; ivThere < numInNeighborCell ; ++ ivThere )
                        {   // For each particle in neighbor cell...
                            const unsigned & rVortIdxThere = neighborCell[ ivThere ] ;
                            accumulateMassDensityGradient( rVortIdxHere , rVortIdxThere ) ;
                        }
                    }
                }
            }
        }
    }
}




#if 1
/** Zero density gradient in certain regions.

    This is an experimental routine aimed at disabling baroclinic generation of
    vorticity in regions where the density gradient approximation is unreliable.
    Such regions include inside or near solid body walls, and where a particle
    has too few neighbors.

    The outcome was not entirely as desired.
*/
static void SelectivelyZeroDensityGradient( VECTOR< Vec3 > & densityGradients
                                            , const VECTOR< SphFluidDensities > & /*fluidDensitiesAtPcls*/
                                            , const VECTOR< float > & /*proximities*/
                                            , const float /*influenceRadius*/
                                            )
{
    PERF_BLOCK( SelectivelyZeroDensityGradient ) ;

    static float densGradMax = 50.0f ;
    const float densGrad2Max = densGradMax * densGradMax ;
    const size_t numPcls = densityGradients.Size() ;
    //ASSERT( fluidDensitiesAtPcls.Size() == numPcls ) ;
    //ASSERT( proximities.Size() == numPcls ) ;
    for( size_t idx = 0 ; idx < numPcls ; ++ idx )
    {   // For each particle...
        Vec3 & densGrad = densityGradients[ idx ] ;
        //if( fluidDensitiesAtPcls[ idx ].mNumberDensity < 1.2f )
        //{   // Particle is "in isolation".
        //    densGrad = Vec3( 0.0f , 0.0f , 0.0f ) ;
        //}
        //if( proximities[ idx ] < influenceRadius )
        //{
        //    densGrad = Vec3( 0.0f , 0.0f , 0.0f ) ;
        //}
        const float densGradMag2 = densGrad.Mag2() ;
        if( densGradMag2 > densGrad2Max )
        {
            const float densGradMag = fsqrtf( densGradMag2 ) ;
            densGrad *= ( densGradMax / densGradMag ) ;
        }
    }
}
#endif




/** Compute fluid particle acceleration due to pressure gradient at each SPH particle using a uniform grid spatial partition.

    This is an O(N*k) operation where N is the number of particles and k is the
    average number of particles in the neighborhood of one of the N particles.
*/
void ComputeSphPressureGradientAcceleration_Grid( VECTOR< Vec3 > & accelerations
                                                , const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                                                , const VECTOR< Vorton > & particles
                                                , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                                                )
{
#if ENABLE_FLUID_BODY_SIMULATION
    PERF_BLOCK( ComputeSphPressureGradientAcceleration_Grid ) ;

    accelerations.Clear() ;
    accelerations.Resize( particles.Size() , Vec3( 0.0f , 0.0f , 0.0f ) ) ;

    const unsigned & nz     = pclIndicesGrid.GetNumPoints( 2 ) ;
    const unsigned   nzm1   = nz - 1 ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , nzm1 / gNumberOfProcessors ) ;
        // Compute particle number density using threading building blocks.
        // Alternate between even and odd z-slices to avoid multiple threads accessing the same particles simultaneously.
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , 
            SphSim_ComputeSphPressureGradientAcceleration_TBB( accelerations , fluidDensitiesAtPcls , particles , pclIndicesGrid , VortonSim::PHASE_EVEN ) ) ;
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) ,
            SphSim_ComputeSphPressureGradientAcceleration_TBB( accelerations , fluidDensitiesAtPcls , particles , pclIndicesGrid , VortonSim::PHASE_ODD  ) ) ;
    #else
        ComputeSphPressureGradientAcceleration_Grid_Slice( accelerations , fluidDensitiesAtPcls , particles , pclIndicesGrid , 0 , nzm1 , VortonSim::PHASE_BOTH ) ;
    #endif
#endif
}




/** Compute fluid particle mass density gradient at each SPH particle using a uniform grid spatial partition.

    This is an O(N*k) operation where N is the number of particles and k is the
    average number of particles in the neighborhood of one of the N particles.
*/
void ComputeSphMassDensityGradient_Grid( VECTOR< Vec3 > &                           massDensityGradients
                                        , const VECTOR< SphFluidDensities > &       fluidDensitiesAtPcls
                                        , const VECTOR< Vorton > &                  particles
                                        , const VECTOR< float > &                   proximities
                                        , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid
                                        , const float                               ambientDensity
                                        )
{
#if ENABLE_FLUID_BODY_SIMULATION
    PERF_BLOCK( ComputeSphMassDensityGradient_Grid ) ;

    massDensityGradients.clear() ;
    massDensityGradients.resize( particles.Size() , Vec3( 0.0f , 0.0f , 0.0f ) ) ;

    const unsigned & nz     = pclIndicesGrid.GetNumPoints( 2 ) ;
    const unsigned   nzm1   = nz - 1 ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , nzm1 / gNumberOfProcessors ) ;
        // Compute particle number density using threading building blocks.
        // Alternate between even and odd z-slices to avoid multiple threads accessing the same particles simultaneously.
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , 
            SphSim_ComputeSphMassDensityGradient_TBB( massDensityGradients , fluidDensitiesAtPcls , particles , proximities , pclIndicesGrid , ambientDensity , VortonSim::PHASE_EVEN ) ) ;
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) ,
            SphSim_ComputeSphMassDensityGradient_TBB( massDensityGradients , fluidDensitiesAtPcls , particles , proximities , pclIndicesGrid , ambientDensity , VortonSim::PHASE_ODD  ) ) ;
    #else
        ComputeSphMassDensityGradient_Grid_Slice( massDensityGradients , fluidDensitiesAtPcls , particles , proximities , pclIndicesGrid , ambientDensity , 0 , nzm1 , VortonSim::PHASE_BOTH ) ;
    #endif

//#error Experimental: Zero density gradients where their estimate is unreliable, to facilitate shutting off baroclinic vorticity generation there, to let linear acceleration operate there instead.
{
    const float pclRad          = particles[ 0 ].GetRadius() ;
    const float influenceRadius = influenceRadiusScale * pclRad ;
    SelectivelyZeroDensityGradient( massDensityGradients , fluidDensitiesAtPcls , proximities , influenceRadius ) ;
}

#endif
}




#else

/** Compute fluid particle acceleration due to pressure gradient at each SPH particle using direct summation.

    This is an O(N^2) operation where N is the number of particles.
*/
void ComputeSphPressureGradientForce_Direct( VECTOR< Vec3 > & accelerations
                                            , const VECTOR< SphFluidDensities > & fluidDensitiesAtPcls
                                            , const VECTOR< Vorton > & particles )
{
    PERF_BLOCK( ComputeSphPressureGradientForce_Direct ) ;

    const size_t numPcls = particles.size() ;
    if( 0 == numPcls )
    {
        return ;
    }

    accelerations.Clear() ;
    accelerations.Resize( numPcls , Vec3( 0.0f , 0.0f , 0.0f ) ) ;

    const float pclRad          = particles[ 0 ].GetRadius() ;
    const float influenceRadius = influenceRadiusScale * pclRad ;

    PressureGradientAccumulator accumulatePressureGradient( accelerations , fluidDensitiesAtPcls , particles , influenceRadius ) ;

    for( size_t idxA = 0 ; idxA < numPcls ; ++ idxA )
    {
        ASSERT( pclRad == particles[ idxA ].GetRadius() ) ; // This implementation assumes all SPH particles have same radius.
        for( size_t idxB = idxA + 1 ; idxB < numPcls ; ++ idxB )
        {
            accumulatePressureGradient( idxA , idxB ) ;
        }
    }
}

#endif // USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH




#if USE_SMOOTHED_PARTICLE_HYDRODYNAMICS

/** Apply a uniform body force (gravity/buoyancy) on each particle.
*/
static void ApplyBodyForce( VECTOR< Vorton > & particles , const VECTOR< Vec3 > & accelerations , float ambientDensity , float timeStep , size_t idxPclStart , size_t idxPclEnd )
{
    PERF_BLOCK( ApplyBodyForce ) ;

    const size_t numPcls = particles.size() ;
    if( 0 == numPcls )
    {
        return ;
    }

    static Vec3         gravityAcceleration( 0.0f , 0.0f , -10.0f ) ;

    for( size_t idxPcl = idxPclStart ; idxPcl < idxPclEnd ; ++ idxPcl )
    {
        // Update velocity.
        Vec3 & velPcl = particles[ idxPcl ].mVelocity ;
        velPcl += accelerations[ idxPcl ] * timeStep ;

        ASSERT( ! IsNan( velPcl ) && ! IsInf( velPcl ) ) ;
        ASSERT( ! IsInf( velPcl.Mag2() ) ) ;
        ASSERT( velPcl.Mag2() < 1e8f ) ;

        // Apply body force (gravity/buoyancy).
        const float & density           = particles[ idxPcl ].mDensity ;
        ASSERT( density > 0.0f ) ;
        const float   relativeDensity   = ( density - ambientDensity ) / density ;
        particles[ idxPcl ].mVelocity += timeStep * gravityAcceleration * relativeDensity ;

        ASSERT( ! IsNan( particles[ idxPcl ].mVelocity ) && ! IsInf( particles[ idxPcl ].mVelocity ) ) ;
    }
}




/** Functor to exchange velocity between two smoothed particles.
*/
class VelocityDiffuser
{
    public:
        /** Construct a functor to exchange velocity between two smoothed particles.
        */
        VelocityDiffuser( 
                    VECTOR< Vorton > &  particles
                ,   const float         influenceRadius
                ,   const float         timeStep
                ,   const float         radialViscosity
            )
            : mParticles( particles )
            , mInfluenceRadius( influenceRadius )
            , mInflRad2( influenceRadius * influenceRadius )
            , mTimeStep( timeStep )
            , mTangentialViscosityGain( 0.0f )
            , mRadialViscosityGain( radialViscosity * timeStep )
        {
            ASSERT( mRadialViscosityGain <= 1.0f ) ; // Otherwise velocity can flip direction, which friction cannot cause.
        }

        void operator()( size_t idxA , size_t idxB )
        {
            Vec3 &  velA    = mParticles[ idxA ].mVelocity ;

            ASSERT( ! IsNan( velA ) && ! IsInf( velA ) ) ;
            ASSERT( ! IsInf( velA.Mag2() ) ) ;
            ASSERT( velA.Mag2() < 1e8f ) ;

            const Vec3  sep     = mParticles[ idxA ].mPosition - mParticles[ idxB ].mPosition ;
            const float dist2   = sep.Mag2() ;
            if( dist2 < mInflRad2 )
            {   // Particles are near enough to exchange velocity.
                const float     dist    = fsqrtf( dist2 ) ;
                const Vec3      sepDir  = sep / dist ;
                Vec3 &          velB    = mParticles[ idxB ].mVelocity ;
                const Vec3      velDiff = velA - velB ;
                const float     velSep  = velDiff * sepDir ;
                if( velSep < 0.0f )
                {   // Particles are approaching.
                    const float infl    = 1.0f - dist / mInfluenceRadius ;
#if 0
                    const float viscImp = infl * ( velSep * mTangentialViscosityGain + velSep * velSep * mRadialViscosityGain ) ;
                    const Vec3  imp     = sepDir * viscImp ;
                    ASSERT( ! IsNan( imp ) && ! IsInf( imp ) ) ;
                    velA += imp ;
                    velB -= imp ;
#else // MJG
                    const float velSepA         = velA * sepDir ;                           // Component of velocity of particle A along separation direction.
                    const float velSepB         = velB * sepDir ;                           // Component of velocity of particle B along separation direction.
                    const float velSepTarget    = ( velSepA + velSepB ) * 0.5f ;            // Component of target velocity along separation direction.
                    const float diffSepA        = velSepTarget - velSepA ;                  // Difference between A's velocity-along-separation and target.
                    const float changeSepA      = mRadialViscosityGain * diffSepA * infl ;  // Amount of velocity change to apply.
                    const Vec3  changeA         = changeSepA * sepDir ;                     // Velocity change to apply.
                    velA += changeA ;                                                       // Apply velocity change to A.
                    velB -= changeA ;                                                       // Apply commensurate change to B.
#endif
                    ASSERT( ! IsNan( velA ) && ! IsInf( velA ) ) ;
                    ASSERT( ! IsNan( velB ) && ! IsInf( velB ) ) ;
                    ASSERT( velA.Mag2() < 1e8f ) ;
                    ASSERT( velB.Mag2() < 1e8f ) ;
                }
            }
        }

    private:
        VelocityDiffuser & operator=( const VelocityDiffuser & ) ; // Prevent assignment

        VECTOR< Vorton > &  mParticles              ; ///< Fluid particles.
        const float         mInfluenceRadius        ; ///< Range of influence each particle has on others.
        const float         mInflRad2               ; ///< Square of mInfluenceRadius.
        const float         mTimeStep               ; ///< Amount of virtual time by which to advance simulation.
        const float         mTangentialViscosityGain; ///< Controls portion of tangential velocity to keep.
        const float         mRadialViscosityGain    ; ///< Controls portion of radial velocity to keep.
} ;




#if USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH

/** Compute fluid particle velocity diffusion and dissipation at each SPH particle using a uniform grid spatial partition, for a subset of the domain.
*/
static void DiffuseAndDissipateVelocitySph_Grid_Slice( VECTOR< Vorton > & particles , const float timeStep , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid , size_t izStart , size_t izEnd , VortonSim::PhaseE phase )
{
    const size_t numPcls = particles.size() ;
    if( 0 == numPcls )
    {
        return ;
    }

    ASSERT( ! pclIndicesGrid.Empty() ) ; // Spatial partition must be populated.

    const float     pclRad          = particles[ 0 ].GetRadius() ;
    const float     inflRad         = 3.0f * pclRad ;

    ASSERT( inflRad <= MIN3( pclIndicesGrid.GetCellSpacing().x , pclIndicesGrid.GetCellSpacing().y , pclIndicesGrid.GetCellSpacing().z ) ) ;

    static float    radialViscosity = 30.0f ;
    static float    viscousGain     = 0.9999f ;
    const  float    speedLimit2     = 2.0f * stiffness ;
    static float    speedLimit      = sqrt( stiffness ) ;

    VelocityDiffuser diffuseVelocity( particles , inflRad , timeStep , radialViscosity ) ;

    const int &     nx      = pclIndicesGrid.GetNumPoints( 0 ) ;
    const size_t    nxm1    = nx - 1 ;
    const int &     ny      = pclIndicesGrid.GetNumPoints( 1 ) ;
    const size_t    nym1    = ny - 1 ;
    const int       nxy     = nx * ny ;

    // Set up increment and offset for alternating odd-even access.
    const size_t   zInc     =   VortonSim::PHASE_BOTH == phase   ? 1 : 2 ;  ///< Amount by which to increment z index.
    const size_t   flipper  = ( VortonSim::PHASE_ODD  == phase ) ? 1 : 0 ;
    const size_t   izShift  = ( izStart & 1 ) ^ flipper ; // Flip lowest bit on odd phase.

    DEBUG_ONLY( const size_t & nz      = pclIndicesGrid.GetNumPoints( 2 ) ) ;
    DEBUG_ONLY( const size_t   nzm1    = nz - 1 ) ;
    ASSERT( nz   > 0  ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( nzm1 < nz ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( izStart <  izEnd ) ;
    ASSERT( izEnd   <= nzm1  ) ;

    // Note that the cell-neighborhood loop visits only half the neighboring
    // cells. That is because each particle visitation is symmetric so there is
    // no need to visit all neighboring cells.
    const int neighborCellOffsets[] =
    {   // Offsets to neighboring cells whose indices exceed this one:
           1            // + , 0 , 0 ( 1)
        , -1 + nx       // - , + , 0 ( 2)
        ,    + nx       // 0 , + , 0 ( 3)
        ,  1 + nx       // + , + , 0 ( 4)
        , -1 - nx + nxy // - , - , + ( 5)
        ,    - nx + nxy // 0 , - , + ( 6)
        ,  1 - nx + nxy // + , - , + ( 7)
        , -1      + nxy // - , 0 , + ( 8)
        ,         + nxy // 0 , 0 , + ( 9)
        ,  1      + nxy // + , 0 , + (10)
        , -1 + nx + nxy // - , 0 , + (11)
        ,      nx + nxy // 0 , + , + (12)
        ,  1 + nx + nxy // + , + , + (13)
    } ;
    static const size_t numNeighborCells = sizeof( neighborCellOffsets ) / sizeof( neighborCellOffsets[ 0 ] ) ;

    const size_t gridCapacity = pclIndicesGrid.GetGridCapacity() ;

    size_t idx[3] ;
    for( idx[2] = izStart + izShift ; idx[2] < izEnd ; idx[2] += zInc )
    {   // For all grid cells along z...
        const size_t offsetZ0 = idx[2] * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all grid cells along y...
            const size_t offsetY0Z0 = idx[1] * nx + offsetZ0 ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all grid cells along x...
                const size_t offsetX0Y0Z0 = idx[0] + offsetY0Z0 ;
                const size_t numInCurrentCell = pclIndicesGrid[ offsetX0Y0Z0 ].Size() ;
                for( unsigned ivHere = 0 ; ivHere < numInCurrentCell ; ++ ivHere )
                {   // For each vorton in this gridcell...
                    const unsigned &    rVortIdxHere    = pclIndicesGrid[ offsetX0Y0Z0 ][ ivHere ] ;

                    // Aggregate density between particles that share a cell.
                    // Notice that the loop only visits vortons with indices
                    // following the "here" particle.  That is because each
                    // visitation symmetrically modifies both vortons in the
                    // pair.  If "here" and "there" is visited, then "there"
                    // and "here" should not also be visited, because it
                    // would be redundant.
                    for( unsigned ivThere = ivHere + 1 ; ivThere < numInCurrentCell ; ++ ivThere )
                    {   // For each OTHER particle within this same cell...
                        const unsigned &    rVortIdxThere   = pclIndicesGrid[ offsetX0Y0Z0 ][ ivThere ] ;
                        diffuseVelocity( rVortIdxHere , rVortIdxThere ) ;
                    }

                    // Aggregate density between particles in adjacent cells.
                    for( size_t idxNeighborCell = 0 ; idxNeighborCell < numNeighborCells ; ++ idxNeighborCell )
                    {   // For each cell in neighborhood...
                        const size_t cellOffset = offsetX0Y0Z0 + neighborCellOffsets[ idxNeighborCell ] ; // offset of adjacent cell
                        if( cellOffset >= gridCapacity ) break ; // Would-be neighbor is out-of-bounds.
                        const VECTOR< unsigned > & cell = pclIndicesGrid[ cellOffset ] ;
                        for( unsigned ivThere = 0 ; ivThere < cell.Size() ; ++ ivThere )
                        {   // For each vorton in the visited cell...
                            const unsigned &    rVortIdxThere   = cell[ ivThere ] ;
                            diffuseVelocity( rVortIdxHere , rVortIdxThere ) ;
                        }
                    }

                    // Dissipate velocity.
                    Vec3 & vel = particles[ rVortIdxHere ].mVelocity ;
                    vel *= viscousGain ;

                    // Apply speed limit.
                    const float velMag2 =  vel.Mag2() ;
                    if( velMag2 > speedLimit2 )
                    {   // Particle is going too fast.
                        const float reduction = fsqrtf( speedLimit2 / velMag2 ) ;
                        vel *= reduction ;
                    }

                    ASSERT( ! IsNan( vel ) && ! IsInf( vel ) ) ;
                    ASSERT( ! IsInf( vel.Mag2() ) ) ;
                    ASSERT( vel.Mag2() < 1e8f ) ;
                }
            }
        }
    }
}




/** Compute fluid particle velocity diffusion and dissipation at each SPH particle using a uniform grid spatial partition.

    This is an O(N*k) operation where N is the number of particles and k is the
    average number of particles in the neighborhood of one of the N particles.
*/
static void DiffuseAndDissipateVelocitySph_Grid( VECTOR< Vorton > & particles , const float timeStep , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid )
{
#if ENABLE_FLUID_BODY_SIMULATION
    PERF_BLOCK( DiffuseAndDissipateVelocitySph_Grid ) ;

    const unsigned & nz     = pclIndicesGrid.GetNumPoints( 2 ) ;
    const unsigned   nzm1   = nz - 1 ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , nzm1 / gNumberOfProcessors ) ;
        // Compute particle number density using threading building blocks.
        // Alternate between even and odd z-slices to avoid multiple threads accessing the same particles simultaneously.
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , 
            SphSim_DiffuseAndDissipateVelocity_TBB( particles , timeStep , pclIndicesGrid , VortonSim::PHASE_EVEN ) ) ;
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) ,
            SphSim_DiffuseAndDissipateVelocity_TBB( particles , timeStep , pclIndicesGrid , VortonSim::PHASE_ODD  ) ) ;
    #else
        DiffuseAndDissipateVelocitySph_Grid_Slice( particles , timeStep , pclIndicesGrid , 0 , nzm1 , VortonSim::PHASE_BOTH ) ;
    #endif
#endif
}



#else

static void DiffuseAndDissipateVelocitySph_Direct( VECTOR< Vorton > & particles , float timeStep )
{
    PERF_BLOCK( DiffuseAndDissipateVelocitySph_Direct ) ;

    const size_t numPcls = particles.size() ;
    if( 0 == numPcls )
    {
        return ;
    }

    const float     pclRad          = particles[ 0 ].GetRadius() ;
    const float     inflRad         = 3.0f * pclRad ;
    static float    radialViscosity = 10.0f ;

    static float    viscousGain     = 0.99f ;
    const  float    speedLimit2     = 2.0f * stiffness ;
    static float    speedLimit      = sqrt( stiffness ) ;

    VelocityDiffuser diffuseVelocity( particles , inflRad , timeStep , radialViscosity ) ;

    for( size_t idxA = 0 ; idxA < numPcls ; ++ idxA )
    {
        for( size_t idxB = idxA + 1 ; idxB < numPcls ; ++ idxB )
        {   // For each OTHER particle following A...
            diffuseVelocity( idxA , idxB ) ;
        }

        // Dissipate velocity.
        Vec3 & vel = particles[ idxA ].mVelocity ;
        vel *= viscousGain ;

        // Apply speed limit.
        const float velMag2 =  vel.Mag2() ;
        if( velMag2 > speedLimit2 )
        {   // Particle is going too fast.
            const float reduction = fsqrtf( speedLimit2 / velMag2 ) ;
            vel *= reduction ;
        }
    }
}

#endif // USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH




/** Update smoothed particle hydrodynamics fluid simulation to next time.

    \param timeStep     Amount of time by which to advance simulation.

    \param uFrame       Frame counter, used to generate diagnostic files.

    \note FindBoundingBox and UpdateBoundingBox must have been called
            before this routine starts, for each timestep.

    Effectively this routine generates the velocity field and prepares the
    simulation for the next step.  It does NOT, however, actually advect
    vortons.  The Evolve particle operation does that, and it is up to the
    caller of this routine to invoke that operation. This separation of
    processes facilitates adding other motion to the vortons which are not due
    to the fluid simulation.

*/
void VortonSim::UpdateSmoothedParticleHydrodynamics( float timeStep , unsigned uFrame )
{
    PERF_BLOCK( VortonSim__UpdateSph ) ;

    ASSERT( ( FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS == mFluidSimTechnique ) || ( FLUID_SIM_VPM_SPH_HYBRID == mFluidSimTechnique ) ) ;

    if( mVortons->Empty() )
    {
        return ;
    }

    VECTOR< Vec3 >              accelerationOfPcls ;

#if USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH
    const float pclRad  = ( * mVortons )[ 0 ].GetRadius() ;
    const float inflRad = influenceRadiusScale * pclRad ;   // Make grid cell big enough to include all possible neighbors.

    UniformGrid< VECTOR< unsigned > >   vortonIndicesGrid  ;   ///< Spatial partition of indices into mVortons.
    PartitionVortons( timeStep , uFrame , vortonIndicesGrid , inflRad ) ;
#endif

    UNUSED_PARAM( uFrame ) ;

#if USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH
    ComputeSphDensityAtParticles_Grid( mFluidDensitiesAtPcls , * mVortons , vortonIndicesGrid ) ;
#else
    ComputeSphDensityAtParticles_Direct( mFluidDensitiesAtPcls , * mVortons ) ;
#endif

#if USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH
    ComputeSphPressureGradientAcceleration_Grid( accelerationOfPcls , mFluidDensitiesAtPcls , * mVortons , vortonIndicesGrid ) ;
#else
    ComputeSphPressureGradientForce_Direct( accelerationOfPcls , mFluidDensitiesAtPcls , * mVortons ) ;
#endif

    ApplyBodyForce( * mVortons , accelerationOfPcls , mAmbientDensity , timeStep , 0 , mVortons->Size() ) ;

#if USE_UNIFORM_GRID_SPATIAL_PARTITION_FOR_SPH
    DiffuseAndDissipateVelocitySph_Grid( * mVortons , timeStep , vortonIndicesGrid ) ;
#else
    DiffuseAndDissipateVelocitySph_Direct( * mVortons , timeStep ) ;
#endif

    // TODO: FIXME: Note that by populating the velocity grid here, instead
    //              of after fluid-body interaction, causes the velocity
    //              grid to lack the influence boundaries have on velocity.
    //              This causes severe artifacts in tracer particle
    //              advection.  This should therefore run after solving
    //              boundary conditions.
    PopulateVelocityGrid( mVelGrid , reinterpret_cast< VECTOR< Particle > & >( * mVortons ) ) ;
}

#endif // USE_SMOOTHED_PARTICLE_HYDRODYNAMICS




////////////////////////////////////////////////////////////////////////////////
// This COMPUTE_PRESSURE_GRADIENT code was an experimental work-around for
// issues in the vorton simulation, as exposed in Part 14. Do not spend time
// trying to unravel its mysteries.  Expect it to disappear soon.
////////////////////////////////////////////////////////////////////////////////




#if COMPUTE_PRESSURE_GRADIENT

void ComputePressureGradient( UniformGridGeometry gridTemplate , UniformGrid< Vec3 > & pressureGradientGrid , VECTOR< Impulsion::PhysicalObject * > & physicalObjects , const float vortonRadius )
{   // Note: This routine is nonsense.  Do not use it.  It was a half-baked idea that did not work out.
    PERF_BLOCK( ComputePressureGradient ) ;

    UniformGrid< float > pressureGrid( gridTemplate ) ;
    pressureGrid.Init( 1.0f ) ; // Reserve memory for and initialize pressure grid.
    FluidBodySim::PoisonPressure( pressureGrid , physicalObjects , vortonRadius ) ;

    pressureGradientGrid.Clear() ;
    pressureGradientGrid.CopyShape( gridTemplate ) ;
    pressureGradientGrid.Init() ;

    ComputeGradient( pressureGradientGrid , pressureGrid ) ;
}

#endif




////////////////////////////////////////////////////////////////////////////////
// This COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS code was an experimental
// work-around for issues in the vorton simulation, as exposed in Part 14. Do
// not spend time trying to unravel its mysteries.  Expect it to disappear soon.
////////////////////////////////////////////////////////////////////////////////




#if COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS

/** Compute density gradient directly from vortons (not using a grid of density values).

    \see ComputeDensityGradientFromVortons
*/
void ComputeDensityGradientFromVortons_Slice( VECTOR< Vec3 > & densityGradients , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid , const VECTOR< Vorton > & vortons , size_t izStart , size_t izEnd )
{
    if( pclIndicesGrid.Empty() )
    {   // No vortons.
        return ;
    }

    const size_t   & nx     = pclIndicesGrid.GetNumPoints( 0 ) ;
    const size_t   & ny     = pclIndicesGrid.GetNumPoints( 1 ) ;
    const size_t   & nz     = pclIndicesGrid.GetNumPoints( 2 ) ;
    ASSERT( izStart <  izEnd ) ;
    ASSERT( izEnd   <= nz    ) ;

    densityGradients.Resize( vortons.Size() ) ;

#if defined( _DEBUG )
    VECTOR< bool > elementsAssigned ;
    elementsAssigned.resize( vortons.size() , false ) ;
#endif

    size_t idx[3] ;
    for( idx[2] = izStart ; idx[2] < izEnd ; ++ idx[2] )
    for( idx[1] = 0       ; idx[1] < ny    ; ++ idx[1] )
    for( idx[0] = 0       ; idx[0] < nx    ; ++ idx[0] )
    {   // For all gridpoints...
        const size_t offsetHere = pclIndicesGrid.OffsetFromIndices( idx ) ;
        const size_t numVortonsHere = pclIndicesGrid[ offsetHere ].Size() ;
        for( unsigned ivHere = 0 ; ivHere < numVortonsHere ; ++ ivHere )
        {   // For each vorton in here gridcell...
            const unsigned  vortIdxHere     = pclIndicesGrid[ offsetHere ][ ivHere ] ;
            const Vorton &  vortonHere      = vortons[ vortIdxHere ] ;
            const Vec3 &    posHere         = vortonHere.mPosition ;
            const float &   densHere        = vortonHere.mDensity ;
            float           weightSum       = 0.0f ;

            Vec3 & rDensGrad = densityGradients[ vortIdxHere ] ;
            rDensGrad = Vec3( 0.0f , 0.0f , 0.0f ) ;

            DEBUG_ONLY( elementsAssigned[ vortIdxHere ] = true ) ;

            size_t jdx[3] ;
            for( jdx[2] = idx[2] - 1 ; jdx[2] < idx[2] + 1 ; ++ jdx[2] )
            for( jdx[1] = idx[1] - 1 ; jdx[1] < idx[1] + 1 ; ++ jdx[1] )
            for( jdx[0] = idx[0] - 1 ; jdx[0] < idx[0] + 1 ; ++ jdx[0] )
            {   // For each cell at or adjacent to that containing current vorton...
                if( ( jdx[0] < nx ) && ( jdx[1] < ny ) && ( jdx[2] < nz ) )
                {
                    const size_t offsetThere        = pclIndicesGrid.OffsetFromIndices( jdx ) ;
                    const size_t numVortonsThere    = pclIndicesGrid[ offsetThere ].Size() ;
                    for( unsigned ivThere = 0 ; ivThere < numVortonsThere ; ++ ivThere )
                    {   // For each vorton in there gridcell...
                        const unsigned  vortIdxThere    = pclIndicesGrid[ offsetThere ][ ivThere ] ;
                        if( vortIdxHere != vortIdxThere )
                        {   // Here and There vortices differ.
                            // NOTE: This routine uses too naive a formulation
                            //      for spatial gradient.  The failure is most
                            //      obvious in a couple of cases: (1) If all
                            //      neighboring vortons have the same density,
                            //      then the density gradient will be zero,
                            //      even for overlapping particles where the
                            //      density would be higher in that region.
                            //      (2) If vortons are on the boundary of a
                            //      region/blob then this formula does not
                            //      compute any derivatives in that direction,
                            //      which in many cases would be the direction
                            //      with the most important contribution to the
                            //      gradient.
                            const Vorton &  vortonThere     = vortons[ vortIdxThere ] ;
                            const Vec3 &    posThere        = vortonThere.mPosition ;
                            const Vec3      separation      = posHere - posThere ;
                            const float     sepLen2         = separation.Mag2() ;
                            const float &   densThere       = vortonHere.mDensity ;
                            const float     densDiff        = densHere - densThere ;
                            const float     weight          = 1.0f / sepLen2 ;  // There are many choices for weight.  This one is singular and non-compact.  Probably worst possible choice.  But it's cheap.
                            const Vec3      densGradTerm    = densDiff * separation / sepLen2 ;
                            rDensGrad += densGradTerm * weight ;
                            ASSERT( ! IsNan( rDensGrad ) && ! IsInf( rDensGrad ) ) ;
                            weightSum += weight ;
                        }
                    }
                }
            }
            ASSERT( weightSum >= 0.0f ) ;
            if( weightSum > 0.0f )
            {
                rDensGrad /= weightSum ;
            }
            ASSERT( ! IsNan( rDensGrad ) && ! IsInf( rDensGrad ) ) ;
        }
    }

#if defined( _DEBUG )
    ASSERT( elementsAssigned.Size() == vortons.Size() ) ;
    for( size_t iElem = 0 ; iElem < elementsAssigned.Size() ; ++ iElem )
    {
        ASSERT( elementsAssigned[ iElem ] ) ;
    }
#endif
}




/** 

    \see ComputeDensityGradientFromVortons_Slice

*/
void ComputeDensityGradientFromVortons( VECTOR< Vec3 > & densityGradient , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid , const VECTOR< Vorton > & vortons )
{
    PERF_BLOCK( ComputeDensityGradientFromVortons ) ;

    const unsigned & nz     = pclIndicesGrid.GetNumPoints( 2 ) ;

    #if USE_TBB && 0
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( 1 , nz / gNumberOfProcessors ) ;
        // Compute ... using threading building blocks.
        parallel_for( tbb::blocked_range<size_t>( 0 , nz , grainSize ) , VortonSim_ComputeDensityGradientFromParticles_TBB( densityGradient , pclIndicesGrid , vortons ) ) ;
    #else
        ComputeDensityGradientFromVortons_Slice( densityGradient , pclIndicesGrid , vortons , 0 , nz ) ;
    #endif
}

#endif // COMPUTE_DENSITY_GRADIENT_AT_AND_WITH_VORTONS




////////////////////////////////////////////////////////////////////////////////
// This REDUCE_CONVERGENCE code was a temporary work-around for issues in the
// vorton simulation introduced in Part 14.  It uses an ad-hoc divergence
// reduction scheme.
////////////////////////////////////////////////////////////////////////////////




#if REDUCE_CONVERGENCE

/** Push particles apart.

    \param  displacementMax Maximum amount of unacceptable particle overlap encountered.

    \see ReduceDivergence, ReduceDivergence_Direct.
*/
static inline void PushParticles( Vorton & rVortonHere , Vorton & rVortonThere , float & displacementMax )
{
    ASSERT( rVortonHere.IsAlive() ) ;
    ASSERT( rVortonThere.IsAlive() ) ;
    const Vec3      separation      = rVortonHere.mPosition - rVortonThere.mPosition ;
    const float     dist2           = separation.Mag2() ;
    const float     radiusSum       = ( rVortonHere.mSize + rVortonThere.mSize ) * 0.5f ; // 1/2 because size=2*radius
    // Note, this should allow for spheres to be closer, such that average
    // density in grid could equal average density of particles.  Highest
    // density of packed spheres has 12 adjacent with average density
    // pi/sqrt(18)~=0.7405.  To make that have an average density of 1 would
    // entail letting spheres overlap with pow(0.7405,1/3)~=0.9048 of their radius.
    // (That is nearly tantamount to shrinking the spheres but retaining their
    // mass, while retaining a closest-packing.) The lowest density of packed
    // spheres has average density V_sphere/V_cube=4pi/24~=0.5236 which
    // corresponds to an overlap of 0.8060.
    const float     closestDist     = radiusSum ;
    const float     closestDist2    = Pow2( closestDist ) ;
    if( dist2 < closestDist2 )
    {   // Particles overlap.
        // Push them apart so they no longer overlap excessively.
        const float displacement2       = closestDist2 - dist2 ;
        const float displacementHalf    = 0.5f * fsqrtf( displacement2 ) ;
        const Vec3  displacementDir     = separation.GetDir() ;
        static const float gain = 0.125f ;
        const Vec3  displacementEach    = gain * displacementHalf * displacementDir ;

        rVortonHere.mPosition  += displacementEach ;
        rVortonThere.mPosition -= displacementEach ;
        displacementMax = Max2( displacementHalf , displacementMax ) ;

    #if ENABLE_DISPLACEMENT_TRACKING
       rVortonHere.mDisplacement  += displacementEach ; // Track vorton displacement to transfer it to tracers.
       rVortonThere.mDisplacement += displacementEach ; // Track vorton displacement to transfer it to tracers.
    #endif
    }
    displacementMax = Max2( 0.0f , displacementMax ) ;
}




/** Push apart particles that are too close, visiting pairs of particles that are near each other.

    \note   This invalidates pclIndicesGrid in the sense that this routine
            pushes particles around and potentially out of the grid cell in
            which they started.  This routine uses pclIndicesGrid to partition
            particles and to find neighbors, and even when they get pushed
            around, the neighbor relationships likely remain the same, so in
            that sense, the invalidity of pclIndicesGrid is not crucial;
            memory will not be accessed out-of-bounds due to how particles
            are pushed out of their grid cells.
            Due to this property, it is possible that this routine could find
            zero displacementMax even though some particles are too close to
            each other. ReduceDivergence_Direct does not have this problem since
            it indiscriminately visits every particle pair, including pairs
            which are very far apart.

    \see PushParticles, ReduceDivergence, ReduceDivergence_Direct.
*/
static void ReduceDivergence_Grid_Slice( VECTOR< Vorton > * vortons , float & displacementMax , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid , size_t izStart , size_t izEnd , VortonSim::PhaseE phase )
{
    if( pclIndicesGrid.Empty() )
    {   // No vortons.
        return ;
    }

    // Exchange vorticity with nearest neighbors

    const int &     nx      = pclIndicesGrid.GetNumPoints( 0 ) ;
    const size_t    nxm1    = nx - 1 ;
    const int &     ny      = pclIndicesGrid.GetNumPoints( 1 ) ;
    const size_t    nym1    = ny - 1 ;
    const int       nxy     = nx * ny ;

    // Set up increment and offset for alternating odd-even access.
    const size_t   zInc     =   VortonSim::PHASE_BOTH == phase   ? 1 : 2 ;  ///< Amount by which to increment z index.
    const size_t   flipper  = ( VortonSim::PHASE_ODD  == phase ) ? 1 : 0 ;
    const size_t   izShift  = ( izStart & 1 ) ^ flipper ; // Flip lowest bit on odd phase.

    DEBUG_ONLY( const size_t & nz      = pclIndicesGrid.GetNumPoints( 2 ) ) ;
    DEBUG_ONLY( const size_t   nzm1    = nz - 1 ) ;
    ASSERT( nz   > 0  ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( nzm1 < nz ) ; // If nz is zero then unsigned nzm1 will be larger.
    ASSERT( izStart <  izEnd ) ;
    ASSERT( izEnd   <= nzm1  ) ;

    // Note that the cell-neighborhood loop visits only half the neighboring
    // cells. That is because each particle visitation is symmetric so there is
    // no need to visit all neighboring cells.
    const int neighborCellOffsets[] =
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

    const size_t gridCapacity = pclIndicesGrid.GetGridCapacity() ;

    displacementMax = - FLT_MAX ;

    size_t idx[3] ;
    for( idx[2] = izStart + izShift ; idx[2] < izEnd ; idx[2] += zInc )
    {   // For all grid cells along z...
        const size_t offsetZ0 =   idx[2]       * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all grid cells along y...
            const size_t offsetY0Z0 =   idx[1]       * nx + offsetZ0 ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all grid cells along x...
                const size_t offsetX0Y0Z0 = idx[0]     + offsetY0Z0 ;
                const size_t numInCurrentCell = pclIndicesGrid[ offsetX0Y0Z0 ].Size() ;
                for( unsigned ivHere = 0 ; ivHere < numInCurrentCell ; ++ ivHere )
                {   // For each vorton in this gridcell...
                    const unsigned &    rVortIdxHere    = pclIndicesGrid[ offsetX0Y0Z0 ][ ivHere ] ;
                    Vorton &            rVortonHere     = (*vortons)[ rVortIdxHere ] ;
                    ASSERT( rVortonHere.IsAlive() ) ;

                    // Push particles that share a cell.
                    // Notice that the loop only visits vortons with indices
                    // following the "here" particle.  That is because each
                    // visitation symmetrically modifies both vortons in the
                    // pair.  If "here" and "there" is visited, then "there" and
                    // "here" should not also be visited, because it would be
                    // redundant.
                    for( unsigned ivThere = ivHere + 1 ; ivThere < numInCurrentCell ; ++ ivThere )
                    {   // For each OTHER particle within this same cell...
                        const unsigned &    rVortIdxThere   = pclIndicesGrid[ offsetX0Y0Z0 ][ ivThere ] ;
                        Vorton &            rVortonThere    = (*vortons)[ rVortIdxThere ] ;
                        PushParticles( rVortonHere , rVortonThere , displacementMax ) ;
                    }

                    // Push particles in adjacent cells.
                    // Note that each of the following sections visits only
                    // cells in the positive direction.  That is because each
                    // visitation is symmetric so there is no need to visit
                    // particles in cells in the negative direction.
                    for( size_t idxNeighborCell = 0 ; idxNeighborCell < numNeighborCells ; ++ idxNeighborCell )
                    {   // For each cell in neighborhood...
                        const size_t cellOffset = offsetX0Y0Z0 + neighborCellOffsets[ idxNeighborCell ] ; // offset of adjacent cell
                        if( cellOffset >= gridCapacity ) break ; // Would-be neighbor is out-of-bounds.
                        const VECTOR< unsigned > & cell = pclIndicesGrid[ cellOffset ] ;
                        for( unsigned ivThere = 0 ; ivThere < cell.Size() ; ++ ivThere )
                        {   // For each vorton in the visited cell...
                            const unsigned &    rVortIdxThere   = cell[ ivThere ] ;
                            Vorton &            rVortonThere    = (*vortons)[ rVortIdxThere ] ;
                            PushParticles( rVortonHere , rVortonThere , displacementMax ) ;
                        }
                    }
                }
            }
        }
    }
}




#if 0

/** Push apart particles that are too close, visiting every pair of particles.
*/
static float ReduceDivergence_Direct( VECTOR< Vorton > * vortons )
{
    PERF_BLOCK( ReduceDivergence_Direct ) ;

    float displacementMax = - FLT_MAX ;
    const size_t numParticles = vortons->Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {   // For each particle...
        Vorton & vortonHere = (*vortons)[ iPcl ] ;
        for( size_t ivThere = iPcl + 1 ; ivThere < numParticles ; ++ ivThere )
        {   // For each other particle...
            Vorton & vortonThere = (*vortons)[ ivThere ] ;
            PushParticles( vortonHere , vortonThere , displacementMax ) ;
        }
    }
    return displacementMax ;
}

#endif




/** Push apart particles that are too close.

    \note   This invalidates mesh bounding box and pclIndicesGrid. 
*/
float VortonSim::ReduceDivergence( VECTOR< Vorton > * vortons , const UniformGrid< VECTOR< unsigned > > & pclIndicesGrid )
{
#if ENABLE_FLUID_BODY_SIMULATION

    PERF_BLOCK( VortonSim__ReduceDivergence ) ;

    float displacementMax = FLT_MAX ;

    const unsigned & nz   = pclIndicesGrid.GetNumPoints( 2 ) ;
    const unsigned   nzm1 = nz - 1 ;

    static const float displacementThreshold = FLT_EPSILON ;
    static const int   maxNumIters           = 256 ;

    {
    #if USE_TBB && 0 // TBB version not implemented.
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  Max2( size_t( 1 ) , nzm1 / gNumberOfProcessors ) ;
        // Compute vorticity diffusion using threading building blocks.
        // Alternate between even and odd z-slices to avoid multiple threads accessing the same vortons simultaneously.
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , VortonSim_ReduceDivergence_TBB( this , pclIndicesGrid , VortonSim::PHASE_EVEN ) ) ;
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , VortonSim_ReduceDivergence_TBB( this , pclIndicesGrid , VortonSim::PHASE_ODD  ) ) ;
    #else
        ReduceDivergence_Grid_Slice( vortons , displacementMax , pclIndicesGrid , 0 , nzm1 , VortonSim::PHASE_BOTH ) ;
    #endif
    }

    //if( 0.0f == displacementMax )
    //{   // Approximate routine yielded zero displacement.
    //    return ReduceDivergence_Direct( vortons ) ;
    //}

    return displacementMax ;

#endif
}

#endif // REDUCE_CONVERGENCE
