/** \file particleSystemConfiguration.cpp

    \brief Routines to configure particle systems.

    These structs and routines stand proxy for defining particle systems using some definitions from a configuration file.

*/
#include "particleSystemConfiguration.h"

#include "Particles/Operation/pclOpFindBoundingBox.h"
#include "Particles/Operation/pclOpWind.h"
#include "Particles/Operation/pclOpAdvect.h"
#include "Particles/Operation/pclOpEvolve.h"
#include "Particles/Operation/pclOpAssignScalarFromGrid.h"
#include "Particles/Operation/pclOpPopulateVelocityGrid.h"
#include "Particles/Operation/pclOpEmit.h"
#include "Particles/Operation/pclOpKillAge.h"
#include "Particles/particleGroup.h"
#include "Particles/particleSystemManager.h"

#include "VortonFluid/pclOpVortonSim.h"

#include "FluidBodySim/fluidBodySim.h"
#include "FluidBodySim/pclOpFluidBodyInteraction.h"

#include <Core/Performance/perfBlock.h>

static const Vec3 sGravityDirection( 0.0f , 0.0f , -1.0f ) ; ///< Direction of acceleration due to gravity
static const Vec3 sGravityAcceleration( 10.0f * sGravityDirection ) ; ///< Acceleration due to gravity

static const size_t tracerOffsetToDensity       = offsetof( Particle , mDensity         ) ;
#if ENABLE_FIRE
static const size_t tracerOffsetToFuelFraction  = offsetof( Particle , mFuelFraction    ) ;
static const size_t tracerOffsetToFlameFraction = offsetof( Particle , mFlameFraction   ) ;
static const size_t tracerOffsetToSmokeFraction = offsetof( Particle , mSmokeFraction   ) ;
#endif




/** Create fluid vorton particle systems.
*/
static ParticleGroup * CreateVortonParticleSystem( FluidVortonPclGrpInfo & vortonPclGrpInfo , const float viscosity , const float ambientFluidDensity , const float fluidSpecificHeatCapacity , const Vec3 & gravityAcceleration , VECTOR< Impulsion::PhysicalObject * > & physicalObjects )
{
    PERF_BLOCK( CreateVortonParticleSystem ) ;

    vortonPclGrpInfo.mParticleGroup = new ParticleGroup() ;

    static const int    killAgeMax  = 90    ;

    vortonPclGrpInfo.mPclOpKillAge          = new PclOpKillAge() ;
    vortonPclGrpInfo.mPclOpKillAge->mAgeMax = killAgeMax ;
    vortonPclGrpInfo.mParticleGroup->PushBack( vortonPclGrpInfo.mPclOpKillAge ) ;

    Particle emitterTemplate , emitterSpread ;
    emitterTemplate.mPosition           = Vec3( -0.5f , 0.0f , 0.0f ) ;
    emitterTemplate.mVelocity           = Vec3( 2.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
    emitterTemplate.mSize               = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
    emitterTemplate.mDensity            = ambientFluidDensity ;
#if ENABLE_FIRE
    emitterTemplate.mFuelFraction       = 0.0f ;
    emitterTemplate.mFlameFraction      = 0.0f ;
    emitterTemplate.mSmokeFraction      = 1.0f ;
#endif
    emitterSpread.mPosition             = Vec3( emitterTemplate.mVelocity.x / 30.0f , 0.5f , 0.5f ) ;

    vortonPclGrpInfo.mPclOpEmit = new PclOpEmit() ;
    vortonPclGrpInfo.mPclOpEmit->mTemplate                  = emitterTemplate ;
    vortonPclGrpInfo.mPclOpEmit->mTemplate.mAngularVelocity = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
    vortonPclGrpInfo.mPclOpEmit->mSpread                    = emitterSpread ;
    vortonPclGrpInfo.mPclOpEmit->mSpread.mDensity           = 0.0f ; // No variance for density.
    vortonPclGrpInfo.mPclOpEmit->mEmitRate                  = 0.0f ; // Set by InitialConditions
    vortonPclGrpInfo.mParticleGroup->PushBack( vortonPclGrpInfo.mPclOpEmit ) ;

    // PclOpVortonSim must run after finding bounding box for tracers
    // because VortonSim populates the velocity grid, which relies on
    // the grid dimensions being set from the bounding box.
    // VortonSim must rum before assign-velocity-from-field because that
    // accesses the velocity grid populated by PclOpVortonSim.
    vortonPclGrpInfo.mPclOpVortonSim = new PclOpVortonSim() ;
    //vortonPclGrpInfo.mPclOpVortonSim->mMinCorner = & mPclOpFindBoundingBox.GetMinCorner() ;
    //vortonPclGrpInfo.mPclOpVortonSim->mMaxCorner = & mPclOpFindBoundingBox.GetMaxCorner() ;
    vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetVortons( reinterpret_cast< VECTOR< Vorton > * >( & vortonPclGrpInfo.mParticleGroup->GetParticles() ) ) ;
    vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetViscosity( viscosity ) ;
    vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetAmbientDensity( ambientFluidDensity ) ;
    vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetGravitationalAcceleration( gravityAcceleration ) ;
    vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSpecificHeatCapacity( fluidSpecificHeatCapacity ) ;

    vortonPclGrpInfo.mParticleGroup->PushBack( vortonPclGrpInfo.mPclOpVortonSim ) ;

    // Wind runs after assign-velocity-from-field, because that overwrites
    // velocity, and Wind adds to that velocity.
    vortonPclGrpInfo.mPclOpWind = new PclOpWind() ;
    // mVortonPclGrpInfo.mPclOpWind->mWind        = Vec3( 0.0f , 0.0f , 0.0f ) ; Set in InitialConditions
    vortonPclGrpInfo.mPclOpWind->mWindWeight = 1.0f ;
    vortonPclGrpInfo.mPclOpWind->mSrcWeight  = 1.0f ;
    vortonPclGrpInfo.mParticleGroup->PushBack( vortonPclGrpInfo.mPclOpWind ) ;

    // Evolve runs after all operations that set velocity.
    vortonPclGrpInfo.mPclOpEvolve = new PclOpEvolve() ;
    vortonPclGrpInfo.mParticleGroup->PushBack( vortonPclGrpInfo.mPclOpEvolve ) ;

    // Fluid-Body interaction must occur after evolve, because evolve will move
    // particles, potentially outside grids, and fluid-body interaction does not
    // depend on those grids. Likewise, no particle motion can happen between
    // when grid bounds are found and when grids are populated.
    vortonPclGrpInfo.mPclOpFluidBodInte = new PclOpFluidBodyInteraction() ;
    vortonPclGrpInfo.mPclOpFluidBodInte->mDensityGrid               = 0 ;
    vortonPclGrpInfo.mPclOpFluidBodInte->mPhysicalObjects           = & physicalObjects ;
    vortonPclGrpInfo.mPclOpFluidBodInte->mAmbientFluidDensity       = ambientFluidDensity ;
    vortonPclGrpInfo.mPclOpFluidBodInte->mFluidSpecificHeatCapacity = fluidSpecificHeatCapacity ;
    vortonPclGrpInfo.mPclOpFluidBodInte->mGravityAcceleration       = gravityAcceleration ;
    vortonPclGrpInfo.mPclOpFluidBodInte->mRespectAngVel             = true ;
    vortonPclGrpInfo.mParticleGroup->PushBack( vortonPclGrpInfo.mPclOpFluidBodInte ) ;

#if 1
    // Populate velocity grid from "vortons", after PclOpFluidBodyInteraction,
    // since satisfying boundary conditions will change velocity.  This is done
    // for the benefit of advecting tracer particles.
    // NOTE that UpdateVortexParticleMethod and UpdateSmoothedParticleHydrodynamics both populate a velocity grid anyway, which is proper, but also apparently redundant with this operation.
    // In fact it is redundant but the point of this operation is to take into account satisfying boundary conditions.
    // This is a half-baked solution and not currently applied consistently; it is disabled for VPM even though the rationale applies equally to SPH and VPM.
    // The rationale has 2 parts:
    //  (a) VPM generates a field everywhere (not just at vortons).  A second pass that assigns velocity from vortons loses velocity information far from vortons.
    //      This is mitigated by having boundary conditions applied to tracers as well.  While that is the case for SPH, it still leads to tracers ending up near corners.
    //      The point of this reassignment is to prevent flow toward corners.  And it does solve that particular problem, which is more prevalent in SPH than in VPM.
    //  (b) This operation effectively makes
    //      accessing the density grid "dangerous" because solving boundary conditions happens after vortons are moved, hence results in out-of-bounds accesses.
    //      (Double-check this.  Density grid domain or access for tracers should not depend on vorton position.)
    //      So, for now, when mPclOpPopulateVelocityGrid is active, the density grid is "deactivated", which means tracers get the wrong (or no) density assignments.
    //      One solution to this could entail having tracers get their density assignments prior to (vorton or tracer) advection, when the density grid is still valid.
    //      Another could entail populating the density grid after vortons have gone through satisfying boundary conditions, but that involves wasted computation, since
    //      density is populated to update vortons.
    // Note that this re-assignment has 2 surprising and negative side effects:
    //  (1) It wastes computation since the velocity grid gets popuated twice.
    //      This could be mitigated by solving velocity only at vortons (not on the grid) during the Update phase.  But then for VPM it would lose information. See above.
    //  (2) It leads to the velocity grid having different domain than the density and other grids.  By itself this does not seem to cause any problems; it's just surprising.
    vortonPclGrpInfo.mPclOpPopulateVelocityGrid = new PclOpPopulateVelocityGrid() ;
    vortonPclGrpInfo.mPclOpPopulateVelocityGrid->mVelocityGrid = & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid() ;
    vortonPclGrpInfo.mPclOpPopulateVelocityGrid->mBoundingBox  = const_cast< Vec3 * >( & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetMinCorner() ) ;
    vortonPclGrpInfo.mParticleGroup->PushBack( vortonPclGrpInfo.mPclOpPopulateVelocityGrid ) ;
#else
    vortonPclGrpInfo.mPclOpPopulateVelocityGrid = 0 ;
#endif

    return vortonPclGrpInfo.mParticleGroup ;
}




static ParticleGroup * CreateTracerParticleSystem( FluidTracerPclGrpInfo & tracerPclGrpInfo , FluidVortonPclGrpInfo & vortonPclGrpInfo , const float ambientFluidDensity )
{
    PERF_BLOCK( CreateTracerParticleSystem ) ;

    tracerPclGrpInfo.mParticleGroup = new ParticleGroup() ;

    tracerPclGrpInfo.mPclOpKillAge = new PclOpKillAge() ;
    static const int    killAgeMax  = 90    ;

    tracerPclGrpInfo.mPclOpKillAge->mAgeMax = killAgeMax ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpKillAge ) ;

    tracerPclGrpInfo.mPclOpAssignDensityFromGrid = new PclOpAssignScalarFromGrid() ;
    tracerPclGrpInfo.mPclOpAssignDensityFromGrid->mMemberOffsetInBytes      = tracerOffsetToDensity ;
    tracerPclGrpInfo.mPclOpAssignDensityFromGrid->mScalarGrid               = & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetDensityGrid() ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpAssignDensityFromGrid ) ;

#if ENABLE_FIRE
    tracerPclGrpInfo.mPclOpAssignFuelFromGrid = new PclOpAssignScalarFromGrid() ;
    tracerPclGrpInfo.mPclOpAssignFuelFromGrid->mMemberOffsetInBytes = tracerOffsetToFuelFraction ;
    tracerPclGrpInfo.mPclOpAssignFuelFromGrid->mScalarGrid          = & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFuelGrid() ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpAssignFuelFromGrid ) ;

    tracerPclGrpInfo.mPclOpAssignFlameFromGrid = new PclOpAssignScalarFromGrid() ;
    tracerPclGrpInfo.mPclOpAssignFlameFromGrid->mMemberOffsetInBytes    = tracerOffsetToFlameFraction ;
    tracerPclGrpInfo.mPclOpAssignFlameFromGrid->mScalarGrid             = & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFlameGrid() ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpAssignFlameFromGrid ) ;

    tracerPclGrpInfo.mPclOpAssignSmokeFromGrid = new PclOpAssignScalarFromGrid() ;
    tracerPclGrpInfo.mPclOpAssignSmokeFromGrid->mMemberOffsetInBytes    = tracerOffsetToSmokeFraction ;
    tracerPclGrpInfo.mPclOpAssignSmokeFromGrid->mScalarGrid             = & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetSmokeGrid() ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpAssignSmokeFromGrid ) ;
#endif

    tracerPclGrpInfo.mPclOpAssignVelocityFromField = new PclOpAssignVelocityFromField() ;
    tracerPclGrpInfo.mPclOpAssignVelocityFromField->mVelocityGrid    =  & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid() ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpAssignVelocityFromField ) ;

    tracerPclGrpInfo.mPclOpWind = new PclOpWind() ;
    * tracerPclGrpInfo.mPclOpWind = * vortonPclGrpInfo.mPclOpWind ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpWind ) ;

#if 1 // TESTING initial surface tracer placement. DO NOT SUBMIT DISABLED. See comments below.
    tracerPclGrpInfo.mPclOpEvolve = new PclOpEvolve() ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpEvolve ) ;
#endif

    tracerPclGrpInfo.mPclOpFluidBodInte = new PclOpFluidBodyInteraction() ;
    tracerPclGrpInfo.mPclOpFluidBodInte->mPhysicalObjects      = vortonPclGrpInfo.mPclOpFluidBodInte->mPhysicalObjects ;
    tracerPclGrpInfo.mPclOpFluidBodInte->mAmbientFluidDensity  = ambientFluidDensity ;
    tracerPclGrpInfo.mPclOpFluidBodInte->mRespectAngVel        = false ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpFluidBodInte ) ;

    // Note that if tracers get emitted outside the bounding box computed last
    // frame, then mVelocityGrid will not encompass them. To be safe, these
    // emitted particles should not be advected on their first frame.  That
    // could lead to weird artifacts where just-emitted particles disobey wind
    // and other advectors, on their first frame.  Could mitigate that either by
    // providing suitable initial velocity, or could move Emit to just after
    // Kill, then each FindBoundingBox call would also include the entire Emit
    // region, in anticipation of particles to be emitted next frame.
    tracerPclGrpInfo.mPclOpEmit = new PclOpEmit() ;
    tracerPclGrpInfo.mPclOpEmit->mTemplate              = vortonPclGrpInfo.mPclOpEmit->mTemplate ;
    tracerPclGrpInfo.mPclOpEmit->mSpread                = vortonPclGrpInfo.mPclOpEmit->mSpread ;
    tracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.x    = vortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.x        ;
    tracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.y    = vortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.y * 0.3f ;
    tracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.z    = vortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.z * 0.3f ;
    tracerPclGrpInfo.mPclOpEmit->mSpread.mDensity       = 0.0f ; // No variance for density. Doesn't really matter since tracers adopt density from vortons eventually.
    tracerPclGrpInfo.mPclOpEmit->mEmitRate              = 0.0f ; // Set by InitialConditions
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpEmit ) ;

    // Finding bounding box must occur late in this group, and before
    // VortonSim in the other group (because VortonSim uses the bounding
    // box to create a grid of appropriate size to capture all relevant
    // terms of the vorticity equation). Safest to run FindBoundingBox last.
//#error TODO: Use the bounding box values found here, instead of recomputing that, in PclOpSeedSurfaceTracers::Replace
tracerPclGrpInfo.mPclOpFindBoundingBox = 0 ;
    tracerPclGrpInfo.mPclOpFindBoundingBox = new PclOpFindBoundingBox() ;
    tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpFindBoundingBox ) ;

// Reassign surface tracers to reside within a specified band about the fluid surface.
tracerPclGrpInfo.mPclOpSeedSurfaceTracers = new PclOpSeedSurfaceTracers() ;
tracerPclGrpInfo.mPclOpSeedSurfaceTracers->mTemplate            = vortonPclGrpInfo.mPclOpEmit->mTemplate ;
tracerPclGrpInfo.mPclOpSeedSurfaceTracers->mSpread              = vortonPclGrpInfo.mPclOpEmit->mSpread ;
tracerPclGrpInfo.mPclOpSeedSurfaceTracers->mBandWidth           = - FLT_MAX ;
tracerPclGrpInfo.mPclOpSeedSurfaceTracers->mAmbientDensity      = ambientFluidDensity ;
tracerPclGrpInfo.mPclOpSeedSurfaceTracers->mSignedDistanceGrid  = NULLPTR ; // & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetSignedDistanceGrid() ; // InitialConditions sets this explicitly.
tracerPclGrpInfo.mPclOpSeedSurfaceTracers->mReferenceGrid       = & vortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid() ;
tracerPclGrpInfo.mParticleGroup->PushBack( tracerPclGrpInfo.mPclOpSeedSurfaceTracers ) ;

    return tracerPclGrpInfo.mParticleGroup ;
}




ParticleSystem * CreateFluidParticleSystems( FluidVortonPclGrpInfo & vortonPclGrpInfo , FluidTracerPclGrpInfo & tracerPclGrpInfo , const float viscosity , const float ambientFluidDensity , const float fluidSpecificHeatCapacity , VECTOR< Impulsion::PhysicalObject * > & physicalObjects )
{
    PERF_BLOCK( CreateFluidParticleSystems ) ;

    ParticleSystem * fluidParticleSystem = new ParticleSystem() ;

    // Set up particle group for vortons.
    CreateVortonParticleSystem( vortonPclGrpInfo , viscosity , ambientFluidDensity , fluidSpecificHeatCapacity , sGravityAcceleration , physicalObjects ) ;

    // Set up particle group for passive tracers.
    CreateTracerParticleSystem( tracerPclGrpInfo , vortonPclGrpInfo , ambientFluidDensity ) ;

    // Process tracers before vortons, so vortons know tracer bounding box.
    fluidParticleSystem->PushBack( tracerPclGrpInfo.mParticleGroup ) ;
    fluidParticleSystem->PushBack( vortonPclGrpInfo.mParticleGroup ) ;

    if( tracerPclGrpInfo.mPclOpFindBoundingBox )
    {
        // Patch vortex particle system to reference results from PclOpFindBoundingBox.
        fluidParticleSystem->AddIndirectAddressAssignment(
                IndirectAddress( vortonPclGrpInfo.mPclOpVortonSim       , & vortonPclGrpInfo.mPclOpVortonSim->mMinCorner           )
            ,   IndirectAddress( tracerPclGrpInfo.mPclOpFindBoundingBox , & tracerPclGrpInfo.mPclOpFindBoundingBox->GetMinCorner() ) ) ;

        fluidParticleSystem->AddIndirectAddressAssignment(
                IndirectAddress( vortonPclGrpInfo.mPclOpVortonSim       , & vortonPclGrpInfo.mPclOpVortonSim->mMaxCorner           )
            ,   IndirectAddress( tracerPclGrpInfo.mPclOpFindBoundingBox , & tracerPclGrpInfo.mPclOpFindBoundingBox->GetMaxCorner() ) ) ;
    }

    return fluidParticleSystem ;
}




ParticleSystem * CreateNonFluidParticleSystem( size_t i )
{
    PERF_BLOCK( CreateNonFluidParticleSystem ) ;

    ParticleSystem  * particleSystem = new ParticleSystem ;

    {
        ParticleGroup * pclGrpFlames = new ParticleGroup ;

        {
            PclOpKillAge * pclOpKillAge = new PclOpKillAge ;
            pclOpKillAge->mAgeMax    = 90 ;
            pclGrpFlames->PushBack( pclOpKillAge ) ;
        }
        {
            Particle emitterTemplate ;
            const float gap = 1.0f * float( i ) ;
            emitterTemplate.mPosition               = Vec3( gap , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity               = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize                   = 0.0625f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensity                = 1.0f ; // Actual value does not matter for this case.
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction           = 0.0f ;
            emitterTemplate.mFlameFraction          = 0.0f ;
            emitterTemplate.mSmokeFraction          = 1.0f ;
        #endif

            Particle emitterSpread ;
            emitterSpread.mPosition                 = Vec3( 0.5f , 0.5f , 0.5f ) ;

            PclOpEmit     * pclOpEmit               = new PclOpEmit ;

            pclOpEmit->mTemplate                    = emitterTemplate ;
            pclOpEmit->mTemplate.mAngularVelocity   = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            pclOpEmit->mSpread                      = emitterSpread ;
            pclOpEmit->mSpread.mDensity             = 0.0f ;
            pclOpEmit->mEmitRate                    = 100.0f ;

            pclGrpFlames->PushBack( pclOpEmit ) ;
        }
        {
            PclOpWind   * pclOpWind = new PclOpWind ;

            pclOpWind->mWindWeight  = 1.0f ;
            pclOpWind->mSrcWeight   = 0.0f ;
            pclOpWind->mWind        = Vec3( 0.0f , 0.0f , 1.0f ) ;
            pclGrpFlames->PushBack( pclOpWind ) ;

        }
        {
            PclOpEvolve * pclOpEvolve = new PclOpEvolve ;
            // Evolve runs after all operations that set velocity.
            pclGrpFlames->PushBack( pclOpEvolve ) ;
        }

        particleSystem->PushBack( pclGrpFlames ) ;
    }

    return particleSystem ;
}
