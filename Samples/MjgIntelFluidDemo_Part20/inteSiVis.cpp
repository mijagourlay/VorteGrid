//#pragma optimize( "" , off )
/** \file inteSiVis.cpp

    \brief Application for interactive simulation and visualization

    \author Copyright 2009-2016 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "inteSiVis.h"

#include <Sim/Vorton/vorticityDistribution.h>

#include <Render/Scene/light.h>

#include <Particles/Operation/pclOpFindBoundingBox.h>
#include <Particles/Operation/pclOpWind.h>
#include <Particles/Operation/pclOpAdvect.h>
#include <Particles/Operation/pclOpEvolve.h>
#include <Particles/Operation/pclOpAssignScalarFromGrid.h>
#include <Particles/Operation/pclOpPopulateVelocityGrid.h>
#include <Particles/Operation/pclOpEmit.h>
#include <Particles/Operation/pclOpKillAge.h>

#include <FluidBodySim/fluidBodySim.h>
#include <FluidBodySim/pclOpFluidBodyInteraction.h>

#include <Core/Math/vec2.h>
#include <Core/Math/vec4.h>
#include <Core/Math/mat4.h>

#include <Core/Performance/perfBlock.h>

#if defined( _DEBUG )
    #define UNIT_TEST
    #include <Collision/sphereShape.h> // For Collision::UnitTests
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#if defined( WIN32 )
    #include <windows.h>
#endif

#include <assert.h>

#include <stdarg.h>

#pragma comment(lib, "glut32.lib")




// Private variables --------------------------------------------------------------

static const float  sOneOver30              = 1.0f / 30.0f ;
static const float  sOneOver60              = 1.0f / 60.0f ;
static const float  sOneOver120             = 1.0f / 120.0f ;
static const float  sTimeStep               = sOneOver30 ;

static InteSiVis * sInstance = 0 ;

static const size_t vortonStride                = sizeof( Vorton ) ;
static const size_t vortonOffsetToAngVel        = offsetof( Vorton , mAngularVelocity   ) ;
static const size_t vortonOffsetToSize          = offsetof( Vorton , mSize              ) ;
static const size_t vortonOffsetToDensity       = offsetof( Vorton , mDensity           ) ;

static const size_t tracerStride                = sizeof( Particle ) ;
static const size_t tracerOffsetToAngVel        = offsetof( Particle , mAngularVelocity ) ;
static const size_t tracerOffsetToSize          = offsetof( Particle , mSize            ) ;

static const size_t tracerOffsetToDensity       = offsetof( Particle , mDensity         ) ;
#if ENABLE_FIRE
static const size_t tracerOffsetToFuelFraction  = offsetof( Particle , mFuelFraction    ) ;
static const size_t tracerOffsetToFlameFraction = offsetof( Particle , mFlameFraction   ) ;
static const size_t tracerOffsetToSmokeFraction = offsetof( Particle , mSmokeFraction   ) ;
#endif

static int sMousePrevX = -999 ;
static int sMousePrevY = -999 ;

static const float gScale = 1.0f ;

#if defined( PROFILE )
static const int sFrameCountMax = 600   ;   // In profile builds, run for a fixed number of frames
static size_t    sNumTracersSum = 0     ;   // Sum of current number of tracers
static size_t    sNumVortonsSum = 0     ;   // Sum of current number of vortons
#endif



// Functions --------------------------------------------------------------

/** Construct application for interactive simulation and visualization.
*/
InteSiVis::InteSiVis( PeGaSys::Render::ApiBase * renderApi )
    : mRenderSystem( renderApi )
    , mFluidScene( & mRenderSystem , & mPclSysMgr )

// TODO: Finish replacing render members (mQdCamera, m*Renderer) below with PeGaSys::Render equivalents
    , mQdCamera( 1920 , 1080 ) // NOTE: Window size here DOES NOT MATTER.  The PeGaSys render window controls actual window size.  See glutReshapeWindow, GlutReshapeGlutCallback.

    , mFrameDurSecAvg( sTimeStep )
    , mFrame( 0 )
    , mTimeNow( 0.0 )
    , mTimeStep( sTimeStep )
    , mTimeStepMin( FLT_MAX )
    , mTimeStepMax( -FLT_MAX )
    , mTimeStepping( PLAY )
    , mInitialized( false )
    , mFluidParticleSystem( NULLPTR )
    , mGridDecorations( GRID_DECO_NONE )
    , mGridField( GRID_FIELD_DENSITY_GRADIENT )
    , mVortonProperty( VORTON_PROPERTY_VORTICITY )

#if PROFILE
    , mDiagnosticText( DIAG_TEXT_NONE )
#else
    , mDiagnosticText( DIAG_TEXT_TIMING )
#endif
    , mEmphasizeCameraTarget( true )
{
    PERF_BLOCK_ENABLE_PROFILING( sFrameCountMax ) ;   // Profile specified number of frames.

    PERF_BLOCK( InteSiVis__InteSiVis ) ;

    ASSERT( 0 == sInstance ) ;
    sInstance = this ;

#if defined( _DEBUG )
    mGridDecorations = GRID_DECO_CELLS   ; // Render grid cells.
    mDiagnosticText  = DIAG_TEXT_SUMMARY ; // Render summary diagnostic text.
#endif

    mMouseButtons[0] = mMouseButtons[1] = mMouseButtons[2] = 0 ;

    InitialConditions(
        1 // Vortex ring
        //  2   // Ball through smoke
        // 12  // Falling drop turns into vortex ring
        // 26  // Drop in a box with SDF
        // 28  // Horizontally swishing box
        // 31  // Horizontally sloshing box
        // 32  // Vertically tumbling box
        ) ;

    // Set the camera in motion.
    //mQdCamera.SetOrbitalTrajectory( Vec3( -0.002f , -0.000f , 0.0f ) ) ;

    glutReshapeWindow( 1920 , 1080 ) ;

    // Initialize system timer used inside Idle. 
    QueryPerformanceCounter( & mSystemTimeBefore ) ; // This is probably redundant with the call inside InitialConditions

#if defined( _DEBUG )
    UniformGrid<unsigned>::UnitTest() ;
    NestedGrid<unsigned>::UnitTest() ;
    NestedGrid<Vec3>::UnitTest() ;
    Collision::UnitTests() ;
    extern void UnitTestPoisson1D() ;
    UnitTestPoisson1D() ;
    extern void UnitTestPoisson2D() ;
    UnitTestPoisson2D() ;
#endif
}




/** Destruct application for interactive simulation and visualization.
*/
InteSiVis::~InteSiVis()
{
    PERF_BLOCK( InteSiVis__dtor) ;

    sInstance = 0 ;
}




/* static */ InteSiVis * InteSiVis::GetInstance()
{
    return sInstance ;
}




/** Create render models and simulation entities that couple physical objects to those models.

    This should get called after both physical objects and render data exist.
    You cannot reliably call this, for example, from InitialConditions,
    since the render window might not have appeared yet, in which case
    various render resources will not yet exist.
*/
void InteSiVis::CreateRigidBodyModelsAndEntities()
{
    PERF_BLOCK( InteSiVis__CreateRigidBodyModelsAndEntities ) ;

    ASSERT( mInitialized ) ; // Cannot create entities until render data exists.

    const size_t numSpheres     = GetSpheres().size() ;
    const size_t numBoxes       = GetBoxes().size() ;
    const size_t numPhysObjs    = numSpheres + numBoxes ;

    mEntities.clear() ;
    mEntities.reserve( numPhysObjs ) ;

    mFluidScene.Clear() ; // TODO: FIXME: This also clears cameras (and lights) so camera (and lights) would need to get created after this.  Could move all Clears into caller or create camera (and lights) entity(s) in here.

    // Populate scene first with lights, camera and opaque objects like the skybox.
    mFluidScene.PopulateCameraLightsSky() ;

    size_t idxPhysObj = 0 ;

    for( size_t idxSphere = 0 ; idxSphere < numSpheres ; ++ idxSphere , ++ idxPhysObj )
    {   // For each sphere physical object...
        // Add a sphere model.
        const float radius = GetSpheres()[ idxSphere ].GetCollisionShape()->GetBoundingSphereRadius() ;

        PeGaSys::Render::ModelNode * sphereModel = mFluidScene.AddSphereModel( Vec3( 0.0f , 0.0f , 0.0f ) , radius ) ;

        // Make an Entity to tie the render model to the physical object.
        mEntities.push_back( Entity( & GetSpheres()[ idxSphere ] , sphereModel ) ) ;
    }

    for( size_t idxBox = 0 ; idxBox < numBoxes ; ++ idxBox , ++ idxPhysObj )
    {   // For each box physical object...
        RbBox & box = GetBoxes()[ idxBox ] ;
        const Vec3 & dimensions = box.GetDimensions() ;

        PeGaSys::Render::ModelNode * boxModel = mFluidScene.AddBoxModel( Vec3( 0.0f , 0.0f , 0.0f ) , dimensions , box.GetIsHole() ) ;

        // Make an Entity to tie the render model to the physical object.
        mEntities.push_back( Entity( & box , boxModel ) ) ;
    }

    // Populate the rest of the fluid scene AFTER adding the solid models above, because PopulateScene adds
    // fluid surface and particles which are translucent, therefore must render after all opaque objects.
    mFluidScene.PopulateSceneWithFluidSurfaceAndParticles( mFluidParticleSystem ) ;

    CopyLightsFromQdToPeGaSys() ;
}




/** Set initial conditions for simulation.

    \param ic - which initial conditions to impose

    This is just a demonstration of a few initial conditions.
    There is nothing fundamental or especially important about
    the initial conditions that this routine implements.
    These are just examples.

*/
void InteSiVis::InitialConditions( unsigned ic )
{
    PERF_BLOCK( InteSiVis__InitialConditions ) ;

#if ENABLE_PARTICLE_HISTORY
    extern size_t gPclHistoryFrame ;
    extern bool gPclHistoryFirstRun ;
    if( gPclHistoryFrame != 0 ) gPclHistoryFirstRun = false ; // This is a subsequent run.  History is already populated with the basis for comparison.
    gPclHistoryFrame = 0 ;  // Reset particle history recorder
    ++ gPclHistoryFrame ; // Advance to next frame in particle history buffer.
#endif

    ASSERT( Impulsion::PhysicalObject::sAmbientTemperature == Particle_sAmbientTemperature ) ; // This is not a necessary condition but I have not tested it otherwise.  If one changes without the other, probably should test a bunch of code to make sure it behaves as intended.

    mScenario               = ic  ;
    sInstance->mFrame       = 0   ;
    sInstance->mTimeNow     = 0.0 ;
    sInstance->mTimeStep    = sTimeStep ;
    sInstance->mTimeStepMin =   FLT_MAX ;
    sInstance->mTimeStepMax = - FLT_MAX ;

    mPclSysMgr.Clear() ; // Clear all particle systems to have a fresh start.  Code below adds them.
    mFluidParticleSystem = NULLPTR ;

    static const float ambientFluidDensity  = 1.0f ;
    {
        static const float viscosity = 0.01f ;
        static const float fluidSpecificHeatCapacity = 10.0f ;

        mFluidParticleSystem = CreateFluidParticleSystem( mVortonPclGrpInfo , mTracerPclGrpInfo , viscosity , ambientFluidDensity , fluidSpecificHeatCapacity , GetPhysicalObjects() ) ;
        mPclSysMgr.PushBack( mFluidParticleSystem ) ;
    }

    SetTallyDiagnosticIntegrals() ;

    GetSpheres().Clear() ;                      // Remove all spherical physical objects from the simulation
    GetBoxes().Clear() ;                        // Remove all box physical objects from the simulation
    mEntities.Clear() ;                         // Remove all simulation entities

    // Set up lights.
    mQdLights.resize( 2 ) ;
    mQdLights[0].mPosition = Vec3( 0.7f , -0.2f , 0.7f ) ; // actually a direction
    mQdLights[0].mColor    = Vec3( 0.9f , 0.9f , 0.9f ) ;
    mQdLights[0].mType     = QdLight::LT_DIRECTIONAL ;

    mQdLights[1].mPosition = Vec3( -0.7f , -0.2f , 0.7f ) ; // actually a direction
    mQdLights[1].mColor    = Vec3( 0.9f , 0.9f , 0.9f ) ;
    mQdLights[1].mType     = QdLight::LT_DIRECTIONAL ;

    mQdLights[0].mAmplitudes [ 1 ] = 0.01f ;
    mQdLights[0].mFrequencies[ 1 ] = 0.0f ;
    mQdLights[0].mAmplitudes [ 2 ] = 0.0f ;
    mQdLights[0].mFrequencies[ 2 ] = 0.0f ;
    mQdLights[0].mAmplitudes [ 3 ] = 0.0f ;
    mQdLights[0].mFrequencies[ 3 ] = 0.0f ;

    static const float          fRadius                     = 1.0f ;
    static const float          fThickness                  = 1.0f ;
    static const float          fMagnitude                  = 20.0f ;
    static const unsigned       numCellsPerDim              = 16 ;
    static const unsigned       numVortonsMax               = numCellsPerDim * numCellsPerDim * numCellsPerDim ;
    unsigned                    numTracersPerCellCubeRoot   = 5 ;
    VortonSim &                 vortonSim                   = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
    VECTOR< Vorton > &          vortons                     = * vortonSim.GetVortons() ;
    static const float          veryLargeMass               = 1.0e8f ; // large but not large enough that 1/mass is a denormal.  Denormal values occur when veryLargeMass is around 1.0e34f. Also needs to be small enough that squaring it fits in FLT_MAX.
    const VECTOR< Particle > *  vortonsImplyingTracers      = 0 ;
    Vec3                        tracerGridScale             ( 1.0f , 1.0f , 1.0f ) ;

    mVortonPclGrpInfo.mPclOpKillAge->mAgeMax    = INT_MAX ;                                     // By default, disable vorton killing.
    mTracerPclGrpInfo.mPclOpKillAge->mAgeMax    = mVortonPclGrpInfo.mPclOpKillAge->mAgeMax ;    // By default, disable tracer killing.
    mVortonPclGrpInfo.mPclOpEmit->mEmitRate     = 0.0f ;                                        // By default, disable vorton emission.
    mTracerPclGrpInfo.mPclOpEmit->mEmitRate     = 0.0f ;                                        // By default, disable tracer emission.
    mVortonPclGrpInfo.mPclOpWind->mWind         = Vec3( 0.0f , 0.0f , 0.0f ) ;                  // By default, disable wind.

    // By default, disable density grid.
    mVortonPclGrpInfo.mPclOpFluidBodInte->mDensityGrid = NULLPTR ;

    // By default, disable SDF generation.
    mTracerPclGrpInfo.mPclOpSeedSurfaceTracers->mSignedDistanceGrid  = NULLPTR ;

    // By default, enable assigning tracer density from vorton density.
    mTracerPclGrpInfo.mPclOpAssignDensityFromGrid->mScalarGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetDensityGrid() ;

    // By default, use fire tracer rendering.
    mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_FIRE ) ;

    // By default, set fluid simulation technique to use vortex particle method.
    vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

    // By default, do not populate SDF from density.
    vortonSim.SetPopulateSdfFromDensity( false ) ;

    srand( 1 ) ;    // Seed pseudo-random number generator to make results repeatable.

    switch( ic )
    {   // Switch on initial conditions
        case 0: // vortex ring -- vorticity in [0,1]
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexRing( fRadius , fThickness , Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 10.f , 0.f , 0.f ) ) ;
            mQdCamera.SetEye( Vec3( 10.f , -10.0f , 0.0f ) ) ;
        break ;

        case 1: // "jet" vortex ring -- velocity in [0,1]
            AssignVortons( vortons , fMagnitude , numVortonsMax , JetRing( fRadius , fThickness , Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            //vortonsImplyingTracers = ( const VECTOR< Particle > * )( & vortons ) ;
            numTracersPerCellCubeRoot -= 2 ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 10.f ,    0.f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 10.f , -10.0f , 0.0f ) ) ;
        break ;

        case 2: // Projectile with no initial spin
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 2.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Make sphere spin

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;

        case 3: // Projectile with spin about longitudinal axis: rifled.
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.3f * fThickness , 0.3f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 5.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres()[0].GetBody()->ApplyImpulsiveTorque( Vec3( 0.002f , 0.0f , 0.0f ) ) ; // Make sphere spin

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        break ;

        case 4: // Projectile with spin about transverse axis: curve ball.
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a sphere.
            GetSpheres().PushBack( RbSphere( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 4.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.0f , 0.05f , 0.0f ) ) ; // Make sphere spin

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;

        case 5: // Spinning sphere in initially stationary fluid
            // Make a box of vortons surrounding the origin.
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 2.0f * fThickness , 2.0f * fThickness , 2.0f * fThickness ) ) ) ;

            // Make a thinner layer of tracers.
            numTracersPerCellCubeRoot += 6 ;
            tracerGridScale = Vec3( 1.0f , 0.125f , 1.0f ) ;

#if 1
            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( 0.f , 0.0f , 0.0f ) , Vec3( 0.f , 0.0f , 0.0f ) , 1000.0f , fThickness * 0.3f ) ) ;
            GetSpheres().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 100.0f ) ) ; // Make sphere spin
#else
            // Add a box
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f ,  0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1000.0f , Vec3( 1.0f , 1.0f , 1.0f ) ) ) ;
            GetBoxes().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 100.0f ) ) ; // Make box spin
#endif

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , 0.0f , 3.0f ) ) ;
        break ;

        case 6: // Sphere in spinning fluid
            AssignVortons( vortons , 2.0f * fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 4.0f * fThickness , 2 , 0 ) ) ;

            numTracersPerCellCubeRoot += 3 ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( 0.f , 0.0f , 0.0f ) , Vec3( 0.f , 0.0f , 0.0f ) , 0.5f , 0.1f ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( -2.0f , 0.0f , 0.0f ) ) ;
        break ;

        case 7: // Fluid moves past ball
        {
            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            //GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Make sphere spin

            mVortonPclGrpInfo.mPclOpKillAge->mAgeMax            = 100 ;
            mTracerPclGrpInfo.mPclOpKillAge->mAgeMax            = mVortonPclGrpInfo.mPclOpKillAge->mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition       = Vec3( -0.2f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity       = Vec3( 1.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize           = 0.0625f ;
            emitterTemplate.mDensity        = ambientFluidDensity ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction   = 0.0f ;
            emitterTemplate.mFlameFraction  = 0.0f ;
            emitterTemplate.mSmokeFraction  = 1.0f ;
        #endif
            emitterSpread.mPosition         = Vec3( emitterTemplate.mVelocity.x / 30.0f , 0.5f , 0.5f ) ;

            mVortonPclGrpInfo.mPclOpEmit->mTemplate                     = emitterTemplate ;
            mVortonPclGrpInfo.mPclOpEmit->mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread                       = emitterSpread ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread.mDensity              = 0.0f ;
            mVortonPclGrpInfo.mPclOpEmit->mEmitRate                     = 500.0f ;

            mTracerPclGrpInfo.mPclOpEmit->mEmitRate                     = 10000.0f ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate                     = mVortonPclGrpInfo.mPclOpEmit->mTemplate ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate.mSize               = 0.0078f ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread                       = mVortonPclGrpInfo.mPclOpEmit->mSpread ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.x           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.x        ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.y           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.y * 0.3f ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.z           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.z * 0.3f ;

            mVortonPclGrpInfo.mPclOpWind->mWind                         = Vec3( 1.0f , 0.0f , 0.0f ) ;

            numTracersPerCellCubeRoot -- ; // Emit rate controls tracer density, but this effectively controls their radius.

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 1.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 1.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 8: // Vortex sheet with spanwise variation
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexSheet( fThickness , /* variation */ 0.2f , /* width */ 7.0f * fThickness ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -20.0f , 0.0f ) ) ;
        break ;

        case 9: // Vortex tube
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 2.0f * fThickness , 2 , 0 ) ) ;
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;
        break ;

        case 10: // 2 orthogonal vortex tubes
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 4.0f * fThickness , 2 , -1 ) ) ;
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 4.0f * fThickness , 2 ,  1 ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        break ;

        ////////////////////////////////////////////////////////////////////////

        case 11: // 2D sheet
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexSheet( fThickness , /* variation */ 0.0f , /* width */ 2.0f * fThickness ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -10.0f , 0.0f ) ) ;
        break ;

        case 12: // ball of initially stationary heavy fluid
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius , 2.0f ) , Vec3( 0.0f , 0.0f , 5.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot += 2 ;

            // Use dye tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_DYE ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -10.0f , 0.0f ) ) ;
        break ;

        case 13: // Two balls of initially stationary fluid, one heavy, one light, heavy one above light one.
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius , -0.1f ) , Vec3( 0.0f , 0.0f , -2.0f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius ,  0.1f ) , Vec3( 0.0f , 0.0f ,  2.0f ) ) ;

            // Use dye tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_DYE ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -5.0f , 0.0f ) ) ;
        break ;

        case 14: // Two boxes of initially stationary fluid, one heavy, one light, light one above heavy one.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) , -0.6f ) , Vec3( 0.0f , 0.0f ,  0.5f ) ) ;

            // Set up density grid to facilitate applying buoyancy to bodies.
            mVortonPclGrpInfo.mPclOpFluidBodInte->mDensityGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetDensityGrid() ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            const float sphereRadius = 0.2f ;
            const float neutrallyBuoyantMass = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetAmbientDensity() * 4.0f * PI * Pow3( sphereRadius ) / 3.0f ;
            GetSpheres().PushBack( RbSphere( Vec3( 0.f , 0.0f , 0.5f ) , Vec3( 0.f , 0.0f , 0.0f ) , neutrallyBuoyantMass , sphereRadius ) ) ;

            // Use dye tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_DYE ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
        }
        break ;

        case 15: // Ball heats fluid
        {
            mVortonPclGrpInfo.mPclOpKillAge->mAgeMax    = 300 ;
            mTracerPclGrpInfo.mPclOpKillAge->mAgeMax    = mVortonPclGrpInfo.mPclOpKillAge->mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition       = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity       = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize           = 0.0625f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensity        = ambientFluidDensity ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction   = 0.0f ;
            emitterTemplate.mFlameFraction  = 0.0f ;
            emitterTemplate.mSmokeFraction  = 0.0f ;
        #endif
            emitterSpread.mPosition         = Vec3( 0.5f , 0.5f , 0.5f ) ;

            mVortonPclGrpInfo.mPclOpEmit->mTemplate                     = emitterTemplate ;
            mVortonPclGrpInfo.mPclOpEmit->mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread                       = emitterSpread ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread.mDensity              = 0.0f ;
            mVortonPclGrpInfo.mPclOpEmit->mEmitRate                     = 100.0f ;

            mTracerPclGrpInfo.mPclOpEmit->mEmitRate                     = 5000.0f ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate                     = mVortonPclGrpInfo.mPclOpEmit->mTemplate ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate.mSize               = 0.0105f ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread                       = mVortonPclGrpInfo.mPclOpEmit->mSpread ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.x           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.x ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.y           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.y ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.z           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.z ;

            mVortonPclGrpInfo.mPclOpWind->mWind                         = Vec3( 0.0f , 0.0f , 0.0f ) ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            GetSpheres().Back().GetThermalProperties().SetTemperature( Impulsion::PhysicalObject::sAmbientTemperature + 300.0f ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 2.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -3.0f , 2.0f ) ) ;
        }
        break ;

        case 16: // Fluid combusts and pushes a ball up.
        {
            // Set up lights.
            mQdLights[0].mPosition    = Vec3( 0.0f , 0.0f , 0.7f ) ;
            mQdLights[0].mColor       = 1.2f * Vec3( 1.0f , 0.5f , 0.3f ) ;
            mQdLights[0].mAttenuation = Vec3( 0.0f , 0.2f , 0.8f ) ;
            mQdLights[0].mType        = QdLight::LT_POINT ;
            // Make light fluctuate to imply it comes from flames.
            mQdLights[0].mAmplitudes [ 1 ] = 0.2f ;
            mQdLights[0].mFrequencies[ 1 ] = 2.0f ;
            mQdLights[0].mAmplitudes [ 2 ] = 0.1f ;
            mQdLights[0].mFrequencies[ 2 ] = 5.0f ;
            mQdLights[0].mAmplitudes [ 3 ] = 0.05f ;
            mQdLights[0].mFrequencies[ 3 ] = 10.0f ;

            mQdLights[1].mPosition    = Vec3( 0.0f , 0.0f , 1.0f ) ; // actually a direction
            mQdLights[1].mColor       = 0.2f * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mQdLights[1].mType        = QdLight::LT_DIRECTIONAL ;

            mVortonPclGrpInfo.mPclOpKillAge->mAgeMax    = 300 ;
            mTracerPclGrpInfo.mPclOpKillAge->mAgeMax    = mVortonPclGrpInfo.mPclOpKillAge->mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition       = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity       = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize           = 0.055f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensity        = ambientFluidDensity ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction   = 0.01f ;   // Add a little bit of fluid. Note that the fuel-air ratio in a combustion engine is around 0.068.
            emitterTemplate.mFlameFraction  = 0.0f  ;   // No flames initially -- those will occur from ignition.
            emitterTemplate.mSmokeFraction  = 0.99f ;   // Mostly smoke.
        #endif
            emitterSpread.mPosition         = Vec3( 0.5f , 0.5f , 0.5f ) ;

            mVortonPclGrpInfo.mPclOpEmit->mTemplate                     = emitterTemplate ;
            mVortonPclGrpInfo.mPclOpEmit->mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread                       = emitterSpread ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread.mDensity              = 0.0f ;
            mVortonPclGrpInfo.mPclOpEmit->mEmitRate                     = 100.0f ;

            mTracerPclGrpInfo.mPclOpEmit->mEmitRate                     = 5000.0f ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate                     = mVortonPclGrpInfo.mPclOpEmit->mTemplate ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate.mSize               = 0.0095f ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread                       = mVortonPclGrpInfo.mPclOpEmit->mSpread ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.x           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.x ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.y           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.y ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.z           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.z ;

            // Use fire tracer rendering
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_FIRE ) ;

            mVortonPclGrpInfo.mPclOpWind->mWind                         = Vec3( 0.0f , 0.0f , 0.0f ) ;

        #if ENABLE_FIRE
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetCombustionTemperature( 1.0f * Particle_sAmbientTemperature + 1.5f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetCombustionRateFactor( 2.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSmokeTemperature( 2000.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSmokeRateFactor( 5.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSpecificFreeEnergy( 10000.0f ) ;
        #endif

            // Add rigid bodies.
            GetSpheres().PushBack( RbSphere( Vec3(  0.0f , 0.0f , -0.5f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            GetSpheres().PushBack( RbSphere( Vec3(  0.2f , 0.0f ,  2.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1.0f /* veryLargeMass */ , 0.5f ) ) ;
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f ,  1.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , Vec3( 1.0f , 0.5f , 0.1f ) ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.
            GetBoxes().Back().GetBody()->SetAngularVelocity( Vec3( 0.01f , 0.02f , 0.0f ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 1.5f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -3.0f , 1.5f ) ) ;
        }
        break ;

        case 17: // Candle.
        {
            // Set up lights.
            mQdLights[0].mPosition    = Vec3( 0.0f , 0.0f , 0.7f ) ;
            mQdLights[0].mColor       = 1.2f * Vec3( 1.0f , 0.5f , 0.3f ) ;
            mQdLights[0].mAttenuation = Vec3( 0.0f , 0.2f , 0.8f ) ;
            mQdLights[0].mType        = QdLight::LT_POINT ;
            // Make light fluctuate to imply it comes from candle flame.
            mQdLights[0].mAmplitudes [ 1 ] = 0.1f ;
            mQdLights[0].mFrequencies[ 1 ] = 1.0f ;
            mQdLights[0].mAmplitudes [ 2 ] = 0.05f ;
            mQdLights[0].mFrequencies[ 2 ] = 2.0f ;
            mQdLights[0].mAmplitudes [ 3 ] = 0.02f ;
            mQdLights[0].mFrequencies[ 3 ] = 5.0f ;

            mQdLights[1].mPosition    = Vec3( 0.0f , 0.0f , 1.0f ) ; // actually a direction
            mQdLights[1].mColor       = 0.2f * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mQdLights[1].mType        = QdLight::LT_DIRECTIONAL ;

            mVortonPclGrpInfo.mPclOpKillAge->mAgeMax    = 400 ;
            mTracerPclGrpInfo.mPclOpKillAge->mAgeMax    = mVortonPclGrpInfo.mPclOpKillAge->mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition       = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity       = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize           = 0.05f ;
            emitterTemplate.mDensity        = ambientFluidDensity ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction   = 0.01f ;   // Add a little bit of fluid. Note that the fuel-air ratio in a combustion engine is around 0.068.
            emitterTemplate.mFlameFraction  = 0.0f  ;   // No flames initially -- those will occur from ignition.
            emitterTemplate.mSmokeFraction  = 0.99f ;   // Mostly smoke.
        #endif
            emitterSpread.mPosition         = Vec3( 0.2f , 0.2f , 0.2f ) ;

            mVortonPclGrpInfo.mPclOpEmit->mTemplate                     = emitterTemplate ;
            mVortonPclGrpInfo.mPclOpEmit->mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread                       = emitterSpread ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread.mDensity              = 0.0f ;
            mVortonPclGrpInfo.mPclOpEmit->mEmitRate                     = 100.0f ;

            mTracerPclGrpInfo.mPclOpEmit->mEmitRate                     = 1000.0f ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate                     = mVortonPclGrpInfo.mPclOpEmit->mTemplate ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate.mSize               = 0.0085f ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread                       = mVortonPclGrpInfo.mPclOpEmit->mSpread ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.x           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.x ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.y           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.y ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.z           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.z ;

            // Use fire tracer rendering
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_FIRE ) ;

            mVortonPclGrpInfo.mPclOpWind->mWind                         = Vec3( 0.0f , 0.0f , 0.0f ) ;

        #if ENABLE_FIRE
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetCombustionTemperature( Particle_sAmbientTemperature + 1.3f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetCombustionRateFactor( 5.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSmokeTemperature( 2000.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSmokeRateFactor( 10.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSpecificFreeEnergy( 10000.0f ) ;
        #endif

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3(  0.0f , 0.0f , -0.35f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            GetSpheres().PushBack( RbSphere( Vec3(  0.2f , 0.0f ,  5.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1.0f /* veryLargeMass */ , 0.5f ) ) ;
            GetSpheres().PushBack( RbSphere( Vec3( -1.0f , 1.0f ,  1.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.3f ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 1.5f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -3.0f , 1.5f ) ) ;
        }
        break ;

        case 18: // Large flame
        {
            // Set up lights.
            mQdLights[0].mPosition    = Vec3( 0.0f , 0.0f , 0.7f ) ;
            mQdLights[0].mColor       = 1.2f * Vec3( 1.0f , 0.5f , 0.3f ) ;
            mQdLights[0].mAttenuation = Vec3( 0.0f , 0.2f , 0.8f ) ;
            mQdLights[0].mType        = QdLight::LT_POINT ;
            // Make light fluctuate to imply it comes from flames.
            mQdLights[0].mAmplitudes [ 1 ] = 0.2f ;
            mQdLights[0].mFrequencies[ 1 ] = 2.0f ;
            mQdLights[0].mAmplitudes [ 2 ] = 0.1f ;
            mQdLights[0].mFrequencies[ 2 ] = 5.0f ;
            mQdLights[0].mAmplitudes [ 3 ] = 0.05f ;
            mQdLights[0].mFrequencies[ 3 ] = 10.0f ;

            mQdLights[1].mPosition    = Vec3( 0.0f , 0.0f , 1.0f ) ; // actually a direction
            mQdLights[1].mColor       = 0.2f * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mQdLights[1].mType        = QdLight::LT_DIRECTIONAL ;

            mVortonPclGrpInfo.mPclOpKillAge->mAgeMax    = 300 ;
            mTracerPclGrpInfo.mPclOpKillAge->mAgeMax    = mVortonPclGrpInfo.mPclOpKillAge->mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition       = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity       = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize           = 0.0625f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensity        = ambientFluidDensity ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction   = 0.01f ;   // Add a little bit of fluid. Note that the fuel-air ratio in a combustion engine is around 0.068.
            emitterTemplate.mFlameFraction  = 0.0f  ;   // No flames initially -- those will occur from ignition.
            emitterTemplate.mSmokeFraction  = 0.99f ;   // Mostly smoke.
        #endif
            emitterSpread.mPosition         = Vec3( 1.0f , 1.0f , 0.5f ) ;

            mVortonPclGrpInfo.mPclOpEmit->mTemplate                     = emitterTemplate ;
            mVortonPclGrpInfo.mPclOpEmit->mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread                       = emitterSpread ;
            mVortonPclGrpInfo.mPclOpEmit->mSpread.mDensity              = 0.0f ;
            mVortonPclGrpInfo.mPclOpEmit->mEmitRate                     = 200.0f ;

            mTracerPclGrpInfo.mPclOpEmit->mEmitRate                     = 10000.0f ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate                     = mVortonPclGrpInfo.mPclOpEmit->mTemplate ;
            mTracerPclGrpInfo.mPclOpEmit->mTemplate.mSize               = 0.0105f ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread                       = mVortonPclGrpInfo.mPclOpEmit->mSpread ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.x           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.x ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.y           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.y ;
            mTracerPclGrpInfo.mPclOpEmit->mSpread.mPosition.z           = mVortonPclGrpInfo.mPclOpEmit->mSpread.mPosition.z ;

            // Use fire tracer rendering
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_FIRE ) ;

            mVortonPclGrpInfo.mPclOpWind->mWind                         = Vec3( 0.0f , 0.0f , 0.0f ) ;

        #if ENABLE_FIRE
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetCombustionTemperature( 1.0f * Particle_sAmbientTemperature + 1.5f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetCombustionRateFactor( 2.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSmokeTemperature( 5000.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSmokeRateFactor( 2.0f ) ;
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSpecificFreeEnergy( 20000.0f ) ;
        #endif

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3(  0.0f , 0.0f , -0.5f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            GetSpheres().PushBack( RbSphere( Vec3(  0.2f , 0.0f ,  5.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1.0f /* veryLargeMass */ , 0.5f ) ) ;
            GetSpheres().PushBack( RbSphere( Vec3( -1.0f , 1.0f ,  1.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.3f ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 1.5f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -3.0f , 1.5f ) ) ;
        }
        break ;

        case 19: // Stationary cube to test RemoveEmbedded
        case 20: // Stationary cube to test RemoveEmbedded
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 0.5f * fThickness , 0.5f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot ++ ;

            // Add a cube
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.01f ,  0.0f ) , Vec3( 0.00f , 0.0f , 0.0f ) , veryLargeMass , Vec3( 0.5f , 0.5f , 0.5f ) ) ) ;
            {
            Mat33 orientation ;
            orientation.SetRotationX( 0.5f ) ;
            GetBoxes().Back().GetBody()->SetOrientation( orientation ) ; // Orient box
            }

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
        break ;

        ////////////////////////////////////////////////////////////////////////

        case 21: // Gradient sphere, for testing SPH mass density and density gradient calculations.
            numTracersPerCellCubeRoot = 1 ;
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax * 64 , DensityGradientBall( fRadius , 1.0f ) ) ;

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
        break ;

        case 22: // Box projectile
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot ++ ;

            // Add a box
        #if 0   // Cube.
            GetBoxes().PushBack( RbBox( Vec3( -3.0f , 0.01f ,  0.0f ) , Vec3( 4.0f , 0.0f , 0.0f ) , 0.01 , Vec3( 0.2f , 0.2f , 0.2f ) ) ) ;
            {
            Mat33 orientation ;
            orientation.SetRotationY( 0.5f ) ;
            GetBoxes().Back().GetBody()->SetOrientation( orientation ) ; // Orient box
            }
        #else   // Long thin board.
            GetBoxes().PushBack( RbBox( Vec3( -3.0f , 0.0f ,  -0.05f ) , Vec3( 10.0f , 0.0f , 0.0f ) , 0.02 , Vec3( 1.0f , 1.0f , 0.1f ) ) ) ;
            {
            Mat33 orientation ;
            orientation.SetRotationY( -0.2f ) ; // Give plate some angle of attack.
            GetBoxes().Back().GetBody()->SetOrientation( orientation ) ; // Orient box
            }
        #endif

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;

        case 23: // Box projectile with spin about longitudinal axis
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.3f * fThickness , 0.3f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a box.
            GetBoxes().PushBack( RbBox( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 4.0f , 0.0f , 0.0f ) , 0.02f , Vec3( 0.2f , 0.2f , 0.2f ) ) ) ;
            GetBoxes().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.001f , 0.0f , 0.0f ) ) ; // Make box spin

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        break ;

        case 24: // Box projectile with spin about transverse axis
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a box.
            GetBoxes().PushBack( RbBox( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 2.0f , 0.0f , 0.0f ) , 0.015f , Vec3( 0.2f , 0.5f , 0.1f ) ) ) ;
            GetBoxes().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.0f , 0.002f , 0.0f ) ) ; // Make box spin

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mQdCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;

        case 25: // particle fountain
        {
            ParticleSystem * pclSys = CreateNonFluidParticleSystem( 0 ) ;

            mPclSysMgr.PushBack( pclSys ) ;

            // TODO: FIXME: Clone this effect multiple times using new virtual constructors.

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,   0.0f , 2.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -10.0f , 2.0f ) ) ;
        }
        break ;

        case 26: // ball of initially stationary heavy fluid inside a box.
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius , 10.0f ) , Vec3( 0.0f , 0.0f , 1.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot += 2 ;

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Enable SDF generation.
            mTracerPclGrpInfo.mPclOpSeedSurfaceTracers->mSignedDistanceGrid  = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetSignedDistanceGrid() ;

            // Use diagnostic tracer rendering
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_DIAGNOSTIC ) ;

// Disable assigning tracer density from vorton density; for surface tracers, density only indicates "sign" of signed distance.
// TODO: Instead, surface tracers should use truly signed sizes, but currently particle system rejects negative sizes.
mTracerPclGrpInfo.mPclOpAssignDensityFromGrid->mScalarGrid = 0 ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 5.0f , 5.0f , 5.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -4.0f , 4.0f ) ) ;
        break ;

        case 27: // Two boxes of initially stationary fluid, one heavy, one light, light one above heavy one, inside container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) , -0.6f ) , Vec3( 0.0f , 0.0f ,  0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSpecificHeatCapacity( 1.0f ) ; // Diagnosing: Does this influence vorton motion?

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 28: // One box of initially stationary fluid, heavy, at bottom of container, sloshing
        {
            //AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.0f , 1.0f , 1.9f ) ,  1000.0f ) , Vec3( 0.0f , 0.0f ,  0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;

            // Decorations and other rendering options
            //mGridDecorations        = GRID_DECO_POINTS ;
            //mGridField              = GRID_FIELD_DENSITY_GRADIENT ;
            //mVortonRendering        = VORTON_RENDER_DIAGNOSTIC_PARTICLES ;
            //mTracerRendering        = TRACER_RENDER_DYE ;
            //mEmphasizeCameraTarget  = false ;
        }
        break ;

        case 29: // One box of initially stationary fluid, light, at top of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) , -0.6f ) , Vec3( 0.0f , 0.0f ,  0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        ////////////////////////////////////////////////////////////////////////

        case 30: // One box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.5f , 0.5f , 2.0f ) ,  100.0f ) , Vec3( 0.75f , 0.0f , 0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 3.0f , 0.5f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -1.8f , 0.0f ) ) ;
        }
        break ;

        case 31: // One box of initially stationary fluid, heavy, at side of container, sloshing
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.5f , 0.5f , 2.0f ) ,  100.0f ) , Vec3( 0.75f , 0.0f , 0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 3.0f , 0.5f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -1.8f , 0.0f ) ) ;
        }
        break ;

        case 32: // One rotating box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.5f , 0.5f , 2.0f ) ,  1000.0f ) , Vec3( 0.75f , 0.0f , 0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1.0f / FLT_EPSILON , Vec3( 3.0f , 0.5f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ;
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ;
            GetBoxes().Back().GetBody()->SetAngularVelocity( Vec3( 0.0f , 1.0f , 0.0f ) ) ;

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
        }
        break ;

        case 33: // One rotating box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.5f , 0.5f , 2.0f ) ,  1000.0f ) , Vec3( 0.75f , 0.0f , 0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1.0f / FLT_EPSILON , Vec3( 3.0f , 0.5f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ;
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ;
            GetBoxes().Back().GetBody()->SetAngularVelocity( Vec3( 0.0f , 2.0f , 0.0f ) ) ;

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
        }
        break ;

        case 34: // One box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , 2 * numVortonsMax , DensityBox( Vec3( 1.5f , 0.5f , 2.0f ) ,  100.0f ) , Vec3( 0.75f , 0.0f , 0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 3.0f , 0.5f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 35: // Two boxes of initially stationary fluid, one heavy, one light, heavy above light, inside container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) , -0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  0.6f ) , Vec3( 0.0f , 0.0f ,  0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSpecificHeatCapacity( 1.0f ) ; // Diagnosing: Does this influence vorton motion?

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 36: // One box of initially stationary fluid, light, at bottom of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) , -0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 37: // One box of initially stationary fluid, heavy, at top of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  1000.0f ) , Vec3( 0.0f , 0.0f ,  0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 38: // One box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( 0.5 , 1.0f , 2.0f ) ,  1000.0f ) , Vec3( 0.0f , 0.0f ,  0.0 ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 39: // One box of initially stationary fluid, heavy, at middle of container, nearly filling container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.0f , 1.0f , 1.9f ) ,  1000.0f ) , Vec3( 0.0f , 0.0f ,  0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 40: // One box of initially stationary fluid, neutral, at middle of container, nearly filling container.
        {
            //AssignVortons( vortons , 1.0f , numVortonsMax , VortexNoise( Vec3( 1.0f , 1.0f , 1.9f ) ) , Vec3( 0.0f , 0.0f ,  0.0f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.0f , 1.0f , 1.9f ) ,  1.0f ) , Vec3( 0.0f , 0.0f ,  0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , /* isHole */ true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Disable tracer rendering.
            mFluidScene.SetTracerRenderingStyle( FluidScene::TRACER_RENDER_NONE ) ;

            // Use SPH simulation technique.
            vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   )  ;

            // Position and orient camera.
            mQdCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mQdCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

    }

    // Apply same wind to tracers that vortons experience.
    * mTracerPclGrpInfo.mPclOpWind = * mVortonPclGrpInfo.mPclOpWind ;

    // Tell Fluid-Body simulation about rigid bodies.
    GetPhysicalObjects().Clear() ;
    for( size_t iSphere = 0 ; iSphere < GetSpheres().Size() ; ++ iSphere )
    {   // For each sphere in the simulation...
        RbSphere * pSphere = & GetSpheres()[ iSphere ] ;
        GetPhysicalObjects().PushBack( pSphere ) ;
    }
    for( size_t iBox = 0 ; iBox < GetBoxes().Size() ; ++ iBox )
    {   // For each box in the simulation...
        RbBox * pBox = & GetBoxes()[ iBox ] ;
        GetPhysicalObjects().PushBack( pBox ) ;
    }
    if( mInitialized )
    {   // Render API is initialized.  Initial conditions probably changed (i.e. this is not the first call to this function).
        // Can and should create entities now, to bind physical objects to their render objects.
        CreateRigidBodyModelsAndEntities() ;
    }

#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS || POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS || POISON_DENSITY_GRADIENT_BASED_ON_GRIDPOINTS_INSIDE_WALLS || COMPUTE_PRESSURE_GRADIENT
    mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetPhysicalObjects( & GetPhysicalObjects() ) ;
#endif

    if( ! vortonSim.GetVortons()->empty() )
    {   // Initial conditions include pre-existing vortons (in contrast to an effect where all particles are emitted later).
    #if( ENABLE_FLUID_BODY_SIMULATION )
        FluidBodySim::RemoveEmbeddedParticles( reinterpret_cast< VECTOR< Particle > & > ( * vortonSim.GetVortons() ) , GetPhysicalObjects() ) ;
        FluidBodySim::RemoveEmbeddedParticles( mTracerPclGrpInfo.mParticleGroup->GetParticles() , GetPhysicalObjects() ) ;
    #endif

        const bool systemTracksFluidSurfaceUsingSdf = mTracerPclGrpInfo.mPclOpSeedSurfaceTracers->mSignedDistanceGrid != NULLPTR ;

        vortonSim.Initialize( systemTracksFluidSurfaceUsingSdf ) ;
        UniformGridGeometry tracerGrid( vortonSim.GetGrid() ) ;
        tracerGrid.Scale( vortonSim.GetGrid() , tracerGridScale ) ;

        if( systemTracksFluidSurfaceUsingSdf )
        {   // This case uses SDF to track fluid surface.
            // Emit initial tracers near SDF zero-crossing.
            PclOpSeedSurfaceTracers::Emit( mTracerPclGrpInfo.mParticleGroup->GetParticles() , 4 , mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetSignedDistanceGrid() , PclOpSeedSurfaceTracers::sBandWidthAutomatic , ambientFluidDensity ) ;
        }
        else
        {   // Tracers do not track SDF.
            // Emit initial tracers based on initial vorton locations.
            PclOpEmit::Emit( mTracerPclGrpInfo.mParticleGroup->GetParticles() , tracerGrid , numTracersPerCellCubeRoot , vortonsImplyingTracers ) ;
        }

        if( ! systemTracksFluidSurfaceUsingSdf
            && vortonSim.GetFluidSimulationTechnique() == VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS )
        {   // Using SPH but not tracking SDF from tracers.
            // For visualization, populate density grid from vortons and populate SDF from density grid.
            vortonSim.SetPopulateSdfFromDensity( true ) ;
        }

    #if( ENABLE_FLUID_BODY_SIMULATION )
        FluidBodySim::RemoveEmbeddedParticles( reinterpret_cast< VECTOR< Particle > & >( * vortonSim.GetVortons() ) , GetPhysicalObjects() ) ;
        FluidBodySim::RemoveEmbeddedParticles( mTracerPclGrpInfo.mParticleGroup->GetParticles() , GetPhysicalObjects() ) ;
    #endif
    }

    QueryPerformanceCounter( & mSystemTimeBefore ) ; // Reset system timer

#if PROFILE
    printf( "Initial condition %i, simulation objects: %i tracers    %i vortons    %i spheres %i boxes\n"
            , ic , mTracerPclGrpInfo.mParticleGroup->GetNumParticles() , vortons.Size() , GetSpheres().Size() , GetBoxes().size() ) ;
#endif

}




/** Update particle systems.
*/
void InteSiVis::UpdateParticleSystems()
{
    PERF_BLOCK( InteSiVis__UpdateParticleSystems ) ;

    if( mTimeStepping == PLAY || mTimeStepping == SINGLE_STEP )
    {
        #if USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
        if(     ( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS == mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFluidSimulationTechnique() )
            ||  ( VortonSim::FLUID_SIM_VPM_SPH_HYBRID                  == mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFluidSimulationTechnique() )
            //||  ( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD          == mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFluidSimulationTechnique() )
            )
        {
            // Set up particle operation to re-populate velocity grid from SPH particles, to take into account SPH particle velocity after satisfying boundary conditions.
            // This is meant to keep tracers from heading where they should not.
            // In principle this should also occur for VPM, but it's only implemented for SPH, because of the problems created by not assigning densities, which the comments below describe.
            ASSERT( mVortonPclGrpInfo.mPclOpPopulateVelocityGrid ) ;
            mVortonPclGrpInfo.mPclOpPopulateVelocityGrid->mVelocityGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid() ;

            // In SPH mode, the density grid isn't updated, so has invalid bounds, which would make AssignScalarFromGrid fail.
            // Clear density grids to avoid that problem.
            // Note that in this mode, because density grid is not populated, tracers do not get their densities updated from vortons.
            // That has 2 effects: linear impulses are different, and rendering is different.
            const_cast< UniformGrid< float > & >( mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetDensityGrid() ) .Clear() ;

        #if ENABLE_FIRE
            const_cast< UniformGrid< float > & >( mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFuelGrid() ).Clear() ;
            const_cast< UniformGrid< float > & >( mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFlameGrid() ).Clear() ;
            const_cast< UniformGrid< float > & >( mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetSmokeGrid() ).Clear() ;
        #endif
        }
        else
        #endif
        {
            ASSERT( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD == mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFluidSimulationTechnique() ) ;
            if( mVortonPclGrpInfo.mPclOpPopulateVelocityGrid )
            {
                mVortonPclGrpInfo.mPclOpPopulateVelocityGrid->mVelocityGrid = 0 ;
            }
        }

        mPclSysMgr.Update( mTimeStep , mFrame ) ;

        #if 0 && USE_SMOOTHED_PARTICLE_HYDRODYNAMICS // Silly VPM-SPH hybrid: Alternate between VPM and SPH.  This yields about the same results as running SPH after VPM each frame.
        if( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS == mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetFluidSimulationTechnique() )
        {   // Previous fluid simulation update used SPH.
            // Switch to VPM for next Update.
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD ) ;
        }
        else
        {   // Previous fluid simulation update used VPM.
            // Switch to SPH for next Update.
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS ) ;
        }
        #endif
    }

#if 0 // DO NOT SUBMIT -- unit testing
    {
        ParticleSystemManager * pclSysMgr = new ParticleSystemManager( mPclSysMgr ) ;
        pclSysMgr->Update( 0.0 , 0 ) ;
        mPclSysMgr.Update( mTimeStep , mFrame ) ;
        delete pclSysMgr ;
        mPclSysMgr.Update( mTimeStep , mFrame ) ;
    }
#endif
}




/** Update rigid bodies.
*/
void InteSiVis::UpdateRigidBodies()
{
    PERF_BLOCK( InteSiVis__UpdateRigidBodies ) ;

    if( mTimeStepping == PLAY || mTimeStepping == SINGLE_STEP )
    {
        if( 28 == mScenario )
        {   // Box of fluid.
            ASSERT( GetBoxes().Size() == 1 ) ;
// TODO: eliminate this compile-time conditional.
#if USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
            static Vec3 amplitude( 0.5f , 0.2f , 0.05f ) ;
            static Vec3 frequency( 1.0f , 2.0f , 0.1f ) ;
#else
            static Vec3 amplitude( 0.0f * 0.5f , 0.0f , 0.0f ) ;
            static Vec3 frequency( 0.2f , 0.0f , 0.0f ) ;
#endif
            if( ( amplitude.Mag2() != 0.0f ) && ( frequency.Mag2() != 0.0f ) )
            {
                RbBox & box = GetBoxes().Back() ;
                Impulsion::RigidBody * rigidBody = box.GetBody() ;
                Vec3 position = rigidBody->GetPosition() ;
                // Make box slosh back and forth.
                position.x = amplitude.x * sin( mTimeNow * frequency.x ) ;
                position.y = amplitude.y * cos( mTimeNow * frequency.y ) ;
                position.z = amplitude.z * sin( mTimeNow * frequency.z ) ;
                rigidBody->SetPosition( position ) ;
            }
        }
        else if( 31 == mScenario )
        {   // Box of fluid.
            ASSERT( GetBoxes().Size() == 1 ) ;
            static Vec3 amplitude( 1.0f , 0.0f , 0.0f ) ;
            static Vec3 frequency( 2.0f , 0.0f , 0.0f ) ;
            if( ( amplitude.Mag2() != 0.0f ) && ( frequency.Mag2() != 0.0f ) )
            {
                RbBox & box = GetBoxes().Back() ;
                Impulsion::RigidBody * rigidBody = box.GetBody() ;
                Vec3 position = rigidBody->GetPosition() ;
                // Make box slosh back and forth.
                position.x = amplitude.x * pow( sin( mTimeNow * frequency.x ) , 3 ) ;
                position.y = amplitude.y * cos( mTimeNow * frequency.y ) ;
                position.z = amplitude.z * sin( mTimeNow * frequency.z ) ;
                rigidBody->SetPosition( position ) ;
            }
        }
        PhysicalObject_UpdateSystem( GetPhysicalObjects() , mTimeStep , mFrame ) ;
    }
}




/** Copy light parameters from QdLights to PeGaSys::Render::Lights
*/
void InteSiVis::CopyLightsFromQdToPeGaSys()
{
    PERF_BLOCK( InteSiVis__CopyLightsFromQdToPeGaSys ) ;

    using namespace PeGaSys::Render ;

    // Set PeGaSys::Lights from QdLights.  This is a temporary workaround until switching completely to PeGaSys::Render.
    const size_t numLights = mQdLights.Size() ;
    ASSERT( numLights == FluidScene::NUM_LIGHTS ) ;
    for( size_t idxLight = 0 ; idxLight < numLights ; ++ idxLight )
    {
        PeGaSys::Render::Light * pgsLight = mFluidScene.GetLight( idxLight ) ;
        QdLight & qdLight = mQdLights[ idxLight ] ;
        pgsLight->SetPosition( qdLight.mPosition ) ;
        pgsLight->SetDirection( - qdLight.mPosition ) ;
        pgsLight->SetAttenuation( qdLight.mAttenuation.x , qdLight.mAttenuation.y , qdLight.mAttenuation.z ) ;
        switch( qdLight.mType )
        {
        case QdLight::LT_POINT      : pgsLight->SetLightType( Light::POINT       ) ; break ;
        case QdLight::LT_SPOT       : pgsLight->SetLightType( Light::SPOT        ) ; break ;
        case QdLight::LT_DIRECTIONAL: pgsLight->SetLightType( Light::DIRECTIONAL ) ; break ;
        default: FAIL() ; break ;
        }
        FluidScene::LightAnimation & anim = mFluidScene.GetLightAnim( idxLight ) ;
        for( int iSpecComp = 0 ; iSpecComp < FluidScene::NUM_SPECTRAL_COMPONENTS ; ++ iSpecComp )
        {
            anim.mAnimComponents[ iSpecComp ].mAmplitude  = qdLight.mColor * qdLight.mAmplitudes[ iSpecComp ] ;
            anim.mAnimComponents[ iSpecComp ].mFrequency  = qdLight.mFrequencies[ iSpecComp ] ;
            anim.mAnimComponents[ iSpecComp ].mPhaseShift = 0.0f ;
        }
    }
}




/** Update rigid body entities -- transfer physical object position and orientation into model.
*/
void InteSiVis::UpdateRigidBodyModelsFromPhysics()
{
    PERF_BLOCK( InteSiVis__UpdateRigidBodyModelsFromPhysics ) ;

    const size_t numEntities = mEntities.Size() ;
    for( size_t idxEntity = 0 ; idxEntity < numEntities ; ++ idxEntity )
    {
        Entity & entity = mEntities[ idxEntity ] ;
        entity.Update() ;
        //entity.mRenderModel->Render( & mQdLights , static_cast< float >( mTimeNow ) ) ; OBSOLETE QdRender
    }
}




/** Gather and record performance profile data.
*/
void InteSiVis::GatherAndRecordProfileData()
{
    PERF_BLOCK( InteSiVis__GatherAndRecordProfileData ) ;

#if defined( PROFILE )
    VortonSim & rVortonSim = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;

    if( mFrame >= sFrameCountMax )
    {   // Reached maximum number of frames, so exit.
        printf( "InteSiVis Ran %i frames\n" , mFrame ) ;
        printf( "region: { %g , %g , %g } , { %g , %g , %g } \n"
            , rVortonSim.GetMinCornerEternal().x
            , rVortonSim.GetMinCornerEternal().y
            , rVortonSim.GetMinCornerEternal().z
            , rVortonSim.GetMaxCornerEternal().x
            , rVortonSim.GetMaxCornerEternal().y
            , rVortonSim.GetMaxCornerEternal().z
            ) ;
        const float tracersPerFrame = (float) sNumTracersSum / (float) ( mFrame - 1 ) ;
        const float vortonsPerFrame = (float) sNumVortonsSum / (float) ( mFrame - 1 ) ;
        printf( "Particles, average per frame: %g tracers    %g vortons\n" , tracersPerFrame , vortonsPerFrame ) ;

        const float tracerHitsPerFrame = (float) FluidBodySim::GetNumTracerBodyHits() / (float) ( mFrame - 1 ) ;
        const float vortonHitsPerFrame = (float) FluidBodySim::GetNumVortonBodyHits() / (float) ( mFrame - 1 ) ;
        printf( "Particles-body hits, average per frame: %g tracers    %g vortons\n" , tracerHitsPerFrame , vortonHitsPerFrame ) ;
        printf( "InteSiVis benchmark DONE: build " __DATE__ " " __TIME__ "\n\n" ) ;

        exit( 0 ) ;
    }
#endif
}




/** Function that GLUT calls when window resizes.
*/
/* static */ void InteSiVis::GlutReshapeGlutCallback(int width, int height)
{
    PERF_BLOCK( InteSiVis__GlutReshapeGlutCallback ) ;

    //const int currentGlutWindow = glutGetWindow() ;
    const int   width_4 = (width / 4) * 4;
    sInstance->mQdCamera.SetViewport( width_4 ,height ) ;
    glViewport( 0 , 0 , sInstance->mQdCamera.GetWidth() , sInstance->mQdCamera.GetHeight() ) ;
    glutReshapeWindow( width_4 , height ) ;
}




/** Copy camera info from QdRender into PeGaSys::Render.
    TODO: FIXME: This is done because at the moment the GUI controls the QdCamera.  Eventually the GUI controls should directly operate on PeGaSys::Render camera, at which point this can go away.
    NOTE: The QD window size does NOT determine the PeGaSys window; it's the other way around.  See GlutReshapeGlutCallback.
*/
void InteSiVis::CopyCameraFromQdToPeGaSys()
{
    PERF_BLOCK( InteSiVis__CopyCameraFromQdToPeGaSys ) ;

    PeGaSys::Render::Camera *   pgsCam  = sInstance->mFluidScene.GetCamera() ;
    const QdCamera &            qdCam   = sInstance->mQdCamera ;
    pgsCam->SetPerspective( qdCam.GetFieldOfViewVertical() , qdCam.GetAspectRatio() , qdCam.GetNearClipDist() , qdCam.GetFarClipDist() ) ;
    pgsCam->SetEye( qdCam.GetEye() ) ;
    pgsCam->SetLookAt( qdCam.GetTarget() ) ;
}




/** Function that GLUT calls to display contents of window.
*/
/* static */ void InteSiVis::GlutDisplayCallback()
{
    PERF_BLOCK( InteSiVis__GlutDisplayCallback ) ;

    {
        if( ! sInstance->mInitialized )
        {   // This is the first frame this app has tried to display anything.
            sInstance->InitializeQdRendering() ;
            sInstance->CreateRigidBodyModelsAndEntities() ;
        }

        CheckGlError() ;
    }

    sInstance->UpdateRigidBodyModelsFromPhysics() ;

    sInstance->CopyCameraFromQdToPeGaSys() ;

    // Animate lights.
    sInstance->mFluidScene.AnimateLights( sInstance->mTimeNow ) ;

    // Update fluid isosurface model.
    {
        VortonSim & vortonSim = sInstance->mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
        if( ! vortonSim.GetSignedDistanceGrid().Empty() )
        {
            sInstance->mFluidScene.UpdateFluidIsosurface( vortonSim.GetSignedDistanceGrid() ) ;
        }
        //else if( ! vortonSim.GetDensityGrid().Empty() )
        //{
        //    sInstance->mFluidScene.UpdateFluidIsosurface( vortonSim.GetDensityGrid() ) ;
        //}
    }

    // Render scene using PeGaSys::Render.
    sInstance->mRenderSystem.UpdateTargets( sInstance->mTimeNow ) ;

    // Render rest of scene using QdRender.
    // TODO: Remove or replace most of below; PeGaSys::Render update above should suffice, once particles & diagnostic rendering use that.
    {
        CheckGlError() ;

        sInstance->mQdCamera.SetCamera() ;
        sInstance->QdRenderDiagnosticGrid() ;
        sInstance->QdRenderParticleDiagnostics() ;
        sInstance->QdRenderSummaryDiagnosticText() ;

        CheckGlError() ;
    }

    {
        // TODO: This call to glutSwapBuffers probably ought to happen in a render routine.  Likewise see call to Present in D3D version.
        // For now it is useful not to swap buffers within UpdateTargets because it lets QdRender composite into same screen buffer.
        PERF_BLOCK( glutSwapBuffers ) ;
        glutSwapBuffers() ;
    }

#if PROFILE > 1
    sInstance->GatherAndRecordProfileData() ;
#endif
}




/// Return PhysicalObject in focus, if any.
Impulsion::PhysicalObject * InteSiVis::GetPhysicalObjectInFocus()
{
    PERF_BLOCK( InteSiVis__GetPhysicalObjectInFocus ) ;

    if( ! GetPhysicalObjects().Empty() )
    {   // Simulation has at at least one PhysicalObject.
        mPhysObjFocus %= GetPhysicalObjects().Size() ;  // Make sure mPhysObjFocus has a legitimate value for current scenario.
        return GetPhysicalObjects()[ mPhysObjFocus ] ;
    }
    else
    {   // Simulation has no PhysicalObjects so return nothing.
        mPhysObjFocus = 0 ;  // Actual value does not matter; code elsewhere cannot legally reference mPhysObjFocus if mPhysicalObjects is empty.
        return 0 ;
    }
}




/** Function to handle mouse motion.
*/
/* static */ void InteSiVis::MouseMotionHandler( int x , int y , int modifierKeys )
{
    PERF_BLOCK( InteSiVis__MouseMotionHandler ) ;

    if( -999 == sMousePrevX )
    {
        sMousePrevX = x ;
        sMousePrevY = y ;
        return ;
    }

    static float fMouseMotionSensitivity = 0.005f ;    // Tune based on how fast you want mouse to move camera.

    const float dx = Clamp( fMouseMotionSensitivity * float( x - sMousePrevX ) , -100.0f , 100.0f ) ;
    const float dy = Clamp( fMouseMotionSensitivity * float( y - sMousePrevY ) , -100.0f , 100.0f ) ;

    if( mMouseButtons[0] )
    {   // User is left-click-dragging mouse
        if( GLUT_ACTIVE_SHIFT == modifierKeys )
        {   // Shift (only) held.
            Vec3                        angles      ;
            Impulsion::PhysicalObject * physObj     = GetPhysicalObjectInFocus() ;
            if( physObj )
            {
                Impulsion::RigidBody *  rigidBody   = physObj->GetBody() ;
                const Mat33 &           orientOld   = rigidBody->GetOrientation() ;
                orientOld.GetAngles( angles ) ;
                angles.x += dy ;
                angles.z += dx ;
                Mat33 orientNew ;
                orientNew.SetRotationXYZ( angles ) ;
                rigidBody->SetOrientation( orientNew ) ;
            }
        }
        else
        {
            QdCamera & qdCam = mQdCamera ;
            float azimuth , elevation , radius ;
            // Obtain previous camera parameters.
            qdCam.GetOrbit( azimuth , elevation , radius ) ;
            // Avoid gimbal lock by limiting elevation angle to avoid the poles.
            static const float sAvoidPoles = 0.001f ;
            elevation = Clamp( elevation - dy , sAvoidPoles , PI * ( 1.0f - sAvoidPoles ) ) ;
            // Set new camera parameters based on how much mouse moved.
            qdCam.SetOrbit( azimuth - dx , elevation , radius ) ;
        }
    }
    else if( mMouseButtons[2] )
    {   // User is right-click-dragging mouse
        if( GLUT_ACTIVE_SHIFT == modifierKeys )
        {   // Shift (only) held.
            Impulsion::PhysicalObject * physObj     = GetPhysicalObjectInFocus() ;
            if( physObj )
            {
                Mat44 viewMatrix ;
                glGetFloatv( GL_MODELVIEW_MATRIX , (GLfloat *) & viewMatrix ) ;
                Vec3 viewRight  ( viewMatrix.m[0][0] , viewMatrix.m[1][0] , viewMatrix.m[2][0] ) ;
                Vec3 viewUp     ( viewMatrix.m[0][1] , viewMatrix.m[1][1] , viewMatrix.m[2][1] ) ;
                Vec3 viewForward( viewMatrix.m[0][2] , viewMatrix.m[1][2] , viewMatrix.m[2][2] ) ;
                Vec3 delta      = 1.0f * viewForward * ( dx + dy ) ;

                Impulsion::RigidBody *  rigidBody   = physObj->GetBody() ;
                Vec3                    trans       = rigidBody->GetPosition() ;
                trans.x += delta.x ;
                trans.y += delta.y ;
                trans.z += delta.z ;
                rigidBody->SetPosition( trans ) ;            }
        }
        else
        {
            QdCamera & qdCam = mQdCamera ;
            float azimuth , elevation , radius ;
            // Obtain previous camera parameters.
            qdCam.GetOrbit( azimuth , elevation , radius ) ;
            // Set new camera parameters based on how much mouse moved.
            const float newRadius = Max2( radius - ( dx + dy ) , 1.0e-4f ) ;
            qdCam.SetOrbit( azimuth , elevation , newRadius ) ;
        }
    }
    else if( mMouseButtons[1] )
    {   // User is middle-click-dragging mouse
        // Extract world space direction vectors associated with view (used to compute camera-facing coordinates).
        // Note that these vectors are the unit vectors of the inverse of the view matrix.
        // They are the world-space unit vectors of the view transformation.
        Mat44 viewMatrix ;
        glGetFloatv( GL_MODELVIEW_MATRIX , (GLfloat *) & viewMatrix ) ;
        Vec3 viewRight  ( viewMatrix.m[0][0] , viewMatrix.m[1][0] , viewMatrix.m[2][0] ) ;
        Vec3 viewUp     ( viewMatrix.m[0][1] , viewMatrix.m[1][1] , viewMatrix.m[2][1] ) ;
        Vec3 viewForward( viewMatrix.m[0][2] , viewMatrix.m[1][2] , viewMatrix.m[2][2] ) ;
        Vec3 delta      = 1.0f * ( - viewRight * dx + viewUp * dy ) ;
        if( GLUT_ACTIVE_SHIFT == modifierKeys )
        {   // Shift (only) held.
            Impulsion::PhysicalObject * physObj     = GetPhysicalObjectInFocus() ;
            if( physObj )
            {
                Impulsion::RigidBody *  rigidBody   = physObj->GetBody() ;
                Vec3                    trans       = rigidBody->GetPosition() ;
                trans.x -= delta.x ;
                trans.y -= delta.y ;
                trans.z -= delta.z ;
                rigidBody->SetPosition( trans ) ;
            }
        }
        else
        {
            QdCamera & qdCam = mQdCamera ;
            const Vec3 & rEye    = qdCam.GetEye() ;
            const Vec3 & rTarget = qdCam.GetTarget() ;
            qdCam.SetEye   ( Vec3( rEye.x    + delta.x , rEye.y    + delta.y , rEye.z    + delta.z ) ) ;
            qdCam.SetTarget( Vec3( rTarget.x + delta.x , rTarget.y + delta.y , rTarget.z + delta.z ) ) ;
        }
    }

    sMousePrevX = x ;
    sMousePrevY = y ;
}




/** Function to run to handle "idle" event, that is, when not doing anything else.
*/
void InteSiVis::Idle()
{
    PERF_BLOCK_CROSS_FRAME_BOUNDARY( false ) ;

    PERF_BLOCK( InteSiVis__Idle ) ;

#if USE_TBB
    tbb::tick_count time0 = tbb::tick_count::now() ;
#endif

    const float originalTimeStep = mTimeStep ; // Remember dictated timeStep in case this changes it and we need to restore it.
    if( ( PLAY == mTimeStepping ) && ( mTimeStep > 0.0f ) )
    {   // Simulation is playing forward normally.

        // Poll system timer.
        LARGE_INTEGER systemTimeNow ;
        QueryPerformanceCounter( & systemTimeNow ) ;

        // Obtain system timer characteristics.
        LARGE_INTEGER    qwTicksPerSec      ;   // Performance counter property; number of ticks per second.
        QueryPerformanceFrequency( & qwTicksPerSec ) ;
        const float secondsPerTick  = 1.0f / float( qwTicksPerSec.QuadPart ) ; // Floating-point reciprocal of ticks-per-second

        LARGE_INTEGER frameDurationInTicks ;
        frameDurationInTicks.QuadPart = systemTimeNow.QuadPart - mSystemTimeBefore.QuadPart ;
        mSystemTimeBefore = systemTimeNow ;
        const float frameDurationInSeconds = secondsPerTick * static_cast< float >( frameDurationInTicks.QuadPart ) ;

        const float delta = frameDurationInSeconds - mFrameDurSecAvg ;
        const float gain = 0.125f ;
        mFrameDurSecAvg += gain * delta ;

        //mTimeStep = Clamp( frameDurationInSeconds , 1.0f * mTimeStep , 4.0f * mTimeStep ) ; // Synchronize virtual time to real time (within some limit).
    }
    mTimeStepMin = Min2( mTimeStepMin , mTimeStep ) ;
    mTimeStepMax = Max2( mTimeStepMax , mTimeStep ) ;

    UpdateParticleSystems() ;

#if USE_TBB
    tbb::tick_count timeFinal = tbb::tick_count::now() ;
    //printf( " tbb duration=%g second\n" , (timeFinal - time0).seconds() ) ;
#endif

    UpdateRigidBodies() ;

    mQdCamera.Update() ;

    InteSiVis::GlutDisplayCallback() ;

    if( mTimeStepping == PLAY || mTimeStepping == SINGLE_STEP )
    {   // Step time by 1.
        if( mTimeStep > 0.0f )
        {   // Step time forward.
            ++ mFrame ;
        }
        else if( mTimeStep < 0.0f )
        {   // Step time backward.
            -- mFrame ;
        }
        mTimeNow += mTimeStep ;
        if( mTimeStepping == SINGLE_STEP )
        {   // Now that simulation single-stepped, return to pause mode.
            mTimeStepping = PAUSE ;
        }
    }

#if PROFILE
    sNumTracersSum += mTracerPclGrpInfo.mParticleGroup->GetNumParticles() ;
    sNumVortonsSum += mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortons()->Size() ;
#endif

    mTimeStep = originalTimeStep ;
}




/** Function that GLUT calls when nothing else is happening.
*/
/* static */ void InteSiVis::GlutIdleCallback()
{
    //PERF_BLOCK( InteSiVis__GlutIdleCallback ) ;

    sInstance->Idle() ;
}




/** Function to handle a special key.
*/
/* static */ void InteSiVis::SpecialKeyHandler( int key , const int modifierKeys , int /* windowRelativeMouseX */ , int /* windowRelativeMouseY */ )
{
    PERF_BLOCK( InteSiVis__SpecialKeyHandler ) ;

    if( ! modifierKeys )
    {   // No modifier keys (Shift, Ctrl or Alt) were pressed.
        static const float timeFactor = 1.0f ;

        switch( key )
        {
            case GLUT_KEY_F1 : InitialConditions(  1 ) ; break;
            case GLUT_KEY_F2 : InitialConditions(  2 ) ; break;
            case GLUT_KEY_F3 : InitialConditions(  3 ) ; break;
            case GLUT_KEY_F4 : InitialConditions(  4 ) ; break;
            case GLUT_KEY_F5 : InitialConditions(  5 ) ; break;
            case GLUT_KEY_F6 : InitialConditions(  6 ) ; break;
            case GLUT_KEY_F7 : InitialConditions(  7 ) ; break;
            case GLUT_KEY_F8 : InitialConditions(  8 ) ; break;
            case GLUT_KEY_F9 : InitialConditions(  9 ) ; break;
            case GLUT_KEY_F11: InitialConditions( 10 ) ; break;

            case GLUT_KEY_UP   : mTimeStep =  sTimeStep * timeFactor ; mTimeStepping   = PLAY  ; break ;
            case GLUT_KEY_DOWN : mTimeStep =  sTimeStep * timeFactor ; mTimeStepping   = PAUSE ; break ;
            case GLUT_KEY_RIGHT: mTimeStep =  sTimeStep * timeFactor ; if( mTimeStepping == PAUSE ) mTimeStepping = SINGLE_STEP ; break ;
            case GLUT_KEY_LEFT : mTimeStep = -sTimeStep * timeFactor ; if( mTimeStepping == PAUSE ) mTimeStepping = SINGLE_STEP ; break ;

            default:
            return;
            break;
        }
    }
    else if( GLUT_ACTIVE_SHIFT == modifierKeys )
    {   // Shift (only) held while special key pressed.
        switch( key )
        {
            case GLUT_KEY_F1 : InitialConditions( 11 ) ; break;
            case GLUT_KEY_F2 : InitialConditions( 12 ) ; break;
            case GLUT_KEY_F3 : InitialConditions( 13 ) ; break;
            case GLUT_KEY_F4 : InitialConditions( 14 ) ; break;
            case GLUT_KEY_F5 : InitialConditions( 15 ) ; break;
            case GLUT_KEY_F6 : InitialConditions( 16 ) ; break;
            case GLUT_KEY_F7 : InitialConditions( 17 ) ; break;
            case GLUT_KEY_F8 : InitialConditions( 18 ) ; break;
            case GLUT_KEY_F9 : InitialConditions( 19 ) ; break;
            case GLUT_KEY_F11: InitialConditions( 20 ) ; break;

            case GLUT_KEY_UP   : mPhysObjFocus  ++ ; break ;
            case GLUT_KEY_DOWN : mPhysObjFocus  -- ; break ;
            case GLUT_KEY_RIGHT: mPhysObjFocus  ++ ; break ;
            case GLUT_KEY_LEFT : mPhysObjFocus  -- ; break ;

            default:
            return;
            break;
        }
    }
    else if( GLUT_ACTIVE_CTRL == modifierKeys )
    {   // Ctrl (only) held while special key pressed.
        static const float timeFactor = 0.001f ;

        switch( key )
        {
            case GLUT_KEY_F1 : InitialConditions( 21 ) ; break;
            case GLUT_KEY_F2 : InitialConditions( 22 ) ; break;
            case GLUT_KEY_F3 : InitialConditions( 23 ) ; break;
            case GLUT_KEY_F4 : InitialConditions( 24 ) ; break;
            case GLUT_KEY_F5 : InitialConditions( 25 ) ; break;
            case GLUT_KEY_F6 : InitialConditions( 26 ) ; break;
            case GLUT_KEY_F7 : InitialConditions( 27 ) ; break;
            case GLUT_KEY_F8 : InitialConditions( 28 ) ; break;
            case GLUT_KEY_F9 : InitialConditions( 29 ) ; break;
            case GLUT_KEY_F11: InitialConditions( 30 ) ; break;

            case GLUT_KEY_UP   : mTimeStep =  sTimeStep * timeFactor ; mTimeStepping   = PLAY  ; break ;
            case GLUT_KEY_DOWN : mTimeStep =  sTimeStep * timeFactor ; mTimeStepping   = PAUSE ; break ;
            case GLUT_KEY_RIGHT: mTimeStep =  sTimeStep * timeFactor ; if( mTimeStepping == PAUSE ) mTimeStepping = SINGLE_STEP ; break ;
            case GLUT_KEY_LEFT : mTimeStep = -sTimeStep * timeFactor ; if( mTimeStepping == PAUSE ) mTimeStepping = SINGLE_STEP ; break ;

            default:
            return;
            break;
        }
    }
    else if( GLUT_ACTIVE_ALT == modifierKeys )
    {   // Alt (only) held while special key pressed.
        static const float timeFactor = 2.0f ;

        switch( key )
        {
            case GLUT_KEY_F1 : InitialConditions( 31 ) ; break;
            case GLUT_KEY_F2 : InitialConditions( 32 ) ; break;
            case GLUT_KEY_F3 : InitialConditions( 33 ) ; break;
            case GLUT_KEY_F4 : InitialConditions( 34 ) ; break;
            case GLUT_KEY_F5 : InitialConditions( 35 ) ; break;
            case GLUT_KEY_F6 : InitialConditions( 36 ) ; break;
            case GLUT_KEY_F7 : InitialConditions( 37 ) ; break;
            case GLUT_KEY_F8 : InitialConditions( 38 ) ; break;
            case GLUT_KEY_F9 : InitialConditions( 39 ) ; break;
            case GLUT_KEY_F11: InitialConditions( 40 ) ; break;

            case GLUT_KEY_UP   : mTimeStep =  sTimeStep * timeFactor ; mTimeStepping   = PLAY  ; break ;
            case GLUT_KEY_DOWN : mTimeStep =  sTimeStep * timeFactor ; mTimeStepping   = PAUSE ; break ;
            case GLUT_KEY_RIGHT: mTimeStep =  sTimeStep * timeFactor ; if( mTimeStepping == PAUSE ) mTimeStepping = SINGLE_STEP ; break ;
            case GLUT_KEY_LEFT : mTimeStep = -sTimeStep * timeFactor ; if( mTimeStepping == PAUSE ) mTimeStepping = SINGLE_STEP ; break ;

            default:
            return;
            break;
        }
    }

    // Make sure mPhysObjFocus has a legitimate value for the current scenario.
    if( ! mPhysicalObjects.Empty() )
    {
        mPhysObjFocus %= mPhysicalObjects.size() ;
    }
    else
    {   // Avoid divide-by-zero.
        mPhysObjFocus = 0 ;  // Actual value does not matter; code elsewhere cannot legally reference mPhysObjFocus if mPhysicalObjects is empty.
    }
}





/** Function that GLUT calls to handle special key events.
*/
/* static */ void InteSiVis::GlutSpecialKeyHandler( int key , int windowRelativeMouseX , int windowRelativeMouseY )
{
    PERF_BLOCK( InteSiVis__GlutSpecialKeyHandler ) ;

    sInstance->mModifierKeys = glutGetModifiers() ;

    sInstance->SpecialKeyHandler( key , sInstance->mModifierKeys , windowRelativeMouseX , windowRelativeMouseY ) ;

    glutPostRedisplay() ;
}




/** Function to handle a regular key.
*/
void InteSiVis::KeyboardHandler( unsigned char key , int /* mouseX */ , int /* mouseY */ )
{
    PERF_BLOCK( InteSiVis__KeyboardHandler ) ;

    mModifierKeys = glutGetModifiers() ;

    QdCamera & cam = mQdCamera ;
    float azimuth , elevation , radius ;
    static bool animateCamera = false ;
    cam.GetOrbit( azimuth , elevation , radius ) ;
    switch( key )
    {
        case '.': radius -= 0.1f ; break ;  // Move camera in.
        case ',': radius += 0.1f ; break ;  // Move camera out.

        case 'c':   // Toggle camera rotation.
        case 'C':
        {
            animateCamera = ! animateCamera ;
            if( animateCamera )
            {   // Camera animation is now on.
                if( 'c' == key )
                {   // Rotate azimuth faster.
                    cam.SetOrbitalTrajectory( Vec3( -0.005f , -0.00f , 0.0f ) ) ;
                }
                else
                {   // Rotate elevation faster.
                    cam.SetOrbitalTrajectory( Vec3( -0.002f , -0.005f , 0.0f ) ) ;
                }
            }
            else
            {
                cam.SetOrbitalTrajectory( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            }
        }
        break ;

        case 'd':   // Cycle through diagnostic text options.
        case 'D':
        {
            switch( mDiagnosticText )
            {
                case DIAG_TEXT_NONE   :
                    mDiagnosticText = DIAG_TEXT_TIMING ;
                    mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetTallyDiagnosticIntegrals( false ) ;
                    break ;
                case DIAG_TEXT_TIMING:
                    mDiagnosticText = DIAG_TEXT_SUMMARY ;
                    mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetTallyDiagnosticIntegrals( true ) ;
                    break ;
                case DIAG_TEXT_SUMMARY:
                    mDiagnosticText = DIAG_TEXT_FULL    ;
                    mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetTallyDiagnosticIntegrals( true ) ;
                    break ;
                case DIAG_TEXT_FULL   :
                    mDiagnosticText = DIAG_TEXT_NONE    ;
                    mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetTallyDiagnosticIntegrals( false ) ;
                    break ;
            }
        }
        break ;

        case 'e':   // Toggle emphasis on objects near camera look-at location.
        case 'E':
        {
            mEmphasizeCameraTarget = ! mEmphasizeCameraTarget ;
        }
        break ;

        case 'f':   // Cycle through which field to render as grid.
        case 'F':
        {
            switch( mGridField )
            {
                case GRID_FIELD_VELOCITY            : mGridField = GRID_FIELD_DENSITY_GRADIENT  ; break ;
                case GRID_FIELD_DENSITY_GRADIENT    : mGridField = 
                #if COMPUTE_PRESSURE_GRADIENT
                                                                    GRID_FIELD_PRESSURE_GRADIENT
                #else
                                                                    GRID_FIELD_NEGATIVE_VORTICITY
                #endif
                                                                                                ; break ;
                case GRID_FIELD_PRESSURE_GRADIENT   : mGridField = GRID_FIELD_NEGATIVE_VORTICITY; break ;

                case GRID_FIELD_NEGATIVE_VORTICITY  : mGridField = GRID_FIELD_VECTOR_POTENTIAL  ; break ;
                case GRID_FIELD_VECTOR_POTENTIAL    : mGridField = GRID_FIELD_DENSITY           ; break ;
                case GRID_FIELD_DENSITY             : mGridField = GRID_FIELD_SIGNED_DISTANCE   ; break ;
                case GRID_FIELD_SIGNED_DISTANCE     : mGridField = GRID_FIELD_VELOCITY          ; break ;
            }
        }
        break ;

        case 'g':   // Cycle through grid decoration options.
        case 'G':
        {
            switch( mGridDecorations )
            {
            case GRID_DECO_NONE             : mGridDecorations = GRID_DECO_BOUNDING_BOX     ; break ;
            case GRID_DECO_BOUNDING_BOX     : mGridDecorations = GRID_DECO_CELLS            ; break ;
            case GRID_DECO_CELLS            : mGridDecorations = GRID_DECO_POINTS           ; break ;
            case GRID_DECO_POINTS           : mGridDecorations = GRID_DECO_CELLS_AND_POINTS ; break ;
            case GRID_DECO_CELLS_AND_POINTS : mGridDecorations = GRID_DECO_NONE             ; break ;
            }
        }
        break ;

        case 12: // Ctrl+L (with or without Shift)
        {
            extern void ResampleNestedGridLayer( int direction ) ;
            ResampleNestedGridLayer( 0 ) ; // Disable forced resampling
        }
        break ;

        case 'l':   // Cycle through color maps for grid
        {
            if( GLUT_ACTIVE_ALT & mModifierKeys )
            {   // Alt+L pressed.
                extern void ResampleNestedGridLayer( int direction ) ;
                ResampleNestedGridLayer( -1 ) ; // Force downsampling
            }
            else
            {   // 'l' pressed without Alt modified
                extern void CycleNestedGridLayer( int direction ) ;
                CycleNestedGridLayer( -1 ) ;
            }
        }
        break ;

        case 'L':   // Cycle through color maps for diagnostic particles
        {
            if( GLUT_ACTIVE_ALT & mModifierKeys )
            {   // Shift+Alt+L key pressed
                extern void ResampleNestedGridLayer( int direction ) ;
                ResampleNestedGridLayer( +1 ) ; // Force upsampling
            }
            else
            {   // Shift+L pressed without Alt modified
                extern void CycleNestedGridLayer( int direction ) ;
                CycleNestedGridLayer( +1 ) ;
            }
        }
        break ;

        case 'm':   // Cycle through color maps for grid
        {
            extern void CycleGridColorMap() ;
            CycleGridColorMap() ;
        }
        break ;

        case 'M':   // Cycle through color maps for diagnostic particles
        {
            extern void CycleDiagnosticVortonColorMap() ;
            CycleDiagnosticVortonColorMap() ;
        }
        break ;

        case 's':   // Cycle through fluid simulation techniques.
        case 'S':
        {
            VortonSim & vortonSim = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
            switch( vortonSim.GetFluidSimulationTechnique() )
            {
            case VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD            : vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   ) ; break ;
            case VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS   : vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VPM_SPH_HYBRID                    ) ; break ;
            case VortonSim::FLUID_SIM_VPM_SPH_HYBRID                    : vortonSim.SetFluidSimulationTechnique( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD            ) ; break ;
            }
        }
        break ;

        case 't':   // Cycle through tracer rendering options.
        case 'T':
        {
            mFluidScene.CycleTracerRenderingStyle() ;
        }
        break ;

        case 'p':   // Cycle through vorton properties.
        case 'P':
        {
            switch( mVortonProperty )
            {
            case VORTON_PROPERTY_POSITION           : mVortonProperty = VORTON_PROPERTY_VELOCITY        ; break ;
            case VORTON_PROPERTY_VELOCITY           : mVortonProperty = VORTON_PROPERTY_VORTICITY       ; break ;
            case VORTON_PROPERTY_VORTICITY          : mVortonProperty = VORTON_PROPERTY_DENSITY         ; break ;
            case VORTON_PROPERTY_DENSITY            : mVortonProperty = VORTON_PROPERTY_DENSITY_SPH     ; break ;
            case VORTON_PROPERTY_DENSITY_SPH        : mVortonProperty = VORTON_PROPERTY_DENSITY_GRADIENT; break ;
            case VORTON_PROPERTY_DENSITY_GRADIENT   : mVortonProperty = VORTON_PROPERTY_PROXIMITY       ; break ;
            case VORTON_PROPERTY_PROXIMITY          : mVortonProperty = VORTON_PROPERTY_POSITION        ; break ;
            }
        }
        break ;

        case 'v':   // Cycle through vorton rendering options.
        case 'V':
        {
            mFluidScene.CycleVortonRenderingStyle() ;
        }
        break ;

        case 'w':   // Cycle through vorticity equation terms
        case 'W':
        {
            VortonSim & vortonSim   = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
            VortonSim::InvestigationTermE & vorticityInvestigationTerm = vortonSim.GetInvestigationTerm() ;
            switch( vorticityInvestigationTerm )
            {
            case VortonSim::INVESTIGATE_ALL               : vorticityInvestigationTerm = VortonSim::INVESTIGATE_STRETCHING_TILTING  ; break ;
            case VortonSim::INVESTIGATE_STRETCHING_TILTING: vorticityInvestigationTerm = VortonSim::INVESTIGATE_BAROCLINIC          ; break ;
            case VortonSim::INVESTIGATE_BAROCLINIC        : vorticityInvestigationTerm = VortonSim::INVESTIGATE_VISCOUS_DIFFUSION   ; break ;
            case VortonSim::INVESTIGATE_VISCOUS_DIFFUSION : vorticityInvestigationTerm = VortonSim::INVESTIGATE_THERMAL_DIFFUSION   ; break ;
            case VortonSim::INVESTIGATE_THERMAL_DIFFUSION : vorticityInvestigationTerm = VortonSim::INVESTIGATE_NONE                ; break ;
            case VortonSim::INVESTIGATE_NONE              : vorticityInvestigationTerm = VortonSim::INVESTIGATE_ALL                 ; break ;
            }
        }
        break ;

        case 'z':   // Zero velocity everywhere
        {
            VortonSim &         vortonSim   = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
            VECTOR< Vorton > &  vortons     = * vortonSim.GetVortons() ;
            const size_t numVortons = vortons.size() ;
            for( size_t vortIdx = 0 ; vortIdx < numVortons ; ++ vortIdx )
            {
                Vorton & vorton = vortons[ vortIdx ] ;
                vorton.mVelocity = Vec3( 0.0f , 0.0f , 0.0f ) ;
            }
        }
        break ;

        case 'Z':   // Zero vorticity everywhere
        {
            VortonSim &         vortonSim   = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
            VECTOR< Vorton > &  vortons     = * vortonSim.GetVortons() ;
            const size_t numVortons = vortons.size() ;
            for( size_t vortIdx = 0 ; vortIdx < numVortons ; ++ vortIdx )
            {
                Vorton & vorton = vortons[ vortIdx ] ;
                vorton.mAngularVelocity = Vec3( 0.0f , 0.0f , 0.0f ) ;
            }
        }
        break ;

        case 127 /* delete */ : ; break;

        case 27 /* escape */ :
#       if PROFILE
            PerfBlock::TerminateAndLogAllThreads() ;
#       endif
            exit( 0 ) ;
        break;

        default:
            return;
            /*NOTREACHED*/
        break;
    }
    cam.SetOrbit( azimuth , elevation , radius ) ;
}




/** Function that GLUT calls to handle regular key events.
*/
/* static */ void InteSiVis::GlutKeyboardHandler( unsigned char key , int mouseX , int mouseY )
{
    PERF_BLOCK( InteSiVis__GlutKeyboardHandler ) ;

    sInstance->KeyboardHandler( key , mouseX , mouseY ) ;
}




/** Function that GLUT calls to handle mouse button events.
*/
/* static */ void InteSiVis::GlutMouseHandler( int button , int state , int x , int y )
{
    PERF_BLOCK( InteSiVis__GlutMouseHandler ) ;

    sInstance->mModifierKeys = glutGetModifiers() ;

    if( GLUT_DOWN == state )
    {   // User is clicking a mouse button
        sInstance->mMouseButtons[ button ] = 1 ;
    }
    else
    {   // User released a mouse button
        sInstance->mMouseButtons[ button ] = 0 ;
    }
    sMousePrevX = x ;
    sMousePrevY = y ;
}




/** Function that GLUT calls to handle mouse motion.
*/
/* static */ void InteSiVis::GlutMouseMotionHandler( int x , int y )
{
    PERF_BLOCK( InteSiVis__GlutMouseMotionHandler ) ;

    sInstance->MouseMotionHandler( x , y , sInstance->mModifierKeys ) ;
}




/** Function that GLUT calls when the window obtains focus.
*/
/* static */ void InteSiVis::GlutEntryHandler( int state )
{
    PERF_BLOCK( InteSiVis__GlutEntryHandler ) ;

    if( GLUT_LEFT == state )
    {   // Focus left this window so act like mouse buttons got released.
        sInstance->mMouseButtons[0] =
        sInstance->mMouseButtons[1] =
        sInstance->mMouseButtons[2] = 0 ;
    }
}




/// Typedef for pointer to SwapInterval function.
typedef BOOL (APIENTRY *PFNWGLSWAPINTERVALFARPROC)( int );

/// Pointer to SwapInterval function.
PFNWGLSWAPINTERVALFARPROC wglSwapIntervalEXT = 0;

/** Set the vertical synchronization interval.

    \param interval Number of frames before swapping buffers.
*/
void setVSync( int interval = 1 )
{
    PERF_BLOCK( setVSync ) ;

    const char * extensions = (char*) glGetString( GL_EXTENSIONS );

    if( strstr( extensions, "WGL_EXT_swap_control" ) == 0 )
    {   // Your computer does NOT support WGL_EXT_swap_control extension.
        return ;
    }
    else
    {
        wglSwapIntervalEXT = (PFNWGLSWAPINTERVALFARPROC)wglGetProcAddress( "wglSwapIntervalEXT" );
        if( wglSwapIntervalEXT )
        {   // Set the vsync mode
            wglSwapIntervalEXT(interval);
        }
    }
}




/// Callback meant for use with glDebugMessageCallbackARB, which nVidia apparently does not yet support.
static void CALLBACK DebugCallback(unsigned int /* source */ , unsigned int /* type */ , unsigned int /* id */ , unsigned int /* severity */ ,  int /* length */ , const char * /* message*/ , void * /* userParam */ )
{
}




/** Initialize display device.
*/
void InteSiVis::InitializeDisplayAndInputDevices()
{
    PERF_BLOCK( InteSiVis__InitializeDisplayAndInputDevices ) ;

#if 0
    for(int iz=-1 ; iz < 2 ; ++ iz )
    for(int iy=-1 ; iy < 2 ; ++ iy )
    for(int ix=-1 ; ix < 2 ; ++ ix )
    {
        int offset = ix + iy * 3 + iz * 9 ;
        if( offset >= 0 )
        {
            printf("[ %2i , %2i , %2i ]=%i\n", ix, iy, iz , offset ) ;
        }
    }
#endif

    // Register GLUT callbacks for main window
    glutIdleFunc( GlutIdleCallback) ;
    glutDisplayFunc( GlutDisplayCallback) ;
    glutKeyboardFunc( GlutKeyboardHandler ) ;
    glutSpecialFunc( GlutSpecialKeyHandler ) ;
    glutReshapeFunc( GlutReshapeGlutCallback ) ;
    glutMotionFunc( GlutMouseMotionHandler ) ;
    glutMouseFunc( GlutMouseHandler ) ;
    glutPassiveMotionFunc( GlutMouseMotionHandler ) ;
    glutEntryFunc( GlutEntryHandler ) ;

    //glDebugMessageCallbackARB( & DebugCallback , NULL ) ;

    setVSync( 0 ) ;
}
