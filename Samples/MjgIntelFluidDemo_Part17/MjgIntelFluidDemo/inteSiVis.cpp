/** \file inteSiVis.cpp

    \brief Application for interactive simulation and visualization

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "inteSiVis.h"

#include "Sim/Vorton/vorticityDistribution.h"

#include "Particles/Operation/pclOpFindBoundingBox.h"
#include "Particles/Operation/pclOpWind.h"
#include "Particles/Operation/pclOpAdvect.h"
#include "Particles/Operation/pclOpEvolve.h"
#include "Particles/Operation/pclOpAssignScalarFromGrid.h"
#include "Particles/Operation/pclOpPopulateVelocityGrid.h"
#include "Particles/Operation/pclOpEmit.h"
#include "Particles/Operation/pclOpKillAge.h"

#include "FluidBodySim/fluidBodySim.h"
#include "FluidBodySim/pclOpFluidBodyInteraction.h"

#include "Core/Math/vec2.h"
#include "Core/Math/vec4.h"
#include "Core/Math/mat4.h"

#include "Core/Performance/perf.h"

#if defined( _DEBUG )
    #define UNIT_TEST
    #include "Collision/sphereShape.h" // For Collision::UnitTests
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

bool gPrintProfileData = false ;

#if defined( PROFILE )
static const int sFrameCountMax = 600   ;   // In profile builds, run for a fixed number of frames
static size_t    sNumTracersSum = 0     ;   // Sum of current number of tracers
static size_t    sNumVortonsSum = 0     ;   // Sum of current number of vortons
#endif



// Functions --------------------------------------------------------------

/** Construct application for interactive simulation and visualization.
*/
InteSiVis::InteSiVis()
#if 0
    : mCamera( 320 , 240 )  // Low resolution
#elif 1
    : mCamera( 640 , 480 )  // Standard definition
#elif 1
    : mCamera( 1280 , 960 ) // 4:3 aspect ratio, high definition
#else
    : mCamera( 1280 , 720 ) // 16:9 720p high definition
#endif
    , mDiagnosticVortonRenderer( 0 , vortonStride , vortonOffsetToAngVel , vortonOffsetToSize , vortonOffsetToDensity , mDiagnosticVortonMaterial )
    , mSimpleVortonRenderer    ( 0 , vortonStride , vortonOffsetToAngVel , vortonOffsetToSize , vortonOffsetToDensity , mSimpleVortonMaterial     )
    , mDiagnosticTracerRenderer( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensity , mDiagnosticTracerMaterial )
    , mDyeRenderer             ( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensity , mDyeMaterial              )
#if ENABLE_FIRE
    , mSmokeRenderer ( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToSmokeFraction   , mSmokeMaterial  )
    , mFlameRenderer ( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToFlameFraction   , mFlameMaterial  )
    , mFuelRenderer  ( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToFuelFraction    , mFuelMaterial   )
#else
    , mSmokeRenderer ( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensity         , mSmokeMaterial  )
    , mFlameRenderer ( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensity         , mFlameMaterial  )
    , mFuelRenderer  ( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensity         , mFuelMaterial   )
#endif
    , mRenderWindow( 0 )
    , mStatusWindow( 0 )
    , mFrame( 0 )
    , mTimeNow( 0.0 )
    , mTimeStep( sTimeStep )
    , mTimeStepping( PLAY )
    , mInitialized( false )
    , mGridDecorations( GRID_DECO_NONE )
    , mGridField( GRID_FIELD_DENSITY_GRADIENT )
    , mVortonRendering( VORTON_RENDER_NONE )
    , mVortonProperty( VORTON_PROPERTY_VORTICITY )
    , mTracerRendering( TRACER_RENDER_FIRE )
    , mDiagnosticText( DIAG_TEXT_NONE )
    , mEmphasizeCameraTarget( true )
{
    ASSERT( 0 == sInstance ) ;
    sInstance = this ;

#if defined( _DEBUG )
    mGridDecorations = GRID_DECO_CELLS   ; // Render grid cells.
    mDiagnosticText  = DIAG_TEXT_SUMMARY ; // Render summary diagnostic text.
#endif

    mMouseButtons[0] = mMouseButtons[1] = mMouseButtons[2] = 0 ;

    InitialConditions( 26 /* 12 */ /* 32 */ /* 28 */) ;

    // Set the camera in motion.
    //mCamera.SetOrbitalTrajectory( Vec3( -0.002f , -0.000f , 0.0f ) ) ;

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
    sInstance = 0 ;
}




/* static */ InteSiVis * InteSiVis::GetInstance()
{
    return sInstance ;
}




/** Create simulation entities that couple physical objects to their render models.

    This should get called after both physical objects and render data exist.
    You cannot reliably call this, for example, from InitialConditions,
    since the render window might not have appeared yet, in which case
    various render resources will not yet exist.
*/
void InteSiVis::CreateEntities()
{
    ASSERT( mInitialized ) ; // Cannot create entities until render data exists.

    const size_t numSpheres     = GetSpheres().size() ;
    const size_t numBoxes       = GetBoxes().size() ;
    const size_t numPhysObjs    = numSpheres + numBoxes ;

    mModels.clear() ;
    mEntities.clear() ;
    mModels.reserve( numPhysObjs ) ;
    mEntities.reserve( numPhysObjs ) ;

    size_t idxPhysObj = 0 ;

    for( size_t idxSphere = 0 ; idxSphere < numSpheres ; ++ idxSphere , ++ idxPhysObj )
    {   // For each sphere physical object...
        // Add a sphere model.
        const float radius = GetSpheres()[ idxSphere ].GetCollisionShape()->GetBoundingSphereRadius() ;
        GLuint sphereDisplayList = MakeSphere( radius ) ;
        mModels.push_back( QdModel( sphereDisplayList , & mPhysObjMaterial ) ) ;
        // Make an Entity to tie the render model to the physical object.
        mEntities.push_back( Entity( & GetSpheres()[ idxSphere ] , & mModels[ idxSphere ] ) ) ;
    }

    for( size_t idxBox = 0 ; idxBox < numBoxes ; ++ idxBox , ++ idxPhysObj )
    {   // For each box physical object...
        RbBox & box = GetBoxes()[ idxBox ] ;
        const float radius = box.GetCollisionShape()->GetBoundingSphereRadius() ;
        const Vec3 & dimensions = box.GetDimensions() ;
        QdMaterial * boxMaterial = & mPhysObjMaterial ;
        if( box.GetIsHole() )
        {   // This box is a container so render it translucent.
            boxMaterial = & mContainerMaterial ;
        }
        // Make a box.
        GLuint boxDisplayList = MakeBox( dimensions , radius ) ;
        // Add a cube model.
        mModels.push_back( QdModel( boxDisplayList , boxMaterial ) ) ;
        // Make an Entity to tie the render model to the physical object.
        mEntities.push_back( Entity( & box , & mModels[ idxPhysObj ] ) ) ;
    }
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
#if ENABLE_PARTICLE_HISTORY
    extern size_t gPclHistoryFrame ;
    extern bool gPclHistoryFirstRun ;
    if( gPclHistoryFrame != 0 ) gPclHistoryFirstRun = false ; // This is a subsequent run.  History is already populated with the basis for comparison.
    gPclHistoryFrame = 0 ;  // Reset particle history recorder
    ++ gPclHistoryFrame ; // Advance to next frame in particle history buffer.
#endif

    ASSERT( Impulsion::PhysicalObject::sAmbientTemperature == Particle_sAmbientTemperature ) ; // This is not a necessary condition but I have not tested it otherwise.  If one changes without the other, probably should test a bunch of code to make sure it behaves as intended.

    mScenario           = ic  ;
    sInstance->mFrame   = 0   ;
    sInstance->mTimeNow = 0.0 ;

    mPclSysMgr.Clear() ;

    static const float ambientFluidDensity  = 1.0f ;
    {
        static const float viscosity = 0.01f ;
        static const float fluidSpecificHeatCapacity = 10.0f ;

        ParticleSystem * fluidParticleSystem = CreateFluidParticleSystems( mVortonPclGrpInfo , mTracerPclGrpInfo , viscosity , ambientFluidDensity , fluidSpecificHeatCapacity , GetPhysicalObjects() ) ;
        mPclSysMgr.PushBack( fluidParticleSystem ) ;
    }

    SetTallyDiagnosticIntegrals() ;

    GetSpheres().Clear() ;                      // Remove all spherical physical objects from the simulation
    GetBoxes().Clear() ;                        // Remove all box physical objects from the simulation
    mEntities.Clear() ;                         // Remove all simulation entities

    // Set up lights.
    mLights.resize( 2 ) ;
    mLights[0].mPosition = Vec3( 0.7f , -0.2f , 0.7f ) ; // actually a direction
    mLights[0].mColor    = Vec3( 0.9f , 0.9f , 0.9f ) ;
    mLights[0].mType     = QdLight::LT_DIRECTIONAL ;

    mLights[1].mPosition = Vec3( -0.7f , -0.2f , 0.7f ) ; // actually a direction
    mLights[1].mColor    = Vec3( 0.9f , 0.9f , 0.9f ) ;
    mLights[1].mType     = QdLight::LT_DIRECTIONAL ;

    mLights[0].mAmplitudes[ 1 ]  = 0.01f ;
    mLights[0].mFrequencies[ 1 ] = 0.0f ;
    mLights[0].mAmplitudes[ 2 ]  = 0.0f ;
    mLights[0].mFrequencies[ 2 ] = 0.0f ;
    mLights[0].mAmplitudes[ 3 ]  = 0.0f ;
    mLights[0].mFrequencies[ 3 ] = 0.0f ;

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
    mVortonPclGrpInfo.mPclOpFluidBodInte->mDensityGrid = 0 ;

    // By default, disable SDF generation.
    mTracerPclGrpInfo.mPclOpSeedSurfaceTracers->mSignedDistanceGrid  = 0 ;

    // By default, enable assigning tracer density from vorton density.
    mTracerPclGrpInfo.mPclOpAssignDensityFromGrid->mScalarGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetDensityGrid() ;

    srand( 1 ) ;    // Seed pseudo-random number generator to make results repeatable.

    switch( ic )
    {   // Switch on initial conditions
        case 0: // vortex ring -- vorticity in [0,1]
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexRing( fRadius , fThickness , Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 10.f , 0.f , 0.f ) ) ;
            mCamera.SetEye( Vec3( 10.f , -10.0f , 0.0f ) ) ;
        break ;

        case 1: // "jet" vortex ring -- velocity in [0,1]
            AssignVortons( vortons , fMagnitude , numVortonsMax , JetRing( fRadius , fThickness , Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            //vortonsImplyingTracers = ( const VECTOR< Particle > * )( & vortons ) ;
            numTracersPerCellCubeRoot -= 2 ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 10.f ,    0.f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 10.f , -10.0f , 0.0f ) ) ;
        break ;

        case 2: // Projectile with no initial spin
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 2.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Make sphere spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;

        case 3: // Projectile with spin about longitudinal axis: rifled.
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.3f * fThickness , 0.3f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 5.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres()[0].GetBody()->ApplyImpulsiveTorque( Vec3( 0.002f , 0.0f , 0.0f ) ) ; // Make sphere spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        break ;

        case 4: // Projectile with spin about transverse axis: curve ball.
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a sphere.
            GetSpheres().PushBack( RbSphere( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 4.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.0f , 0.05f , 0.0f ) ) ; // Make sphere spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
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

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , 0.0f , 3.0f ) ) ;
        break ;

        case 6: // Sphere in spinning fluid
            AssignVortons( vortons , 2.0f * fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 4.0f * fThickness , 2 , 0 ) ) ;

            numTracersPerCellCubeRoot += 3 ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( 0.f , 0.0f , 0.0f ) , Vec3( 0.f , 0.0f , 0.0f ) , 0.5f , 0.1f ) ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( -2.0f , 0.0f , 0.0f ) ) ;
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

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 1.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 8: // Vortex sheet with spanwise variation
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexSheet( fThickness , /* variation */ 0.2f , /* width */ 7.0f * fThickness ) ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -20.0f , 0.0f ) ) ;
        break ;

        case 9: // Vortex tube
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 2.0f * fThickness , 2 , 0 ) ) ;
        break ;

        case 10: // 2 orthogonal vortex tubes
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 4.0f * fThickness , 2 , -1 ) ) ;
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 4.0f * fThickness , 2 ,  1 ) ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        break ;

        ////////////////////////////////////////////////////////////////////////

        case 11: // 2D sheet
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexSheet( fThickness , /* variation */ 0.0f , /* width */ 2.0f * fThickness ) ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -10.0f , 0.0f ) ) ;
        break ;

        case 12: // ball of initially stationary heavy fluid
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius , 2.0f ) , Vec3( 0.0f , 0.0f , 5.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot += 2 ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -10.0f , 0.0f ) ) ;
        break ;

        case 13: // Two balls of initially stationary fluid, one heavy, one light, heavy one above light one.
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius , -0.1f ) , Vec3( 0.0f , 0.0f , -2.0f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius ,  0.1f ) , Vec3( 0.0f , 0.0f ,  2.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -5.0f , 0.0f ) ) ;
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

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
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

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 2.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -3.0f , 2.0f ) ) ;
        }
        break ;

        case 16: // Fluid combusts and pushes a ball up.
        {
            // Set up lights.
            mLights[0].mPosition    = Vec3( 0.0f , 0.0f , 0.7f ) ;
            mLights[0].mColor       = 1.2f * Vec3( 1.0f , 0.5f , 0.3f ) ;
            mLights[0].mAttenuation = Vec3( 0.0f , 0.2f , 0.8f ) ;
            mLights[0].mType        = QdLight::LT_POINT ;
            // Make light fluctuate to imply it comes from flames.
            mLights[0].mAmplitudes [ 1 ] = 0.1f ;
            mLights[0].mFrequencies[ 1 ] = 2.0f ;
            mLights[0].mAmplitudes [ 2 ] = 0.05f ;
            mLights[0].mFrequencies[ 2 ] = 5.0f ;
            mLights[0].mAmplitudes [ 3 ] = 0.02f ;
            mLights[0].mFrequencies[ 3 ] = 10.0f ;

            mLights[1].mPosition    = Vec3( 0.0f , 0.0f , 1.0f ) ; // actually a direction
            mLights[1].mColor       = 0.2f * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mLights[1].mType        = QdLight::LT_DIRECTIONAL ;

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

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 1.5f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -3.0f , 1.5f ) ) ;
        }
        break ;

        case 17: // Candle.
        {
            // Set up lights.
            mLights[0].mPosition    = Vec3( 0.0f , 0.0f , 0.7f ) ;
            mLights[0].mColor       = 1.2f * Vec3( 1.0f , 0.5f , 0.3f ) ;
            mLights[0].mAttenuation = Vec3( 0.0f , 0.2f , 0.8f ) ;
            mLights[0].mType        = QdLight::LT_POINT ;
            // Make light fluctuate to imply it comes from candle flame.
            mLights[0].mAmplitudes [ 1 ] = 0.05f ;
            mLights[0].mFrequencies[ 1 ] = 1.0f ;
            mLights[0].mAmplitudes [ 2 ] = 0.02f ;
            mLights[0].mFrequencies[ 2 ] = 2.0f ;
            mLights[0].mAmplitudes [ 3 ] = 0.01f ;
            mLights[0].mFrequencies[ 3 ] = 5.0f ;

            mLights[1].mPosition    = Vec3( 0.0f , 0.0f , 1.0f ) ; // actually a direction
            mLights[1].mColor       = 0.2f * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mLights[1].mType        = QdLight::LT_DIRECTIONAL ;

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

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 1.5f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -3.0f , 1.5f ) ) ;
        }
        break ;

        case 18: // Large flame
        {
            // Set up lights.
            mLights[0].mPosition    = Vec3( 0.0f , 0.0f , 0.7f ) ;
            mLights[0].mColor       = 1.2f * Vec3( 1.0f , 0.5f , 0.3f ) ;
            mLights[0].mAttenuation = Vec3( 0.0f , 0.2f , 0.8f ) ;
            mLights[0].mType        = QdLight::LT_POINT ;
            // Make light fluctuate to imply it comes from flames.
            mLights[0].mAmplitudes [ 1 ] = 0.1f ;
            mLights[0].mFrequencies[ 1 ] = 2.0f ;
            mLights[0].mAmplitudes [ 2 ] = 0.05f ;
            mLights[0].mFrequencies[ 2 ] = 5.0f ;
            mLights[0].mAmplitudes [ 3 ] = 0.02f ;
            mLights[0].mFrequencies[ 3 ] = 10.0f ;

            mLights[1].mPosition    = Vec3( 0.0f , 0.0f , 1.0f ) ; // actually a direction
            mLights[1].mColor       = 0.2f * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mLights[1].mType        = QdLight::LT_DIRECTIONAL ;

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

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 1.5f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -3.0f , 1.5f ) ) ;
        }
        break ;

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
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
        break ;

        ////////////////////////////////////////////////////////////////////////

        case 21: // Gradient sphere, for testing SPH mass density and density gradient calculations.
            numTracersPerCellCubeRoot = 1 ;
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax * 64 , DensityGradientBall( fRadius , 1.0f ) ) ;
            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
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

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;

        case 23: // Box projectile with spin about longitudinal axis
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.3f * fThickness , 0.3f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a box.
            GetBoxes().PushBack( RbBox( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 4.0f , 0.0f , 0.0f ) , 0.02f , Vec3( 0.2f , 0.2f , 0.2f ) ) ) ;
            GetBoxes().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.001f , 0.0f , 0.0f ) ) ; // Make box spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        break ;

        case 24: // Box projectile with spin about transverse axis
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a box.
            GetBoxes().PushBack( RbBox( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 2.0f , 0.0f , 0.0f ) , 0.015f , Vec3( 0.2f , 0.5f , 0.1f ) ) ) ;
            GetBoxes().Back().GetBody()->ApplyImpulsiveTorque( Vec3( 0.0f , 0.002f , 0.0f ) ) ; // Make box spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;

        case 25: // particle fountain
        {
            ParticleSystem * pclSys = CreateNonFluidParticleSystem( 0 ) ;

            mPclSysMgr.PushBack( pclSys ) ;

            // TODO: FIXME: Clone this effect multiple times using new virtual constructors.

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 2.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -10.0f , 2.0f ) ) ;
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
mTracerRendering = TRACER_RENDER_DIAGNOSTIC ;

// Disable assigning tracer density from vorton density; for surface tracers, density only indicates "sign" of signed distance.
// TODO: Instead, surface tracers should use truly signed sizes, but currently particle system rejects negative sizes.
mTracerPclGrpInfo.mPclOpAssignDensityFromGrid->mScalarGrid = 0 ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 5.0f , 5.0f , 5.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -4.0f , 4.0f ) ) ;
        break ;

        case 27: // Two boxes of initially stationary fluid, one heavy, one light, light one above heavy one, inside container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) , -0.6f ) , Vec3( 0.0f , 0.0f ,  0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetSpecificHeatCapacity( 1.0f ) ; // Diagnosing: Does this influence vorton motion?


            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 28: // One box of initially stationary fluid, heavy, at bottom of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;

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
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        ////////////////////////////////////////////////////////////////////////

        case 32: // One rotating box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.5f , 0.5f , 2.0f ) ,  1000.0f ) , Vec3( 0.75f , 0.0f , 0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1.0f / FLT_EPSILON , Vec3( 3.0f , 0.5f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ;
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ;
            GetBoxes().Back().GetBody()->SetAngularVelocity( Vec3( 0.0f , 2.0f , 0.0f ) ) ;

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
        }
        break ;

        case 33: // One box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.5f , 0.5f , 2.0f ) ,  100.0f ) , Vec3( 0.75f , 0.0f , 0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 3.0f , 0.5f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -1.8f , 0.0f ) ) ;
        }
        break ;

        case 34: // One box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , 2 * numVortonsMax , DensityBox( Vec3( 1.5f , 0.5f , 2.0f ) ,  100.0f ) , Vec3( 0.75f , 0.0f , 0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 3.0f , 0.5f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
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
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 36: // One box of initially stationary fluid, light, at bottom of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) , -0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 37: // One box of initially stationary fluid, heavy, at top of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  1000.0f ) , Vec3( 0.0f , 0.0f ,  0.5f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 38: // One box of initially stationary fluid, heavy, at side of container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( 0.5 , 1.0f , 2.0f ) ,  1000.0f ) , Vec3( 0.0f , 0.0f ,  0.0 ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 39: // One box of initially stationary fluid, heavy, at middle of container, nearly filling container.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.0f , 1.0f , 1.9f ) ,  1000.0f ) , Vec3( 0.0f , 0.0f ,  0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
        }
        break ;

        case 40: // One box of initially stationary fluid, neutral, at middle of container, nearly filling container.
        {
            //AssignVortons( vortons , 1.0f , numVortonsMax , VortexNoise( Vec3( 1.0f , 1.0f , 1.9f ) ) , Vec3( 0.0f , 0.0f ,  0.0f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBox( Vec3( 1.0f , 1.0f , 1.9f ) ,  1.0f ) , Vec3( 0.0f , 0.0f ,  0.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const VECTOR< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            // Add a box-shaped hole.
            GetBoxes().PushBack( RbBox( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 100000.0f , Vec3( 1.0f , 1.0f , 2.0f ) , true ) ) ;
            GetBoxes().Back().GetThermalProperties().SetOneOverHeatCapacity( 1.0f ) ; // Very small heat capacity.  Want this box to respond to heat quickly, so it does not alter temperature.
            GetBoxes().Back().GetThermalProperties().SetThermalConductivity( 0.0f ) ; // Prevent heat transfer between fluid and container.

#if REDUCE_CONVERGENCE
            mVortonPclGrpInfo.mPclOpFluidBodInte->mVortonIndicesGrid = & mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortonIndicesGrid() ;
#endif

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -2.0f , 0.0f ) ) ;
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
        CreateEntities() ;
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

        vortonSim.Initialize() ;
        UniformGridGeometry tracerGrid( vortonSim.GetGrid() ) ;
        tracerGrid.Scale( vortonSim.GetGrid() , tracerGridScale ) ;

        PclOpEmit::Emit( mTracerPclGrpInfo.mParticleGroup->GetParticles() , tracerGrid , numTracersPerCellCubeRoot , vortonsImplyingTracers ) ;

        if( mTracerPclGrpInfo.mPclOpSeedSurfaceTracers->mSignedDistanceGrid )
        {
//#error For testing, emit initial surface tracers.
PclOpSeedSurfaceTracers::Emit( mTracerPclGrpInfo.mParticleGroup->GetParticles() , 4 , mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetSignedDistanceGrid() , PclOpSeedSurfaceTracers::sBandWidthAutomatic , ambientFluidDensity ) ;
        }

    #if( ENABLE_FLUID_BODY_SIMULATION )
        FluidBodySim::RemoveEmbeddedParticles( reinterpret_cast< VECTOR< Particle > & >( * vortonSim.GetVortons() ) , GetPhysicalObjects() ) ;
        FluidBodySim::RemoveEmbeddedParticles( mTracerPclGrpInfo.mParticleGroup->GetParticles() , GetPhysicalObjects() ) ;
    #endif
    }

#if PROFILE
    fprintf( stderr , "Initial condition %i, simulation objects: %i tracers    %i vortons    %i spheres %i boxes\n"
        , ic , mTracerPclGrpInfo.mParticleGroup->GetNumParticles() , vortons.Size() , GetSpheres().Size() , GetBoxes().size() ) ;
#endif

}




/** Update particle systems.
*/
void InteSiVis::UpdateParticleSystems()
{
    QUERY_PERFORMANCE_ENTER ;
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
    QUERY_PERFORMANCE_EXIT( InteSiVis_ParticleSystem_Update ) ;

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
    QUERY_PERFORMANCE_ENTER ;
    if( mTimeStepping == PLAY || mTimeStepping == SINGLE_STEP )
    {
        if( 28 == mScenario )
        {   // Box of fluid.
            ASSERT( GetBoxes().Size() == 1 ) ;
// TODO: eliminate this compile-time conditional.
#if USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
            static Vec3 amplitude( 0.5f , 0.2f , 0.05f ) ;
            static Vec3 frequency( 3.0f , 5.0f , 7.0f ) ;
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
        PhysicalObject_UpdateSystem( GetPhysicalObjects() , mTimeStep , mFrame ) ;
    }
    QUERY_PERFORMANCE_EXIT( InteSiVis_RigidBody_UpdateSystem ) ;
}




/** Gather and record performance profile data.
*/
void InteSiVis::GatherAndRecordProfileData()
{
#if defined( PROFILE )
    VortonSim & rVortonSim = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;

    if( mFrame == sFrameCountMax - 1 )
    {   // Reached last frame, so enable profile reporting
        gPrintProfileData = true ;
    }
    else if( mFrame >= sFrameCountMax )
    {   // Reached maximum number of frames, so exit.
        fprintf( stderr , "Ran %i frames\n" , mFrame ) ;
        fprintf( stderr , "region: { %g , %g , %g } , { %g , %g , %g } \n"
            , rVortonSim.GetMinCornerEternal().x
            , rVortonSim.GetMinCornerEternal().y
            , rVortonSim.GetMinCornerEternal().z
            , rVortonSim.GetMaxCornerEternal().x
            , rVortonSim.GetMaxCornerEternal().y
            , rVortonSim.GetMaxCornerEternal().z
            ) ;
        const float tracersPerFrame = (float) sNumTracersSum / (float) ( mFrame - 1 ) ;
        const float vortonsPerFrame = (float) sNumVortonsSum / (float) ( mFrame - 1 ) ;
        fprintf( stderr , "Particles, average per frame: %g tracers    %g vortons\n"
            , tracersPerFrame , vortonsPerFrame ) ;

        const float tracerHitsPerFrame = (float) FluidBodySim::GetNumTracerBodyHits() / (float) ( mFrame - 1 ) ;
        const float vortonHitsPerFrame = (float) FluidBodySim::GetNumVortonBodyHits() / (float) ( mFrame - 1 ) ;
        fprintf( stderr , "Particles-body hits, average per frame: %g tracers    %g vortons\n"
            , tracerHitsPerFrame , vortonHitsPerFrame ) ;
        fprintf( stderr , "Benchmark DONE: " __DATE__ " " __TIME__ "\n\n" ) ;

        exit( 0 ) ;
    }
#endif
}




/** Function that GLUT calls when window resizes.
*/
/* static */ void InteSiVis::GlutReshapeGlutCallback(int width, int height)
{
    const int   width_4 = (width / 4) * 4;
    sInstance->mCamera.SetViewport( width_4 ,height ) ;
    glViewport( 0 , 0 , sInstance->mCamera.GetWidth() , sInstance->mCamera.GetHeight() ) ;
    glutReshapeWindow( width_4 , height ) ;
}




/** Function that GLUT calls to display contents of window.
*/
/* static */ void InteSiVis::GlutDisplayCallback()
{
    QUERY_PERFORMANCE_ENTER ;

    if( ! sInstance->mInitialized )
    {   // This is the first time this app has displayed anything.
        sInstance->InitializeRendering() ;
        sInstance->CreateEntities() ;
    }

    CheckGlError() ;

    sInstance->mCamera.SetCamera() ;
    sInstance->RenderSky() ;
    sInstance->RenderRigidBodies() ;
    sInstance->RenderDiagnosticGrid() ;
    sInstance->RenderParticles() ;
    sInstance->RenderSummaryDiagnosticText() ;

    QUERY_PERFORMANCE_ENTER ;
    glutSwapBuffers() ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_SwapBuffers ) ;

#if PROFILE > 1
    sInstance->GatherAndRecordProfileData() ;
#endif

    CheckGlError() ;

    QUERY_PERFORMANCE_EXIT( InteSiVis_Render ) ;
}




/// Return PhysicalObject in focus, if any.
Impulsion::PhysicalObject * InteSiVis::GetPhysicalObjectInFocus()
{
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
        {   // Shift held.
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
            QdCamera & cam = mCamera ;
            float azimuth , elevation , radius ;
            // Obtain previous camera parameters.
            cam.GetOrbit( azimuth , elevation , radius ) ;
            // Avoid gimbal lock by limiting elevation angle to avoid the poles.
            static const float sAvoidPoles = 0.001f ;
            elevation = Clamp( elevation - dy , sAvoidPoles , PI * ( 1.0f - sAvoidPoles ) ) ;
            // Set new camera parameters based on how much mouse moved.
            cam.SetOrbit( azimuth - dx , elevation , radius ) ;
        }
    }
    else if( mMouseButtons[2] )
    {   // User is right-click-dragging mouse
        if( GLUT_ACTIVE_SHIFT == modifierKeys )
        {   // Shift held.
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
            QdCamera & cam = mCamera ;
            float azimuth , elevation , radius ;
            // Obtain previous camera parameters.
            cam.GetOrbit( azimuth , elevation , radius ) ;
            // Set new camera parameters based on how much mouse moved.
            const float newRadius = Max2( radius - ( dx + dy ) , 1.0e-4f ) ;
            cam.SetOrbit( azimuth , elevation , newRadius ) ;
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
        {   // Shift held.
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
            QdCamera & cam = mCamera ;
            const Vec3 & rEye    = cam.GetEye() ;
            const Vec3 & rTarget = cam.GetTarget() ;
            cam.SetEye   ( Vec3( rEye.x    + delta.x , rEye.y    + delta.y , rEye.z    + delta.z ) ) ;
            cam.SetTarget( Vec3( rTarget.x + delta.x , rTarget.y + delta.y , rTarget.z + delta.z ) ) ;
        }
    }

    sMousePrevX = x ;
    sMousePrevY = y ;
}




/** Function to run to handle "idle" event, that is, when not doing anything else.
*/
void InteSiVis::Idle()
{
    QUERY_PERFORMANCE_ENTER ;
#if USE_TBB
    tbb::tick_count time0 = tbb::tick_count::now() ;
#endif

    UpdateParticleSystems() ;

#if USE_TBB
    tbb::tick_count timeFinal = tbb::tick_count::now() ;
    //fprintf( stderr , " tbb duration=%g second\n" , (timeFinal - time0).seconds() ) ;
#endif

    mCamera.Update() ;

    InteSiVis::GlutDisplayCallback() ;

    UpdateRigidBodies() ;

    QUERY_PERFORMANCE_EXIT( InteSiVis_UPDATE_Michael_J_Gourlay_2009 ) ;

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
}




/** Function that GLUT calls when nothing else is happening.
*/
/* static */ void InteSiVis::GlutIdleCallback()
{
    sInstance->Idle() ;
}




/** Function to handle a special key.
*/
/* static */ void InteSiVis::SpecialKeyHandler( int key , const int modifierKeys , int /* windowRelativeMouseX */ , int /* windowRelativeMouseY */ )
{
    QdCamera &      cam     = mCamera ;
    Vec3            vTarget = cam.GetTarget() ;

    if( ! modifierKeys )
    {   // No modifier keys (Shift, Ctrl or Alt) were pressed.
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

            case GLUT_KEY_UP   : mTimeStepping   = PLAY  ; break ;
            case GLUT_KEY_DOWN : mTimeStepping   = PAUSE ; break ;
            case GLUT_KEY_RIGHT: mTimeStep       =  sTimeStep ; if( mTimeStepping == PAUSE ) mTimeStepping = SINGLE_STEP ; break ;
            case GLUT_KEY_LEFT : mTimeStep       = -sTimeStep ; if( mTimeStepping == PAUSE ) mTimeStepping = SINGLE_STEP ; break ;

            default:
            return;
            break;
        }
    }
    else if( GLUT_ACTIVE_SHIFT == modifierKeys )
    {   // Shift held while special key pressed.
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
    {   // Ctrl held while special key pressed.
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

            default:
            return;
            break;
        }
    }
    else if( GLUT_ACTIVE_ALT == modifierKeys )
    {   // Alt held while special key pressed.
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
    sInstance->mModifierKeys = glutGetModifiers() ;

    sInstance->SpecialKeyHandler( key , sInstance->mModifierKeys , windowRelativeMouseX , windowRelativeMouseY ) ;

    glutPostRedisplay() ;
}




/** Function to handle a regular key.
*/
void InteSiVis::KeyboardHandler( unsigned char key , int /* mouseX */ , int /* mouseY */ )
{
    mModifierKeys = glutGetModifiers() ;

    QdCamera & cam = mCamera ;
    float azimuth , elevation , radius ;
    static bool animateCamera = false ;
    cam.GetOrbit( azimuth , elevation , radius ) ;
    switch( key )
    {
        case '?': gPrintProfileData = ! gPrintProfileData ; break ;

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
                case GRID_FIELD_VELOCITY         : mGridField = GRID_FIELD_DENSITY_GRADIENT ; break ;
                case GRID_FIELD_DENSITY_GRADIENT : mGridField = 
                #if COMPUTE_PRESSURE_GRADIENT
                                                                GRID_FIELD_PRESSURE_GRADIENT
                #else
                                                                GRID_FIELD_DENSITY
                #endif
                                                                                            ; break ;
                case GRID_FIELD_PRESSURE_GRADIENT: mGridField = GRID_FIELD_DENSITY          ; break ;
                case GRID_FIELD_DENSITY          : mGridField = GRID_FIELD_SIGNED_DISTANCE  ; break ;
                case GRID_FIELD_SIGNED_DISTANCE  : mGridField = GRID_FIELD_VELOCITY         ; break ;
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

        case 'm':   // Cycle through color maps.
        case 'M':
        {
            extern void CycleGridColorMap() ;
            CycleGridColorMap() ;
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
            switch( mTracerRendering )
            {
            case TRACER_RENDER_NONE      : mTracerRendering = TRACER_RENDER_FUEL        ; break ;
            case TRACER_RENDER_FUEL      : mTracerRendering = TRACER_RENDER_FLAME       ; break ;
            case TRACER_RENDER_FLAME     : mTracerRendering = TRACER_RENDER_SMOKE       ; break ;
            case TRACER_RENDER_SMOKE     : mTracerRendering = TRACER_RENDER_FIRE        ; break ;
            case TRACER_RENDER_FIRE      : mTracerRendering = TRACER_RENDER_DYE         ; break ;
            case TRACER_RENDER_DYE       : mTracerRendering = TRACER_RENDER_DIAGNOSTIC  ; break ;
            case TRACER_RENDER_DIAGNOSTIC: mTracerRendering = TRACER_RENDER_NONE        ; break ;
            }
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
            switch( mVortonRendering )
            {
            case VORTON_RENDER_NONE                 : mVortonRendering = VORTON_RENDER_DIAGNOSTIC_PARTICLES ; break ;
            case VORTON_RENDER_DIAGNOSTIC_PARTICLES : mVortonRendering = VORTON_RENDER_SIMPLE_PARTICLES     ; break ;
            case VORTON_RENDER_SIMPLE_PARTICLES     : mVortonRendering = VORTON_RENDER_VECTORS              ; break ;
            case VORTON_RENDER_VECTORS              : mVortonRendering = VORTON_RENDER_PARTICLES_AND_VECTORS; break ;
            case VORTON_RENDER_PARTICLES_AND_VECTORS: mVortonRendering = VORTON_RENDER_PATHLINES            ; break ;
            case VORTON_RENDER_PATHLINES            : mVortonRendering = VORTON_RENDER_ALL                  ; break ;
            case VORTON_RENDER_ALL                  : mVortonRendering = VORTON_RENDER_NONE                 ; break ;
            }
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
    sInstance->KeyboardHandler( key , mouseX , mouseY ) ;
}




/** Function that GLUT calls to handle mouse button events.
*/
/* static */ void InteSiVis::GlutMouseHandler( int button , int state , int x , int y )
{
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
    sInstance->MouseMotionHandler( x , y , sInstance->mModifierKeys ) ;
}




/** Function that GLUT calls when the window obtains focus.
*/
/* static */ void InteSiVis::GlutEntryHandler( int state )
{
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
void InteSiVis::InitializeDisplayandInputDevices( int * pArgc , char ** argv )
{
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

    glutInit( pArgc , argv ) ;

    unsigned int mode =     GLUT_RGBA
                        |   GLUT_DEPTH
                        |   GLUT_DOUBLE
                        //|   GLUT_ACCUM
                        //|   GLUT_STENCIL
                        //|   GLUT_MULTISAMPLE
                        ;

    // Initialize the GL utility toolkit.
    glutInitDisplayMode( mode ) ;

    // Create render window
    glutInitWindowPosition( 0 , 0 ) ;
    glutInitWindowSize( sInstance->mCamera.GetWidth() , sInstance->mCamera.GetHeight() ) ;
    mRenderWindow = glutCreateWindow( "(C) 2009-2013 Michael J. Gourlay: VorteGrid Render Window" ) ;

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

    // Set the current window to the main window
    glutSetWindow( mRenderWindow ) ;

    setVSync( 0 ) ;
}
