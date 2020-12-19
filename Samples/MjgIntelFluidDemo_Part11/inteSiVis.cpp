/*! \file inteSiVis.cpp

    \brief Application for interactive simulation and visualization

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdarg.h>

#include <assert.h>
#if defined( WIN32 )
    #include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include "glut.h"


#include "Core/Math/vec4.h"
#include "Core/Math/mat4.h"
#include "Core/Performance/perf.h"
#include "Space/uniformGridMath.h"
#include "Sim/Vorton/vorticityDistribution.h"

#include "inteSiVis.h"

// Private variables --------------------------------------------------------------

static const float  sOneMinusEpsilon        = 1.0f - FLT_EPSILON ;
static const float  sOnePlusEpsilon         = 1.0f + FLT_EPSILON ;
static const float  sOneOverThirty          = 1.0f / 30.0f ;

static InteSiVis * sInstance = 0 ;
static const Vec3 sGravityDirection( 0.0f , 0.0f , -1.0f ) ; ///< Direction of acceleration due to gravity
static const Vec3 sGravityAcceleration( 10.0f * sGravityDirection ) ; ///< Acceleration due to gravity

static const size_t vortonStride                = sizeof( Vorton ) ;
static const size_t vortonOffsetToAngVel        = offsetof( Vorton , mAngularVelocity  ) ;
static const size_t vortonOffsetToSize          = offsetof( Vorton , mSize             ) ;
static const size_t vortonOffsetToDensityDev    = offsetof( Vorton , mDensityDeviation ) ;

static const size_t tracerStride                = sizeof( Particle ) ;
static const size_t tracerOffsetToAngVel        = offsetof( Particle , mAngularVelocity  ) ;
static const size_t tracerOffsetToSize          = offsetof( Particle , mSize             ) ;
static const size_t tracerOffsetToDensityDev    = offsetof( Particle , mDensityDeviation ) ;
#if ENABLE_FIRE
static const size_t tracerOffsetToFuelFraction  = offsetof( Particle , mFuelFraction     ) ;
static const size_t tracerOffsetToFlameFraction = offsetof( Particle , mFlameFraction    ) ;
static const size_t tracerOffsetToSmokeFraction = offsetof( Particle , mSmokeFraction    ) ;
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

static Vec4 sColorRamp[] =
{
    Vec4( 0.0f , 0.0f , 1.0f , 0.2f ) ,
    Vec4( 0.0f , 1.0f , 1.0f , 0.3f ) ,
    Vec4( 0.0f , 1.0f , 0.0f , 0.4f  ) ,
    Vec4( 1.0f , 0.5f , 0.0f , 0.5f  ) ,
    Vec4( 1.0f , 0.0f , 0.0f , 0.6f  ) ,
    Vec4( 1.0f , 0.0f , 1.0f , 0.8f  )
} ;
static const int NUM_COLORS_IN_RAMP = sizeof( sColorRamp ) / sizeof( sColorRamp[0] ) ;

// Functions --------------------------------------------------------------

/** Given a color ramp and a value in [0,1], compute and return a blended color.

    \param colorRamp - array of colors.

    \param numColorsInRamp - number of elements in colorRamp

    \param zeroToOne - value in [0,1] used to choose a color.

    This routine assumes the colors in the ramp should be uniformly distributed
    across the range [0,1] of input values.  So, colorRamp[0] corresponds to 0,
    colorRamp[numColorsInRamp-1] corresponds to 1, and all colorRamp values in
    between are evenly distributed in [0,1].

*/
static inline Vec4 GetColorFromRamp( const Vec4 * colorRamp , int numColorsInRamp , float zeroToOne )
{
    const float         idxMinFlt               = zeroToOne * float( numColorsInRamp - 1 ) * sOneMinusEpsilon ;
    const int           idxMin                  = int( idxMinFlt ) ;
    // Interpolate between adjacent colors within the ramp.
    const int           idxMax                  = idxMin + 1 ;
    const float         spanBetweenRampElements = sOnePlusEpsilon / float( numColorsInRamp - 1 ) ;
    const float         tween                   = ( zeroToOne - spanBetweenRampElements * float( idxMin ) ) / spanBetweenRampElements ;
    return colorRamp[ idxMin ] * ( 1.0f - tween ) + colorRamp[ idxMax ] * tween ;
}




/** Construct application for interactive simulation and visualization.

    \param viscosity - Fluid viscosity.

    \param ambientFluidDensity - Ambient fluid density.  This is the density of
            the fluid in the absence of particles.  Particles have a density,
            and when particles are present then they can modify this ambient density.

    \param fluidSpecificHeatCapacity - Heat capacity per unit mass of the fluid.

*/
InteSiVis::InteSiVis( float viscosity , float ambientFluidDensity , float fluidSpecificHeatCapacity )
    : mTracers( mParticleGroupTracers.mParticles )
#if 0
    , mCamera( 320 , 240 )  // Low resolution
#elif 1
    , mCamera( 640 , 480 )  // Standard definition
#elif 1
    : mCamera( 1280 , 960 ) // 4:3 aspect ratio, high definition
#else
    : mCamera( 1280 , 720 ) // 16:9 720p high definition
#endif
    , mVortonRenderer( 0 , vortonStride , vortonOffsetToAngVel , vortonOffsetToSize , vortonOffsetToDensityDev , mVortonMaterial )
#if ENABLE_FIRE
    , mHeavyLightRenderer( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToFuelFraction , mHeavyLightTracerMaterial )
    , mSmokeRenderer( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToSmokeFraction , mSmokeMaterial )
    , mFlameRenderer( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToFlameFraction , mFlameMaterial )
#else
    , mHeavyLightRenderer( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensityDev , mHeavyLightTracerMaterial )
    , mSmokeRenderer( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensityDev , mSmokeMaterial )
    , mFlameRenderer( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensityDev , mFlameMaterial )
#endif
    , mRenderWindow( 0 )
    , mStatusWindow( 0 )
    , mFrame( 0 )
    , mTimeNow( 0.0 )
    , mTimeStep( sOneOverThirty )
    , mTimeStepping( PLAY )
    , mInitialized( false )
    , mGridDecorations( GRID_DECO_NONE )
    , mDiagnosticText( DIAG_TEXT_NONE )
    , mRenderVortons( false )
    , mTracerRendering( TRACER_RENDER_ALL )
{
    sInstance = this ;


    mMouseButtons[0] = mMouseButtons[1] = mMouseButtons[2] = 0 ;

    {
        static const int    killAgeMax  = 90    ;
        static const float  windGain    = 1.0f  ;

        // Set up particle group for vortons.
        {
            mPclOpKillAgeVortons.mAgeMax    = killAgeMax ;
            mParticleGroupVortons.mParticleOps.PushBack( & mPclOpKillAgeVortons ) ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition           = Vec3( -0.5f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity           = Vec3( 2.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize               = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensityDeviation   = 0.0f ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction       = 0.0f ;
            emitterTemplate.mFlameFraction      = 0.0f ;
            emitterTemplate.mSmokeFraction      = 1.0f ;
        #endif
            emitterSpread.mPosition             = Vec3( emitterTemplate.mVelocity.x / 30.0f , 0.5f , 0.5f ) ;

            mPclOpEmitVortons.mTemplate                     = emitterTemplate ;
            mPclOpEmitVortons.mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mPclOpEmitVortons.mSpread                       = emitterSpread ;
            mPclOpEmitVortons.mEmitRate                     = 0.0f ; // Set by InitialConditions
            mParticleGroupVortons.mParticleOps.PushBack( & mPclOpEmitVortons ) ;

            // PclOpVortonSim must occur after finding bounding box for tracers
            // and before advection.  Advection will move particles outside the
            // bounding box, which defines the grid for velocity and other terms.
            mPclOpVortonSim.mMinCorner = & mPclOpFindBoundingBox.GetMinCorner() ;
            mPclOpVortonSim.mMaxCorner = & mPclOpFindBoundingBox.GetMaxCorner() ;
            mPclOpVortonSim.mVortonSim.SetVortons( reinterpret_cast< Vector< Vorton > * >( & mParticleGroupVortons.mParticles ) ) ;
            mPclOpVortonSim.mVortonSim.SetViscosity( viscosity ) ;
            mPclOpVortonSim.mVortonSim.SetAmbientDensity( ambientFluidDensity ) ;
            mPclOpVortonSim.mVortonSim.SetGravitationalAcceleration( sGravityAcceleration ) ;
            mParticleGroupVortons.mParticleOps.PushBack( & mPclOpVortonSim ) ;

            // Wind must occur before advection, because advection uses the
            // total velocity to move particles, and Wind adds to that velocity.
            //mPclOpWindVortons.mWind        = Vec3( 0.0f , 0.0f , 0.0f ) ; Set in InitialConditions
            mPclOpWindVortons.mGain        = windGain ;
            mParticleGroupVortons.mParticleOps.PushBack( & mPclOpWindVortons ) ;

            mPclOpAdvectVortons.mVelocityGrid   = & mPclOpVortonSim.mVortonSim.GetVelocityGrid() ;
            mParticleGroupVortons.mParticleOps.PushBack( & mPclOpAdvectVortons ) ;

            // Fluid-Body interaction must occur after advection,
            // because advection will move particles inside and near rigid bodies.
            mPclOpFluidBodInteVortons.mVelocityGrid             = & mPclOpVortonSim.mVortonSim.GetVelocityGrid() ;
            mPclOpFluidBodInteVortons.mDensityDeviationGrid     = 0 ;
            mPclOpFluidBodInteVortons.mRigidBodies              = (Vector< RigidBody * > *) & GetRigidBodies() ;
            mPclOpFluidBodInteVortons.mAmbientFluidDensity      = ambientFluidDensity ;
            mPclOpFluidBodInteVortons.mFluidSpecificHeatCapacity= fluidSpecificHeatCapacity ;
            mPclOpFluidBodInteVortons.mGravityAcceleration      = sGravityAcceleration ;
            mPclOpFluidBodInteVortons.mRespectAngVel            = true ;
            mParticleGroupVortons.mParticleOps.PushBack( & mPclOpFluidBodInteVortons ) ;
        }

        // Set up particle group for passive tracers.
        {
            mPclOpKillAgeTracers.mAgeMax    = killAgeMax ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpKillAgeTracers ) ;

            mPclOpWindTracers.mWind        = mPclOpWindVortons.mWind ;
            mPclOpWindTracers.mGain        = windGain ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpWindTracers ) ;

            mPclOpAssignDensityFromGrid.mMemberOffsetInBytes    = tracerOffsetToDensityDev ;
            mPclOpAssignDensityFromGrid.mScalarGrid             = & mPclOpVortonSim.mVortonSim.GetDensityDeviationGrid() ;
            mPclOpAssignDensityFromGrid.mDivideByParticleCount = true ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpAssignDensityFromGrid ) ;

        #if ENABLE_FIRE
            mPclOpAssignFuelFromGrid.mMemberOffsetInBytes   = tracerOffsetToFuelFraction ;
            mPclOpAssignFuelFromGrid.mScalarGrid            = & mPclOpVortonSim.mVortonSim.GetFuelGrid() ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpAssignFuelFromGrid ) ;

            mPclOpAssignFlameFromGrid.mMemberOffsetInBytes  = tracerOffsetToFlameFraction ;
            mPclOpAssignFlameFromGrid.mScalarGrid           = & mPclOpVortonSim.mVortonSim.GetFlameGrid() ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpAssignFlameFromGrid ) ;

            mPclOpAssignSmokeFromGrid.mMemberOffsetInBytes  = tracerOffsetToSmokeFraction ;
            mPclOpAssignSmokeFromGrid.mScalarGrid           = & mPclOpVortonSim.mVortonSim.GetSmokeGrid() ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpAssignSmokeFromGrid ) ;
        #endif

            mPclOpAdvectTracers.mVelocityGrid   = mPclOpAdvectVortons.mVelocityGrid ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpAdvectTracers ) ;

            mPclOpFluidBodInteTracers.mVelocityGrid         = mPclOpFluidBodInteVortons.mVelocityGrid ;
            mPclOpFluidBodInteTracers.mRigidBodies          = mPclOpFluidBodInteVortons.mRigidBodies ;
            mPclOpFluidBodInteTracers.mAmbientFluidDensity  = ambientFluidDensity ;
            mPclOpFluidBodInteTracers.mRespectAngVel        = false ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpFluidBodInteTracers ) ;

            // Note that if tracers get emitted outside the bounding box
            // computed last frame, then mVelocityGrid will not encompass them.
            // To be safe, these emitted particles should not be advected on
            // their first frame.  That could lead to weird artifacts where
            // just-emitted particles disobey wind and other advectors, on their
            // first frame.  Could mitigate that either by providing suitable
            // initial velocity, or could move Emit to just after Kill, then
            // each FindBoudingBox call would also include the entire Emit
            // region, in anticipation of particles to be emitted next frame.
            mPclOpEmitTracers.mTemplate             = mPclOpEmitVortons.mTemplate ;
            mPclOpEmitTracers.mSpread               = mPclOpEmitVortons.mSpread ;
            mPclOpEmitTracers.mSpread.mPosition.x   = mPclOpEmitVortons.mSpread.mPosition.x        ;
            mPclOpEmitTracers.mSpread.mPosition.y   = mPclOpEmitVortons.mSpread.mPosition.y * 0.3f ;
            mPclOpEmitTracers.mSpread.mPosition.z   = mPclOpEmitVortons.mSpread.mPosition.z * 0.3f ;
            mPclOpEmitTracers.mEmitRate             = 0.0f ; // Set by InitialConditions
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpEmitTracers ) ;

            // Finding bounding box must occur last in this group, and before
            // VortonSim in the other group (because VortonSim uses the bounding
            // box to create a grid of appropriate size to capture all relevant
            // terms of the vorticity equation). Safest to FindBoundingBox last.
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpFindBoundingBox ) ;
        }

        // Process tracers before vortons, so vortons know tracer bounding box.
        mParticleSystem.mParticleGroups.PushBack( & mParticleGroupTracers ) ;
        mParticleSystem.mParticleGroups.PushBack( & mParticleGroupVortons ) ;
    }

    InitialConditions( 16 ) ;

    // Set the camera in motion.
    //mCamera.SetOrbitalTrajectory( Vec3( -0.002f , -0.000f , 0.0f ) ) ;

}




/** Return a value based on the distance of the given position to the camera target.

    Value lies in [0,1] where the value 1 is for any distance smaller than a grid cell.
*/
float InteSiVis::CameraFocusEmphasis( const Vec3 position ) const
{
    const Vec3 &    target               = mCamera.GetTarget() ;
    const Vec3      displacementToLookat = position - target ;
    const float     distToEye            = displacementToLookat.Magnitude() ;
    const float     characteristicSize   = powf( mPclOpVortonSim.mVortonSim.GetVelocityGrid().GetCellVolume() , 0.33333333333333333333333f ) ;
    const float     distRatio            = MIN2( characteristicSize / distToEye , 1.0f ) ;
    return POW3( distRatio ) ;
}




#if USE_ORIENTED_SORTED_PARTICLES
/** Cheap lighting hack to make an internal glow effect inside smoke.

    \param vGlowPos - position of point light

*/
static void SetGlow( const Vec3 vGlowPos )
{
    glEnable( GL_LIGHTING ) ;

    // Code below here belongs in "QdLight"
    // Simplify (and speed up) specular computation
    glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE ) ;

    // Global ambient light
    {
        static const int iLight = GL_LIGHT0 ;
        float globalAmbientLight[] = { 1.0f , 1.0f , 1.0f , 1.0f } ;
        glLightModelfv( GL_LIGHT_MODEL_AMBIENT , globalAmbientLight ) ;
        glEnable( iLight ) ;
    }

    // Glow point light
    {
        glPushMatrix() ;
        static const int iLight = GL_LIGHT1 ;
        const Vec4 diffuse( 18.0 , 9.0 , 0.0 , 1.0 ) ;
        glLightfv( iLight , GL_DIFFUSE , (float*) & diffuse ) ;
        Vec4 pos( vGlowPos , 1.0 ) ;
        glLightfv( iLight , GL_POSITION , (float*) & pos ) ;
        glLightf( iLight , GL_CONSTANT_ATTENUATION , 0.0f ) ;
        glLightf( iLight , GL_LINEAR_ATTENUATION , 0.0f ) ;
        glLightf( iLight , GL_QUADRATIC_ATTENUATION , 10.0 ) ;
        glEnable( iLight ) ;
        glPopMatrix() ;
    }

    // Directional light from above
    {
        glPushMatrix() ;
        static const int iLight = GL_LIGHT2 ;
        const Vec4 ambient( 0.05 , 0.05 , 0.05 , 1.0 ) ;
        glLightfv( iLight , GL_AMBIENT , (float*) & ambient ) ;
        const Vec4 diffuse( 1.0 , 1.0 , 1.0 , 1.0 ) ;
        glLightfv( iLight , GL_DIFFUSE , (float*) & diffuse ) ;
        const Vec4 position( 0.0 , 0.0 , -1.0 , 0.0 ) ;
        glLightfv( iLight , GL_POSITION , (float*) & position ) ;
        glEnable( iLight ) ;
        glPopMatrix() ;
    }
}
#endif




/** Construct application for interactive simulation and visualization.

    \param ic - which initial conditions to impose

    This is just a demonstration of a few initial conditions.
    There is nothing fundamental or especially important about
    the initial conditions that this routine implements.
    These are just examples.

*/
void InteSiVis::InitialConditions( unsigned ic )
{
    mScenario           = ic  ;
    sInstance->mFrame   = 0   ;
    sInstance->mTimeNow = 0.0 ;

    VortonSim & vortonSim = mPclOpVortonSim.mVortonSim ;

    vortonSim.Clear() ;     // Reset the fluid simulator
    mParticleGroupTracers.mParticles.Clear() ;     // Remove all passive tracer particles
    GetSpheres().Clear() ;  // Remove all rigid bodies from the simulation

    // Set up lights.
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
    Vector< Vorton > &          vortons                     = * vortonSim.GetVortons() ;
    static const float          veryLargeMass               = 1.0e16f ; // large but not large enough that 1/mass is a denormal.  Denormal values occur when veryLargeMass is around 1.0e34f.
    const Vector< Particle > *  vortonsImplyingTracers      = 0 ;
    Vec3                        tracerGridScale             ( 1.0f , 1.0f , 1.0f ) ;

    mPclOpKillAgeVortons.mAgeMax    = INT_MAX ;                         // By default, disable vorton killing.
    mPclOpKillAgeTracers.mAgeMax    = mPclOpKillAgeVortons.mAgeMax ;    // By default, disable tracer killing.
    mPclOpEmitVortons.mEmitRate     = 0.0f ;                            // By default, disable vorton emission.
    mPclOpEmitTracers.mEmitRate     = 0.0f ;                            // By default, disable tracer emission.
    mPclOpWindVortons.mWind         = Vec3( 0.0f , 0.0f , 0.0f ) ;      // By default, disable wind.

    mPclOpFluidBodInteVortons.mDensityDeviationGrid = 0 ;

    srand( 1 ) ;    // Seed pseudo-random number generator to make results repeatable.

    switch( ic )
    {   // Switch on initial conditions
        case 0: // vortex ring -- vorticity in [0,1]
            AssignVortons( vortons , 5.0f * fMagnitude , numVortonsMax , VortexRing( fRadius , fThickness , Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 10.f , 0.f , 0.f ) ) ;
            mCamera.SetEye( Vec3( 10.f , -10.0f , 0.0f ) ) ;
        break ;
        case 1: // "jet" vortex ring -- velocity in [0,1]
            AssignVortons( vortons ,  5.0f * fMagnitude , numVortonsMax , JetRing( fRadius , fThickness , Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            //vortonsImplyingTracers = ( const Vector< Particle > * )( & vortons ) ;
            numTracersPerCellCubeRoot -= 2 ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 10.f ,    0.f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 10.f , -10.0f , 0.0f ) ) ;
        break ;
        case 2: // Projectile
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.25f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot ++ ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( -3.0f , 0.0f , 0.0f ) , Vec3( 4.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Make sphere spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;
        case 3: // Projectile with spin about longitudinal axis
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 4.0f * fThickness , 0.3f * fThickness , 0.3f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( -2.0f , 0.0f , 0.0f ) , Vec3( 10.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.005f , 0.0f , 0.0f ) ) ; // Make sphere spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;
        case 4: // Projectile with spin about transverse axis
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax , VortexNoise( Vec3( 4.0f * fThickness , 0.125f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -- ;

            // Add a sphere.
            GetSpheres().PushBack( RbSphere( Vec3( -2.0f , 0.0f , 0.0f ) , Vec3( 9.0f , 0.0f , 0.0f ) , 0.02f , 0.2f ) ) ;
            GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.01f , 0.0f ) ) ; // Make sphere spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
        break ;
        case 5: // Spinning sphere in initially stationary fluid
            AssignVortons( vortons , 0.125f * FLT_EPSILON , numVortonsMax / 2 , VortexNoise( Vec3( 2.0f * fThickness , 2.0f * fThickness , 2.0f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot += 6 ;
            tracerGridScale = Vec3( 1.0f , 0.125f , 1.0f ) ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( 0.f , 0.0f , 0.0f ) , Vec3( 0.f , 0.0f , 0.0f ) , 1000.0f , fThickness * 0.3f ) ) ;
            //GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 100.0f ) ) ; // Make sphere spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 2.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -4.0f , 2.0f ) ) ;
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
            AssignVortons( vortons , 0.125f * FLT_EPSILON , 512 , VortexNoise( Vec3( 0.5f * fThickness , 0.5f * fThickness , 0.5f * fThickness ) ) ) ;
            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            //GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Make sphere spin

            mPclOpKillAgeVortons.mAgeMax            = 100 ;
            mPclOpKillAgeTracers.mAgeMax            = mPclOpKillAgeVortons.mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition           = Vec3( -0.5f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity           = Vec3( 1.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize               = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensityDeviation   = 0.0f ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction       = 0.0f ;
            emitterTemplate.mFlameFraction      = 0.0f ;
            emitterTemplate.mSmokeFraction      = 1.0f ;
        #endif
            emitterSpread.mPosition             = Vec3( emitterTemplate.mVelocity.x / 30.0f , 0.5f , 0.5f ) ;

            mPclOpEmitVortons.mTemplate                     = emitterTemplate ;
            mPclOpEmitVortons.mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mPclOpEmitVortons.mSpread                       = emitterSpread ;
            mPclOpEmitVortons.mEmitRate                     = 500.0f ;

            mPclOpEmitTracers.mEmitRate                     = 10000.0f ;
            mPclOpEmitTracers.mTemplate                     = mPclOpEmitVortons.mTemplate ;
            mPclOpEmitTracers.mSpread                       = mPclOpEmitVortons.mSpread ;
            mPclOpEmitTracers.mSpread.mPosition.x           = mPclOpEmitVortons.mSpread.mPosition.x        ;
            mPclOpEmitTracers.mSpread.mPosition.y           = mPclOpEmitVortons.mSpread.mPosition.y * 0.3f ;
            mPclOpEmitTracers.mSpread.mPosition.z           = mPclOpEmitVortons.mSpread.mPosition.z * 0.3f ;
            mPclOpWindVortons.mWind                         = Vec3( 1.0f , 0.0f , 0.0f ) ;

            numTracersPerCellCubeRoot -- ; // Emit rate controls tracer density, but this effectively controls their radius.

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 1.0f ,  0.0f , 0.0f ) ) ;
            mCamera.SetEye   ( Vec3( 1.0f , -3.0f , 0.0f ) ) ;
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
        case 11: // 2D sheet
            AssignVortons( vortons , fMagnitude , numVortonsMax , VortexSheet( fThickness , /* variation */ 0.0f , /* width */ 2.0f * fThickness ) ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -10.0f , 0.0f ) ) ;
        break ;
        case 12: // ball of initially stationary heavy fluid
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius , 10.0f ) , Vec3( 0.0f , 0.0f , 5.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const Vector< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot += 2 ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -10.0f , 0.0f ) ) ;
        break ;
        case 13: // Two balls of initially stationary fluid, one heavy, one light, heavy one above light one.
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius , -0.9f ) , Vec3( 0.0f , 0.0f , -2.0f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax , DensityBall( fRadius ,  0.9f ) , Vec3( 0.0f , 0.0f ,  2.0f ) ) ;

            vortonsImplyingTracers = reinterpret_cast< const Vector< Particle > * >( & vortons ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -5.0f , 0.0f ) ) ;
        break ;
        case 14: // Two boxes of initially stationary fluid, one heavy, one light, light one above heavy one.
        {
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) ,  0.6f ) , Vec3( 0.0f , 0.0f , -0.5f ) ) ;
            AssignVortons( vortons , 1.0f , numVortonsMax / 2 , DensityBox( Vec3( fRadius , fRadius , fRadius ) , -0.6f ) , Vec3( 0.0f , 0.0f ,  0.5f ) ) ;
            mPclOpFluidBodInteVortons.mDensityDeviationGrid = & mPclOpVortonSim.mVortonSim.GetDensityDeviationGrid() ;

            vortonsImplyingTracers = reinterpret_cast< const Vector< Particle > * >( & vortons ) ;
            numTracersPerCellCubeRoot -- ;

            const float sphereRadius = 0.2f ;
            const float neutrallyBuoyantMass = mPclOpVortonSim.mVortonSim.GetAmbientDensity() * 4.0f * PI * POW3( sphereRadius ) / 3.0f ;
            GetSpheres().PushBack( RbSphere( Vec3( 0.f , 0.0f , 0.5f ) , Vec3( 0.f , 0.0f , 0.0f ) , neutrallyBuoyantMass , sphereRadius ) ) ;

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
            mCamera.SetEye( Vec3( 0.0f , -3.0f , 0.0f ) ) ;
        }
        break ;
        case 15: // Ball heats fluid
        {
            AssignVortons( vortons , 0.125f * FLT_EPSILON , 512 , VortexNoise( Vec3( 0.5f * fThickness , 0.5f * fThickness , 0.5f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -= 2 ;

            mPclOpKillAgeVortons.mAgeMax            = 300 ;
            mPclOpKillAgeTracers.mAgeMax            = mPclOpKillAgeVortons.mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition           = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity           = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize               = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensityDeviation   = 0.0f ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction       = 0.0f ;
            emitterTemplate.mFlameFraction      = 0.0f ;
            emitterTemplate.mSmokeFraction      = 0.0f ;
        #endif
            emitterSpread.mPosition             = Vec3( 0.5f , 0.5f , 0.5f ) ;

            mPclOpEmitVortons.mTemplate                     = emitterTemplate ;
            mPclOpEmitVortons.mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mPclOpEmitVortons.mSpread                       = emitterSpread ;
            mPclOpEmitVortons.mEmitRate                     = 100.0f ;
            mPclOpEmitTracers.mEmitRate                     = 5000.0f ;
            mPclOpEmitTracers.mTemplate                     = mPclOpEmitVortons.mTemplate ;
            mPclOpEmitTracers.mSpread                       = mPclOpEmitVortons.mSpread ;
            mPclOpEmitTracers.mSpread.mPosition.x           = mPclOpEmitVortons.mSpread.mPosition.x ;
            mPclOpEmitTracers.mSpread.mPosition.y           = mPclOpEmitVortons.mSpread.mPosition.y ;
            mPclOpEmitTracers.mSpread.mPosition.z           = mPclOpEmitVortons.mSpread.mPosition.z ;
            mPclOpWindVortons.mWind                         = Vec3( 0.0f , 0.0f , 0.0f ) ;

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            //GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Make sphere spin
            GetSpheres()[0].SetTemperature( sAmbientTemperature + 300.0f ) ;

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

            AssignVortons( vortons , 0.125f * FLT_EPSILON , 400 , VortexNoise( Vec3( 0.22f * fThickness , 0.22f * fThickness , 0.22f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -= 2 ;

            mPclOpKillAgeVortons.mAgeMax            = 300 ;
            mPclOpKillAgeTracers.mAgeMax            = mPclOpKillAgeVortons.mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition           = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity           = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize               = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensityDeviation   = 0.0f ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction       = 0.01f ;   // Add a little bit of fluid. Note that the fuel-air ratio in a combustion engine is around 0.068.
            emitterTemplate.mFlameFraction      = 0.0f  ;   // No flames initially -- those will occur from ignition.
            emitterTemplate.mSmokeFraction      = 0.99f ;   // Mostly smoke.
        #endif
            emitterSpread.mPosition             = Vec3( 0.5f , 0.5f , 0.5f ) ;

            mPclOpEmitVortons.mTemplate                     = emitterTemplate ;
            mPclOpEmitVortons.mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mPclOpEmitVortons.mSpread                       = emitterSpread ;
            mPclOpEmitVortons.mEmitRate                     = 100.0f ;
            mPclOpEmitTracers.mEmitRate                     = 5000.0f ;
            mPclOpEmitTracers.mTemplate                     = mPclOpEmitVortons.mTemplate ;
            mPclOpEmitTracers.mSpread                       = mPclOpEmitVortons.mSpread ;
            mPclOpEmitTracers.mSpread.mPosition.x           = mPclOpEmitVortons.mSpread.mPosition.x ;
            mPclOpEmitTracers.mSpread.mPosition.y           = mPclOpEmitVortons.mSpread.mPosition.y ;
            mPclOpEmitTracers.mSpread.mPosition.z           = mPclOpEmitVortons.mSpread.mPosition.z ;
            mPclOpWindVortons.mWind                         = Vec3( 0.0f , 0.0f , 0.0f ) ;

        #if ENABLE_FIRE
            mPclOpVortonSim.mVortonSim.SetCombustionTemperature( 1.0f * sAmbientTemperature + 1.5f ) ;
            mPclOpVortonSim.mVortonSim.SetCombustionRateFactor( 2.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSmokeTemperature( 2000.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSmokeRateFactor( 5.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSpecificFreeEnergy( 10000.0f ) ;
        #endif

            // Add a sphere
            GetSpheres().PushBack( RbSphere( Vec3(  0.0f , 0.0f , -0.5f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            GetSpheres().PushBack( RbSphere( Vec3(  0.2f , 0.0f ,  2.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1.0f /* veryLargeMass */ , 0.5f ) ) ;
            GetSpheres().PushBack( RbSphere( Vec3( -1.0f , 1.0f ,  1.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.3f ) ) ;

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

            AssignVortons( vortons , 0.125f * FLT_EPSILON , 400 , VortexNoise( Vec3( 0.2f * fThickness , 0.2f * fThickness , 0.2f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -= 2 ;

            mPclOpKillAgeVortons.mAgeMax            = 800 ;
            mPclOpKillAgeTracers.mAgeMax            = mPclOpKillAgeVortons.mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition           = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity           = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize               = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensityDeviation   = 0.0f ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction       = 0.01f ;   // Add a little bit of fluid. Note that the fuel-air ratio in a combustion engine is around 0.068.
            emitterTemplate.mFlameFraction      = 0.0f  ;   // No flames initially -- those will occur from ignition.
            emitterTemplate.mSmokeFraction      = 0.99f ;   // Mostly smoke.
        #endif
            emitterSpread.mPosition             = Vec3( 0.2f , 0.2f , 0.2f ) ;

            mPclOpEmitVortons.mTemplate                     = emitterTemplate ;
            mPclOpEmitVortons.mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mPclOpEmitVortons.mSpread                       = emitterSpread ;
            mPclOpEmitVortons.mEmitRate                     = 100.0f ;
            mPclOpEmitTracers.mEmitRate                     = 1000.0f ;
            mPclOpEmitTracers.mTemplate                     = mPclOpEmitVortons.mTemplate ;
            mPclOpEmitTracers.mSpread                       = mPclOpEmitVortons.mSpread ;
            mPclOpEmitTracers.mSpread.mPosition.x           = mPclOpEmitVortons.mSpread.mPosition.x ;
            mPclOpEmitTracers.mSpread.mPosition.y           = mPclOpEmitVortons.mSpread.mPosition.y ;
            mPclOpEmitTracers.mSpread.mPosition.z           = mPclOpEmitVortons.mSpread.mPosition.z ;
            mPclOpWindVortons.mWind                         = Vec3( 0.0f , 0.0f , 0.0f ) ;

        #if ENABLE_FIRE
            mPclOpVortonSim.mVortonSim.SetCombustionTemperature( sAmbientTemperature + 1.3f ) ;
            mPclOpVortonSim.mVortonSim.SetCombustionRateFactor( 5.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSmokeTemperature( 2000.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSmokeRateFactor( 10.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSpecificFreeEnergy( 10000.0f ) ;
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

            AssignVortons( vortons , 0.125f * FLT_EPSILON , 400 , VortexNoise( Vec3( 0.25f * fThickness , 0.25f * fThickness , 0.25f * fThickness ) ) ) ;

            numTracersPerCellCubeRoot -= 2 ;

            mPclOpKillAgeVortons.mAgeMax            = 300 ;
            mPclOpKillAgeTracers.mAgeMax            = mPclOpKillAgeVortons.mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition           = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity           = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize               = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensityDeviation   = 0.0f ;
        #if ENABLE_FIRE
            emitterTemplate.mFuelFraction       = 0.05f ;   // Add a little bit of fluid. Note that the fuel-air ratio in a combustion engine is around 0.068.
            emitterTemplate.mFlameFraction      = 0.0f  ;   // No flames initially -- those will occur from ignition.
            emitterTemplate.mSmokeFraction      = 0.99f ;   // Mostly smoke.
        #endif
            emitterSpread.mPosition             = Vec3( 1.0f , 1.0f , 0.5f ) ;

            mPclOpEmitVortons.mTemplate                     = emitterTemplate ;
            mPclOpEmitVortons.mTemplate.mAngularVelocity    = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
            mPclOpEmitVortons.mSpread                       = emitterSpread ;
            mPclOpEmitVortons.mEmitRate                     = 200.0f ;
            mPclOpEmitTracers.mEmitRate                     = 10000.0f ;
            mPclOpEmitTracers.mTemplate                     = mPclOpEmitVortons.mTemplate ;
            mPclOpEmitTracers.mSpread                       = mPclOpEmitVortons.mSpread ;
            mPclOpEmitTracers.mSpread.mPosition.x           = mPclOpEmitVortons.mSpread.mPosition.x ;
            mPclOpEmitTracers.mSpread.mPosition.y           = mPclOpEmitVortons.mSpread.mPosition.y ;
            mPclOpEmitTracers.mSpread.mPosition.z           = mPclOpEmitVortons.mSpread.mPosition.z ;
            mPclOpWindVortons.mWind                         = Vec3( 0.0f , 0.0f , 0.0f ) ;

        #if ENABLE_FIRE
            mPclOpVortonSim.mVortonSim.SetCombustionTemperature( 1.0f * sAmbientTemperature + 1.5f ) ;
            mPclOpVortonSim.mVortonSim.SetCombustionRateFactor( 2.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSmokeTemperature( 5000.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSmokeRateFactor( 2.0f ) ;
            mPclOpVortonSim.mVortonSim.SetSpecificFreeEnergy( 20000.0f ) ;
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
        default:
        break ;
    }

    // Apply same wind to tracers that vortons experience.
    mPclOpWindTracers.mWind = mPclOpWindVortons.mWind ;

    // Tell Fluid-Body simulation about rigid bodies.
    GetRigidBodies().Clear() ;
    for( size_t iRigidBody = 0 ; iRigidBody < GetSpheres().Size() ; ++ iRigidBody )
    {   // For each rigid body in the simulation...
        RbSphere * pSphere = & GetSpheres()[ iRigidBody ] ;
        GetRigidBodies().PushBack( pSphere ) ;
    }

#if( ENABLE_FLUID_BODY_SIMULATION )
    FluidBodySim::RemoveEmbeddedParticles( reinterpret_cast< Vector< Particle > & > ( * vortonSim.GetVortons() ) , GetRigidBodies() ) ;
    FluidBodySim::RemoveEmbeddedParticles( mParticleGroupTracers.mParticles , GetRigidBodies() ) ;
#endif

    vortonSim.Initialize() ;
    UniformGridGeometry tracerGrid( vortonSim.GetGrid() ) ;
    tracerGrid.Scale( vortonSim.GetGrid() , tracerGridScale ) ;

    Particles::Emit( mParticleGroupTracers.mParticles , tracerGrid , numTracersPerCellCubeRoot , vortonsImplyingTracers ) ;

#if( ENABLE_FLUID_BODY_SIMULATION )
    FluidBodySim::RemoveEmbeddedParticles( reinterpret_cast< Vector< Particle > & >( * vortonSim.GetVortons() ) , GetRigidBodies() ) ;
    FluidBodySim::RemoveEmbeddedParticles( mTracers , GetRigidBodies() ) ;
#endif

#if PROFILE
    fprintf( stderr , "Initial condition %i, simulation objects: %i tracers    %i vortons    %i spheres\n"
        , ic , mTracers.Size() , vortons.Size() , GetSpheres().Size() ) ;
#endif

}




/** Destruct application for interactive simulation and visualization.
*/
InteSiVis::~InteSiVis()
{
    sInstance = 0 ;
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




/// Set the projection matrix to orthographic, for rendering text.
void setOrthographicProjection()
{
	// switch to projection mode
	glMatrixMode(GL_PROJECTION);

	// save previous matrix which contains the
	//settings for the perspective projection
	glPushMatrix();

	// reset matrix
	glLoadIdentity();

    int viewportXYWH[4] ;
    glGetIntegerv( GL_VIEWPORT , viewportXYWH ) ;

	// set a 2D orthographic projection
	gluOrtho2D(0, viewportXYWH[ 2 ] , viewportXYWH[ 3 ] , 0);

	// switch back to modelview mode
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glLoadIdentity();
}




/// Set the projection matrix to projection, for rendering 3D objects.
void restorePerspectiveProjection()
{
	glMatrixMode(GL_PROJECTION);
	// restore previous projection matrix
	glPopMatrix();

	// get back to modelview mode
	glMatrixMode(GL_MODELVIEW);
    glPopMatrix() ;
}




/// Render the given string using the given font.
static void oglRenderBitmapString( void * font , const char * string )
{
    for( const char * c = string ; *c != '\0' ; c ++ )
    {
        glutBitmapCharacter( font , *c ) ;
    }
}




static void oglRenderBitmapString( const Vec3 & pos , void * font , const Vec4 & color , const char * string , bool useScreenSpace )
{
    if( useScreenSpace )
    {
        setOrthographicProjection() ;
    }

    glPushAttrib( GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT ) ; // Should include GL_ENABLE_BIT or GL_ALL_ATTRIB_BITS but for some reason that *sometimes* makes things disappear, like vortex lines.

    glDisable( GL_LIGHTING ) ;
    glDisable( GL_TEXTURE_2D ) ;
    glEnable( GL_BLEND ) ;
    glBlendFunc( GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA ) ;  // Typical alpha blending

    if( useScreenSpace )
    {   // Only when drawing in screen space...
        glDisable( GL_DEPTH_TEST ) ;    // Disable depth test so text appears as if in front of everything already drawn.
    }

    glColor4fv( reinterpret_cast< const GLfloat * >( & color ) ) ;

    //int viewportXYWH[4] ;
    //glGetIntegerv( GL_VIEWPORT , viewportXYWH ) ;

    glRasterPos3f( pos.x , pos.y , pos.z ) ;
    oglRenderBitmapString( font , string ) ;

    glPopAttrib() ;

    if( useScreenSpace )
    {
        restorePerspectiveProjection() ;
    }
}




/// Render the given string at the given screen-space position using the given font.
void oglRenderString( const Vec3 & pos , void * font , const char * format , ... )
{
    char stringBuffer[ 512 ] ;
    va_list args ;
    va_start( args , format ) ;
    vsprintf( stringBuffer , format , args ) ;
    va_end( args ) ;
    oglRenderBitmapString( pos , font , Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) , stringBuffer , true ) ;
}




/// Render the given string at the given world-space position using the given font.
void oglRenderStringWorld( const Vec3 & pos , void * font , const Vec4 & color , const char * format , ... )
{
    char stringBuffer[ 512 ] ;
    va_list args ;
    va_start( args , format ) ;
    vsprintf( stringBuffer , format , args ) ;
    va_end( args ) ;
    oglRenderBitmapString( pos , font , color , stringBuffer , false ) ;
}




/** Draw bounding box and grid cells.

    \param grid         Grid to draw.

    \param material     Material to use to draw grid.

    \parm bDrawCells    Whether to draw all cells (if true) or just bounding box (if false).

*/
static void DrawGrid( const UniformGrid<Vec3> & grid , const QdMaterial & material , bool bDrawCells , bool bRenderDiagnosticText )
{
    if( grid.HasZeroExtent() )
    {   // Grid has no size.
        return ;    // There is nothing to draw.
    }

    // Draw bounding box
    material.UseMaterial() ;
    {
        glPushMatrix() ;

        const Vec3 vCenter = grid.GetCenter() ;
        glTranslatef( vCenter.x , vCenter.y , vCenter.z ) ;
        const Vec3 gridExtent = grid.GetExtent() ;
        glScalef( gridExtent.x , gridExtent.y , gridExtent.z ) ;

        glutWireCube( 1.0 ) ;

        glPopMatrix() ;
    }

    // Gather min and max statistics for grid contents (speeds)
    float magMin ;
    float magMax ;
    FindMagnitudeRange( grid , magMin , magMax ) ;
    magMin *= ( 1.0f - FLT_EPSILON ) ;  // Account for diff between fsqrtf and sqrtf
    magMax *= ( 1.0f + FLT_EPSILON ) ;  // Account for diff between fsqrtf and sqrtf
    const float oneOverMagRange = 1.0f / ( magMax - magMin ) ;
    const bool speedRangeIsNonZero = magMax != magMin ;

    const Vec3  cellSize   = grid.GetCellSpacing() /* shrink cell to visualize each, otherwise edges overlap */ * 0.99f ;
    const float cellLength = powf( grid.GetCellVolume() , 0.33333333333333333f ) ;

    if( bDrawCells )
    {
        // Draw each grid cell.
        Vec4            color   = material.GetColor() ;
        const unsigned  nx      = grid.GetNumCells( 0 ) ;
        const unsigned  ny      = grid.GetNumCells( 1 ) ;
        const unsigned  nz      = grid.GetNumCells( 2 ) ;
        const unsigned  npx     = grid.GetNumPoints( 0 ) ;
        const unsigned  npy     = grid.GetNumPoints( 1 ) ;
        const unsigned  npxy    = npx * npy ;
        unsigned        indices[ 4 ] ;
        unsigned &      ix      = indices[ 0 ] ;
        unsigned &      iy      = indices[ 1 ] ;
        unsigned &      iz      = indices[ 2 ] ;
        for( iz = 0 ; iz < nz ; ++ iz )
        {
            const unsigned zOffset = iz * npxy ;
            for( iy = 0 ; iy < ny ; ++ iy )
            {
                const unsigned yzOffset = zOffset + iy * npx ;
                for( ix = 0 ; ix < nx ; ++ ix )
                {
                    const unsigned xyzOffset = ix + yzOffset ;

                    // Note: pushing and popping matrix per cell is probably crazy slow.
                    // This routine could be sped up considerably.
                    glPushMatrix() ;

                    const Vec3 vCenter = grid.GetCellCenter( ix , iy , iz ) ;
                    glTranslatef( vCenter.x , vCenter.y , vCenter.z ) ;
                    glScalef( cellSize.x , cellSize.y , cellSize.z ) ;

                    const Vec3 & vel      = grid[ xyzOffset ] ;
                    const float speed     = vel.Magnitude() ;
                    const float speed0to1 = ( speed - magMin ) * oneOverMagRange ;

                    // Choose color based on speed of gridpoint at minimal corner.
                    // Note: a more proper rendering would use different velocity at each gridpoint.
                    if( speedRangeIsNonZero )
                    {
                        
                        color = GetColorFromRamp( sColorRamp , NUM_COLORS_IN_RAMP , speed0to1 ) ; // Color based on speed.
                        color.w *= sInstance->CameraFocusEmphasis( vCenter ) ;  // Modulate opacity based on distance to camera target.
                        glColor4f( color.x , color.y , color.z , color.w ) ;
                    }

                    glutWireCube( 1.0 ) ;

                    glPopMatrix() ;

                    if( bRenderDiagnosticText && color.w > 0.01f )
                    {   // Diagnostic text is enabled and opacity is sufficient to render it.
                        Vec3 minCorner ;
                        grid.PositionFromIndices( minCorner , indices ) ;
                        const Vec3 & value = grid[ xyzOffset ] ;
                    #if 1
                        // Draw value magnitude
                        oglRenderStringWorld( minCorner , GLUT_BITMAP_HELVETICA_10 , 2.0f * color , "%g" , value.Magnitude() ) ;
                    #else
                        // Draw position and value.
                        oglRenderStringWorld( minCorner , GLUT_BITMAP_HELVETICA_10 , 2.0f * color , "%g,%g,%g %g,%g,%g"
                            , minCorner.x , minCorner.y , minCorner.z
                            , value.x , value.y , value.z
                            ) ;
                    #endif
                        // Draw an arrow indicating value direction and magnitude.
                        // Scale arrow according to cell size and speed.
                        Vec3 arrowEnd( minCorner + value.GetDir() * speed0to1 * cellLength ) ;
                        glBegin( GL_LINES ) ;
                        glVertex3fv( reinterpret_cast< GLfloat * >( & minCorner ) ) ;
                        glVertex3fv( reinterpret_cast< GLfloat * >( & arrowEnd  ) ) ;
                        glEnd() ;
                    }

                }
            }
        }
    }
}




/** Create materials for rendering particles.
*/
void InteSiVis::CreateParticleMaterials()
{

    // Light & heavy vorton materials.
    {
        mVortonMaterial.AllocateTexture( 256 , 512, 2 ) ;
        const Vec4 opaqueLight( 0.9f , 0.9f , 0.1f , 0.125f ) ; // yellowish
        const Vec4 opaqueHeavy( 0.1f , 0.9f , 0.9f , 0.125f ) ; // cyanish
        DecoratedNoiseBallTextureProc noiseBallLight( 1.5f , opaqueLight ) ;
        DecoratedNoiseBallTextureProc noiseBallHeavy( 1.5f , opaqueHeavy ) ;
        mVortonMaterial.AssignTexturePage( noiseBallLight , 1 ) ;
        mVortonMaterial.AssignTexturePage( noiseBallHeavy , 0 ) ;
        mVortonMaterial.FinalizeTexture() ;
        mVortonMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mVortonMaterial.SetDepthWrite( false ) ;
        mVortonMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
    }

    // Light & heavy tracer materials
    {
        mHeavyLightTracerMaterial.AllocateTexture( 128 , 256 , 2 ) ;
        const Vec4 opaqueLight( 0.9f , 0.1f , 0.1f , 0.125f ) ; // redish
        const Vec4 opaqueHeavy( 0.1f , 0.1f , 0.9f , 0.125f ) ; // blueish
        NoiseBallTextureProc noiseBallLight( 0.5f , opaqueLight ) ;
        NoiseBallTextureProc noiseBallHeavy( 0.5f , opaqueHeavy ) ;
        mHeavyLightTracerMaterial.AssignTexturePage( noiseBallLight , 1 ) ;
        mHeavyLightTracerMaterial.AssignTexturePage( noiseBallHeavy , 0 ) ;
        mHeavyLightTracerMaterial.FinalizeTexture() ;
        mHeavyLightTracerMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mHeavyLightTracerMaterial.SetDepthWrite( false ) ;
        mHeavyLightTracerMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        mHeavyLightTracerMaterial.SetScale( 2.0f ) ;
    #if ENABLE_FIRE
        mHeavyLightTracerMaterial.SetDensityVisibility( 255.0f * 100.0f ) ;
    #else
        mHeavyLightTracerMaterial.SetDensityVisibility( FLT_MAX ) ;
    #endif
    }

    // Smoke material
    {
        mSmokeMaterial.AllocateTexture( 128 , 128 , 1 ) ;
        const Vec4 translucentGray( 0.7f , 0.7f , 0.7f , 0.125f ) ; // light gray
        NoiseBallTextureProc noiseBall( 0.5f , translucentGray ) ;
        mSmokeMaterial.AssignTexturePage( noiseBall , 0 ) ;
        mSmokeMaterial.FinalizeTexture() ;
        mSmokeMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mSmokeMaterial.SetDepthWrite( false ) ;
        mSmokeMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        mSmokeMaterial.SetScale( 4.0f ) ;
    #if ENABLE_FIRE
        mSmokeMaterial.SetDensityVisibility( 100.0f ) ;
    #else
        mSmokeMaterial.SetDensityVisibility( 0.0f ) ;
    #endif
    }

    // Flame material
    {
        mFlameMaterial.AllocateTexture( 128 , 128 , 1 ) ;
        const Vec4 orange( 1.0f , 0.5f , 0.1f , 1.0f ) ; // orange.
        NoiseBallTextureProc noiseBall( 0.5f , orange ) ;
        mFlameMaterial.AssignTexturePage( noiseBall , 0 ) ;
        mFlameMaterial.FinalizeTexture() ;
        mFlameMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mFlameMaterial.SetDepthWrite( false ) ;
        mFlameMaterial.SetBlendMode( QdMaterial::BM_ADDITIVE ) ;
        mFlameMaterial.SetScale( 4.0f ) ;
    #if ENABLE_FIRE
        mFlameMaterial.SetDensityVisibility( 255.0f * 100.0f ) ;
    #else
        mFlameMaterial.SetDensityVisibility( 0.0f ) ;
    #endif
    }
}




/** Create rendering materials.
*/
void InteSiVis::CreateMaterials()
{

    CreateParticleMaterials() ;

    mBallMaterial.MakeSimpleNoiseTexture( 256 , 256 , 1 , 0.2f ) ;
    mBallMaterial.SetColor( Vec4( 0.8f , 0.4f , 0.8f , 1.0f ) ) ;

    mBoundingBoxMaterial.MakeSimpleNoiseTexture( 32 , 32 , 1 , 0.0f ) ;
    mBoundingBoxMaterial.SetColor( Vec4( 0.7f , 0.9f , 0.7f , 0.01f ) ) ;
    mBoundingBoxMaterial.SetDepthWrite( false ) ;
    mBoundingBoxMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;

    mSkyMaterial.MakeSimpleNoiseTexture( 256 , 256 , 1 , 0.05f ) ;
    mSkyMaterial.SetColor( Vec4( 0.3f , 0.3f , 0.3f , 1.0f ) ) ;

    mInitialized = true ;
}




/** Perform initializations that cannot happen before render device is initialized.

    For example, the device cannot generate a texture until OpenGL has been initialized.
*/
void InteSiVis::InitializeRendering()
{

    CreateMaterials() ;

	mBallDisplayList = glGenLists( 1 ) ; // Create the id for the display list
	glNewList( mBallDisplayList , GL_COMPILE ) ;
    glTexGeni( GL_S , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glTexGeni( GL_T , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glEnable( GL_TEXTURE_GEN_S ) ;
    glEnable( GL_TEXTURE_GEN_T ) ;
    glutSolidSphere( 1.0f , 32 , 32 ) ;
    glDisable( GL_TEXTURE_GEN_S ) ;
    glDisable( GL_TEXTURE_GEN_T ) ;
    glEndList() ;

    mInitialized = true ;
}




/** Render sky.
*/
void InteSiVis::RenderSky()
{
    QUERY_PERFORMANCE_ENTER ;
    mSkyMaterial.UseMaterial() ;
    QdLight::DisableLights() ;
    {
        glCullFace(GL_FRONT) ; // Render /inside/ of sphere
        glPushMatrix() ;
        glTranslatef( 0.0f , 0.0f , 0.0f ) ; // should translate to camera eye
        static const float skyRadius = 500.0f ;
        glScalef( skyRadius , skyRadius , skyRadius ) ;
		glCallList( mBallDisplayList ) ;
        glPopMatrix() ;
    }
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_Sky ) ;
}




/** Render rigid bodies.
*/
void InteSiVis::RenderRigidBodies()
{
    // Render balls
    mLights[0].SetLight( 0 , mTimeNow ) ;
    mLights[1].SetLight( 1 , mTimeNow ) ;
    mBallMaterial.UseMaterial() ;
    const size_t numSpheres = GetSpheres().Size() ;
    for( size_t iSphere = 0 ; iSphere < numSpheres ; ++ iSphere )
    {
        RbSphere & sphere = GetSpheres()[ iSphere ] ;
        glPushMatrix() ;
    #if 1   // Use OpenGL to apply transforms.
        glTranslatef( sphere.GetPosition().x , sphere.GetPosition().y , sphere.GetPosition().z ) ;
        {
            const float orientMag = sphere.GetOrientation().Magnitude() ;
            if( orientMag >  TWO_PI )
            {   // Orientation angle is larger than a full revolution so truncate it.
                const float orientMagNew = orientMag - TWO_PI ;
                sphere.SetOrientation( sphere.GetOrientation() * ( orientMagNew / orientMag ) ) ;
            }
            else if( orientMag < 0 )
            {   // Orientation angle is larger than a full revolution so truncate it.
                const float orientMagNew = orientMag - TWO_PI ;
                sphere.SetOrientation( sphere.GetOrientation() * ( orientMagNew / orientMag ) ) ;
            }
        }
        const float angle = sphere.GetOrientation().Magnitude() ;
        Vec4 rotAxis( sphere.GetOrientation() ) ;
        if( fabsf( angle ) > FLT_EPSILON )
        {   // angle is not near zero.  (Angle near zero would cause problems for Normalize below.)
            rotAxis.Normalizev3() ;
        }
        else
        {
            rotAxis = Vec4( 1.0f , 0.0f , 0.0f , 0.0f ) ;
        }
        glRotatef( angle * RAD2DEG , rotAxis.x , rotAxis.y , rotAxis.z ) ;
        glScalef( sphere.mRadius , sphere.mRadius , sphere.mRadius ) ;
    #else   // Build local-to-world transform manually.
        Mat4 xLocalToWorld( Mat4_xIdentity ) ;
        xLocalToWorld.SetTranslation( Vec4( sphere.mPosition , 1.0f ) ) ;
        glMultMatrixf( (GLfloat*) & xLocalToWorld ) ;
    #endif
		glCallList( mBallDisplayList ) ;
        glPopMatrix() ;
    }
}




/** Render diagnostic information for vortex particles.
*/
void InteSiVis::RenderVortonDiagnostics()
{

    VortonSim &                 rVortonSim  = mPclOpVortonSim.mVortonSim ;
    Vector< Vorton > &          vortons     = * rVortonSim.GetVortons() ;
    const UniformGrid< Vec3 > & velGrid     = mPclOpVortonSim.mVortonSim.GetVelocityGrid() ;

    size_t numVortons = vortons.size() ;
    for( size_t iVort = 0 ; iVort < numVortons ; ++ iVort )
    {   // For each vorton...
        const Vorton & vort = vortons[ iVort ] ;

        unsigned    nearestGridPointIndices[ 4 ]    ; // indices of grid cell containing vorton.
        velGrid.IndicesOfNearestGridPoint( nearestGridPointIndices , vort.mPosition ) ; // Get indices of grid cell containing vorton.
        Vec3        nearestGridPointPosition        ;
        velGrid.PositionFromIndices( nearestGridPointPosition , nearestGridPointIndices ) ;
        const Vec3  displacementToNearestGridPoint  = nearestGridPointPosition - vort.mPosition ;
        const float distToNearestGridPoint          = displacementToNearestGridPoint.Magnitude() ;

        void * font = GLUT_BITMAP_HELVETICA_10 ;
        Vec4   color( 0.2f , 1.0f , 0.2f , 0.25f ) ;

        // Make opacity depend on distance to gridpoint.
        //color.w = 0.25f * MIN2( vort.GetRadius() / distToNearestGridPoint , 1.0f ) ;

        if( distToNearestGridPoint < vort.GetRadius() )
        {   // Gridpoint is inside vorton.
            font  = GLUT_BITMAP_HELVETICA_12 ;
            color = Vec4( 1.0f , 0.2f , 0.2f , 0.5f ) ;
        }

        // Modulate opacity based on distance to camera target.
        color.w *= CameraFocusEmphasis( vort.mPosition ) ;

        if( color.w > 0.01f )
        {   // Diagnostic information is opaque enough to render.
            // Render diagnostic information about the vorton.
        #if 1
            oglRenderStringWorld( vort.mPosition , font , color , "%i w=%g s=%g d=%.2f:%.2f,%s"
                , iVort
                , vort.GetVorticity().Magnitude()
                , vort.mVelocity.Magnitude()
                , distToNearestGridPoint , distToNearestGridPoint / vort.GetRadius() , distToNearestGridPoint < vort.GetRadius() ? "IN" : "out"
                ) ;
        #else
            oglRenderStringWorld( vort.mPosition , font , color , "%i p=%g,%g,%g v=%g,%g,%g d=%g:%g %s"
                , iVort
                , vort.mPosition.x , vort.mPosition.y , vort.mPosition.z
                , vort.mVelocity.x , vort.mVelocity.y , vort.mVelocity.z
                , distToNearestGridPoint , distToNearestGridPoint / vort.GetRadius()
                , distToNearestGridPoint < vort.GetRadius() ? "IN" : "out"
                ) ;
        #endif

            // Draw a line from the vorton to the nearest gridpoint.
            glBegin( GL_LINES ) ;
            glColor4f( 1.0f , 0.0f , 1.0f , color.w ) ;
            glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition           ) ) ;
            glVertex3fv( reinterpret_cast< const GLfloat * >( & nearestGridPointPosition ) ) ;
            glEnd() ;

            {
                // Draw a line indicating vorticity direction.
                Vec3 vortDirEnd = vort.mPosition + vort.GetVorticity().GetDir() * vort.GetRadius() ;
                glColor4f( 1.0f , 1.0f , 0.0f , color.w ) ;
                glBegin( GL_LINES ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vortDirEnd      ) ) ;
                glEnd() ;
            }

            {
                // Draw a line indicating velocity direction.
                Vec3 velDirEnd = vort.mPosition + vort.mVelocity.GetDir() * vort.GetRadius() ;
                glBegin( GL_LINES ) ;
                glColor4f( 0.0f , 1.0f , 1.0f , color.w ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & velDirEnd       ) ) ;
                glEnd() ;
            }
        }
    }
}




/** Render vortex particles.
*/
void InteSiVis::RenderVortons()
{

    QUERY_PERFORMANCE_ENTER ;

    VortonSim &         rVortonSim  = mPclOpVortonSim.mVortonSim ;
    Vector< Vorton > &  vortons     = * rVortonSim.GetVortons() ;

    //  // Modern STL supports vortons.data() but not older versions like MSVS7.
    mVortonRenderer.SetParticleData( reinterpret_cast< char * >( & vortons.operator[]( 0 ) ) ) ; // Modern STL supports vortons.data() but not older versions like MSVS7.
    mVortonRenderer.Render( mTimeNow , mTimeStep , vortons.Size() ) ;

    if( mDiagnosticText == DIAG_TEXT_FULL )
    {
        RenderVortonDiagnostics() ;
    }

    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderVortons ) ;
}




/** Render tracer particles.
*/
void InteSiVis::RenderTracers()
{
#if ENABLE_FIRE

    if( ( TRACER_RENDER_ALL == mTracerRendering ) || ( TRACER_RENDER_FUEL == mTracerRendering ) )
    {
        // Render tracers as fuel:
        QUERY_PERFORMANCE_ENTER ;
        mHeavyLightRenderer.SetParticleData( (char*) & mTracers[0] ) ;
        mHeavyLightRenderer.Render( mTimeNow , mTimeStep , mTracers.Size() ) ;
        QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
    }

    if( ( TRACER_RENDER_ALL == mTracerRendering ) || ( TRACER_RENDER_SMOKE == mTracerRendering ) )
    {
        // Render tracers as smoke:
        QUERY_PERFORMANCE_ENTER ;
        mSmokeRenderer.SetParticleData( (char*) & mTracers[0] ) ;
        mSmokeRenderer.Render( mTimeNow , mTimeStep , mTracers.Size() ) ;
        QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
    }

    if( ( TRACER_RENDER_ALL == mTracerRendering ) || ( TRACER_RENDER_FLAME == mTracerRendering ) )
    {
        // Render tracers as flame:
        QUERY_PERFORMANCE_ENTER ;
        mFlameRenderer.SetParticleData( (char*) & mTracers[0] ) ;
        mFlameRenderer.Render( mTimeNow , mTimeStep , mTracers.Size() ) ;
        QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
    }

#else
    // Render tracers with texture that varies according to buoyancy:
    QUERY_PERFORMANCE_ENTER ;
    mHeavyLightRenderer.SetParticleData( (char*) & mTracers[0] ) ;
    mHeavyLightRenderer.Render( mTimeNow , mTimeStep , mTracers.Size() ) ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
#endif
}




/** Render particles.
*/
void InteSiVis::RenderParticles()
{
#if ! USE_ORIENTED_SORTED_PARTICLES
    QUERY_PERFORMANCE_ENTER ;
//    switch( mScenario )
//    {
//        case 0: case 1:
//            const Vec3      vCoM        = Particles::ComputeGeometricCenter( mTracers ) ;
//            const Vec3 &    vMin        = mPclOpVortonSim.mVortonSim.GetVelocityGrid().GetMinCorner() ;
//            const Vec3 &    vExtent     = mPclOpVortonSim.mVortonSim.GetVelocityGrid().GetExtent() ;
//            const Vec3      vMax        = vMin + vExtent ;
//            const Vec3      vFireball   = Vec3( 0.5f * ( vCoM.x + vMax.x ) , vCoM.y , vCoM.z ) ;
//            SetGlow( vFireball ) ;
//        break ;
//    }
    QdLight::DisableLights() ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_DisableLighitngForParticles ) ;
#endif

    if( mGridDecorations != GRID_DECO_NONE )
    {   // Grid decorations are enabled.
        QUERY_PERFORMANCE_ENTER ;
        const bool bDrawCells            = GRID_DECO_CELLS == mGridDecorations ; 
        const bool bRenderDiagnosticText = DIAG_TEXT_FULL  == mDiagnosticText  ;
        // Render bounding box -- intentionally before all opaque objects, with depth write disabled, so grid is behind everything else.
        DrawGrid( mPclOpVortonSim.mVortonSim.GetVelocityGrid() , mBoundingBoxMaterial , bDrawCells , bRenderDiagnosticText ) ;
        QUERY_PERFORMANCE_EXIT( InteSiVis_Render_BoundingBox ) ;
    }

    if( mRenderVortons )
    {   // Vorton rendering is enabled.
        RenderVortons() ;
    }

    if( mTracerRendering != TRACER_RENDER_NONE )
    {   // Tracer rendering is enabled.
        RenderTracers() ;
    }
}



static void RenderDiagnosticIntegralsText( float yPos , const char * name , const VortonSim::Integrals & integralsBefore , const VortonSim::Integrals & integralsAfter )
{
    const float delta = integralsBefore.ComputeMaxRelativeDifference( integralsAfter ) ;
    oglRenderString( Vec3( 10.0f , yPos , 0.0f ) , GLUT_BITMAP_8_BY_13 , "%s %6.1f I0={% 8.1f,% 8.1f,% 8.1f} I1={% 8.1f,% 8.1f,% 8.1f},{% 8.1f,% 8.1f,% 8.1f} I2={% 8.1f,% 8.1f,% 8.1f}"
        , name
        , delta
        , integralsAfter.mTotalCirculation.x            , integralsAfter.mTotalCirculation.y            , integralsAfter.mTotalCirculation.z
        , integralsAfter.mLinearImpulseFromVorticity.x  , integralsAfter.mLinearImpulseFromVorticity.y  , integralsAfter.mLinearImpulseFromVorticity.z
        , integralsAfter.mLinearImpulseFromVelocity.x   , integralsAfter.mLinearImpulseFromVelocity.y   , integralsAfter.mLinearImpulseFromVelocity.z
        , integralsAfter.mAngularImpulse.x              , integralsAfter.mAngularImpulse.y              , integralsAfter.mAngularImpulse.z              ) ;
}




/** Render summary diagnostic text.
*/
void InteSiVis::RenderSummaryDiagnosticText()
{
    QUERY_PERFORMANCE_ENTER ;

    if( mDiagnosticText != DIAG_TEXT_NONE )
    {   // Diagnostic text is enabled.
        // Render frame counter as text on screen.
        oglRenderString( Vec3( 10.0f , 10.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "frame=%5i time=%5.2g" , mFrame , mTimeNow ) ;
        // Render vorticity integrals.
        {
            const VortonSim::DiagnosticIntegrals & di = mPclOpVortonSim.mVortonSim.GetDiagnosticIntegrals() ;
            oglRenderString( Vec3( 10.0f , 20.0f , 0.0f ) , GLUT_BITMAP_8_BY_13 , "init        I0={% 8.1f,% 8.1f,% 8.1f} I1={% 8.1f,% 8.1f,% 8.1f},{% 8.1f,% 8.1f,% 8.1f} I2={% 8.1f,% 8.1f,% 8.1f}"
                , di.mInitial.mTotalCirculation.x                   , di.mInitial.mTotalCirculation.y                   , di.mInitial.mTotalCirculation.z
                , di.mInitial.mLinearImpulseFromVorticity.x         , di.mInitial.mLinearImpulseFromVorticity.y         , di.mInitial.mLinearImpulseFromVorticity.z
                , di.mInitial.mLinearImpulseFromVelocity.x          , di.mInitial.mLinearImpulseFromVelocity.y          , di.mInitial.mLinearImpulseFromVelocity.z
                , di.mInitial.mAngularImpulse.x                     , di.mInitial.mAngularImpulse.y                     , di.mInitial.mAngularImpulse.z              ) ;
            RenderDiagnosticIntegralsText(  30.0f , "befr" , di.mInitial         , di.mBefore           ) ;
            RenderDiagnosticIntegralsText(  40.0f , "advc" , di.mBefore          , di.mAfterAdvect      ) ;
            RenderDiagnosticIntegralsText(  50.0f , "rgrd" , di.mAfterAdvect     , di.mAfterRegrid      ) ;
            RenderDiagnosticIntegralsText(  60.0f , "velg" , di.mAfterRegrid     , di.mAfterVelGrid     ) ;
            RenderDiagnosticIntegralsText(  70.0f , "stre" , di.mAfterVelGrid    , di.mAfterStretch     ) ;
            RenderDiagnosticIntegralsText(  80.0f , "baro" , di.mAfterStretch    , di.mAfterBaroclinic  ) ;
            RenderDiagnosticIntegralsText(  90.0f , "difs" , di.mAfterBaroclinic , di.mAfterDiffuse     ) ;
            RenderDiagnosticIntegralsText( 100.0f , "heat" , di.mAfterDiffuse    , di.mAfterHeat        ) ;
        }
        // Render vorticity statistics.
        {
            float vorticityMin , vorticityMax , vorticityMean , vorticityStdDev ;
            mPclOpVortonSim.mVortonSim.GatherVorticityStats( vorticityMin , vorticityMax , vorticityMean , vorticityStdDev ) ;
            oglRenderString( Vec3( 10.0f , 110.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "omega:[%.0f,%.0f]<%.0f>+-%-.0f #=%i" , vorticityMin , vorticityMax , vorticityMean , vorticityStdDev , mPclOpVortonSim.mVortonSim.GetVortons()->size() ) ;
        }
        // Render fluid temperature statistics.
        {
            float temperatureMin , temperatureMax , temperatureMean , temperatureStdDev ;
            mPclOpVortonSim.mVortonSim.GatherTemperatureStats( temperatureMin , temperatureMax , temperatureMean , temperatureStdDev ) ;
            oglRenderString( Vec3( 10.0f , 120.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "T:[%.0f,%.0f]<%.0f>+-%-.0f" , temperatureMin , temperatureMax , temperatureMean , temperatureStdDev ) ;
        }
        // Render rigid body temperatures.
        for( size_t iSphere = 0 ; iSphere < GetSpheres().size() ; ++ iSphere )
        {   // For each sphere...
            // Print its temperature.
            const float yPos = 130.0f + float( 10 * iSphere ) ;
            oglRenderString( Vec3( 10.0f , yPos , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "ball[%i].T=%g" , iSphere , GetSpheres()[ iSphere ].GetTemperature() ) ;
        }
    }
    else
    {
    }

    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_SummaryDiagnosticText ) ;
}




/** Gather and record performance profile data.
*/
void InteSiVis::GatherAndRecordProfileData()
{
#if defined( PROFILE )
    VortonSim & rVortonSim = mPclOpVortonSim.mVortonSim ;

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
        extern unsigned gNumVortonBodyHits , gNumTracerBodyHits ;
        const float tracerHitsPerFrame = (float) gNumTracerBodyHits / (float) ( mFrame - 1 ) ;
        const float vortonHitsPerFrame = (float) gNumVortonBodyHits / (float) ( mFrame - 1 ) ;
        fprintf( stderr , "Particles-body hits, average per frame: %g tracers    %g vortons\n"
            , tracerHitsPerFrame , vortonHitsPerFrame ) ;
        fprintf( stderr , "Benchmark DONE: " __DATE__ " " __TIME__ "\n\n" ) ;

        exit( 0 ) ;
    }
#endif
}




/** Update particle system.
*/
void InteSiVis::UpdateParticleSystem()
{

    QUERY_PERFORMANCE_ENTER ;

    // $$HACK CODE START$$ Kludgy way to make all particles the same size.
    if( ! sInstance->mPclOpVortonSim.mVortonSim.GetVortons()->Empty() )
    {   // Make all vortons have the same size and mass.
        //      This is likely not correct since it does not yield uniform
        //      coverage of vorticity, since that depends also on emit rate and
        //      flow velocity, which this ignores.
        sInstance->mPclOpEmitVortons.mTemplate.mSize                = sInstance->mPclOpVortonSim.mVortonSim.GetVortons()->operator[](0).mSize               ;
        sInstance->mPclOpEmitVortons.mTemplate.mDensityDeviation    = sInstance->mPclOpVortonSim.mVortonSim.GetVortons()->operator[](0).mDensityDeviation   ;
    }

    if( ! sInstance->mTracers.Empty() )
    {   // Make all tracers have the same size and mass.
        sInstance->mPclOpEmitTracers.mTemplate.mSize                = sInstance->mTracers[0].mSize              ;
        sInstance->mPclOpEmitTracers.mTemplate.mDensityDeviation    = sInstance->mTracers[0].mDensityDeviation  ;
    #if ENABLE_FIRE
        sInstance->mPclOpEmitTracers.mTemplate.mFuelFraction     = sInstance->mTracers[0].mFuelFraction ;
        sInstance->mPclOpEmitTracers.mTemplate.mFlameFraction    = sInstance->mTracers[0].mFlameFraction ;
        sInstance->mPclOpEmitTracers.mTemplate.mSmokeFraction    = sInstance->mTracers[0].mSmokeFraction ;
    #endif
    }
    // $$HACK CODE END$$

    if( sInstance->mTimeStepping == PLAY || sInstance->mTimeStepping == SINGLE_STEP )
    {
        sInstance->mParticleSystem.Update( sInstance->mTimeStep , sInstance->mFrame ) ;
    }
    QUERY_PERFORMANCE_EXIT( InteSiVis_ParticleSystem_Update ) ;
}




/** Update rigid bodies.
*/
void InteSiVis::UpdateRigidBodies()
{
    // Update rigid bodies.
    QUERY_PERFORMANCE_ENTER ;
    if( sInstance->mTimeStepping == PLAY || sInstance->mTimeStepping == SINGLE_STEP )
    {
        RigidBody::UpdateSystem( (const Vector< RigidBody * > &) sInstance->GetRigidBodies() , sInstance->mTimeStep , sInstance->mFrame ) ;
    }
    QUERY_PERFORMANCE_EXIT( InteSiVis_RigidBody_UpdateSystem ) ;
}




/** Function that GLUT calls to display contents of window.
*/
/* static */ void InteSiVis::GlutDisplayCallback(void)
{
    QUERY_PERFORMANCE_ENTER ;

    if( ! sInstance->mInitialized )
    {   // This is the first time this app has displayed anything.
        sInstance->InitializeRendering() ;
    }

    sInstance->mCamera.SetCamera() ;
    sInstance->RenderSky() ;
    sInstance->RenderRigidBodies() ;
    sInstance->RenderParticles() ;
    sInstance->RenderSummaryDiagnosticText() ;

    QUERY_PERFORMANCE_ENTER ;
    glutSwapBuffers() ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_SwapBuffers ) ;

#if PROFILE > 1
    sInstance->GatherAndRecordProfileData() ;
#endif

    QUERY_PERFORMANCE_EXIT( InteSiVis_Render ) ;
}




/** Function that GLUT calls when nothing else is happening.
*/
/* static */ void InteSiVis::GlutIdleCallback(void)
{
    QUERY_PERFORMANCE_ENTER ;
#if USE_TBB
    tbb::tick_count time0 = tbb::tick_count::now() ;
#endif

    sInstance->UpdateParticleSystem() ;

#if USE_TBB
    tbb::tick_count timeFinal = tbb::tick_count::now() ;
    //fprintf( stderr , " tbb duration=%g second\n" , (timeFinal - time0).seconds() ) ;
#endif

    sInstance->mCamera.Update() ;

    InteSiVis::GlutDisplayCallback() ;

    sInstance->UpdateRigidBodies() ;

    QUERY_PERFORMANCE_EXIT( InteSiVis_UPDATE_Michael_J_Gourlay_2009 ) ;

    if( sInstance->mTimeStepping == PLAY || sInstance->mTimeStepping == SINGLE_STEP )
    {   // Step time by 1.
        if( sInstance->mTimeStep > 0.0f )
        {   // Step time forward.
            ++ sInstance->mFrame ;
        }
        else if( sInstance->mTimeStep < 0.0f )
        {   // Step time backward.
            -- sInstance->mFrame ;
        }
        sInstance->mTimeNow += sInstance->mTimeStep ;
        if( sInstance->mTimeStepping == SINGLE_STEP )
        {   // Now that simulation single-stepped, return to pause mode.
            sInstance->mTimeStepping = PAUSE ;
        }
    }

#if PROFILE
    sNumTracersSum += sInstance->mTracers.Size() ;
    sNumVortonsSum += sInstance->mPclOpVortonSim.mVortonSim.GetVortons()->Size() ;
#endif
}




/** Function that GLUT calls to handle a special key.
*/
/* static */ void InteSiVis::GlutSpecialKeyHandler( int key , int x , int y )
{
    (void) x ; // Avoid "unreferenced formal parameter" warning.
    (void) y ; // Avoid "unreferenced formal parameter" warning.
    QdCamera &      cam     = sInstance->mCamera ;
    Vec3            vTarget = cam.GetTarget() ;
    const int       modKeys = glutGetModifiers() ;

    if( ! modKeys )
    {   // No modifier keys (Shift, Ctrl or Alt) were pressed.
        switch (key)
        {
            case GLUT_KEY_F1 : sInstance->InitialConditions(  1 ) ; break;
            case GLUT_KEY_F2 : sInstance->InitialConditions(  2 ) ; break;
            case GLUT_KEY_F3 : sInstance->InitialConditions(  3 ) ; break;
            case GLUT_KEY_F4 : sInstance->InitialConditions(  4 ) ; break;
            case GLUT_KEY_F5 : sInstance->InitialConditions( 15 ) ; break;
            case GLUT_KEY_F6 : sInstance->InitialConditions( 16 ) ; break;
            case GLUT_KEY_F7 : sInstance->InitialConditions( 17 ) ; break;
            case GLUT_KEY_F8 : sInstance->InitialConditions( 12 ) ; break;
            case GLUT_KEY_F9 : sInstance->InitialConditions( 13 ) ; break;
            case GLUT_KEY_F11: sInstance->InitialConditions( 14 ) ; break;

            case GLUT_KEY_UP   : sInstance->mTimeStepping   = PLAY  ; break ;
            case GLUT_KEY_DOWN : sInstance->mTimeStepping   = PAUSE ; break ;
            case GLUT_KEY_RIGHT: sInstance->mTimeStep       =  sOneOverThirty ; if( sInstance->mTimeStepping == PAUSE ) sInstance->mTimeStepping = SINGLE_STEP ; break ;
            case GLUT_KEY_LEFT : sInstance->mTimeStep       = -sOneOverThirty ; if( sInstance->mTimeStepping == PAUSE ) sInstance->mTimeStepping = SINGLE_STEP ; break ;

            default:
            return;
            break;
        }
    }

    glutPostRedisplay() ;
}




/** Function that GLUT calls to handle a regular key.
*/
/* static */ void InteSiVis::GlutKeyboardHandler( unsigned char key , int x , int y )
{
    (void) x ; // Avoid "unreferenced formal parameter" warning.
    (void) y ; // Avoid "unreferenced formal parameter" warning.
    QdCamera & cam = sInstance->mCamera ;
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

        case 'd':   // Toggle diagnostic text.
        case 'D':
        {
            DiagnosticTextE & diag = sInstance->mDiagnosticText;
            switch( diag )
            {
                case DIAG_TEXT_NONE   :
                    diag = DIAG_TEXT_SUMMARY ;
                    sInstance->mPclOpVortonSim.mVortonSim.SetTallyDiagnosticIntegrals( true ) ;
                    break ;
                case DIAG_TEXT_SUMMARY:
                    diag = DIAG_TEXT_FULL    ;
                    sInstance->mPclOpVortonSim.mVortonSim.SetTallyDiagnosticIntegrals( true ) ;
                    break ;
                case DIAG_TEXT_FULL   :
                    diag = DIAG_TEXT_NONE    ;
                    sInstance->mPclOpVortonSim.mVortonSim.SetTallyDiagnosticIntegrals( false ) ;
                    break ;
            }
        }
        break ;

        case 'g':   // Cycle through grid decoration options.
        case 'G':
        {
            GridDecorationsE & deco = sInstance->mGridDecorations ;
            switch( deco )
            {
            case GRID_DECO_NONE         : deco = GRID_DECO_BOUNDING_BOX ; break ;
            case GRID_DECO_BOUNDING_BOX : deco = GRID_DECO_CELLS        ; break ;
            case GRID_DECO_CELLS        : deco = GRID_DECO_NONE         ; break ;
            }
        }
        break ;

        case 't':   // Cycle throgh tracer rendering options.
        case 'T':
        {
            TracerRenderingE & tracerRendering = sInstance->mTracerRendering ;
            switch( tracerRendering )
            {
            case TRACER_RENDER_NONE : tracerRendering = TRACER_RENDER_FUEL  ; break ;
            case TRACER_RENDER_FUEL : tracerRendering = TRACER_RENDER_FLAME ; break ;
            case TRACER_RENDER_FLAME: tracerRendering = TRACER_RENDER_SMOKE ; break ;
            case TRACER_RENDER_SMOKE: tracerRendering = TRACER_RENDER_ALL   ; break ;
            case TRACER_RENDER_ALL  : tracerRendering = TRACER_RENDER_NONE  ; break ;
            }
        }
        break ;

        case 'v':   // Toggle vorton rendering.
        case 'V':
        {
            bool & renderVortons = sInstance->mRenderVortons ;
            renderVortons = ! renderVortons ;
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




/** Function that GLUT calls to handle mouse button events.
*/
/* static */ void InteSiVis::GlutMouseHandler( int button , int state , int x , int y )
{
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
    if( -999 == sMousePrevX )
    {
        sMousePrevX = x ;
        sMousePrevY = y ;
        return ;
    }

    static float fMouseMotionSensitivity = 0.005f ;    // Tune based on how fast you want mouse to move camera.

    const float dx = CLAMP( fMouseMotionSensitivity * float( x - sMousePrevX ) , -100.0f , 100.0f ) ;
    const float dy = CLAMP( fMouseMotionSensitivity * float( y - sMousePrevY ) , -100.0f , 100.0f ) ;

    if( sInstance->mMouseButtons[0] )
    {   // User is left-click-dragging mouse
        QdCamera & cam = sInstance->mCamera ;
        float azimuth , elevation , radius ;
        // Obtain previous camera parameters.
        cam.GetOrbit( azimuth , elevation , radius ) ;
        // Avoid gimbal lock by limiting elevation angle to avoid the poles.
        static const float sAvoidPoles = 0.001f ;
        elevation = CLAMP( elevation - dy , sAvoidPoles , PI * ( 1.0f - sAvoidPoles ) ) ;
        // Set new camera parameters based on how much mouse moved.
        cam.SetOrbit( azimuth - dx , elevation , radius ) ;
    }
    else if( sInstance->mMouseButtons[2] )
    {   // User is right-click-dragging mouse
        QdCamera & cam = sInstance->mCamera ;
        float azimuth , elevation , radius ;
        // Obtain previous camera parameters.
        cam.GetOrbit( azimuth , elevation , radius ) ;
        // Set new camera parameters based on how much mouse moved.
        const float newRadius = MAX2( radius - ( dx + dy ) , 1.0e-4f ) ;
        cam.SetOrbit( azimuth , elevation , newRadius ) ;
    }
    else if( sInstance->mMouseButtons[1] )
    {   // User is middle-click-dragging mouse
        QdCamera & cam = sInstance->mCamera ;
        const Vec3 & rEye    = cam.GetEye() ;
        const Vec3 & rTarget = cam.GetTarget() ;
        // Extract world space direction vectors associated with view (used to compute camera-facing coordinates).
        // Note that these vectors are the unit vectors of the inverse of the view matrix.
        // They are the world-space unit vectors of the view transformation.
        Mat4 viewMatrix ;
        glGetFloatv( GL_MODELVIEW_MATRIX , (GLfloat *) & viewMatrix ) ;
        Vec3 viewRight  ( viewMatrix.m[0][0] , viewMatrix.m[1][0] , viewMatrix.m[2][0] ) ;
        Vec3 viewUp     ( viewMatrix.m[0][1] , viewMatrix.m[1][1] , viewMatrix.m[2][1] ) ;
        Vec3 viewForward( viewMatrix.m[0][2] , viewMatrix.m[1][2] , viewMatrix.m[2][2] ) ;
        Vec3 delta = 2.0f * ( - viewRight * dx + viewUp * dy ) ;
        // This ought to use the view to change position.
        cam.SetEye   ( Vec3( rEye.x    + delta.x , rEye.y    + delta.y , rEye.z    + delta.z ) ) ;
        cam.SetTarget( Vec3( rTarget.x + delta.x , rTarget.y + delta.y , rTarget.z + delta.z ) ) ;
    }

    sMousePrevX = x ;
    sMousePrevY = y ;
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
static void CALLBACK DebugCallback(unsigned int source, unsigned int type, unsigned int id, unsigned int severity,  int length, const char* message, void* userParam )
{
}




/** Initialize display device.
*/
void InteSiVis::InitDevice( int * pArgc , char ** argv )
{
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
    mRenderWindow = glutCreateWindow( "(C) 2009 Michael J. Gourlay: InteSiVis Render Window" ) ;

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

    glutMainLoop() ; // Relinquish control to GLUT.  This never returns.
}




/// Application entry point.
int main( int argc , char ** argv )
{
    fprintf( stderr , "Benchmark: " __DATE__ " " __TIME__ "\n" ) ;

    XMM_Set_FlushToZero_DenormalAreZero() ;
    Setx87Precision( PRECISION_SINGLE ) ;
    //Unmaskx87FpExceptions() ;

#if PROFILE
    {   // Redirect stderr to a file to collect profile data.
        freopen( "profile.log" , "a" , stderr ) ;
        fprintf( stderr , "Benchmark: " __DATE__ " " __TIME__ "\n" ) ;
    }
#endif

    static const float viscosity = 0.01f ;
    static const float ambientFluidDensity  = 1.0f ;
    static const float fluidSpecificHeatCapacity = 1.0f ;
    InteSiVis inteSiVis( viscosity , ambientFluidDensity , fluidSpecificHeatCapacity ) ;
    inteSiVis.InitDevice( & argc , argv ) ;
    return 0 ;
}
