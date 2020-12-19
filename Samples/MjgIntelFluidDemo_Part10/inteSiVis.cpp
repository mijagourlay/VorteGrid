/*! \file inteSiVis.cpp

    \brief Application for interactive simulation and visualization

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.
*/

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

static InteSiVis * sInstance = 0 ;
static const float  timeStep    = 1.0f / 30.0f ;
static const Vec3 sGravityDirection( 0.0f , 0.0f , -1.0f ) ; ///< Direction of acceleration due to gravity
static const Vec3 sGravityAcceleration( 10.0f * sGravityDirection ) ; ///< Acceleration due to gravity

static const Vorton vortonDummy ;
static const size_t vortonStride            = sizeof( Vorton ) ;
static const size_t vortonOffsetToAngVel    = OFFSET_OF_MEMBER( vortonDummy , mAngularVelocity ) ;
static const size_t vortonOffsetToSize      = OFFSET_OF_MEMBER( vortonDummy , mSize ) ;
static const size_t vortonOffsetToDensity   = OFFSET_OF_MEMBER( vortonDummy , mDensity ) ;

static const Particle tracerDummy ;
static const size_t tracerStride            = sizeof( Particle ) ;
static const size_t tracerOffsetToAngVel    = OFFSET_OF_MEMBER( tracerDummy , mAngularVelocity ) ;
static const size_t tracerOffsetToSize      = OFFSET_OF_MEMBER( tracerDummy , mSize ) ;
static const size_t tracerOffsetToDensity   = OFFSET_OF_MEMBER( tracerDummy , mDensity ) ;

static int sMousePrevX = -999 ;
static int sMousePrevY = -999 ;

static const float gScale = 1.0f ;

bool gPrintProfileData = false ;

#if defined( PROFILE )
static const int sFrameCountMax = 60   ;   // In profile builds, run for a fixed number of frames
static size_t    sNumTracersSum = 0     ;   // Sum of current number of tracers
static size_t    sNumVortonsSum = 0     ;   // Sum of current number of vortons
#endif

static Vec4 sColorRamp[] =
{
    Vec4( 0.0f , 0.0f , 1.0f , 0.005f ) ,
    Vec4( 0.0f , 1.0f , 1.0f , 0.01f  ) ,
    Vec4( 0.0f , 1.0f , 0.0f , 0.02f  ) ,
    Vec4( 1.0f , 0.5f , 0.0f , 0.05f  ) ,
    Vec4( 1.0f , 0.0f , 0.0f , 0.1f   ) ,
    Vec4( 1.0f , 0.0f , 1.0f , 0.2f   )
} ;
static const int NUM_COLORS_IN_RAMP = sizeof( sColorRamp ) / sizeof( sColorRamp[0] ) ;

// Functions --------------------------------------------------------------

/*! \brief Given a color ramp and a value in [0,1], compute and return a blended color

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




/*! \brief Construct application for interactive simulation and visualization

    \param viscosity - fluid viscosity

    \param ambientFluidDensity - ambient fluid density.  This is the density of the fluid
                in the absence of particles.  Particles have a density, and when particles
                are present then they can modify this ambient density.

*/
InteSiVis::InteSiVis( float viscosity , float ambientFluidDensity )
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
    , mVortonRenderer( 0 , vortonStride , vortonOffsetToAngVel , vortonOffsetToSize , vortonOffsetToDensity )
    , mTracerRenderer( 0 , tracerStride , tracerOffsetToAngVel , tracerOffsetToSize , tracerOffsetToDensity )
    , mRenderWindow( 0 )
    , mStatusWindow( 0 )
    , mFrame( 0 )
    , mTimeNow( 0.0 )
    , mInitialized( false )
    , mDecorations( DECO_NONE )
{
    sInstance = this ;


    mMouseButtons[0] = mMouseButtons[1] = mMouseButtons[2] = 0 ;

    mLights[0].mPosition = Vec3( 0.7f , -0.2f , 0.7f ) ; // actually a direction
    mLights[0].mColor    = Vec3( 0.9f , 0.9f , 0.9f ) ;
    mLights[0].mType     = QdLight::LT_DIRECTIONAL ;

    mLights[1].mPosition = Vec3( -0.7f , -0.2f , 0.7f ) ; // actually a direction
    mLights[1].mColor    = Vec3( 0.9f , 0.9f , 0.9f ) ;
    mLights[1].mType     = QdLight::LT_DIRECTIONAL ;

    {
        static const int    killAgeMax  = 90    ;
        static const float  windGain    = 1.0f  ;

        // Set up particle group for vortons.
        {

            mPclOpKillAgeVortons.mAgeMax    = killAgeMax ;
            mParticleGroupVortons.mParticleOps.PushBack( & mPclOpKillAgeVortons ) ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition   = Vec3( -0.5f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity   = Vec3( 2.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize       = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensity    = 0.0f ;
            emitterSpread.mPosition     = Vec3( emitterTemplate.mVelocity.x / 30.0f , 0.5f , 0.5f ) ;

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
            mPclOpFluidBodInteVortons.mVelocityGrid         = & mPclOpVortonSim.mVortonSim.GetVelocityGrid() ;
            mPclOpFluidBodInteVortons.mDensityDeviationGrid = 0 ;
            mPclOpFluidBodInteVortons.mRigidBodies          = (Vector< RigidBody * > *) & GetRigidBodies() ;
            mPclOpFluidBodInteVortons.mAmbientFluidDensity  = ambientFluidDensity ;
            mPclOpFluidBodInteVortons.mGravityAcceleration  = sGravityAcceleration ;
            mPclOpFluidBodInteVortons.mRespectAngVel        = true ;
            mParticleGroupVortons.mParticleOps.PushBack( & mPclOpFluidBodInteVortons ) ;
        }

        // Set up particle group for passive tracers.
        {
            mPclOpKillAgeTracers.mAgeMax    = killAgeMax ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpKillAgeTracers ) ;

            mPclOpWindTracers.mWind        = mPclOpWindVortons.mWind ;
            mPclOpWindTracers.mGain        = windGain ;
            mParticleGroupTracers.mParticleOps.PushBack( & mPclOpWindTracers ) ;

            mPclOpAdvectTracers.mDensityGrid    = & mPclOpVortonSim.mVortonSim.GetDensityDeviationGrid() ;
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

    InitialConditions( 15 ) ;

    // Set the camera in motion.
    //mCamera.SetOrbitalTrajectory( Vec3( -0.001f , -0.002f , 0.0f ) ) ;

}




#if USE_FANCY_PARTICLES
/*! \brief Cheap lighting hack to make an internal glow effect inside smoke

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




/*! \brief Construct application for interactive simulation and visualization

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

    static const float          fRadius                     = 1.0f ;
    static const float          fThickness                  = 1.0f ;
    static const float          fMagnitude                  = 20.0f ;
    static const unsigned       numCellsPerDim              = 16 ;
    static const unsigned       numVortonsMax               = numCellsPerDim * numCellsPerDim * numCellsPerDim ;
    unsigned                    numTracersPerCellCubeRoot   = 5 ;
    Vector< Vorton > &          vortons                     = * vortonSim.GetVortons() ;
    static const float          veryLargeMass               = 1.0e16f ; // large but not large enough that 1/mass is a denormal.
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
            mCamera.SetTarget( Vec3( 10.f , 0.f , 0.f ) ) ;
            mCamera.SetEye( Vec3( 10.f , -10.0f , 0.0f ) ) ;
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
            // Note: Denormal values occur when veryLargeMass is around 1.0e34f.
            GetSpheres().PushBack( RbSphere( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            //GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Make sphere spin

            mPclOpKillAgeVortons.mAgeMax            = 100 ;
            mPclOpKillAgeTracers.mAgeMax            = mPclOpKillAgeVortons.mAgeMax ;

            Particle emitterTemplate , emitterSpread ;
            emitterTemplate.mPosition   = Vec3( -0.5f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity   = Vec3( 1.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize       = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensity    = 0.0f ;
            emitterSpread.mPosition     = Vec3( emitterTemplate.mVelocity.x / 30.0f , 0.5f , 0.5f ) ;

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
            emitterTemplate.mPosition   = Vec3( 0.0f , 0.0f , 0.0f ) ;
            emitterTemplate.mVelocity   = Vec3( 0.0f , 0.0f , 0.0f ) ; // Mostly useless when Wind is active
            emitterTemplate.mSize       = 0.05f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
            emitterTemplate.mDensity    = 0.0f ;
            emitterSpread.mPosition     = Vec3( 0.5f , 0.5f , 0.5f ) ;

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
            // Note: Denormal values occur when veryLargeMass is around 1.0e34f.
            GetSpheres().PushBack( RbSphere( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , veryLargeMass , 0.2f ) ) ;
            //GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Make sphere spin

            // Position and orient camera.
            mCamera.SetTarget( Vec3( 0.0f ,  0.0f , 2.0f ) ) ;
            mCamera.SetEye   ( Vec3( 0.0f , -3.0f , 2.0f ) ) ;
        }
        break ;

        default:
        break ;
    }

    // Apply same wind to tracers that vortons experience.
    mPclOpWindTracers.mWind = mPclOpWindVortons.mWind ;

    //const unsigned numTracersPerCell = POW3( numTracersPerCellCubeRoot ) ;

#if USE_FANCY_PARTICLES
    switch( mScenario )
    {   // For some scenarios, place a point light inside the cloud to make it seem to glow from within.
        case 0: // vortex ring -- vorticity in [0,1]
        case 1: // "jet" vortex ring -- velocity in [0,1]
            SetGlow( Particles::ComputeGeometricCenter( mParticleGroupTracers.mParticles ) ) ;
        break ;
        default:
            QdLight::DisableLights() ;
        break ;
    }
#else   // When not using "fancy particles", disable all lighting and shading -- just use so-called "flat" shading.
    QdLight::DisableLights() ;
#endif

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




/*! \brief Destruct application for interactive simulation and visualization
*/
InteSiVis::~InteSiVis()
{
    sInstance = 0 ;
}




/*! \brief Function that GLUT calls when window resizes
*/
/* static */ void InteSiVis::GlutReshapeGlutCallback(int width, int height)
{
    const int   width_4 = (width / 4) * 4;
    sInstance->mCamera.SetViewport( width_4 ,height ) ;
    glViewport( 0 , 0 , sInstance->mCamera.GetWidth() , sInstance->mCamera.GetHeight() ) ;
    glutReshapeWindow( width_4 , height ) ;
}




/*! \brief Draw bounding box and grid cells
    \param grid - grid to draw

    \param material - material to use to draw grid

    \parm bDrawCells - whether to draw all cells (if true) or just bounding box (if false)

    \note This uses glutWireCube, one per cell.
            That unnecessarily doubly-renders each cell edge,
            so this routine could be sped up by at least a factor of 2
            by drawing individual lines instead.

*/
static void DrawGrid( const UniformGrid<Vec3> & grid , const QdMaterial & material , bool bDrawCells )
{
    if( grid.HasZeroExtent() )
    {   // Grid has no size.
        return ;    // There is nothing to draw.
    }

    // Draw bounding box
    material.UseMaterial() ;
    glPushMatrix() ;

    const Vec3 vCenter = grid.GetCenter() ;
    glTranslatef( vCenter.x , vCenter.y , vCenter.z ) ;
    const Vec3 gridExtent = grid.GetExtent() ;
    glScalef( gridExtent.x , gridExtent.y , gridExtent.z ) ;

    glutWireCube( 1.0 ) ;

    glPopMatrix() ;

    // Gather min and max statistics for grid contents (speeds)
    float magMin ;
    float magMax ;
    FindMagnitudeRange( grid , magMin , magMax ) ;
    magMin *= ( 1.0f - FLT_EPSILON ) ;  // Account for diff between fsqrtf and sqrtf
    magMax *= ( 1.0f + FLT_EPSILON ) ;  // Account for diff between fsqrtf and sqrtf
    const float oneOverMagRange = 1.0f / ( magMax - magMin ) ;
    const bool speedRangeIsNonZero = magMax != magMin ;

    const Vec3 cellSize = grid.GetCellSpacing() /* shrink cell to visualize each, otherwise edges overlap */ * 0.95f ;

    if( bDrawCells )
    {
        // Draw each grid cell.
        const unsigned nx    = grid.GetNumCells( 0 ) ;
        const unsigned ny    = grid.GetNumCells( 1 ) ;
        const unsigned nz    = grid.GetNumCells( 2 ) ;
        const unsigned npx   = grid.GetNumPoints( 0 ) ;
        const unsigned npy   = grid.GetNumPoints( 1 ) ;
        //const int npz   = grid.GetNumPoints( 2 ) ;
        const unsigned npxy  = npx * npy ;
        
        for( unsigned iz = 0 ; iz < nz ; ++ iz )
        {
            const int zOffset = iz * npxy ;
            for( unsigned iy = 0 ; iy < ny ; ++ iy )
            {
                const int yzOffset = zOffset + iy * npx ;
                for( unsigned ix = 0 ; ix < nx ; ++ ix )
                {
                    // Note: pushing and popping matrix per cell is probably crazy slow.
                    // This routine could be sped up considerably.
                    glPushMatrix() ;

                    const Vec3 vCenter = grid.GetCellCenter( ix , iy , iz ) ;
                    glTranslatef( vCenter.x , vCenter.y , vCenter.z ) ;
                    glScalef( cellSize.x , cellSize.y , cellSize.z ) ;

                    // Choose color based on speed of gridpoint at minimal corner.
                    // Note: a more proper rendering would use different velocity at each gridpoint.
                    if( speedRangeIsNonZero )
                    {
                        const int xyzOffset = ix + yzOffset ;
                        
                        const Vec3 & vel = grid[ xyzOffset ] ;
                        const float speed = vel.Magnitude() ;
                        const float speed0to1 = ( speed - magMin ) * oneOverMagRange ;
                        Vec4 color = GetColorFromRamp( sColorRamp , NUM_COLORS_IN_RAMP , speed0to1 ) ;
                        glColor4f( color.x , color.y , color.z , color.w ) ;
                    }

                    glutWireCube( 1.0 ) ;

                    glPopMatrix() ;
                }
            }
        }
    }
}




/*! \brief Function that GLUT calls to display contents of window
*/
/* static */ void InteSiVis::GlutDisplayCallback(void)
{
    QUERY_PERFORMANCE_ENTER ;

	static GLuint ballDisplayList ;

    if( ! sInstance->mInitialized )
    {   // This is the first time this app has displayed anything.
        // Cannot generate a texture until OpenGL has been initialized.

        // Light & heavy vorton textures.
        {
            sInstance->mVortonMaterial.AllocateTexture( 16 , 32 , 2 ) ;
            const Vec4 opaqueLight( 0.9f , 0.9f , 0.1f , 1.0f ) ; // yellowish
            const Vec4 opaqueHeavy( 0.1f , 0.9f , 0.9f , 1.0f ) ; // cyanish
            sInstance->mVortonMaterial.AssignTexturePage( QdMaterial::TP_NOISE_BALL , 1.5f , 1 , opaqueLight ) ;
            sInstance->mVortonMaterial.AssignTexturePage( QdMaterial::TP_NOISE_BALL , 1.5f , 0 , opaqueHeavy ) ;
            sInstance->mVortonMaterial.FinalizeTexture() ;
            sInstance->mVortonMaterial.mColor      = Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) ;
            sInstance->mVortonMaterial.mDepthWrite = false ;
        }

        // Light & heavy tracer textures
        {
            sInstance->mTracerMaterial.AllocateTexture( 16 , 32 , 2 ) ;
            const Vec4 opaqueLight( 0.9f , 0.1f , 0.1f , 1.0f ) ; // redish
            const Vec4 opaqueHeavy( 0.1f , 0.1f , 0.9f , 1.0f ) ; // blueish
            sInstance->mTracerMaterial.AssignTexturePage( QdMaterial::TP_NOISE_BALL , 0.5f , 1 , opaqueLight ) ;
            sInstance->mTracerMaterial.AssignTexturePage( QdMaterial::TP_NOISE_BALL , 0.5f , 0 , opaqueHeavy ) ;
            sInstance->mTracerMaterial.FinalizeTexture() ;
            sInstance->mTracerMaterial.mColor      = Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) ;
            sInstance->mTracerMaterial.mDepthWrite = false ;
        }

        sInstance->mBallMaterial.Initialize( 32 , 32 , 1 , QdMaterial::TP_NOISE_OPAQUE , 0.2f ) ;
        sInstance->mBallMaterial.mColor = Vec4( 0.95f , 0.95f , 0.1f , 1.0f ) ;

        sInstance->mBoundingBoxMaterial.Initialize( 32 , 32 , 1 , QdMaterial::TP_NOISE_OPAQUE , 0.0f ) ;
        sInstance->mBoundingBoxMaterial.mColor = Vec4( 0.7f , 0.9f , 0.7f , 0.01f ) ;
        sInstance->mBoundingBoxMaterial.mDepthWrite = false ;

        sInstance->mSkyMaterial.Initialize( 64 , 64 , 1 , QdMaterial::TP_NOISE_OPAQUE , 0.05f ) ;
        sInstance->mSkyMaterial.mColor = Vec4( 0.9f , 0.9f , 0.9f , 1.0f ) ;

	    ballDisplayList = glGenLists( 1 ) ; // Create the id for the display list
	    glNewList( ballDisplayList , GL_COMPILE ) ;
        glTexGeni( GL_S , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
        glTexGeni( GL_T , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
        glEnable( GL_TEXTURE_GEN_S ) ;
        glEnable( GL_TEXTURE_GEN_T ) ;
        glutSolidSphere( 1.0f , 32 , 32 ) ;
        glDisable( GL_TEXTURE_GEN_S ) ;
        glDisable( GL_TEXTURE_GEN_T ) ;
        glEndList() ;

        sInstance->mInitialized = true ;
    }
    sInstance->mCamera.SetCamera() ;

    QUERY_PERFORMANCE_ENTER ;

    sInstance->mSkyMaterial.UseMaterial() ;
    QdLight::DisableLights() ;
    // Render sky ball
    {
        glCullFace(GL_FRONT) ; // Render /inside/ of sphere
        glPushMatrix() ;
        glTranslatef( 0.0f , 0.0f , 0.0f ) ; // should translate to camera eye
        static const float skyRadius = 500.0f ;
        glScalef( skyRadius , skyRadius , skyRadius ) ;
		glCallList( ballDisplayList ) ;
        glPopMatrix() ;
    }
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_NonFluidObjects ) ;

    if( sInstance->mDecorations != DECO_NONE )
    {
        QUERY_PERFORMANCE_ENTER ;
        const bool bDrawCells = DECO_GRID_CELLS == sInstance->mDecorations ; 
        // Render bounding box -- intentionally before all opaque objects, with depth write disabled, so grid is behind everything else.
        DrawGrid( sInstance->mPclOpVortonSim.mVortonSim.GetVelocityGrid() , sInstance->mBoundingBoxMaterial , bDrawCells ) ;
        QUERY_PERFORMANCE_EXIT( InteSiVis_Render_BoundingBox ) ;
    }
    // Render balls
    sInstance->mLights[0].SetLight( 0 ) ;
    sInstance->mLights[1].SetLight( 1 ) ;
    sInstance->mBallMaterial.UseMaterial() ;
    const size_t numSpheres = sInstance->GetSpheres().Size() ;
    for( size_t iSphere = 0 ; iSphere < numSpheres ; ++ iSphere )
    {
        RbSphere & sphere = sInstance->GetSpheres()[ iSphere ] ;
        glPushMatrix() ;
    #if 1
        glTranslatef( sphere.mPosition.x , sphere.mPosition.y , sphere.mPosition.z ) ;
        //if( true )
        {
            const float orientMag = sphere.mOrientation.Magnitude() ;
            if( orientMag >  TWO_PI )
            {   // orientation angle is larger than a full revolution so truncate it.
                const float orientMagNew = orientMag - TWO_PI ;
                sphere.mOrientation *= ( orientMagNew / orientMag ) ;
            }
            else if( orientMag < 0 )
            {   // orientation angle is larger than a full revolution so truncate it.
                const float orientMagNew = orientMag - TWO_PI ;
                sphere.mOrientation *= ( orientMagNew / orientMag ) ;
            }
        }
        const float angle = sphere.mOrientation.Magnitude() ;
        Vec4 rotAxis( sphere.mOrientation ) ;
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
    #else
        Mat4 xLocalToWorld( Mat4_xIdentity ) ;
        xLocalToWorld.SetTranslation( Vec4( sphere.mPosition , 1.0f ) ) ;
        glMultMatrixf( (GLfloat*) & xLocalToWorld ) ;
    #endif
		glCallList( ballDisplayList ) ;
        glPopMatrix() ;
    }

    QUERY_PERFORMANCE_ENTER ;
    QdLight::DisableLights() ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_DisableLights ) ;

#if USE_FANCY_PARTICLES
    QUERY_PERFORMANCE_ENTER ;
    switch( sInstance->mScenario )
    {
        
        case 0: case 1:
            const Vec3      vCoM        = Particles::ComputeGeometricCenter( sInstance->mTracers ) ;
            const Vec3 &    vMin        = sInstance->mPclOpVortonSim.mVortonSim.GetVelocityGrid().GetMinCorner() ;
            const Vec3 &    vExtent     = sInstance->mPclOpVortonSim.mVortonSim.GetVelocityGrid().GetExtent() ;
            const Vec3      vMax        = vMin + vExtent ;
            const Vec3      vFireball   = Vec3( 0.5f * ( vCoM.x + vMax.x ) , vCoM.y , vCoM.z ) ;
            SetGlow( vFireball ) ;
        break ;
    }
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_SetGlow ) ;
#endif

#if ( PROFILE > 1 ) || defined( _DEBUG )
    VortonSim & rVortonSim = sInstance->mPclOpVortonSim.mVortonSim ;
#endif


    // Render tracers:
    QUERY_PERFORMANCE_ENTER ;
    sInstance->mTracerMaterial.UseMaterial() ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_SetParticleMaterial ) ;

    QUERY_PERFORMANCE_ENTER ;
    sInstance->mTracerRenderer.SetParticleData( (char*) & sInstance->mTracers[0] ) ;
    sInstance->mTracerRenderer.Render( sInstance->mTimeNow , timeStep , sInstance->mTracers.Size() ) ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;

    QUERY_PERFORMANCE_ENTER ;
    glutSwapBuffers() ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_SwapBuffers ) ;

#if PROFILE > 1
    if( sInstance->mFrame == sFrameCountMax - 1 )
    {   // Reached last frame, so enable profile reporting
        gPrintProfileData = true ;
    }
    else if( sInstance->mFrame >= sFrameCountMax )
    {   // Reached maximum number of frames, so exit.
        fprintf( stderr , "Ran %i frames\n" , sInstance->mFrame ) ;
        fprintf( stderr , "region: { %g , %g , %g } , { %g , %g , %g } \n"
            , rVortonSim.GetMinCornerEternal().x
            , rVortonSim.GetMinCornerEternal().y
            , rVortonSim.GetMinCornerEternal().z
            , rVortonSim.GetMaxCornerEternal().x
            , rVortonSim.GetMaxCornerEternal().y
            , rVortonSim.GetMaxCornerEternal().z
            ) ;
        const float tracersPerFrame = (float) sNumTracersSum / (float) ( sInstance->mFrame - 1 ) ;
        const float vortonsPerFrame = (float) sNumVortonsSum / (float) ( sInstance->mFrame - 1 ) ;
        fprintf( stderr , "Particles, average per frame: %g tracers    %g vortons\n"
            , tracersPerFrame , vortonsPerFrame ) ;
        extern unsigned gNumVortonBodyHits , gNumTracerBodyHits ;
        const float tracerHitsPerFrame = (float) gNumTracerBodyHits / (float) ( sInstance->mFrame - 1 ) ;
        const float vortonHitsPerFrame = (float) gNumVortonBodyHits / (float) ( sInstance->mFrame - 1 ) ;
        fprintf( stderr , "Particles-body hits, average per frame: %g tracers    %g vortons\n"
            , tracerHitsPerFrame , vortonHitsPerFrame ) ;
        fprintf( stderr , "Benchmark DONE: " __DATE__ " " __TIME__ "\n\n" ) ;

        exit( 0 ) ;
    }
#endif

    QUERY_PERFORMANCE_EXIT( InteSiVis_Render ) ;
}




/*! \brief Function that GLUT calls when nothing else is happening
*/
/* static */ void InteSiVis::GlutIdleCallback(void)
{
#define SYNCHRONIZE_VIRTUAL_AND_WALL_CLOCKS 0
#if SYNCHRONIZE_VIRTUAL_AND_WALL_CLOCKS
    float timeStep ;
    {
        LARGE_INTEGER ticksPerSec ;
        QueryPerformanceFrequency( & ticksPerSec ) ;
        const float secondsPerTick = 1.0f / float( ticksPerSec.QuadPart ) ;
        static LARGE_INTEGER sTimePrevious ;

        LARGE_INTEGER timeNow ;
        QueryPerformanceCounter( & timeNow ) ;
        LARGE_INTEGER liDuration ;
        liDuration.QuadPart = timeNow.QuadPart - sTimePrevious.QuadPart ;
        const float fDuration = secondsPerTick * float( liDuration.QuadPart ) ;

        if( sTimePrevious.QuadPart == 0 )
        {
            timeStep = 1.0f / 60.0f ;
        }
        else
        {
            timeStep = fDuration ;
        }
        sTimePrevious = timeNow ;
    }
#endif

    QUERY_PERFORMANCE_ENTER ;

#if USE_TBB
    tbb::tick_count time0 = tbb::tick_count::now() ;
#endif

    QUERY_PERFORMANCE_ENTER ;

    // $$HACK CODE START$$ Kludgy way to make all particles the same size.
    if( ! sInstance->mPclOpVortonSim.mVortonSim.GetVortons()->Empty() )
    {   // Make all vortons have the same size and mass.
        //      This is likely not correct since it does not yield uniform
        //      coverage of vorticity, since that depends also on emit rate and
        //      flow velocity, which this ignores.
        sInstance->mPclOpEmitVortons.mTemplate.mSize    = sInstance->mPclOpVortonSim.mVortonSim.GetVortons()->operator[](0).mSize    ;
        sInstance->mPclOpEmitVortons.mTemplate.mDensity = sInstance->mPclOpVortonSim.mVortonSim.GetVortons()->operator[](0).mDensity ;
    }

    if( ! sInstance->mTracers.Empty() )
    {   // Make all tracers have the same size and mass.
        sInstance->mPclOpEmitTracers.mTemplate.mSize    = sInstance->mTracers[0].mSize    ;
        sInstance->mPclOpEmitTracers.mTemplate.mDensity = sInstance->mTracers[0].mDensity ;
    }
    // $$HACK CODE END$$


    sInstance->mParticleSystem.Update( timeStep , sInstance->mFrame ) ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_ParticleSystem_Update ) ;

#if USE_TBB
    tbb::tick_count timeFinal = tbb::tick_count::now() ;
    //fprintf( stderr , " tbb duration=%g second\n" , (timeFinal - time0).seconds() ) ;
#endif

    {
        QdCamera & cam = sInstance->mCamera ;
        cam.Update() ;
    }

    InteSiVis::GlutDisplayCallback() ;

    // Update rigid bodies.
    QUERY_PERFORMANCE_ENTER ;
    RigidBody::UpdateSystem( (const Vector< RigidBody * > &) sInstance->GetRigidBodies() , timeStep , sInstance->mFrame ) ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_RigidBody_UpdateSystem ) ;

    QUERY_PERFORMANCE_EXIT( InteSiVis_UPDATE_Michael_J_Gourlay_2009 ) ;

    ++ sInstance->mFrame ;
    sInstance->mTimeNow += timeStep ;

#if PROFILE
    sNumTracersSum += sInstance->mTracers.Size() ;
    sNumVortonsSum += sInstance->mPclOpVortonSim.mVortonSim.GetVortons()->Size() ;
#endif
}




/*! \brief Function that GLUT calls to handle a special key
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
            case GLUT_KEY_F5 : sInstance->InitialConditions(  5 ) ; break;
            case GLUT_KEY_F6 : sInstance->InitialConditions(  6 ) ; break;
            case GLUT_KEY_F7 : sInstance->InitialConditions( 15 ) ; break;
            case GLUT_KEY_F8 : sInstance->InitialConditions( 12 ) ; break;
            case GLUT_KEY_F9 : sInstance->InitialConditions( 13 ) ; break;
            case GLUT_KEY_F11: sInstance->InitialConditions( 14 ) ; break;

            case GLUT_KEY_UP   : vTarget.z += 0.1f ; cam.SetTarget( vTarget ) ; break ;
            case GLUT_KEY_DOWN : vTarget.z -= 0.1f ; cam.SetTarget( vTarget ) ; break ;
            case GLUT_KEY_RIGHT: vTarget.x -= 1.0f ; cam.SetTarget( vTarget ) ; break ;
            case GLUT_KEY_LEFT : vTarget.x += 1.0f ; cam.SetTarget( vTarget ) ; break ;

            default:
            return;
            break;
        }
    }

    glutPostRedisplay() ;
}




/*! \brief Function that GLUT calls to handle a regular key
*/
/* static */ void InteSiVis::GlutKeyboardHandler (unsigned char key, int x, int y)
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

        case '.': radius -= 0.1f ; break ;  // Move camera in
        case ',': radius += 0.1f ; break ;  // Move camera out

        case 'c':   // Toggle camera rotation
        case 'C':
            {
                animateCamera = ! animateCamera ;
                if( animateCamera )
                {   // Camera animation is now on.
                    if( 'c' == key )
                    {   // Rotate azimuth faster
                        cam.SetOrbitalTrajectory( Vec3( -0.005f , -0.00f , 0.0f ) ) ;
                    }
                    else
                    {   // Rotate elevation faster
                        cam.SetOrbitalTrajectory( Vec3( -0.002f , -0.005f , 0.0f ) ) ;
                    }
                }
                else
                {
                    cam.SetOrbitalTrajectory( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
                }
            }
            break ;

        case 'g':   // Cycle through grid decorations
        case 'G':
            {
                DecorationsE & deco = sInstance->mDecorations ;
                switch( deco )
                {
                case DECO_NONE          : deco = DECO_BOUNDING_BOX  ; break ;
                case DECO_BOUNDING_BOX  : deco = DECO_GRID_CELLS    ; break ;
                case DECO_GRID_CELLS    : deco = DECO_NONE          ; break ;
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




/*! \brief Function that GLUT calls to handle mouse button events
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




/*! \brief Function that GLUT calls to handle mouse motion
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
        cam.SetOrbit( azimuth , elevation , radius - ( dx + dy ) ) ;
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




/*! \brief Function that GLUT calls when the window obtains focus
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




typedef BOOL (APIENTRY *PFNWGLSWAPINTERVALFARPROC)( int );
PFNWGLSWAPINTERVALFARPROC wglSwapIntervalEXT = 0;

void setVSync(int interval=1)
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




/*! \brief Initialize display device
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

    // Set the current window to the main window
    glutSetWindow( mRenderWindow ) ;

    setVSync( 0 ) ;

    glutMainLoop() ; // Relinquish control to GLUT.  This never returns.
}




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
    static const float ambientFluidDensity   = 1.0f ;
    InteSiVis inteSiVis( viscosity , ambientFluidDensity ) ;
    inteSiVis.InitDevice( & argc , argv ) ;
    return 0 ;
}
