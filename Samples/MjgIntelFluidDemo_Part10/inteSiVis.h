/*! \file inteSiVis.h

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

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef INTE_SI_VIS_H
#define INTE_SI_VIS_H

#include "Sim/fluidBodySim.h"
#include "Render/qdMaterial.h"
#include "Render/qdCamera.h"
#include "Render/qdLight.h"
#include "Render/particleRenderer.h"

#include "Sim/Particle/pclOpFindBoundingBox.h"
#include "Sim/Particle/pclOpWind.h"
#include "Sim/Particle/pclOpAdvect.h"
#include "Sim/Particle/pclOpEmit.h"
#include "Sim/Particle/pclOpKillAge.h"
#include "Sim/Vorton/pclOpVortonSim.h"
#include "Sim/pclOpFluidBodyInteraction.h"

/*! \brief Application for interactive simulation and visualization
*/
class InteSiVis
{
    public:
        InteSiVis( float viscosity , float ambientFluidDensity ) ;
        ~InteSiVis() ;

        void InitDevice( int * pArgc , char ** argv ) ;
        void InitialConditions( unsigned ic ) ;

    private:
        enum DecorationsE
        {
            DECO_NONE           ,
            DECO_BOUNDING_BOX   ,
            DECO_GRID_CELLS     ,
        } ;

        InteSiVis( const InteSiVis & re) ;                // Disallow copy construction.
        InteSiVis & operator=( const InteSiVis & re ) ;   // Disallow assignment

        static void GlutDisplayCallback(void) ;
        static void GlutReshapeGlutCallback(int width, int height) ;
        static void GlutSpecialKeyHandler (int key, int x, int y) ;
        static void GlutKeyboardHandler (unsigned char key, int x, int y) ;
        static void GlutMouseMotionHandler( int x , int y ) ;
        static void GlutIdleCallback(void) ;
        static void GlutMouseHandler( int button , int state , int x , int y ) ;
        static void GlutEntryHandler( int state ) ;

        Vector< RbSphere > &    GetSpheres( void )      { return mSpheres ; }
        Vector< RigidBody * > & GetRigidBodies( void )  { return mRigidBodies ; }

        Vector< RbSphere >          mSpheres                    ;   ///< Spherical rigid bodies to simulate
        Vector< RigidBody * >       mRigidBodies                ;   ///< Dynamic array of rigid body addresses

        ParticleSystem              mParticleSystem             ;   ///< Particle system

        ParticleGroup               mParticleGroupVortons       ;   ///< Particle group for vortons
        PclOpEmit                   mPclOpEmitVortons           ;   ///< Emit vortons
        PclOpVortonSim              mPclOpVortonSim             ;   ///< Update velocity grid due to vorton-based fluid simulation
        PclOpFluidBodyInteraction   mPclOpFluidBodInteVortons   ;   ///< Interact vortons with rigid bodies
        PclOpAdvect                 mPclOpAdvectVortons         ;   ///< Advect vortons according to velocity field
        PclOpWind                   mPclOpWindVortons           ;   ///< Apply wind to vortons
        PclOpKillAge                mPclOpKillAgeVortons        ;   ///< Kill old vortons

        ParticleGroup               mParticleGroupTracers       ;   ///< Particle group for tracers
        Vector< Particle > &        mTracers                    ;   ///< Passive tracer particles, used to visualize fluid
        PclOpEmit                   mPclOpEmitTracers           ;   ///< Emit passive tracer particles
        PclOpFindBoundingBox        mPclOpFindBoundingBox       ;   ///< Find bounding box containing all tracers
        PclOpFluidBodyInteraction   mPclOpFluidBodInteTracers   ;   ///< Interact passive tracers with rigid bodies
        PclOpAdvect                 mPclOpAdvectTracers         ;   ///< Advect tracers according to velocity field
        PclOpWind                   mPclOpWindTracers           ;   ///< Apply wind to tracers
        PclOpKillAge                mPclOpKillAgeTracers        ;   ///< Kill old passive tracer particles

        QdCamera                    mCamera                     ;   ///< Camera for rendering
        QdLight                     mLights[ 2 ]                ;   ///< Lights
        QdMaterial                  mVortonMaterial             ;   ///< Material used to render vortons
        QdMaterial                  mTracerMaterial             ;   ///< Material used to render tracers
        QdMaterial                  mBallMaterial               ;   ///< Material used to render ball
        QdMaterial                  mBoundingBoxMaterial        ;   ///< Material used to render bounding box
        QdMaterial                  mSkyMaterial                ;   ///< Material used to render sky dome
        ParticleRenderer            mVortonRenderer             ;   ///< Renderer for vortons
        ParticleRenderer            mTracerRenderer             ;   ///< Renderer for tracers
        int                         mRenderWindow               ;   ///< Identifier for render window
        int                         mStatusWindow               ;   ///< Identifier for status window
        unsigned                    mFrame                      ;   ///< Frame counter
        double                      mTimeNow                    ;   ///< Current virtual time
        int                         mMouseButtons[3]            ;   ///< Mouse buttons pressed
        bool                        mInitialized                ;   ///< Whether this application has been initialized
        int                         mScenario                   ;   ///< Which scenario is being simulated now
        DecorationsE                mDecorations                ;   ///< Which decorations to draw, if any

    #if USE_TBB
        tbb::task_scheduler_init tbb_init ;
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
