/*! \file inteSiVis.h

    \brief Application for interactive simulation and visualization

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef INTE_SI_VIS_H
#define INTE_SI_VIS_H

#include "Sim/fluidBodySim.h"
#include "Render/qdMaterial.h"
#include "Render/qdCamera.h"
#include "Render/qdLight.h"
#include "Render/particleRenderer.h"

/*! \brief Application for interactive simulation and visualization
*/
class InteSiVis
{
    public:
        InteSiVis( float viscosity , float density ) ;
        ~InteSiVis() ;

        void InitDevice( int * pArgc , char ** argv ) ;
        void InitialConditions( unsigned ic ) ;

    private:
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

        FluidBodySim        mFluidBodySim       ;   ///< Simulation of fluid and rigid bodies
        QdCamera            mCamera             ;   ///< Camera for rendering
        QdLight             mLights[ 2 ]        ;   ///< Lights
        QdMaterial          mParticleMaterial   ;   ///< Material used to render vortons
        QdMaterial          mBallMaterial       ;   ///< Material used to render ball
        QdMaterial          mSkyMaterial        ;   ///< Material used to render sky dome
        ParticleRenderer    mVortonRenderer     ;   ///< Renderer for vortons
        ParticleRenderer    mTracerRenderer     ;   ///< Renderer for tracers
        int                 mRenderWindow       ;   ///< Identifier for render window
        int                 mStatusWindow       ;   ///< Identifier for status window
        unsigned            mFrame              ;   ///< Frame counter
        double              mTimeNow            ;   ///< Current virtual time
        int                 mMouseButtons[3]    ;   ///< Mouse buttons pressed
        bool                mInitialized        ;   ///< Whether this application has been initialized
        int                 mScenario           ;   ///< Which scenario is being simulated now

    #if USE_TBB
        tbb::task_scheduler_init tbb_init ;
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
