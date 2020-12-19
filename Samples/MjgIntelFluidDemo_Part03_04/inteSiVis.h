/*! \file inteSiVis.h

    \brief Application for interactive simulation and visualization

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef INTE_SI_VIS_H
#define INTE_SI_VIS_H

#include "Sim/fluidBodySim.h"
#include "Render/qdMaterial.h"
#include "Render/qdCamera.h"
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

        FluidBodySim        mFluidBodySim       ;   ///< Simulation of fluid and rigid bodies
        QdCamera            mCamera             ;   ///< Camera for rendering
        QdMaterial          mParticleMaterial   ;   ///< Material used to render vortons
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

    private:
        InteSiVis( const InteSiVis & re) ;                // Disallow copy construction.
        InteSiVis & operator=( const InteSiVis & re ) ;   // Disallow assignment
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
