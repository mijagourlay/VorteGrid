/** \file inteSiVis.h

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
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \mainpage Interactive simulation and visualization of fluids that use vortex elements.

    This code accompanies a series of articles published on the Intel Software Network and Gamasutra.

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
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

        - http://www.mijagourlay.com/

*/
#ifndef INTE_SI_VIS_H
#define INTE_SI_VIS_H

#include "Sim/fluidBodySim.h"
#include "Render/qdMaterial.h"
#include "Render/qdCamera.h"
#include "Render/qdLight.h"
#include "Render/particleMaterial.h"
#include "Render/particleRenderer.h"

#include "Sim/Particle/pclOpFindBoundingBox.h"
#include "Sim/Particle/pclOpWind.h"
#include "Sim/Particle/pclOpAdvect.h"
#include "Sim/Particle/pclOpAssignScalarFromGrid.h"
#include "Sim/Particle/pclOpEmit.h"
#include "Sim/Particle/pclOpKillAge.h"
#include "Sim/Vorton/pclOpVortonSim.h"
#include "Sim/pclOpFluidBodyInteraction.h"

/*! \brief Application for interactive simulation and visualization
*/
class InteSiVis
{
    public:
        InteSiVis( float viscosity , float ambientFluidDensity , float fluidSpecificHeatCapacity ) ;
        ~InteSiVis() ;

        void InitDevice( int * pArgc , char ** argv ) ;
        void InitialConditions( unsigned ic ) ;

        float CameraFocusEmphasis( const Vec3 position ) const ;

    private:
        /** Tracer particle rendering options.
        */
        enum TracerRenderingE
        {
            TRACER_RENDER_NONE  ,   ///< Do not render tracer particles.
            TRACER_RENDER_FUEL  ,   ///< Render only the fuel fraction of tracers.
            TRACER_RENDER_FLAME ,   ///< Render only the flame fraction of tracers.
            TRACER_RENDER_SMOKE ,   ///< Render only the smoke fraction of tracers.
            TRACER_RENDER_ALL   ,   ///< Render all tracer varieties.
        } ;

        /** Vortex particle rendering options.
        */
        enum VortonRenderingE
        {
            VORTON_RENDER_NONE      ,   ///< Do not render vortex particles.
            VORTON_RENDER_SIMPLE    ,   ///< Render only the vortons themselves -- no decorations.
            VORTON_RENDER_VECTORS   ,   ///< Render vortons and their line decorations.
            VORTON_RENDER_PATHLINES ,   ///< Render vortons and their pathlines.
            VORTON_RENDER_ALL       ,   ///< Render vortons, their decorations and pathlines.
        } ;

        /** Grid decoration rendering options.
        */
        enum GridDecorationsE
        {
            GRID_DECO_NONE           ,   ///< Do not render grid.
            GRID_DECO_BOUNDING_BOX   ,   ///< Draw bounding box around domain.
            GRID_DECO_CELLS     ,   ///< Draw grid cells.
        } ;

        /** Diagnostic text rendering options.
        */
        enum DiagnosticTextE
        {
            DIAG_TEXT_NONE      ,   ///< Do not render diagnostic text.
            DIAG_TEXT_SUMMARY   ,   ///< Render only summary text (not enough to affect frame rate).
            DIAG_TEXT_FULL      ,   ///< Render all diagnostic text.
        } ;

        /** Simulation clock options.
        */
        enum TimeSteppingE
        {
            PAUSE       ,   ///< Pause simulation clock.
            SINGLE_STEP ,   ///< Unpause simulation clock for 1 tick, then pause again.
            PLAY        ,   ///< Advance simulation clock normally.
        } ;

        InteSiVis( const InteSiVis & re) ;                /// Disallow copy construction.
        InteSiVis & operator=( const InteSiVis & re ) ;   /// Disallow assignment

        void    CreateParticleMaterials() ;
        void    CreateMaterials() ;
        void    InitializeRendering() ;
        void    RenderSky() ;
        void    RenderRigidBodies() ;
        void    RenderVortonDiagnosticText() ;
        void    RenderVortonDiagnosticVectors() ;
        void    RenderVortons() ;
        void    RenderTracers() ;
        void    RenderPathlines( Vector< Particle > & Particles ) ;
        void    RenderParticles() ;
        void    RenderSummaryDiagnosticText() ;
        void    GatherAndRecordProfileData() ;
        void    UpdateParticleSystem() ;
        void    UpdateRigidBodies() ;

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
        PclOpAssignScalarFromGrid   mPclOpAssignDensityFromGrid ;   ///< Assign density to tracers from grid populated from vortons.
        PclOpAssignScalarFromGrid   mPclOpAssignFuelFromGrid    ;   ///< Assign fuel to tracers from grid populated from vortons.
        PclOpAssignScalarFromGrid   mPclOpAssignFlameFromGrid   ;   ///< Assign flame to tracers from grid populated from vortons.
        PclOpAssignScalarFromGrid   mPclOpAssignSmokeFromGrid   ;   ///< Assign smoke to tracers from grid populated from vortons.
        PclOpAdvect                 mPclOpAdvectTracers         ;   ///< Advect tracers according to velocity field.
        PclOpWind                   mPclOpWindTracers           ;   ///< Apply wind to tracers
        PclOpKillAge                mPclOpKillAgeTracers        ;   ///< Kill old passive tracer particles

        QdCamera                    mCamera                     ;   ///< Camera for rendering
        QdLight                     mLights[ 2 ]                ;   ///< Lights
        GLuint                      mBallDisplayList            ;   ///< Identifier for ball geometry cached in OpenGL server.
        QdMaterial                  mBallMaterial               ;   ///< Material used to render ball
        QdMaterial                  mBoundingBoxMaterial        ;   ///< Material used to render bounding box
        QdMaterial                  mPathlineMaterial           ;   ///< Material used to render pathlines
        QdMaterial                  mSkyMaterial                ;   ///< Material used to render sky dome
        ParticleMaterial            mVortonMaterial             ;   ///< Material used to render vortons
        ParticleMaterial            mHeavyLightTracerMaterial   ;   ///< Material used to render tracers whose texture depends on density.
        ParticleMaterial            mSmokeMaterial              ;   ///< Material used to render tracers as smoke
        ParticleMaterial            mFlameMaterial              ;   ///< Material used to render tracers as flame
        ParticleRenderer            mVortonRenderer             ;   ///< Renderer for vortons
        ParticleRenderer            mHeavyLightRenderer         ;   ///< Renderer for tracers whose texture depends on density.
        ParticleRenderer            mSmokeRenderer              ;   ///< Renderer for smoke tracers
        ParticleRenderer            mFlameRenderer              ;   ///< Renderer for flame tracers

        int                         mRenderWindow               ;   ///< Identifier for render window
        int                         mStatusWindow               ;   ///< Identifier for status window

        unsigned                    mFrame                      ;   ///< Frame counter
        double                      mTimeNow                    ;   ///< Current virtual time
        float                       mTimeStep                   ;   ///< Amount by which to increment virtual time
        TimeSteppingE               mTimeStepping               ;   ///< How to step virtual time
        int                         mMouseButtons[3]            ;   ///< Mouse buttons pressed
        bool                        mInitialized                ;   ///< Whether this application has been initialized
        int                         mScenario                   ;   ///< Which scenario is being simulated now

        // Controls for diagnostic information.
        GridDecorationsE            mGridDecorations            ;   ///< Which decorations to draw, if any.
        VortonRenderingE            mVortonRendering            ;   ///< How to render vortons.
        TracerRenderingE            mTracerRendering            ;   ///< How to render tracers.
        DiagnosticTextE             mDiagnosticText             ;   ///< Which diagnostic text to render, if any.
        bool                        mEmphasizeCameraTarget      ;   ///< Whether to emphasize objects near the camera look-at location.

    #if USE_TBB
        tbb::task_scheduler_init tbb_init ;
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
