/** \file inteSiVis.h

    \brief Application for interactive simulation and visualization.

    \mainpage Interactive simulation and visualization of fluids that use vortex elements.

    This code accompanies a series of articles published on the Intel Software Network and Gamasutra.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-18/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/

*/
#ifndef INTE_SI_VIS_H
#define INTE_SI_VIS_H

#include "entity.h"

#include "particleSystemConfiguration.h"

#include "Scene/fluidScene.h"

#include "Render/Scene/model.h"
#include "Render/system.h"

#include "QdRender/qdMaterial.h"
#include "QdRender/qdCamera.h"
#include "QdRender/qdLight.h"
#include "QdRender/qdModel.h"
#include "QdRender/qdParticleMaterial.h"
#include "QdRender/qdParticleRenderer.h"

#include "Particles/particleSystemManager.h"

#include "VortonFluid/pclOpVortonSim.h"

#include "Impulsion/rbSphere.h"
#include "Impulsion/rbBox.h"




/** Application for interactive simulation and visualization.
*/
class InteSiVis
{
    public:
        /** Grid decoration rendering options.
        */
        enum GridDecorationsE
        {
            GRID_DECO_NONE              ,   ///< Do not render grid.
            GRID_DECO_BOUNDING_BOX      ,   ///< Draw bounding box around domain.
            GRID_DECO_CELLS             ,   ///< Draw grid cells.
            GRID_DECO_POINTS            ,   ///< Draw grid points.
            GRID_DECO_CELLS_AND_POINTS  ,   ///< Draw grid cells and points.
        } ;

        InteSiVis( PeGaSys::Render::ApiBase * renderApi ) ;
        ~InteSiVis() ;

        static InteSiVis * GetInstance() ;

        void InitializeDisplayAndInputDevices() ;
        void InitialConditions( unsigned ic ) ;

        float CameraFocusEmphasis( const Vec3 position ) const ;

    private:
        /** Diagnostic text rendering options.
        */
        enum DiagnosticTextE
        {
            DIAG_TEXT_NONE      ,   ///< Do not render diagnostic text.
            DIAG_TEXT_SUMMARY   ,   ///< Render only summary text (not enough to affect frame rate).
            DIAG_TEXT_FULL      ,   ///< Render all diagnostic text.
        } ;

        /** Which value grid field rendering should represent.
        */
        enum GridFieldE
        {
            GRID_FIELD_VELOCITY             ,
            GRID_FIELD_DENSITY_GRADIENT     ,
            GRID_FIELD_PRESSURE_GRADIENT    ,

            GRID_FIELD_DENSITY              ,
            GRID_FIELD_SIGNED_DISTANCE      ,
        } ;

        /** Vortex particle rendering options.
        */
        enum VortonRenderingE
        {
            VORTON_RENDER_NONE                  ,   ///< Do not render vortex particles.
            VORTON_RENDER_DIAGNOSTIC_PARTICLES  ,   ///< Render only the vortons themselves -- no decorations.  Use diagnostic vorton rendering.
            VORTON_RENDER_SIMPLE_PARTICLES      ,   ///< Render only the vortons themselves -- no decorations.
            VORTON_RENDER_VECTORS               ,   ///< Render line decorations (hedgehogs).
            VORTON_RENDER_PARTICLES_AND_VECTORS ,   ///< Render vortons and their line decorations.
            VORTON_RENDER_PATHLINES             ,   ///< Render vortons and their pathlines.
            VORTON_RENDER_ALL                   ,   ///< Render vortons, their decorations and pathlines.
        } ;

        /** Which property to render, both as text next to each vorton, and as a glyph per vorton.
        */
        enum VortonPropertyE
        {
            VORTON_PROPERTY_POSITION            ,
            VORTON_PROPERTY_VELOCITY            ,
            VORTON_PROPERTY_VORTICITY           ,
            VORTON_PROPERTY_DENSITY             ,
            VORTON_PROPERTY_DENSITY_SPH         ,
            VORTON_PROPERTY_DENSITY_GRADIENT    ,
            VORTON_PROPERTY_PROXIMITY           ,
        } ;

        /** Tracer particle rendering options.
        */
        enum TracerRenderingE
        {
            TRACER_RENDER_NONE          ,   ///< Do not render tracer particles.
            TRACER_RENDER_FUEL          ,   ///< Render only the fuel fraction of tracers.
            TRACER_RENDER_FLAME         ,   ///< Render only the flame fraction of tracers.
            TRACER_RENDER_SMOKE         ,   ///< Render only the smoke fraction of tracers.
            TRACER_RENDER_FIRE          ,   ///< Render fuel, flame and smoke.
            TRACER_RENDER_DYE           ,   ///< Render only density as a two-color dye.
            TRACER_RENDER_DIAGNOSTIC    ,   ///< Render only density as a two-color dye.
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

        void            CreateRigidBodyModelsAndEntities() ;
        void            SetDiagnosticPropertyValueRangeScale( float & rangeScale ) const ;
        void            GatherAndRecordProfileData() ;
        void            UpdateParticleSystems() ;
        void            CopyCameraFromQdToPeGaSys() ;
        void            CopyLightsFromQdToPeGaSys() ;
        void            UpdateRigidBodies() ;
        void            UpdateRigidBodyModelsFromPhysics() ;
        const char *    FluidSimulationTechniqueString() const ;
        const char *    GridDecorationString() const ;
        const char *    GridFieldString() const ;
        const char *    VortonRenderingString() const ;
        const char *    VortonPropertyString() const ;
        const char *    TracerRenderingString() const ;

        void            SetTallyDiagnosticIntegrals() ;

        void            KeyboardHandler( unsigned char key , int mouseX , int mouseY ) ;
        void            SpecialKeyHandler ( int key , int modifierKeys , int windowRelativeMouseX , int windowRelativeMouseY ) ;
        void            MouseMotionHandler( int x , int y , int modifierKeys) ;
        void            Idle() ;

// TODO: Remove all these QD render methods and/or replace them with PeGaSys::Render analogs.
        void            CreateParticleQdMaterials() ;
        void            CreateQdMaterials() ;
        void            QdRenderVortonDiagnosticText() ;
        void            QdRenderVortonDiagnosticVectors() ;
        void            QdRenderVortons() ;
        void            QdRenderTracers() ;
        void            QdRenderNonFluidParticles() ;
        void            QdRenderDiagnosticPathlines( VECTOR< Particle > & Particles ) ;
        void            QdRenderDiagnosticGrid() ;
        void            QdRenderParticles() ;
        void            QdRenderSummaryDiagnosticText() ;
        void            InitializeQdRendering() ;
// END TODO

        static void     GlutDisplayCallback(void) ;
        static void     GlutReshapeGlutCallback(int width, int height) ;
        static void     GlutSpecialKeyHandler( int key , int windowRelativeMouseX , int windowRelativeMouseY ) ;
        static void     GlutKeyboardHandler (unsigned char key, int x, int y) ;
        static void     GlutMouseMotionHandler( int x , int y ) ;
        static void     GlutIdleCallback() ;
        static void     GlutMouseHandler( int button , int state , int x , int y ) ;
        static void     GlutEntryHandler( int state ) ;

        VECTOR< RbSphere >                    &  GetSpheres()           { return mSpheres ; }
        VECTOR< RbBox >                       &  GetBoxes()             { return mBoxes ; }
        VECTOR< Impulsion::PhysicalObject * > &  GetPhysicalObjects()   { return mPhysicalObjects ; }
        Impulsion::PhysicalObject *              GetPhysicalObjectInFocus() ;

        VECTOR< RbSphere >                      mSpheres                    ;   ///< Spherical physical objects.
        VECTOR< RbBox >                         mBoxes                      ;   ///< Box-shaped physical objects.
        VECTOR< Impulsion::PhysicalObject * >   mPhysicalObjects            ;   ///< Dynamic array of physical object addresses.
        size_t                                  mPhysObjFocus               ;   ///< Index info mPhysicalObjects that currently has user focus.

        ParticleSystemManager       mPclSysMgr                  ;   ///< Particle system manager.
        FluidVortonPclGrpInfo       mVortonPclGrpInfo           ;   // Fluid vorton particle system configuration
        FluidTracerPclGrpInfo       mTracerPclGrpInfo           ;   // Fluid tracer particle system configuration

        PeGaSys::Render::System     mRenderSystem               ;   ///< Render system
        FluidScene                  mFluidScene                 ;   ///< Demo scene to render.

// TODO: Remove all these QD render members and/or replace them with PeGaSys::Render analogs.
        QdCamera                    mQdCamera                   ;   ///< Camera for rendering QD scene
        VECTOR< QdLight >           mQdLights                   ;   ///< Lights for QD scene
        QdMaterial                  mBoundingBoxMaterial        ;   ///< Material used to render bounding box
        QdMaterial                  mPathlineMaterial           ;   ///< Material used to render pathlines
        QdParticleMaterial          mDiagnosticVortonMaterial   ;   ///< Material used to render vortons for diagnostics
        QdParticleMaterial          mSimpleVortonMaterial       ;   ///< Material used to render vortons
        QdParticleMaterial          mDiagnosticTracerMaterial   ;   ///< Material used to render tracers for diagnostics
        QdParticleMaterial          mDyeMaterial                ;   ///< Material used to render tracers as dye that depends on density
        QdParticleMaterial          mSmokeMaterial              ;   ///< Material used to render tracers as smoke
        QdParticleMaterial          mFlameMaterial              ;   ///< Material used to render tracers as flame
        QdParticleMaterial          mFuelMaterial               ;   ///< Material used to render tracers as fuel
        QdParticleRenderer          mDiagnosticVortonRenderer   ;   ///< Renderer for vortons for diagnostics
        QdParticleRenderer          mSimpleVortonRenderer       ;   ///< Renderer for vortons
        QdParticleRenderer          mDiagnosticTracerRenderer   ;   ///< Renderer for tracers for diagnostics
        QdParticleRenderer          mDyeRenderer                ;   ///< Renderer for tracers whose texture depends on density.
        QdParticleRenderer          mSmokeRenderer              ;   ///< Renderer for smoke tracers
        QdParticleRenderer          mFlameRenderer              ;   ///< Renderer for flame tracers
        QdParticleRenderer          mFuelRenderer               ;   ///< Renderer for fuel tracers
// END of members to remove

        VECTOR< Entity >            mEntities                   ;   ///< Simulation entities.

        unsigned                    mFrame                      ;   ///< Frame counter
        double                      mTimeNow                    ;   ///< Current virtual time
        float                       mTimeStep                   ;   ///< Amount by which to increment virtual time
        TimeSteppingE               mTimeStepping               ;   ///< How to step virtual time

        int                         mMouseButtons[3]            ;   ///< Mouse buttons pressed
        int                         mModifierKeys               ;   ///< Modifier (shift, control, alt) keys pressed during input event.

        bool                        mInitialized                ;   ///< Whether this application has been initialized
        int                         mScenario                   ;   ///< Which scenario is being simulated now

        // Controls for diagnostic information.
        GridDecorationsE            mGridDecorations            ;   ///< Which decorations to draw, if any.
        GridFieldE                  mGridField                  ;   ///< Which value to render as grid.
        VortonRenderingE            mVortonRendering            ;   ///< How to render vortons.
        VortonPropertyE             mVortonProperty             ;   ///< Which vorton property to render as text next to each vorton.
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
