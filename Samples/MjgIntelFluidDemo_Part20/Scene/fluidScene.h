/** \file fluidScene.h

    \brief Fluid render demo scene

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/
#ifndef PEGASYS_RENDER_FLUID_SCENE_H
#define PEGASYS_RENDER_FLUID_SCENE_H

#include <ParticlesRender/particlesRenderModel.h>

#include <Particles/particleSystemManager.h>

#include <Render/Resource/vertexBuffer.h>

#include <Image/image.h>

#include <Core/SpatialPartition/uniformGrid.h>

#include <Core/Math/vec3.h>

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------


namespace PeGaSys
{
    namespace Render
    {
        class ApiBase ;
        class System ;
        class Window ;
        class SceneManagerBase ;
        class Camera ;
        class Light ;
        class ModelNode ;
        class Pass ;
    }

    namespace ParticlesRender
    {
        class ParticlesRenderModel ;
    } ;
}

/** Fluid demo scene
*/
class FluidScene
{
public:
    static const size_t NUM_LIGHTS              = 2 ;
    //static const size_t NUM_MODELS              = 0 ;
    static const size_t NUM_SPECTRAL_COMPONENTS = 4 ;

    enum ShapeE
    {
        SHAPE_BOX       ,
        SHAPE_SPHERE    ,
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
        VORTON_RENDER_MAX                   ,   ///< Special value indicating one past last legal value
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
        TRACER_RENDER_MAX           ,   ///< Special value indicating one past last legal value
    } ;
    struct SpectralComponent
    {
        Vec3    mAmplitude  ;
        float   mFrequency  ;
        float   mPhaseShift ;
    } ;

    struct LightAnimation
    {
        SpectralComponent   mAnimComponents[ NUM_SPECTRAL_COMPONENTS ] ;
    } ;

    FluidScene( PeGaSys::Render::System * renderSystem , ParticleSystemManager * pclSysMgr ) ;
    ~FluidScene() ;

    void PopulateCameraLightsSky() ;
    void PopulateSceneWithFluidSurfaceAndParticles( ParticleSystem * fluidParticleSystem ) ;

    PeGaSys::Render::System *           GetRenderSystem()   { return mRenderSystem ; }
    PeGaSys::Render::SceneManagerBase * GetSceneManager()   { return mSceneManager ; }
    PeGaSys::Render::Window *           GetWindow()         { return mWindow ; }
    PeGaSys::Render::Camera *           GetCamera()         { return mCamera ; }
    PeGaSys::Render::Light *            GetLight( size_t index ) ;

    void AnimateLights( double timeNow ) ;

    LightAnimation &                    GetLightAnim( size_t index ) { return mLightAnims[ index ] ; }

    PeGaSys::Render::ModelNode * AddSphereModel( const Vec3 & position , float radius ) ;
    PeGaSys::Render::ModelNode * AddBoxModel( const Vec3 & position , const Vec3 & dimensions , bool isHole ) ;

    //PeGaSys::Render::ModelNode * GetModel( size_t index ) ;

    void SetTracerRenderingStyle( TracerRenderingE tracerRenderingStyle ) ;
    void SetVortonRenderingStyle( VortonRenderingE vortonRenderingStyle ) ;

    VortonRenderingE    GetVortonRenderingStyle() const { return mVortonRendering ; }
    TracerRenderingE    CycleTracerRenderingStyle() ;
    VortonRenderingE    CycleVortonRenderingStyle() ;
    const char *        GetTracerRenderingString() const ;
    const char *        GetVortonRenderingString() const ;

    PeGaSys::Render::ModelNode * GetFluidIsosurfaceModel() { return mFluidIsosurfaceModel ; }

    void UpdateFluidIsosurface( const UniformGrid< float > & gridOfValues ) ;

    void Clear() ;

private:

#if defined( WIN32 ) && defined( CreateWindow )
#   undef CreateWindow
#endif
    PeGaSys::Render::Window *           (CreateWindow)() ; // Parentheses around CreateWindow to protect against expanding macro defined in <windows.h>.
    PeGaSys::Render::SceneManagerBase * CreateSceneManager() ;
    PeGaSys::Render::Camera *           AddCamera() ;
    PeGaSys::Render::Light *            AddLight( const Vec3 & direction ) ;

    PeGaSys::Render::ModelNode *        AddDemoModel( const Vec3 & position , PeGaSys::Render::VertexDeclaration::VertexFormatE vertexFormat , bool isHole ) ;
    PeGaSys::Render::ModelNode *        AddSkyModel() ;
    PeGaSys::Render::ModelNode *        AddFluidIsosurfaceModel() ;
    PeGaSys::Render::ModelNode *        AddFluidParticleSystemModel( ParticleSystem * fluidParticleSystem ) ;

    void CreateLights() ;
    void CreateCamera() ;
    void CreateSkybox() ;
    void CreateFluidIsosurfaceModel() ;
    void CreateFluidParticleSystemModel( ParticleSystem * fluidParticleSystem ) ;

    void UpdateTracerRenderingStyle() ;
    void UpdateVortonRenderingStyle() ;

    PeGaSys::Render::System *           mRenderSystem               ;   /// Shallow pointer
    PeGaSys::Render::SceneManagerBase * mSceneManager               ;   /// Shallow pointer
    PeGaSys::Render::Window *           mWindow                     ;   /// Shallow pointer

    // Caches (shallow pointers) of scene nodes:
    PeGaSys::Render::Camera *           mCamera                     ;
    PeGaSys::Render::ModelNode *        mSkyBox                     ;
    PeGaSys::Render::ModelNode *        mFluidIsosurfaceModel       ;
    PeGaSys::Render::ModelNode *        mFluidParticleSystemModel   ;

    PeGaSys::Render::Pass *             mDiagnosticTracerPass       ;
    PeGaSys::Render::Pass *             mDyePass                    ;

    PeGaSys::Render::Pass *             mFuelPass                   ;
    PeGaSys::Render::Pass *             mSmokePass                  ;
    PeGaSys::Render::Pass *             mFlamePass                  ;

    PeGaSys::Render::Pass *             mDiagnosticVortonPass       ;
    PeGaSys::Render::Pass *             mDiagnosticSimpleVortonPass ;

    //PeGaSys::Render::ModelNode *        mModels[ NUM_MODELS ]       ;   /// Not used

    PeGaSys::Render::Light *            mLights[ NUM_LIGHTS ]       ;
    LightAnimation                      mLightAnims[ NUM_LIGHTS ]   ;

    ParticleSystemManager *             mPclSysMgr                  ;   /// Shallow pointer

    TracerRenderingE                    mTracerRendering            ;   ///< How to render tracers.
    VortonRenderingE                    mVortonRendering            ;   ///< How to render vortons.

    PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric   mTracerDiagDyeFuelVertBufFiller ;
    PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric   mTracerSmokeVertBufFiller       ;
    PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric   mTracerFlameVertBufFiller       ;

    PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric   mVortonVertBufFiller            ;

    PeGaSys::ParticlesRender::ParticlesRenderModel::PclSysVertexBufferFillerContainers  mPclSysVertBufFillerContainers ;
} ;

#endif

extern void MakeSimpleNoiseImage( PeGaSys::Image & noiseImage , int texWidth , int texHeight , int numTexPages , float gamma ) ;
extern void MakeDiagnosticVortonImage( PeGaSys::Image & noiseBall ) ;
extern void MakeDiagnosticSimpleVortonImage( PeGaSys::Image & noiseBall ) ;
extern void MakeDyeImage( PeGaSys::Image & noiseBall ) ;
extern void MakeDiagnosticTracerImage( PeGaSys::Image & noiseBall ) ;
extern void MakeSmokeImage( PeGaSys::Image & noiseBall ) ;
extern void MakeFlameImage( PeGaSys::Image & noiseBall ) ;
extern void MakeFuelImage( PeGaSys::Image & noiseBall ) ;