/** \file fluidScene.cpp

    \brief Scene creation routines for InteSiVis.

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "inteSiVis.h"

#include <Image/image.h>
#include <Image/Operation/imgOpMakeNoise.h>
#include <Image/Operation/imgOpBlur.h>
#include <Image/imgOpSequences.h>

#include <ParticlesRender/particlesRenderModel.h>

#include <Render/Platform/OpenGL/OpenGL_VertexBuffer.h>

#include <Render/Scene/sceneManagerBase.h>
#include <Render/Scene/camera.h>
#include <Render/Scene/light.h>
#include <Render/Scene/model.h>

#include <Render/Device/window.h>

#include <Render/Resource/mesh.h>
#include <Render/Resource/marchingCubes.h>
#include <Render/Resource/material.h>
#include <Render/Resource/pass.h>
#include <Render/Resource/texture.h>

#include <Render/system.h>

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#include <Scene/fluidScene.h>




// Types -----------------------------------------------------------------------

typedef PeGaSys::Render::OpenGL_VertexBuffer::VertexFormatPositionColor4Texture2 VertexFormatPos3Col4Tex2 ;

// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

/// Routine to set given Vertex Buffer filler to state suitable for Diagnostic or Dye rendering.
void VertexBufferFiller_SetDiagnosticOrDye( PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric & vbFiller )
{
    vbFiller.SetPos3FCol4BTex2F() ;
    vbFiller.mUseDensityForTextureCoordinate    = true ;
    vbFiller.mDensityVisibility                 = FLT_MAX ;
}




void VertexBufferFiller_SetFuel( PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric & vbFiller )
{
    vbFiller.SetPos3FCol4BTex2F() ;
    vbFiller.mOffsetToDensityInfo               = offsetof( Particle , mFuelFraction ) ;
    vbFiller.mDensityVisibility                 = 254.8f * 100.0f ;
    vbFiller.mScale                             = 2.0987f ;
}




void VertexBufferFiller_SetSmoke( PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric & vbFiller )
{
    vbFiller.SetPos3FCol4BTex2F() ;
    vbFiller.mOffsetToDensityInfo               = offsetof( Particle , mSmokeFraction ) ;
    vbFiller.mDensityVisibility                 = 100.0f ;
    vbFiller.mScale                             = 4.0123f ;
}




void VertexBufferFiller_SetFlame( PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric & vbFiller )
{
    vbFiller.SetPos3FCol4BTex2F() ;
    vbFiller.mOffsetToDensityInfo               = offsetof( Particle , mFlameFraction ) ;
    vbFiller.mDensityVisibility                 = 254.9f * 100.0f ;
    vbFiller.mBlendMode                         = PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric::BLEND_MODE_ADDITIVE ;
    vbFiller.mScale                             = 4.0f ;
}




void VertexBufferFiller_SetVorton( PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric & vbFiller )
{
    vbFiller.SetPos3FCol4BTex2F() ;
    vbFiller.mUseDensityForTextureCoordinate    = true ;
}




/** Construct fluid demo scene.
*/
FluidScene::FluidScene( PeGaSys::Render::System * renderSystem
    , ParticleSystemManager * pclSysMgr
    )
    : mRenderSystem ( renderSystem )
    , mSceneManager( NULLPTR )
    , mWindow( NULLPTR )

    , mCamera( NULLPTR )
    , mSkyBox( NULLPTR )
    , mFluidIsosurfaceModel( NULLPTR )
    , mFluidParticleSystemModel( NULLPTR )

    , mDiagnosticTracerPass      ( NULLPTR )
    , mDyePass                   ( NULLPTR )

    , mFuelPass                  ( NULLPTR )
    , mSmokePass                 ( NULLPTR )
    , mFlamePass                 ( NULLPTR )

    , mPclSysMgr( pclSysMgr )

    , mTracerRendering( TRACER_RENDER_FIRE )
    , mVortonRendering( VORTON_RENDER_NONE )

    , mDiagnosticVortonPass      ( NULLPTR )
    , mDiagnosticSimpleVortonPass( NULLPTR )
{
    PERF_BLOCK( FluidScene__FluidScene ) ;

    memset( mLights     , 0 , sizeof( mLights     ) ) ;
    memset( mLightAnims , 0 , sizeof( mLightAnims ) ) ;

    using namespace PeGaSys::Render ;

#if defined( WIN32 ) && defined( CreateWindow )
#   undef CreateWindow
#endif

    mWindow         = (CreateWindow)() ;  // Parentheses around CreateWindow to protect against expanding macro defined in <windows.h>.
    mSceneManager   = CreateSceneManager() ;

    // Use multiple vb fillers, one per tracer type (fuel, smoke, flame) that requires different vertex data.
    //
    // In principle, all rendering passes of the tracers could use the same vertex buffer,
    // if instead of using vertex data, a shader varied the parameters we want to change for each pass.
    //
    // Examples of those parameters are size and vertex color.
    //
    // Size can vary per particle, but only uniformly depends on render pass.  That is, all smoke particles
    // have the same scale, relative to flame or fuel.  In that case, the same effect could be achieved by
    // making all tracers the largest needed by any pass, then shrinking textures for passes that can use smaller particles.
    // But that could spend fill rate and we have to refill VB's for other reasons anyway, so might as well
    // include scaling in the VB filler.
    //
    // Vertex color is used to convey transparency, and varies per particle, according to species fractions,
    // hence depends on the render pass.

    // mTracerVertBufFiller is used to render diagnostic, dye and fuel tracers.
    // There is no need to render those simultaneously, so the same VB filler can be used for any of them,
    // even if they use incompatible filler properties, just by changing those VB filler properties at runtime.
    // To choose between them, change the VB filler properties and also activate the appropriate render pass.
    VertexBufferFiller_SetFuel( mTracerDiagDyeFuelVertBufFiller ) ;
    mTracerDiagDyeFuelVertBufFiller.mIsActive = false ;

    VertexBufferFiller_SetSmoke( mTracerSmokeVertBufFiller ) ;
    mTracerSmokeVertBufFiller.mIsActive = false ;

    VertexBufferFiller_SetFlame( mTracerFlameVertBufFiller ) ;
    mTracerFlameVertBufFiller.mIsActive = false ;

    VertexBufferFiller_SetVorton( mVortonVertBufFiller ) ;
    mVortonVertBufFiller.mIsActive = false ;
}




/** Destruct fluid demo scene.
*/
FluidScene::~FluidScene()
{
    PERF_BLOCK( FluidScene__dtor ) ;

    Clear() ;
}




void FluidScene::CreateLights()
{
    PERF_BLOCK( FluidScene__CreateLights ) ;

    ASSERT( NULLPTR == mLights[ 0 ] ) ;
    ASSERT( NULLPTR == mLights[ 1 ] ) ;
    mLights[ 0 ] = AddLight( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
    mLights[ 1 ] = AddLight( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
}




/** Add a camera to view the scene.

    This also incidentally creates a viewport, that contains a diagnostic text overlay.
*/
void FluidScene::CreateCamera()
{
    PERF_BLOCK( FluidScene__CreateCamera ) ;

    ASSERT( NULLPTR == mCamera ) ;
    mCamera = AddCamera() ;
}




void FluidScene::CreateSkybox()
{
    PERF_BLOCK( FluidScene__CreateSkybox ) ;

    ASSERT( NULLPTR == mSkyBox ) ;
    mSkyBox = AddSkyModel() ;
}




void FluidScene::CreateFluidIsosurfaceModel()
{
    PERF_BLOCK( FluidScene__CreateFluidIsosurfaceModel ) ;

    ASSERT( NULLPTR == mFluidIsosurfaceModel ) ;
    mFluidIsosurfaceModel = AddFluidIsosurfaceModel() ;
}




void FluidScene::CreateFluidParticleSystemModel( ParticleSystem * fluidParticleSystem )
{
    PERF_BLOCK( FluidScene__CreateParticleSystemModel ) ;

    ASSERT( NULLPTR == mFluidParticleSystemModel ) ;
    mFluidParticleSystemModel = AddFluidParticleSystemModel( fluidParticleSystem ) ;
}




void FluidScene::PopulateCameraLightsSky()
{
    PERF_BLOCK( FluidScene__PopulateCameraLightsSky ) ;

    CreateCamera() ;
    CreateLights() ;
    CreateSkybox() ;

#if 0
    {
        using namespace PeGaSys::Render ;

        Viewport *              viewport        = mWindow->GetViewports().Front() ;
        DiagnosticTextOverlay & diagTextOver    = viewport->GetDiagnosticTextOverlay() ;
        diagTextOver.Clear() ;
        for( int line = 0 ; line < 256 ; ++ line )
        {
            diagTextOver.AppendLine( "Hello, World! %i" , line ) ;
        }
    }
#endif
}




void FluidScene::PopulateSceneWithFluidSurfaceAndParticles( ParticleSystem * fluidParticleSystem )
{
    PERF_BLOCK( FluidScene__PopulateSceneWithFluidSurfaceAndParticles ) ;

    // The fluid models are both translucent, and need to render after all opaque objects.
    CreateFluidIsosurfaceModel() ;
    CreateFluidParticleSystemModel( fluidParticleSystem ) ;

    UpdateTracerRenderingStyle() ;
    UpdateVortonRenderingStyle() ;
}




/** Animate light parameters.

    \note   This could also be done with a NativeCallback if/when using PeGaSys::Entity.
*/
void FluidScene::AnimateLights( double timeNow )
{
    PERF_BLOCK( FluidScene__AnimateLights ) ;

    for( int iLight = 0 ; iLight < NUM_LIGHTS ; ++ iLight )
    {
        if( mLights[ iLight ] != NULLPTR )
        {   // This light exists.
            Vec3 color = Vec3( 0.0f , 0.0f , 0.0f ) ;
            const LightAnimation & anim = GetLightAnim( iLight ) ;
            for( int iSpecComp = 0 ; iSpecComp < NUM_SPECTRAL_COMPONENTS ; ++ iSpecComp )
            {   // For each spectral component...
                const SpectralComponent & specComp = anim.mAnimComponents[ iSpecComp ] ;
                static float freqShift = 1.0f ;
                freqShift = Clamp( freqShift + RandomSpread( 0.0001f ) , 0.8f , 1.1f ) ;
                const double phase = timeNow * specComp.mFrequency * freqShift + specComp.mPhaseShift ;
                color += specComp.mAmplitude * static_cast< float >( cos( phase ) ) ;
            }
            mLights[ iLight ]->SetAmbientColor( Vec4( color * 0.2f , 1.0f ) ) ;
            mLights[ iLight ]->SetDiffuseColor( Vec4( color * 0.8f , 1.0f ) ) ;
        }
    }
}




PeGaSys::Render::Light *     FluidScene::GetLight( size_t index )
{
    PERF_BLOCK( FluidScene__GetLight ) ;

    ASSERT( index < NUM_LIGHTS ) ;
    return mLights[ index ] ;
}




/** Regenerate isosurface from fluid grid.

    \note   This could also be done with a NativeCallback if/when using PeGaSys::Entity.
*/
void FluidScene::UpdateFluidIsosurface( const UniformGrid< float > & gridOfValues )
{
    PERF_BLOCK( FluidScene__UpdateFluidIsosurface ) ;

    using namespace PeGaSys::Render ;

    const VertexDeclaration::VertexFormatE vertFmt = VertexDeclaration::POSITION_NORMAL ;

    MeshBase *  mesh        = GetFluidIsosurfaceModel()->GetModelData()->GetMesh( 0 ) ;
    ApiBase *   renderApi   = mRenderSystem->GetApi() ;

    static const size_t numSegments = 32 ;

#if 0 // WORK IN PROGRESS: Generate sphere mesh, vertices and normals.
    // Regenerate sphere grid using different radius, then refill mesh.
static float fakeTimer = 0.0f ;
fakeTimer += 0.1f ;
    Mesh_MakeSphere( mesh , renderApi , numSegments , /* radius */ 0.05f * ( cos( fakeTimer ) + 2.0f ) , vertFmt ) ;

    (void) gridOfValues ;

#else // WORK IN PROGRESS: This fluid surface generation is temporarily disabled while testing normals-from-gradient in Mesh_MakeSphere.
    GridWrapper gridWrapper ;
    gridWrapper.values = const_cast< float * >( gridOfValues.Data() ) ;
    gridWrapper.number[ 0 ] = gridOfValues.GetNumPoints( 0 ) ;
    gridWrapper.number[ 1 ] = gridOfValues.GetNumPoints( 1 ) ;
    gridWrapper.number[ 2 ] = gridOfValues.GetNumPoints( 2 ) ;
    gridWrapper.strides[ 0 ] = sizeof( float ) ;
    gridWrapper.strides[ 1 ] = sizeof( float ) * gridOfValues.GetNumPoints( 0 ) ;
    gridWrapper.strides[ 2 ] = sizeof( float ) * gridOfValues.GetNumPoints( 0 ) * gridOfValues.GetNumPoints( 1 ) ;
    gridWrapper.minPos = gridOfValues.GetMinCorner() ;
    gridWrapper.directions[ 0 ] = Vec3( gridOfValues.GetCellSpacing().x , 0.0f , 0.0f ) ;
    gridWrapper.directions[ 1 ] = Vec3( 0.0f , gridOfValues.GetCellSpacing().y , 0.0f ) ;
    gridWrapper.directions[ 2 ] = Vec3( 0.0f , 0.0f , gridOfValues.GetCellSpacing().z ) ;

    Mesh_MakeFromVolume( mesh , renderApi , 0.0f , & gridWrapper , vertFmt ) ;
#endif
}




/** Create a window for rendering the scene.

    \note   windows.h defines CreateWindow as a macro.
            If you get compile errors, put parentheses around CreateWindow to protect against expanding macro defined in <windows.h>.
*/
PeGaSys::Render::Window * FluidScene::CreateWindow()
{
    PERF_BLOCK( FluidScene__CreateWindow ) ;

    ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;
    if( mRenderSystem && mRenderSystem->GetApi() )
    {
        PeGaSys::Render::Window * mainWindow = mRenderSystem->GetApi()->NewWindow( mRenderSystem ) ;
        mainWindow->Create() ;
        mRenderSystem->AddTarget( mainWindow ) ;
        return mainWindow ;
    }
    return NULL ;
}




PeGaSys::Render::SceneManagerBase * FluidScene::CreateSceneManager()
{
    PERF_BLOCK( FluidScene__CreateSceneManager ) ;

    ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;

    if( mRenderSystem && mRenderSystem->GetApi() )
    {
        PeGaSys::Render::SceneManagerBase * sceneManager = NEW PeGaSys::Render::SceneManagerBase( mRenderSystem->GetApi() ) ;
        return sceneManager ;
    }
    return NULL ;
}




/** Add a camera to the scene.
*/
PeGaSys::Render::Camera * FluidScene::AddCamera()
{
    PERF_BLOCK( FluidScene__AddCamera ) ;

    ASSERT( mSceneManager && mWindow ) ;

    if( mSceneManager )
    {
        PeGaSys::Render::Camera * camera = mSceneManager->AddCamera() ;
        camera->SetPosition( Vec3( 5.0f , 0.0f , 0.0f ) ) ;
        ASSERT( mWindow->GetViewports().Empty() ) ;
        mWindow->AddViewport( camera ) ;
        return camera ;
    }
    return NULL ;
}




/** Add a light node to the scene.
*/
PeGaSys::Render::Light * FluidScene::AddLight( const Vec3 & direction )
{
    PERF_BLOCK( FluidScene__AddLight ) ;

    ASSERT( mSceneManager ) ;

    if( mSceneManager )
    {
        PeGaSys::Render::Light * light = mSceneManager->AddLight() ;
        //light->SetLightType( Render::Light::POINT ) ;
        light->SetDirection( direction ) ;
        return light ;
    }
    return NULL ;
}




PeGaSys::Render::ModelNode * FluidScene::AddSphereModel( const Vec3 & position , float radius )
{
    PERF_BLOCK( FluidScene__AddSphereModel ) ;

    using namespace PeGaSys::Render ;

#if 1
    const VertexDeclaration::VertexFormatE vertFmt = VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE ;
#else
    const VertexDeclaration::VertexFormatE vertFmt = VertexDeclaration::POSITION ;
#endif
    ModelNode * model       = FluidScene::AddDemoModel( position , vertFmt , /* isHole */ false ) ;
    MeshBase *  mesh        = model->GetModelData()->GetMesh( 0 ) ;
    ApiBase *   renderApi   = mRenderSystem->GetApi() ;

    static const size_t numSegments = 32 ;

#if 1
    mesh->MakeSphere( renderApi , radius , numSegments , numSegments * 2 , vertFmt ) ;
#else
    Mesh_MakeSphere( mesh , renderApi , numSegments , radius , vertFmt ) ;
#endif

    return model ;
}




PeGaSys::Render::ModelNode * FluidScene::AddBoxModel( const Vec3 & position , const Vec3 & dimensions , bool isHole )
{
    PERF_BLOCK( FluidScene__AddBoxModel ) ;

    using namespace PeGaSys::Render ;
    
    const VertexDeclaration::VertexFormatE vertFmt = isHole ? VertexDeclaration::POSITION_NORMAL_TEXTURE : VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE ;
    ModelNode * model       = FluidScene::AddDemoModel( position , vertFmt , isHole ) ;
    MeshBase *  mesh        = model->GetModelData()->GetMesh( 0 ) ;
    ApiBase *   renderApi   = mRenderSystem->GetApi() ;

    mesh->MakeBox( renderApi , dimensions , vertFmt ) ;

    return model ;
}




/// Update internal rendering state to reflect the value of mTraderRendering.
void FluidScene::UpdateTracerRenderingStyle()
{
    // Start by disabling all vbFillers and passes.  Then, below, activate only those appropriate.
    mTracerDiagDyeFuelVertBufFiller.mIsActive = false ;
    mTracerSmokeVertBufFiller.mIsActive = false ;
    mTracerFlameVertBufFiller.mIsActive = false ;

    VertexBufferFiller_SetFuel( mTracerDiagDyeFuelVertBufFiller ) ;

    if( NULLPTR == mDiagnosticTracerPass )
    {   // Passes have not yet been initialized.
        // This can happen when setting up initial conditions, before the render system has been initialized, and materials created.
        // UpdateTracerRenderingStyle will need to be called again later.  But since tracer rendering style can be
        // changed either at initial conditions, or during rendering, the code calls UpdateTracerRenderingStyle
        // in both places, and is robust against early calls to it.
        return ;
    }

    mDiagnosticTracerPass->SetActive( false ) ;
    mDyePass->SetActive( false ) ;
    mFuelPass->SetActive( false ) ;
    mSmokePass->SetActive( false ) ;
    mFlamePass->SetActive( false ) ;

    switch( mTracerRendering )
    {
    case TRACER_RENDER_NONE:
        break ;
    case TRACER_RENDER_FUEL:
        mTracerDiagDyeFuelVertBufFiller.mIsActive = true ;
        mFuelPass->SetActive( true ) ;
        break ;
    case TRACER_RENDER_FLAME:
        mTracerFlameVertBufFiller.mIsActive = true ;
        mFlamePass->SetActive( true ) ;
        break ;
    case TRACER_RENDER_SMOKE:
        mTracerSmokeVertBufFiller.mIsActive = true ;
        mSmokePass->SetActive( true ) ;
        break ;
    case TRACER_RENDER_FIRE      :
        mTracerDiagDyeFuelVertBufFiller.mIsActive = true ;
        mTracerSmokeVertBufFiller.mIsActive = true ;
        mTracerFlameVertBufFiller.mIsActive = true ;
        mFuelPass->SetActive( true ) ;
        mSmokePass->SetActive( true ) ;
        mFlamePass->SetActive( true ) ;
        break ;
    case TRACER_RENDER_DYE       :
        VertexBufferFiller_SetDiagnosticOrDye( mTracerDiagDyeFuelVertBufFiller ) ;
        mDyePass->SetActive( true ) ;
        mTracerDiagDyeFuelVertBufFiller.mIsActive = true ;
        break ;
    case TRACER_RENDER_DIAGNOSTIC:
        VertexBufferFiller_SetDiagnosticOrDye( mTracerDiagDyeFuelVertBufFiller ) ;
        mDiagnosticTracerPass->SetActive( true ) ;
        mTracerDiagDyeFuelVertBufFiller.mIsActive = true ;
        break ;
    }
}




/// Update internal rendering state to reflect the value of mVortonRendering.
void FluidScene::UpdateVortonRenderingStyle()
{
    mVortonVertBufFiller.mIsActive = false ;

    if( NULLPTR == mDiagnosticVortonPass )
    {   // Passes have not yet been initialized.
        // This can happen when setting up initial conditions, before the render system has been initialized, and materials created.
        // UpdateTracerRenderingStyle will need to be called again later.  But since tracer rendering style can be
        // changed either at initial conditions, or during rendering, the code calls UpdateTracerRenderingStyle
        // in both places, and is robust against early calls to it.
        return ;
    }

    mDiagnosticVortonPass->SetActive( false ) ;
    mDiagnosticSimpleVortonPass->SetActive( false ) ;

    switch( mVortonRendering )
    {
    case VORTON_RENDER_NONE:
        mVortonVertBufFiller.mIsActive = false ;
        break ;
    case VORTON_RENDER_DIAGNOSTIC_PARTICLES:
        mVortonVertBufFiller.mIsActive = true ;
        mDiagnosticVortonPass->SetActive( true ) ;
        break ;
    case VORTON_RENDER_SIMPLE_PARTICLES:
        mVortonVertBufFiller.mIsActive = true ;
        mDiagnosticSimpleVortonPass->SetActive( true ) ;
        break ;
    case VORTON_RENDER_VECTORS:
        mVortonVertBufFiller.mIsActive = false ;
        break ;
    case VORTON_RENDER_PARTICLES_AND_VECTORS:
        mVortonVertBufFiller.mIsActive = true ;
        mDiagnosticVortonPass->SetActive( true ) ;
        break ;
    case VORTON_RENDER_PATHLINES:
        break ;
    case VORTON_RENDER_ALL:
        mVortonVertBufFiller.mIsActive = true ;
        mDiagnosticVortonPass->SetActive( true ) ;
        break ;
    }
}




void FluidScene::SetTracerRenderingStyle( TracerRenderingE tracerRenderingStyle )
{
    mTracerRendering = tracerRenderingStyle ;
    UpdateTracerRenderingStyle() ;
}




void FluidScene::SetVortonRenderingStyle( VortonRenderingE vortonRenderingStyle )
{
    mVortonRendering = vortonRenderingStyle ;
    UpdateVortonRenderingStyle() ;
}




FluidScene::TracerRenderingE FluidScene::CycleTracerRenderingStyle()
{
    switch( mTracerRendering )
    {
    case TRACER_RENDER_NONE      : SetTracerRenderingStyle( TRACER_RENDER_FUEL       ) ; break ;
    case TRACER_RENDER_FUEL      : SetTracerRenderingStyle( TRACER_RENDER_FLAME      ) ; break ;
    case TRACER_RENDER_FLAME     : SetTracerRenderingStyle( TRACER_RENDER_SMOKE      ) ; break ;
    case TRACER_RENDER_SMOKE     : SetTracerRenderingStyle( TRACER_RENDER_FIRE       ) ; break ;
    case TRACER_RENDER_FIRE      : SetTracerRenderingStyle( TRACER_RENDER_DYE        ) ; break ;
    case TRACER_RENDER_DYE       : SetTracerRenderingStyle( TRACER_RENDER_DIAGNOSTIC ) ; break ;
    case TRACER_RENDER_DIAGNOSTIC: SetTracerRenderingStyle( TRACER_RENDER_NONE       ) ; break ;
    }
    return mTracerRendering ;
}




FluidScene::VortonRenderingE FluidScene::CycleVortonRenderingStyle()
{
    switch( mVortonRendering )
    {
    case VORTON_RENDER_NONE                 : SetVortonRenderingStyle( VORTON_RENDER_DIAGNOSTIC_PARTICLES  ) ; break ;
    case VORTON_RENDER_DIAGNOSTIC_PARTICLES : SetVortonRenderingStyle( VORTON_RENDER_SIMPLE_PARTICLES      ) ; break ;
    case VORTON_RENDER_SIMPLE_PARTICLES     : SetVortonRenderingStyle( VORTON_RENDER_VECTORS               ) ; break ;
    case VORTON_RENDER_VECTORS              : SetVortonRenderingStyle( VORTON_RENDER_PARTICLES_AND_VECTORS ) ; break ;
    case VORTON_RENDER_PARTICLES_AND_VECTORS: SetVortonRenderingStyle( VORTON_RENDER_PATHLINES             ) ; break ;
    case VORTON_RENDER_PATHLINES            : SetVortonRenderingStyle( VORTON_RENDER_ALL                   ) ; break ;
    case VORTON_RENDER_ALL                  : SetVortonRenderingStyle( VORTON_RENDER_NONE                  ) ; break ;
    }
    return mVortonRendering ;
}




const char * FluidScene::GetTracerRenderingString() const
{
    switch( mTracerRendering )
    {
        CASE_STRING_FROM_TOKEN( TRACER_RENDER_NONE       ) ;
        CASE_STRING_FROM_TOKEN( TRACER_RENDER_FUEL       ) ;
        CASE_STRING_FROM_TOKEN( TRACER_RENDER_FLAME      ) ;
        CASE_STRING_FROM_TOKEN( TRACER_RENDER_SMOKE      ) ;
        CASE_STRING_FROM_TOKEN( TRACER_RENDER_FIRE       ) ;
        CASE_STRING_FROM_TOKEN( TRACER_RENDER_DYE        ) ;
        CASE_STRING_FROM_TOKEN( TRACER_RENDER_DIAGNOSTIC ) ;
    }
    return "(invalid TracerRenderingE)" ;
}




const char * FluidScene::GetVortonRenderingString() const
{
    switch( mVortonRendering )
    {
        CASE_STRING_FROM_TOKEN( VORTON_RENDER_NONE                  ) ;
        CASE_STRING_FROM_TOKEN( VORTON_RENDER_DIAGNOSTIC_PARTICLES  ) ;
        CASE_STRING_FROM_TOKEN( VORTON_RENDER_SIMPLE_PARTICLES      ) ;
        CASE_STRING_FROM_TOKEN( VORTON_RENDER_VECTORS               ) ;
        CASE_STRING_FROM_TOKEN( VORTON_RENDER_PARTICLES_AND_VECTORS ) ;
        CASE_STRING_FROM_TOKEN( VORTON_RENDER_PATHLINES             ) ;
        CASE_STRING_FROM_TOKEN( VORTON_RENDER_ALL                   ) ;
    }
    return "(invalid VortonRenderingE)" ;
}




PeGaSys::Render::ModelNode * FluidScene::AddFluidIsosurfaceModel()
{
    PERF_BLOCK( FluidScene__AddFluidIsosurfaceModel ) ;

    using namespace PeGaSys::Render ;

    const VertexDeclaration::VertexFormatE vertFmt = VertexDeclaration::POSITION_NORMAL ;

    ModelNode * model       = FluidScene::AddDemoModel( Vec3( 0.0f , 0.0f , 0.0f ) , vertFmt , /* isHole */ false ) ;
    MeshBase *  mesh        = model->GetModelData()->GetMesh( 0 ) ;
    ApiBase *   renderApi   = mRenderSystem->GetApi() ;

    // Note: numSegments indirectly sets the vertex buffer initial capacity.
    static const size_t numSegments = 32 ;

    // Placeholder geometry: zero-size sphere.
    Mesh_MakeSphere( mesh , renderApi , numSegments , /* radius */ 0.0f  , vertFmt ) ;

    return model ;
}




/**
    Caller must create vertex and index buffers and populate with shape geometry, like so:

        if( SHAPE_BOX == shape )
        {
            mesh->MakeBox( renderApi , Vec3( 0.2f , 0.5f , 1.0f ) , vertexFormat ) ;
        }
        else
        {
            ASSERT( SHAPE_SPHERE == shape ) ;
            mesh->MakeSphere( renderApi , 0.5f , 8 , 16, vertexFormat ) ;
        }
*/
PeGaSys::Render::ModelNode * FluidScene::AddDemoModel( const Vec3 & position , const PeGaSys::Render::VertexDeclaration::VertexFormatE vertexFormat , bool isHole )
{
    PERF_BLOCK( FluidScene__AddDemoModel ) ;

    using namespace PeGaSys ;
    using namespace PeGaSys::Render ;

    ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;

    ModelNode * modelNode   = NULLPTR ;

    if( mRenderSystem && mRenderSystem->GetApi() )
    {
        ApiBase *   renderApi       = mRenderSystem->GetApi() ;
        Effect *    shapeEffect     = NEW Effect() ;
        Technique * shapeTechnique  = shapeEffect->GetTechniques().Front() ;
        Pass *      shapePass       = shapeTechnique->GetPasses().Front() ;

        if( VertexDeclaration( vertexFormat ).HasTextureCoordinates() )
        {   // Requested vertex format includes texture coordinates, so create a texture.
            shapePass->AddTextureStage() ;
            TextureStage &  shapeTextureStage = shapePass->GetTextureStage( 0 ) ;
            shapeTextureStage.mTexture = mRenderSystem->GetApi()->NewTexture() ;
            {
                Image image( 128 , 128 , 4 , 1 ) ;
                ImgOpSeq_Noise( image , 0.9f , Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) ) ;
                shapeTextureStage.mTexture->CreateFromImages( & image , 1 ) ;
            }
            shapeTextureStage.mSamplerState.mAddressU = SamplerState::ADDRESS_REPEAT ;
            shapeTextureStage.mSamplerState.mAddressV = SamplerState::ADDRESS_CLAMP  ;

            if( isHole )
            {   // This model is a "container" -- an inside-out box.
                shapePass->GetRenderState().mRasterState.mCullMode = RasterStateS::CULL_MODE_NONE ; // Allow drawing from inside model
                //shapePass->GetRenderState().mAlphaState.mAlphaTest = false ; // Render all opacities.
                shapePass->GetRenderState().mDepthState.mDepthWriteEnabled = false ; // Do not modify depth buffer
                shapePass->GetRenderState().mBlendState.SetAlpha() ; // Set up alpha blending
                shapePass->GetRenderState().mMaterialProperties.mAmbientColor = Vec4( 0.8f , 0.8f , 0.8f , 0.1f ) ; // Set translucent material
                shapePass->GetRenderState().mMaterialProperties.mDiffuseColor = Vec4( 0.8f , 0.8f , 0.8f , 0.1f ) ; // Set translucent material
            }
            //else
            //{
            //    shapePass->GetRenderState().mMaterialProperties.mAmbientColor = Vec4( 0.8f , 0.4f , 0.8f , 1.0f ) ;
            //    shapePass->GetRenderState().mMaterialProperties.mDiffuseColor = Vec4( 0.8f , 0.4f , 0.8f , 1.0f ) ;
            //}
        }
        else if( VertexDeclaration::POSITION_NORMAL == vertexFormat )
        {   // Vertices have positions and normals.
            // Hack: Assume this material is for fluid isosurface and set material to green shiny tranlucent.
            shapePass->GetRenderState().mRasterState.mCullMode = RasterStateS::CULL_MODE_NONE ; // Allow drawing from inside model
            shapePass->GetRenderState().mDepthState.mDepthWriteEnabled = false ; // Do not modify depth buffer
            shapePass->GetRenderState().mBlendState.SetAlpha() ; // Set up alpha blending
            shapePass->GetRenderState().mMaterialProperties.mAmbientColor = Vec4( 0.2f , 0.8f , 0.5f , 0.5f ) ; // Set translucent material
            shapePass->GetRenderState().mMaterialProperties.mDiffuseColor = Vec4( 0.2f , 0.8f , 0.5f , 0.5f ) ; // Set translucent material
            shapePass->GetRenderState().mMaterialProperties.mSpecularColor = Vec4( 0.8f , 0.2f , 0.5f , 1.0f ) ; // Set translucent material
            shapePass->GetRenderState().mMaterialProperties.mSpecularPower = 128 ;
        }

        modelNode   = NEW ModelNode( mSceneManager ) ;

        modelNode->SetPosition( position ) ;

        ModelData * modelData   = modelNode->NewModelData() ;
        MeshBase *  mesh        = modelData->NewMesh( renderApi ) ;

        ASSERT( shapeEffect->GetTechniques().Size() == 1 ) ;
        mesh->SetTechnique( shapeTechnique ) ;
        mSceneManager->GetRootSceneNode()->AdoptChild( modelNode ) ;
    }

    return modelNode ;
}




/** Create an inside-out sphere meant to be rendered from the inside.
*/
PeGaSys::Render::ModelNode * FluidScene::AddSkyModel()
{
    PERF_BLOCK( FluidScene__AddSkyModel ) ;

    using namespace PeGaSys ;
    using namespace PeGaSys::Render ;

    ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;

    VertexDeclaration::VertexFormatE vertexFormat = VertexDeclaration::POSITION_TEXTURE ;

    ModelNode * modelNode   = NULLPTR ;

    if( mRenderSystem && mRenderSystem->GetApi() )
    {
        ApiBase *   renderApi       = mRenderSystem->GetApi() ;
        Effect *    shapeEffect     = NEW Render::Effect() ;
        Technique * shapeTechnique  = shapeEffect->GetTechniques().Front() ;

        {   // Requested vertex format includes texture coordinates, so create a texture.
            Pass *          shapePass         = shapeTechnique->GetPasses().Front() ;
            shapePass->GetRenderState().mRasterState.mCullMode = RasterStateS::CULL_MODE_NONE ; // Allow drawing from inside model
            shapePass->AddTextureStage() ;
            TextureStage &  shapeTextureStage = shapePass->GetTextureStage( 0 ) ;
            shapeTextureStage.mTexture = mRenderSystem->GetApi()->NewTexture() ;
            {
                const unsigned width = 128 , height = 128 , numChannels = 4 , numPages = 1 ;
                Image image( width , height , numChannels , numPages ) ;
                const Vec4 blankColor( 1.0f , 1.0f , 1.0f , 1.0f ) ;
                const float gamma = 0.2f ; 
                ImgOpSeq_GradientNoise( image , gamma , blankColor ) ;
                shapeTextureStage.mTexture->CreateFromImages( & image , 1 ) ;
            }
            shapeTextureStage.mSamplerState.mAddressU = SamplerState::ADDRESS_REPEAT ;
            shapeTextureStage.mSamplerState.mAddressV = SamplerState::ADDRESS_CLAMP ;
            shapePass->GetRenderState().mMaterialProperties.mAmbientColor = Vec4( 0.5f , 0.5f , 0.5f , 1.0f ) ;
            shapePass->GetRenderState().mMaterialProperties.mDiffuseColor = Vec4( 0.5f , 0.5f , 0.5f , 1.0f ) ;
        }

        modelNode   = NEW ModelNode( mSceneManager ) ;

        modelNode->SetPosition( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
        Mat33 skyOrientation ;
        skyOrientation.SetRotationX( -0.5f * PI ) ;
        modelNode->SetOrientation( skyOrientation ) ;

        ModelData * modelData   = modelNode->NewModelData() ;
        MeshBase *  mesh        = modelData->NewMesh( renderApi ) ;

        // Create vertex and index buffers and populate with shape geometry.
        mesh->MakeSphere( renderApi , 100.0f , 64 , 128, vertexFormat ) ;

        ASSERT( shapeEffect->GetTechniques().Size() == 1 ) ;
        mesh->SetTechnique( shapeTechnique ) ;
        mSceneManager->GetRootSceneNode()->AdoptChild( modelNode ) ;
    }

    return modelNode ;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void MakeSimpleNoiseTexture( PeGaSys::Render::TextureStage::TexturePtr texture , int texWidth , int texHeight , int numTexPages , float gamma )
{
    PERF_BLOCK( MakeSimpleNoiseTexture ) ;

    PeGaSys::Image noise ;
    MakeSimpleNoiseImage( noise , texWidth , texHeight , numTexPages , gamma ) ;

    texture->CreateFromImages( & noise , 1 ) ;
}




void MakeDiagnosticVortonTexture( PeGaSys::Render::TextureStage::TexturePtr texture )
{
    PERF_BLOCK( MakeDiagnosticVortonTexture ) ;

    PeGaSys::Image noiseBall ;
    MakeDiagnosticVortonImage( noiseBall ) ;

    texture->CreateFromImages( & noiseBall , 1 ) ;
}




void MakeDiagnosticSimpleVortonTexture( PeGaSys::Render::TextureStage::TexturePtr texture )
{
    PERF_BLOCK( MakeDiagnosticSimpleVortonTexture ) ;

    PeGaSys::Image noiseBall ;
    MakeDiagnosticSimpleVortonImage( noiseBall ) ;

    texture->CreateFromImages( & noiseBall , 1 ) ;
}




void MakeDyeTexture( PeGaSys::Render::TextureStage::TexturePtr texture )
{
    PERF_BLOCK( MakeDyeTexture ) ;

    PeGaSys::Image noiseBall ;
    MakeDyeImage( noiseBall ) ;

    texture->CreateFromImages( & noiseBall , 1 ) ;
}




void MakeDiagnosticTracerTexture( PeGaSys::Render::TextureStage::TexturePtr texture )
{
    PERF_BLOCK( MakeDiagnosticTracerTexture ) ;

    PeGaSys::Image noiseBall ;
    MakeDiagnosticTracerImage( noiseBall ) ;

    texture->CreateFromImages( & noiseBall , 1 ) ;
}




void MakeSmokeTexture( PeGaSys::Render::TextureStage::TexturePtr texture )
{
    PERF_BLOCK( MakeSmokeTexture ) ;

    PeGaSys::Image noiseBall ;
    MakeSmokeImage( noiseBall ) ;

    texture->CreateFromImages( & noiseBall , 1 ) ;
}




void MakeFlameTexture( PeGaSys::Render::TextureStage::TexturePtr texture )
{
    PERF_BLOCK( MakeFlameTexture ) ;

    PeGaSys::Image noiseBall ;
    MakeFlameImage( noiseBall ) ;

    texture->CreateFromImages( & noiseBall , 1 ) ;
}




void MakeFuelTexture( PeGaSys::Render::TextureStage::TexturePtr texture )
{
    PERF_BLOCK( MakeFuelTexture ) ;

    PeGaSys::Image noiseBall ;
    MakeFuelImage( noiseBall ) ;

    texture->CreateFromImages( & noiseBall , 1 ) ;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




PeGaSys::Render::Pass * MakeDiagnosticTracerPass( PeGaSys::Render::System * renderSystem , PeGaSys::Render::Pass * pass )
{
    PERF_BLOCK( MakeDiagnosticTracerPass ) ;

    pass->AddTextureStage() ;
    PeGaSys::Render::TextureStage &  textureStage = pass->GetTextureStage( 0 ) ;
    textureStage.mTexture = renderSystem->GetApi()->NewTexture() ;
    MakeDiagnosticTracerTexture( textureStage.mTexture ) ;

    pass->GetRenderState().mDepthState.mDepthWriteEnabled = false ;
    pass->GetRenderState().mBlendState.SetAlpha() ;

    return pass ;
}




PeGaSys::Render::Pass * MakeDyePass( PeGaSys::Render::System * renderSystem , PeGaSys::Render::Pass * pass )
{
    PERF_BLOCK( MakeDyePass ) ;

    pass->AddTextureStage() ;
    PeGaSys::Render::TextureStage &  textureStage = pass->GetTextureStage( 0 ) ;
    textureStage.mTexture = renderSystem->GetApi()->NewTexture() ;
    MakeDyeTexture( textureStage.mTexture ) ;

    pass->GetRenderState().mDepthState.mDepthWriteEnabled = false ;
    pass->GetRenderState().mBlendState.SetAlpha() ;

    return pass ;
}




PeGaSys::Render::Pass * MakeFuelPass( PeGaSys::Render::System * renderSystem , PeGaSys::Render::Pass * pass )
{
    PERF_BLOCK( MakeFuelPass ) ;

    pass->AddTextureStage() ;
    PeGaSys::Render::TextureStage &  textureStage = pass->GetTextureStage( 0 ) ;
    textureStage.mTexture = renderSystem->GetApi()->NewTexture() ;
    MakeFuelTexture( textureStage.mTexture ) ;

    pass->GetRenderState().mDepthState.mDepthWriteEnabled = false ;
    pass->GetRenderState().mBlendState.SetAlpha() ;

    return pass ;
}




PeGaSys::Render::Pass * MakeSmokePass( PeGaSys::Render::System * renderSystem , PeGaSys::Render::Pass * pass )
{
    PERF_BLOCK( MakeSmokePass ) ;

    pass->AddTextureStage() ;
    PeGaSys::Render::TextureStage &  textureStage = pass->GetTextureStage( 0 ) ;
    textureStage.mTexture = renderSystem->GetApi()->NewTexture() ;
    MakeSmokeTexture( textureStage.mTexture ) ;

    pass->GetRenderState().mDepthState.mDepthWriteEnabled = false ;
    pass->GetRenderState().mBlendState.SetAlpha() ;

    return pass ;
}




PeGaSys::Render::Pass * MakeFlamePass( PeGaSys::Render::System * renderSystem , PeGaSys::Render::Pass * pass )
{
    PERF_BLOCK( MakeFlamePass ) ;

    pass->AddTextureStage() ;
    PeGaSys::Render::TextureStage &  textureStage = pass->GetTextureStage( 0 ) ;
    textureStage.mTexture = renderSystem->GetApi()->NewTexture() ;
    MakeFlameTexture( textureStage.mTexture ) ;

    pass->GetRenderState().mDepthState.mDepthWriteEnabled = false ;
    pass->GetRenderState().mBlendState.SetAdditive() ; // NOTE: ADDITIVE (not ALPHA)

    return pass ;
}




PeGaSys::Render::Pass * MakeDiagnosticVortonPass( PeGaSys::Render::System * renderSystem , PeGaSys::Render::Pass * pass )
{
    PERF_BLOCK( MakeDiagnosticVortonPass ) ;

    pass->AddTextureStage() ;
    PeGaSys::Render::TextureStage &  textureStage = pass->GetTextureStage( 0 ) ;
    textureStage.mTexture = renderSystem->GetApi()->NewTexture() ;
    MakeDiagnosticVortonTexture( textureStage.mTexture ) ;

    pass->GetRenderState().mDepthState.mDepthWriteEnabled = false ;
    pass->GetRenderState().mBlendState.SetAlpha() ;

    return pass ;
}




PeGaSys::Render::Pass * MakeDiagnosticSimpleVortonPass( PeGaSys::Render::System * renderSystem , PeGaSys::Render::Pass * pass )
{
    PERF_BLOCK( MakeDiagnosticSimpleVortonPass ) ;

    pass->AddTextureStage() ;
    PeGaSys::Render::TextureStage &  textureStage = pass->GetTextureStage( 0 ) ;
    textureStage.mTexture = renderSystem->GetApi()->NewTexture() ;
    MakeDiagnosticSimpleVortonTexture( textureStage.mTexture ) ;

    pass->GetRenderState().mDepthState.mDepthWriteEnabled = false ;
    pass->GetRenderState().mBlendState.SetAlpha() ;

    return pass ;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/// Wrap the already-created particle system with a Model.  Create meshes and materials.
PeGaSys::Render::ModelNode * FluidScene::AddFluidParticleSystemModel( ParticleSystem * fluidParticleSystem )
{
    using namespace PeGaSys ;
    using namespace PeGaSys::Render ;
    using namespace PeGaSys::ParticlesRender ;

    ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;

    using namespace PeGaSys::Render ;

    ApiBase * renderApi = mRenderSystem->GetApi() ;

    ParticlesRender::ParticlesRenderModel * pclRenderModel   = NULLPTR ;

    VertexDeclaration  vertexDeclaration( VertexDeclaration::POSITION_COLOR_TEXTURE ) ;

    if( mRenderSystem && mRenderSystem->GetApi() )
    {
        const size_t numGroups = fluidParticleSystem->GetNumGroups() ;
        ASSERT( 2 == numGroups ) ; // For this demo, assume 2 groups: 0 is vortons, 1 is tracers.

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // Populate VB fillers.
        mPclSysVertBufFillerContainers.Reserve( numGroups ) ; // Reserve VB Filler containers (one per group) for this system.

        for( size_t groupIndex = 0 ; groupIndex < numGroups ; ++ groupIndex )
        {   // For each particle group...
            // Create VB Fillers container for this particle group.
            mPclSysVertBufFillerContainers.PushBack( ParticlesRenderModel::PclGrpVertexBufferFillerContainer() ) ;
            ParticlesRenderModel::PclGrpVertexBufferFillerContainer & vbFillersPerGroup = mPclSysVertBufFillerContainers.Back() ;

            if( 0 == groupIndex )
            {   // This group contains tracers.
                // Order here matters, both for rendering order and for matching up with material assignment below.
                vbFillersPerGroup.PushBack( & mTracerDiagDyeFuelVertBufFiller ) ;
                vbFillersPerGroup.PushBack( & mTracerSmokeVertBufFiller ) ;
                vbFillersPerGroup.PushBack( & mTracerFlameVertBufFiller ) ;
            }
            else
            {   // This group contains vortons.
                ASSERT( 1 == groupIndex ) ;
                vbFillersPerGroup.PushBack( & mVortonVertBufFiller ) ;
            }
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////

        pclRenderModel   = NEW ParticlesRender::ParticlesRenderModel( mSceneManager ) ;
        pclRenderModel->AssociateWithParticleSystem( fluidParticleSystem , & mPclSysVertBufFillerContainers ) ;

        // Set vertex format and material for each pcl group
        for( size_t groupIndex = 0 ; groupIndex < numGroups ; ++ groupIndex )
        {   // For each particle group...
            // Create a new material.

//#error TODO: Control which passes are enabled depending on particle type and application render settings (i.e. debug flags).

            MeshBase * mesh = NULLPTR ;
            Render::Technique * shapeTechnique = NULLPTR ;

            if( 0 == groupIndex )
            {   // Current group contains tracers.
                ASSERT( pclRenderModel->GetNumVertBufFillersPerGroup( groupIndex ) == 3 ) ;
                {
                    mesh = pclRenderModel->GetMesh( groupIndex , 0 ) ; // Use [0] mesh of tracer group for diagnostic, dye and fuel rendering.
                    Render::Effect * shapeEffect = NEW Render::Effect( /* # techniques */ 1 , /* # passes */ 3 ) ;
                    shapeTechnique = shapeEffect->GetTechniques().Front() ;
                    mDiagnosticTracerPass       = MakeDiagnosticTracerPass      ( mRenderSystem , shapeTechnique->GetPasses().Front() ) ; // NOTE: This uses the pre-existing pass added by the Effect ctor.  All other passes below are created manually.
                    mDiagnosticTracerPass->SetActive( false ) ;
                    mDyePass                    = MakeDyePass                   ( mRenderSystem , shapeTechnique->AddNewPass() ) ;
                    mDyePass->SetActive( false ) ;
                    mFuelPass                   = MakeFuelPass                  ( mRenderSystem , shapeTechnique->AddNewPass()  ) ;
                    mFuelPass->SetActive( false ) ;
                    // Assign material (created above) with mesh associated with current particle group.
                    mesh->SetTechnique( shapeTechnique ) ;
                    // Declare vertex format for mesh associated with this particle group.
                    VertexBufferBase * vertBuf = mesh->NewVertexBuffer( renderApi ) ;
                    vertBuf->DeclareVertexFormat( vertexDeclaration ) ;
                }
                {
                    mesh = pclRenderModel->GetMesh( groupIndex , 1 ) ; // Use [1] mesh of tracer group for smoke.
                    Render::Effect * shapeEffect     = NEW Render::Effect( /* # techniques */ 1 , /* # passes */ 1 ) ;
                    shapeTechnique = shapeEffect->GetTechniques().Front() ;
                    mSmokePass                  = MakeSmokePass                 ( mRenderSystem , shapeTechnique->GetPasses().Front() ) ;
                    mSmokePass->SetActive( false ) ;
                    // Assign material (created above) with mesh associated with current particle group.
                    mesh->SetTechnique( shapeTechnique ) ;
                    // Declare vertex format for mesh associated with this particle group.
                    VertexBufferBase * vertBuf = mesh->NewVertexBuffer( renderApi ) ;
                    vertBuf->DeclareVertexFormat( vertexDeclaration ) ;
                }
                {
                    mesh = pclRenderModel->GetMesh( groupIndex , 2 ) ; // Use [2] mesh of tracer group for flame.
                    Render::Effect * shapeEffect = NEW Render::Effect( /* # techniques */ 1 , /* # passes */ 1 ) ;
                    shapeTechnique  = shapeEffect->GetTechniques().Front() ;
                    mFlamePass                  = MakeFlamePass                 ( mRenderSystem , shapeTechnique->GetPasses().Front()  ) ;
                    mFlamePass->SetActive( false ) ;
                    // Assign material (created above) with mesh associated with current particle group.
                    mesh->SetTechnique( shapeTechnique ) ;
                    // Declare vertex format for mesh associated with this particle group.
                    VertexBufferBase * vertBuf = mesh->NewVertexBuffer( renderApi ) ;
                    vertBuf->DeclareVertexFormat( vertexDeclaration ) ;
                }
            }
            else
            {   // Current group contains vortons.
                ASSERT( 1 == groupIndex ) ;

                ASSERT( pclRenderModel->GetNumVertBufFillersPerGroup( groupIndex ) == 1 ) ;

                mesh = pclRenderModel->GetMesh( groupIndex , 0 ) ; // Vorton group has only 1 mesh.

                Render::Effect *        shapeEffect     = NEW Render::Effect( /* # techniques */ 1 , /* # passes */ 2 ) ;
                
                shapeTechnique  = shapeEffect->GetTechniques().Front() ;

                mDiagnosticVortonPass       = MakeDiagnosticVortonPass      ( mRenderSystem , shapeTechnique->GetPasses().Front() ) ; // NOTE: This uses the pre-existing pass added by the Effect ctor.  All other passes below are created manually.
                mDiagnosticSimpleVortonPass = MakeDiagnosticSimpleVortonPass( mRenderSystem , shapeTechnique->AddNewPass() ) ;

                mDiagnosticVortonPass->SetActive( false ) ;
                mDiagnosticSimpleVortonPass->SetActive( false ) ;

                // Assign material (created above) with mesh associated with current particle group.
                mesh->SetTechnique( shapeTechnique ) ;

                // Declare vertex format for mesh associated with this particle group.
                VertexBufferBase * vertBuf = mesh->NewVertexBuffer( renderApi ) ;
                vertBuf->DeclareVertexFormat( vertexDeclaration ) ;
            }
        }

        // Place particle system in scene graph.
        mSceneManager->GetRootSceneNode()->AdoptChild( pclRenderModel ) ;
    }

    return pclRenderModel ;
}




void FluidScene::Clear()
{
    PERF_BLOCK( FluidScene__Clear ) ;

    mPclSysVertBufFillerContainers.Clear() ;

    // Remove all nodes from the scene.  This delete the nodes, making the cache variables (cleared below) invalid.
    mSceneManager->Clear() ;

    // Clear cache of scene nodes.
    mCamera = NULLPTR ;
    for( int i = 0 ; i < NUM_LIGHTS ; ++ i )
    {
        mLights[ i ] = NULLPTR ;
    }
    //for( int i = 0 ; i < NUM_MODELS ; ++ i )
    //{
    //    mModels[ i ] = NULLPTR ;
    //}
    mSkyBox                     = NULLPTR ;
    mFluidIsosurfaceModel       = NULLPTR ;
    mFluidParticleSystemModel   = NULLPTR ;

    // Remove viewports from target; The viewpors in this target referenced camera node, which was just erased above.
    mWindow->Clear() ;
}
