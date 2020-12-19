/** \file fluidScene.cpp

    \brief Scene creation routines for InteSiVis.

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "inteSiVis.h"

#include "Image/image.h"
#include "Image/Operation/imgOpMakeNoise.h"
#include "Image/Operation/imgOpBlur.h"
#include "Image/imgOpSequences.h"

#include "Render/Scene/sceneManagerBase.h"
#include "Render/Scene/camera.h"
#include "Render/Scene/light.h"
#include "Render/Scene/model.h"

#include "Render/Device/window.h"

#include "Render/Resource/mesh.h"
#include "Render/Resource/marchingCubes.h"
#include "Render/Resource/material.h"
#include "Render/Resource/pass.h"
#include "Render/Resource/texture.h"

#include "Render/system.h"

#include "Core/Performance/perfBlock.h"
#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#include "Scene/fluidScene.h"

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------


/** Construct fluid demo scene.
*/
FluidScene::FluidScene( PeGaSys::Render::System * renderSystem )
    : mRenderSystem ( renderSystem )
    , mSceneManager( NULLPTR )
    , mWindow( NULLPTR )

    , mCamera( NULLPTR )
    , mSkyBox( NULLPTR )
    , mFluidIsosurfaceModel( NULLPTR )
{
    PERF_BLOCK( FluidScene__FluidScene ) ;

    memset( mModels     , 0 , sizeof( mModels     ) ) ;
    memset( mLights     , 0 , sizeof( mLights     ) ) ;
    memset( mLightAnims , 0 , sizeof( mLightAnims ) ) ;

    using namespace PeGaSys::Render ;

#if defined( WIN32 ) && defined( CreateWindow )
#   undef CreateWindow
#endif

    mWindow         = (CreateWindow)() ;  // Parentheses around CreateWindow to protect against expanding macro defined in <windows.h>.
    mSceneManager   = CreateSceneManager() ;

    CreateLightsCameraSkybox() ;

    Viewport *              viewport        = mWindow->GetViewports().Front() ;
    DiagnosticTextOverlay & diagTextOver    = viewport->GetDiagnosticTextOverlay() ;
    diagTextOver.Clear() ;
    for( int line = 0 ; line < 256 ; ++ line )
    {
        diagTextOver.AppendLine( "Hello, World! %i" , line ) ;
    }
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

    This also creates a viewport, that contains a diagnostic text overlay.
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




void FluidScene::CreateLightsCameraSkybox()
{
    PERF_BLOCK( FluidScene__CreateLightsCameraSkybox ) ;

    CreateCamera() ;
    CreateLights() ;
    CreateSkybox() ;
    CreateFluidIsosurfaceModel() ;
}




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




PeGaSys::Render::ModelNode * FluidScene::GetModel( size_t index )
{
    PERF_BLOCK( FluidScene__GetModel ) ;

    ASSERT( index < NUM_MODELS ) ;
    return mModels[ index ] ;
}




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

        if(     ( VertexDeclaration::POSITION_TEXTURE              == vertexFormat )
            ||  ( VertexDeclaration::POSITION_NORMAL_TEXTURE       == vertexFormat )
            ||  ( VertexDeclaration::POSITION_COLOR_TEXTURE        == vertexFormat )
            ||  ( VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE == vertexFormat )
            )
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

        if(     ( VertexDeclaration::POSITION_TEXTURE              == vertexFormat )
            ||  ( VertexDeclaration::POSITION_NORMAL_TEXTURE       == vertexFormat )
            ||  ( VertexDeclaration::POSITION_COLOR_TEXTURE        == vertexFormat )
            ||  ( VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE == vertexFormat )
            )
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




void FluidScene::Clear()
{
    PERF_BLOCK( FluidScene__Clear ) ;

    // Remove all nodes from the scene.  This delete the nodes, making the cache variables invalid.
    mSceneManager->Clear() ;

    // Clear cache of scene nodes.
    mCamera = NULLPTR ;
    for( int i = 0 ; i < NUM_LIGHTS ; ++ i )
    {
        mLights[ i ] = NULLPTR ;
    }
    for( int i = 0 ; i < NUM_MODELS ; ++ i )
    {
        mModels[ i ] = NULLPTR ;
    }
    mSkyBox = NULLPTR ;
    mFluidIsosurfaceModel = NULLPTR ;

    // Remove viewports from target; The viewpors in this target referenced camera node, which was just erased above.
    mWindow->Clear() ;
}
