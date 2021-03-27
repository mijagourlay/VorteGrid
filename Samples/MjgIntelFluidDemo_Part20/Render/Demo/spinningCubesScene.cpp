/** \file spinningCubes.cpp

    \brief Spinning cube render demo scene

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#include "Render/Demo/spinningCubesScene.h"

#include "Render/Scene/sceneManagerBase.h"
#include "Render/Scene/camera.h"
#include "Render/Scene/light.h"
#include "Render/Scene/model.h"

#include "Render/Device/window.h"

#include "Render/Resource/mesh.h"
#include "Render/Resource/material.h"
#include "Render/Resource/pass.h"
#include "Render/Resource/texture.h"

#include "Render/system.h"

#include <Image/image.h>
#include <Image/Operation/imgOpMakeNoise.h>
#include <Image/Operation/imgOpBlur.h>
#include <Image/imgOpSequences.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {
        namespace Demo {

            /** Construct spinning cube demo scene.

                Create a test scene, to demonstrate how to create a scene.
            */
            SpinningCubesScene::SpinningCubesScene( System * renderSystem )
                : mRenderSystem ( renderSystem )
                , mSceneManager( 0 )
                , mWindow( 0 )
                , mCamera( 0 )
            {
                PERF_BLOCK( SpinningCubesScene__SpinningCubesScene ) ;

                mWindow         = CreateAWindow() ;
                mSceneManager   = CreateSceneManager() ;

                // Add a camera to view the scene.  This also creates a viewport, that contains a diagnostic text overlay.
                mCamera = AddCamera() ;
                Viewport *              viewport        = mWindow->GetViewports().Front() ;
                DiagnosticTextOverlay & diagTextOver    = viewport->GetDiagnosticTextOverlay() ;

                mLights[ 0 ] = AddLight( Vec3( 0.0f , 0.0f ,  5.0f ) ) ;
                mLights[ 1 ] = AddLight( Vec3( 0.0f , 0.0f , -5.0f ) ) ;

                // Add one object per vertex format.
                mModels[ 0 ] = AddDemoModel( Vec3( 0.0f ,  0.0f , -3.0f ) , VertexDeclaration::POSITION                      , SHAPE_BOX ) ;

                mModels[ 1 ] = AddDemoModel( Vec3( 0.0f , -2.0f , -1.0f ) , VertexDeclaration::POSITION_NORMAL               , SHAPE_BOX ) ;
                mModels[ 2 ] = AddDemoModel( Vec3( 0.0f ,  0.0f , -1.0f ) , VertexDeclaration::POSITION_COLOR                , SHAPE_BOX ) ;
                mModels[ 3 ] = AddDemoModel( Vec3( 0.0f ,  2.0f , -1.0f ) , VertexDeclaration::POSITION_TEXTURE              , SHAPE_BOX ) ;

                mModels[ 4 ] = AddDemoModel( Vec3( 0.0f , -2.0f ,  1.0f ) , VertexDeclaration::POSITION_NORMAL_COLOR         , SHAPE_SPHERE ) ;
                mModels[ 5 ] = AddDemoModel( Vec3( 0.0f ,  0.0f ,  1.0f ) , VertexDeclaration::POSITION_NORMAL_TEXTURE       , SHAPE_SPHERE ) ;
                mModels[ 6 ] = AddDemoModel( Vec3( 0.0f ,  2.0f ,  1.0f ) , VertexDeclaration::POSITION_COLOR_TEXTURE        , SHAPE_SPHERE ) ;

                mModels[ 7 ] = AddDemoModel( Vec3( 0.0f ,  0.0f ,  3.0f ) , VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE , SHAPE_SPHERE ) ;

                mSkyBox = AddSkyModel() ;

                diagTextOver.Clear() ;
                for( int line = 0 ; line < 256 ; ++ line )
                {
                    diagTextOver.AppendLine( "Hello, World! %i" , line ) ;
                }
            }




            /** Destruct spinning cube demo scene.
            */
            SpinningCubesScene::~SpinningCubesScene()
            {
                PERF_BLOCK( SpinningCubesScene__dtor ) ;
            }




            Light *     SpinningCubesScene::GetLight( size_t index )
            {
                PERF_BLOCK( SpinningCubesScene__GetLight ) ;

                ASSERT( index < NUM_LIGHTS ) ;
                return mLights[ index ] ;
            }




            ModelNode * SpinningCubesScene::GetModel( size_t index )
            {
                PERF_BLOCK( SpinningCubesScene__GetModel ) ;

                ASSERT( index < NUM_MODELS ) ;
                return mModels[ index ] ;
            }




            /** Create a window for rendering the scene.
            */
            Window * SpinningCubesScene::CreateAWindow()
            {
                PERF_BLOCK( SpinningCubesScene__CreateAWindow ) ;

                ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;
                if( mRenderSystem && mRenderSystem->GetApi() )
                {
                    Window * mainWindow = mRenderSystem->GetApi()->NewWindow( mRenderSystem ) ;
                    mainWindow->Create() ;
                    mRenderSystem->AddTarget( mainWindow ) ;
                    return mainWindow ;
                }
                return NULL ;
            }




            SceneManagerBase * SpinningCubesScene::CreateSceneManager()
            {
                PERF_BLOCK( SpinningCubesScene__CreateSceneManager ) ;

                ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;

                if( mRenderSystem && mRenderSystem->GetApi() )
                {
                    SceneManagerBase * sceneManager = NEW SceneManagerBase( mRenderSystem->GetApi() ) ;
                    return sceneManager ;
                }
                return NULL ;
            }




            /** Add a camera to the scene.
            */
            Camera * SpinningCubesScene::AddCamera()
            {
                PERF_BLOCK( SpinningCubesScene__AddCamera ) ;

                ASSERT( mSceneManager && mWindow ) ;

                if( mSceneManager )
                {
                    Camera * camera = mSceneManager->AddCamera() ;
                    camera->SetPosition( Vec3( 5.0f , 0.0f , 0.0f ) ) ;
                    mWindow->AddViewport( camera ) ;
                    return camera ;
                }
                return NULL ;
            }




            /** Add a light node to the scene.
            */
            Light * SpinningCubesScene::AddLight( const Vec3 & direction )
            {
                PERF_BLOCK( SpinningCubesScene__AddLight ) ;

                ASSERT( mSceneManager ) ;

                if( mSceneManager )
                {
                    Render::Light * light = mSceneManager->AddLight() ;
                    //light->SetLightType( Render::Light::POINT ) ;
                    light->SetDirection( direction ) ;
                    return light ;
                }
                return NULL ;
            }




            ModelNode * SpinningCubesScene::AddDemoModel( const Vec3 & position , const PeGaSys::Render::VertexDeclaration::VertexFormatE vertexFormat , ShapeE shape )
            {
                PERF_BLOCK( SpinningCubesScene__AddDemoModel ) ;

                ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;

                ModelNode * modelNode   = NULLPTR ;

                if( mRenderSystem && mRenderSystem->GetApi() )
                {
                    ApiBase *   renderApi       = mRenderSystem->GetApi() ;
                    Effect *    shapeEffect     = NEW Render::Effect() ;
                    Technique * shapeTechnique  = shapeEffect->GetTechniques().Front() ;

                    if( VertexDeclaration( vertexFormat ).HasTextureCoordinates() )
                    {   // Requested vertex format includes texture coordinates, so create a texture.
                        Pass *          shapePass         = shapeTechnique->GetPasses().Front() ;
                        shapePass->AddTextureStage() ;
                        TextureStage &  shapeTextureStage = shapePass->GetTextureStage( 0 ) ;
                        shapeTextureStage.mTexture = mRenderSystem->GetApi()->NewTexture() ;
                        {
                            Image image( 256 , 256 , 4 , 1 ) ;
                            ImgOpMakeNoise::MakeNoise( image ) ;
                            ImgOpBlur::Blur( image , 1 ) ;
                            shapeTextureStage.mTexture->CreateFromImages( & image , 1 ) ;
                        }
                        shapeTextureStage.mSamplerState.mAddressU = SamplerState::ADDRESS_REPEAT ;
                        shapeTextureStage.mSamplerState.mAddressV = SamplerState::ADDRESS_CLAMP  ;
                    }

                    modelNode   = NEW ModelNode( mSceneManager ) ;

                    modelNode->SetPosition( position ) ;

                    ModelData * modelData   = modelNode->NewModelData() ;
                    MeshBase *  mesh        = modelData->NewMesh( renderApi ) ;

                    // Create vertex and index buffers and populate with shape geometry.
                    if( SHAPE_BOX == shape )
                    {
                        mesh->MakeBox( renderApi , Vec3( 0.2f , 0.5f , 1.0f ) , vertexFormat ) ;
                    }
                    else
                    {
                        ASSERT( SHAPE_SPHERE == shape ) ;
                        mesh->MakeSphere( renderApi , 0.5f , 8 , 16, vertexFormat ) ;
                    }

                    ASSERT( shapeEffect->GetTechniques().Size() == 1 ) ;
                    mesh->SetTechnique( shapeTechnique ) ;
                    mSceneManager->GetRootSceneNode()->AdoptChild( modelNode ) ;
                }

                return modelNode ;
            }




            ModelNode * SpinningCubesScene::AddSkyModel()
            {
                PERF_BLOCK( SpinningCubesScene__AddSkyModel ) ;

                ASSERT( mRenderSystem && mRenderSystem->GetApi() ) ;

                PeGaSys::Render::VertexDeclaration::VertexFormatE vertexFormat = VertexDeclaration::POSITION_COLOR_TEXTURE ;

                ModelNode * modelNode   = NULLPTR ;

                if( mRenderSystem && mRenderSystem->GetApi() )
                {
                    ApiBase *   renderApi       = mRenderSystem->GetApi() ;
                    Effect *    shapeEffect     = NEW Render::Effect() ;
                    Technique * shapeTechnique  = shapeEffect->GetTechniques().Front() ;

                    {   // Requested vertex format includes texture coordinates, so create a texture.
                        Pass *          shapePass         = shapeTechnique->GetPasses().Front() ;
                        shapePass->GetRenderState().mRasterState.mCullMode = RasterStateS::CULL_MODE_NONE ;
                        shapePass->AddTextureStage() ;
                        TextureStage &  shapeTextureStage = shapePass->GetTextureStage( 0 ) ;
                        shapeTextureStage.mTexture = mRenderSystem->GetApi()->NewTexture() ;
                        {
                            const unsigned width = 128 , height = 128 , numChannels = 4 , numPages = 1 ;
                            Image image( width , height , numChannels , numPages ) ;
                            const Vec4 blankColor( 1.0f , 1.0f , 1.0f , 1.0f ) ;
                            const float gamma = 0.2f ; 
                            PeGaSys::ImgOpSeq_GradientNoise( image , gamma , blankColor ) ;
                            shapeTextureStage.mTexture->CreateFromImages( & image , 1 ) ;
                        }
                        shapeTextureStage.mSamplerState.mAddressU = SamplerState::ADDRESS_REPEAT ;
                        shapeTextureStage.mSamplerState.mAddressV = SamplerState::ADDRESS_CLAMP  ;
                    }

                    modelNode   = NEW ModelNode( mSceneManager ) ;

                    modelNode->SetPosition( Vec3( 0.0f , 0.0f , 0.0f ) ) ;

                    ModelData * modelData   = modelNode->NewModelData() ;
                    MeshBase *  mesh        = modelData->NewMesh( renderApi ) ;

                    // Create vertex and index buffers and populate with shape geometry.
                    mesh->MakeSphere( renderApi , 100.0f , 8 , 16, vertexFormat ) ;

                    ASSERT( shapeEffect->GetTechniques().Size() == 1 ) ;
                    mesh->SetTechnique( shapeTechnique ) ;
                    mSceneManager->GetRootSceneNode()->AdoptChild( modelNode ) ;
                }

                return modelNode ;
            }

        } ;
    } ;
} ;
