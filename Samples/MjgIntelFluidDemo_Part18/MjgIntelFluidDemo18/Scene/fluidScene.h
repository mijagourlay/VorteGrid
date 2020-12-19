/** \file fluidScene.h

    \brief Fluid render demo scene

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/
#ifndef PEGASYS_RENDER_FLUID_SCENE_H
#define PEGASYS_RENDER_FLUID_SCENE_H

#include <Render/Resource/vertexBuffer.h>

#include "Core/SpatialPartition/uniformGrid.h"

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
    }
}

/** Fluid demo scene
*/
class FluidScene
{
public:
    static const size_t NUM_LIGHTS              = 2 ;
    static const size_t NUM_MODELS              = 8 ;
    static const size_t NUM_SPECTRAL_COMPONENTS = 4 ;

    enum ShapeE
    {
        SHAPE_BOX       ,
        SHAPE_SPHERE    ,
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

    FluidScene( PeGaSys::Render::System * renderSystem ) ;
    ~FluidScene() ;

    void CreateLightsCameraSkybox() ;

    PeGaSys::Render::System * GetRenderSystem()
    {
        return mRenderSystem ;
    }

    PeGaSys::Render::SceneManagerBase * GetSceneManager()
    {
        return mSceneManager ;
    }

    PeGaSys::Render::Window * GetWindow()
    {
        return mWindow ;
    }

    PeGaSys::Render::Camera *    GetCamera()
    {
        return mCamera ;
    }

    void AnimateLights( double timeNow ) ;

    PeGaSys::Render::Light *     GetLight( size_t index ) ;

    LightAnimation & GetLightAnim( size_t index ) { return mLightAnims[ index ] ; }

    PeGaSys::Render::ModelNode * AddSphereModel( const Vec3 & position , float radius ) ;
    PeGaSys::Render::ModelNode * AddBoxModel( const Vec3 & position , const Vec3 & dimensions , bool isHole ) ;

    PeGaSys::Render::ModelNode * GetModel( size_t index ) ;
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

    void CreateLights() ;
    void CreateCamera() ;
    void CreateSkybox() ;
    void CreateFluidIsosurfaceModel() ;


    PeGaSys::Render::System *           mRenderSystem              ;
    PeGaSys::Render::SceneManagerBase * mSceneManager              ;
    PeGaSys::Render::Window *           mWindow                    ;

    // Caches of scene nodes:
    PeGaSys::Render::Camera *           mCamera                    ;
    PeGaSys::Render::ModelNode *        mSkyBox                    ;
    PeGaSys::Render::ModelNode *        mFluidIsosurfaceModel      ;
    PeGaSys::Render::ModelNode *        mModels[ NUM_MODELS ]      ;
    PeGaSys::Render::Light *            mLights[ NUM_LIGHTS ]      ;
    LightAnimation                      mLightAnims[ NUM_LIGHTS ]  ;
} ;

#endif
