/** \file spinningCubes.h

    \brief Spinning cube render demo scene

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/
#ifndef PEGASYS_RENDER_DEMO_SPINNING_CUBE_SCENE_H
#define PEGASYS_RENDER_DEMO_SPINNING_CUBE_SCENE_H

#include <Render/Resource/vertexBuffer.h>

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

struct Vec3 ;

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

        namespace Demo
        {
            /** Spinning cubes demo scene
            */
            class SpinningCubesScene
            {
            public:
                static const size_t NUM_LIGHTS = 2 ;
                static const size_t NUM_MODELS = 8 ;

                enum ShapeE
                {
                    SHAPE_BOX       ,
                    SHAPE_SPHERE    ,
                } ;

                SpinningCubesScene( System * renderSystem ) ;
                ~SpinningCubesScene() ;

                System * GetRenderSystem()
                {
                    return mRenderSystem ;
                }

                SceneManagerBase * GetSceneManager()
                {
                    return mSceneManager ;
                }

                Window * GetWindow()
                {
                    return mWindow ;
                }

                Camera *    GetCamera()
                {
                    return mCamera ;
                }

                Light *     GetLight( size_t index ) ;
                ModelNode * GetModel( size_t index ) ;

            private:
                Window *            CreateAWindow() ;
                SceneManagerBase *  CreateSceneManager() ;
                Camera *            AddCamera() ;
                Light *             AddLight( const Vec3 & direction ) ;
                ModelNode *         AddDemoModel( const Vec3 & position , VertexDeclaration::VertexFormatE vertexFormat , ShapeE shape ) ;
                ModelNode *         AddSkyModel() ;

                System *            mRenderSystem           ;
                SceneManagerBase *  mSceneManager           ;
                Window *            mWindow                 ;
                Camera *            mCamera                 ;
                Light *             mLights[ NUM_LIGHTS ]   ;
                ModelNode *         mModels[ NUM_MODELS ]   ;
                ModelNode *         mSkyBox                 ;
            } ;
        } ;
    } ;
} ;

#endif
