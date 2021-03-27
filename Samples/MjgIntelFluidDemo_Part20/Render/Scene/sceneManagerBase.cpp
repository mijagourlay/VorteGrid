/** \file sceneManagerBase.cpp

    \brief Base class for a manager for a scene of nodes

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Scene/sceneManagerBase.h"

#include "Render/Scene/light.h"
#include "Render/Scene/model.h"
#include "Render/Device/api.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

// Types -----------------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Functor to update lights cache for a light receiver.

            Rendering a model entails shading it, which requires applying one or
            more lights to the model.  A scene can have more lights than the
            underlying render system can readily support, so the render system
            has to determine which lights are most appropriate for each model.
            The model caches which lights to use to render it.

            This LightVisitor determines, for each model, which lights to use
            to render it.
        */
        class LightVisitor : public ISceneNode::IVisitor
        {
        public:
            /** Initialize a light visitor for a given light receiver.

                \param lightReceiver    ModelNode that will receive light during rendering
            */
            LightVisitor( ModelNode & lightReceiver )
                : mLightReceiver( lightReceiver )
            {
                PERF_BLOCK( LightVisitor__LightVisitor ) ;

                ASSERT( lightReceiver.GetNumLights() == 0 ) ;
            }


            /** Update light cache of model to receive light.

                \param sceneNode    reference to scene node to visit,
                    which could be a light.

                This method is run on each node in a scene, including nodes
                which are not lights.  This method ignores nodes which are
                not lights.  For nodes that are lights, this method
                determines whether the light is appropriate for shading the
                model assigned to it at construction time, and if so,
                caches the given sceneNode (a light) for that model to use
                during the shading phase.
            */
            virtual void operator()( ISceneNode & sceneNode )
            {
                PERF_BLOCK( LightVisitor__invoke ) ;

                SceneNodeBase & snb = static_cast< SceneNodeBase & >( sceneNode ) ;
                const TypeId & typeId = snb.GetTypeId() ;
                if( Light::sTypeId == typeId )
                {   // Found a light.
                    const Light * light = & static_cast< Light & >( snb ) ;
                    mLightReceiver.UpdateLightsCache( light ) ;
                }
            }

        private:
            LightVisitor() ; ///< Disallow default construction.
            LightVisitor & operator=( const LightVisitor & ) ; ///< Disallow assignment.

            ModelNode & mLightReceiver  ;   ///< Address of model to light.
        } ;
    } ;
} ;




// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct base part of manager for a scene of nodes.
        */
        SceneManagerBase::SceneManagerBase( ApiBase * renderApi )
#pragma warning( push )
#pragma warning( disable: 4355 ) // 'this' used in base member initializer list
            : mRootSceneNode( this , SceneNodeBase::sTypeId )
            , mLightsContainer( this , SceneNodeBase::sTypeId )
            , mCamerasContainer( this , SceneNodeBase::sTypeId )
#pragma warning( pop )
            , mApi( renderApi )
            , mCurrentVirtualTimeInSeconds( 0.0 )
        {
            PERF_BLOCK( SceneManagerBase__SceneManagerBase ) ;
        }




        /** Destruct base part of manager for a scene of nodes.
        */
        SceneManagerBase::~SceneManagerBase()
        {
            PERF_BLOCK( SceneManagerBase__dtor ) ;

            Clear() ;
        }




        void SceneManagerBase::Clear()
        {
            PERF_BLOCK( SceneManagerBase__Clear ) ;

            mLightsContainer.Clear( ISceneNode::DO_NOT_DELETE_NOTES ) ;
            mCamerasContainer.Clear( ISceneNode::DO_NOT_DELETE_NOTES ) ;
            mRootSceneNode.Clear() ;
        }




        class RenderSceneVisitor : public ISceneNode::IVisitor
        {
        public:
            void operator()( ISceneNode & sceneNode )
            {
                PERF_BLOCK( RenderSceneVisitor__invoke ) ;

                sceneNode.Render() ;
            }
        } ;




        /** Render a scene with the given camera.
        */
        void SceneManagerBase::RenderScene( const Camera & camera , const double & currentVirtualTimeInSeconds )
        {
            PERF_BLOCK( SceneManagerBase__RenderScene ) ;

            mCurrentCamera = & camera ;
            CompileLights() ;
            RenderSceneVisitor renderSceneVisitor ;
            mRootSceneNode.Visit( & renderSceneVisitor ) ;
            mCurrentCamera = NULL ;
            mCurrentVirtualTimeInSeconds = currentVirtualTimeInSeconds ;
        }




        /** Convenience routine to create and add new a camera to the scene.

            \note   Since cameras are a special kind of node, unlikely to be rendered,
                    we might want to make AddCamera a method of ISceneManager instead of
                    just a utility routine in this class.  See comments in AddLight.
        */
        Camera * SceneManagerBase::AddCamera()
        {
            PERF_BLOCK( SceneManagerBase__AddCamera ) ;

            Camera * camera = NEW Camera( this ) ;
            mRootSceneNode.AdoptChild( camera ) ;
            mCamerasContainer.AdoptChild( camera ) ;
            return camera ;
        }




        /** Convenience routine to create and add a new light to the scene.

            \note   Searches on lights are a very different kind of search than other
                    scene lookups, because we want to know the closest N lights to each
                    model in the scene.  To perform quickly, that requires a specialized
                    algorithm.  Furthermore, the optimal spatial partitioning data
                    structure could be a different kind, or at least, could be a
                    different structure, to keep that structure free from models --
                    thereby potentially increasing lookup performance.  The upshot of
                    all this is that it might be worthwhile to make AddLight a method of
                    ISceneManager instead of just a utility routine in this base class.

                    The Scene would still need to have the light (that is, the light
                    would still need to know its place within a scene) so that the light
                    can transform as the child of another node.

                    Adding a light might entail these steps:
                    #   Create the light node.
                    #   Add it as a child of the appropriate scene node. (That could be the root.)
                    #   Inform the scene manager that the node is a Light, so that it can go into an appropriate spatial partition or container.
        */
        Light * SceneManagerBase::AddLight()
        {
            PERF_BLOCK( SceneManagerBase__AddLight ) ;

            Light * light = NEW Light( this ) ;
            mRootSceneNode.AdoptChild( light ) ;
            mLightsContainer.AdoptChild( light ) ;
            return light ;
        }




        class LightModelVisitor : public ISceneNode::IVisitor
        {
        public:
            LightModelVisitor( ISceneNode & lightsContainer )
                : mLightsContainer( lightsContainer )
            {
                PERF_BLOCK( LightModelVisitor__LightModelVisitor ) ;
            }

            /// Visit each model in the scene and compile its lights.
            void operator()( ISceneNode & lightReceiver )
            {
                PERF_BLOCK( LightModelVisitor__invoke ) ;

                SceneNodeBase & snb = static_cast< SceneNodeBase & >( lightReceiver ) ;
                if( snb.GetTypeId() == ModelNode::sTypeId )
                {   // This node is a model, therefore a potential light receiver.
                    ModelNode & mn = static_cast< ModelNode & >( snb ) ;
                    mn.ClearLightsCache() ;
                    LightVisitor lightVisitor( mn ) ;

                    mLightsContainer.Visit( & lightVisitor ) ;
                }
            }

        private:
            LightModelVisitor & operator=( const LightModelVisitor & ) ; /// Disallow assignment
            ISceneNode & mLightsContainer ;
        } ;




        /** Find best lights for each model in the scene.
        */
        void SceneManagerBase::CompileLights()
        {
            PERF_BLOCK( SceneManagerBase__CompileLights ) ;

            LightModelVisitor lightModelVisitor( mLightsContainer ) ;
            mRootSceneNode.Visit( & lightModelVisitor ) ;
        }




#if defined( _DEBUG )

        void PeGaSys_Render_SceneManagerBase_UnitTest( void )
        {
            DebugPrintf( "SceneManagerBase::UnitTest ----------------------------------------------\n" ) ;

            {
                SceneManagerBase sceneManagerBase( 0 ) ;
                Camera camera( & sceneManagerBase ) ;
                sceneManagerBase.RenderScene( camera , 0.0 ) ;
            }

            DebugPrintf( "SceneManagerBase::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
