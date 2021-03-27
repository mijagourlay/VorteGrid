/** \file sceneManagerBase.h

    \brief Base class for a manager for a scene of nodes

    \author Copyright 2010-2011 MJG; All rights reserved.
*/
#ifndef SCENE_MANAGER_BASE_H
#define SCENE_MANAGER_BASE_H

#include "Core/Containers/vector.h"

#include "Render/Scene/sceneNodeBase.h"

#include "Render/Scene/iSceneManager.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        // Forward declarations
        class ApiBase   ;
        class Light     ;
        class ModelNode ;

        /** Base class for a manager for a scene of nodes.
        */
        class SceneManagerBase : public ISceneManager
        {
                typedef VECTOR< ISceneNode * >          SceneNodeContainerT ;
                typedef SceneNodeContainerT::Iterator   SceneNodeIteratorT  ;

            public:
                explicit SceneManagerBase( ApiBase * renderApi ) ;
                virtual ~SceneManagerBase() ;

                // ISceneManager methods

                virtual ISceneNode *    GetRootSceneNode()          { return & mRootSceneNode    ; }
                virtual ISceneNode *    GetAuxLightsContainer()     { return & mLightsContainer  ; }
                virtual ISceneNode *    GetAuxCamerasContainer()    { return & mCamerasContainer ; }

                /// Return address of camera currently used to render scene.
                virtual const Camera *  GetCurrentCamera() const    { return mCurrentCamera ; }

                /// Return current virtual time in seconds
                virtual double          GetCurrentVirtualTimeInSeconds() const    { return mCurrentVirtualTimeInSeconds ; }

                /// Return address of underlying rendering API object that this manager uses to render its scene.
                virtual ApiBase *       GetApi() const { return mApi ; }

                virtual void            RenderScene( const Camera & camera , const double & currentVirtualTimeInSeconds ) ;

                virtual void            Clear() ;

                // Utility methods
                Camera * AddCamera() ;
                Light  * AddLight() ;

            private:
                void CompileLights() ;

                SceneNodeBase   mRootSceneNode                  ;   ///< Ancestor of all scene nodes.
                SceneNodeBase   mLightsContainer                ;   ///< Proxy scene shallow-containing each light node.
                SceneNodeBase   mCamerasContainer               ;   ///< Proxy scene shallow-containing each camera node.
                ApiBase *       mApi                            ;   ///< Non-owned address of low-level render system device
                const Camera  * mCurrentCamera                  ;   ///< Address of camera currently being used to render scene.
                double          mCurrentVirtualTimeInSeconds    ;   ///< Current virtual time in seconds.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
