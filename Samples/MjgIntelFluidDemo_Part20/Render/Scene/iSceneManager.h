/** \file iSceneManager.h

    \brief Interface for a scene manager

    \author Copyright 2010-2012 MJG; All rights reserved.
*/
#ifndef I_SCENE_MANAGER_H
#define I_SCENE_MANAGER_H

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class ApiBase       ;
        class ISceneNode    ;
        class Camera        ;

        /** Interface for a scene manager.

            A scene manager processes renderable nodes at a relatively high
            level.  This processing might entail object-level frustum-culling,
            occlusion, deciding which lights affect which geometry, deciding
            which geometry level-of-detail to use for a particular model, or
            establishing scene-wide parameters for shadow-casting.

            The scene manager typically will use some spatial partitioning
            data structures and algorithms; the details depend on the particular
            implementation of scene manager.

            It would be up to the scene manager, for example, to implement a
            portal-style rendering algorithm, and to decide whether to use BSP
            trees or octrees, or simply to render everything on every frame.
            The majority of client code should neither know nor care about such
            details (except perhaps when instantiating the scene manager, if
            there is a choice among multiple of them).  Furthermore, the
            RenderQueue should be totally independent of all aspects details of
            the concrete scene manager.

            The scene manager can also be queried to provide feedback about
            which objects were rendered and, if not, why; where they behind the
            camera, beyond the far-clip plane or hidden behind some other
            object?  And because it likely spatially partitions all nodes, a
            scene manager could also potentially provide information about
            ray-casting, such as for picking.

            The input to a scene manager is data that is, for the most part,
            platform-independent.  Contents of a RenderQueue, in contrast,
            would typically contain platform-specific data also specific to
            a particular lower-level rendering API (such as DirectX versus
            OpenGL).  So a concrete SceneManager is expected not to have any
            platform- or API-specific code.

            The result of processing a scene would typically result in a
            populating one or more RenderQueues with vertex buffers and render
            states.  The RenderQueues themselves might further sort objects
            by material or there could be a RenderQueue per material.  There
            might be separate RenderQueues for opaque and translucent objects.
            World geometry and UI might also reside in separate RenderQueues.

            It would be hypothetically possible to implement the scene manager
            differently, where there would be a single kind of scene manager,
            and its various policies would be implemented via abstractions.
            But with this approach, a given concrete scene manager implements
            all policy, partly for the sake of reducing the number of virtual
            calls.

            \see ISceneNode
        */
        class ISceneManager
        {
            public:
                virtual ~ISceneManager() {}

                /// Get root scene node that contains all other nodes in this scene.
                virtual ISceneNode *    GetRootSceneNode() = 0 ;

                /// Get auxiliary container for all lights in scene.
                /// The main scene also contains all of these lights.  This is
                /// strictly a secondary, non-owning container.
                virtual ISceneNode *    GetAuxLightsContainer() = 0 ;

                /// Get auxiliary container for all cameras in scene.
                /// The main scene also contains all of these cameras.  This is
                /// strictly a secondary, non-owning container.
                virtual ISceneNode *    GetAuxCamerasContainer() = 0 ;

                /// Return address of camera currently used to render scene.
                virtual const Camera *  GetCurrentCamera() const = 0 ;

                /// Return current virtual time in seconds
                virtual double          GetCurrentVirtualTimeInSeconds() const = 0 ;

                /// Return address of underlying rendering API object that this manager uses to render its scene.
                virtual ApiBase *       GetApi() const = 0 ;

                /// Render a scene with the given camera.
                virtual void            RenderScene( const Camera & camera , const double & currentVirtualTimeInSeconds ) = 0 ;

                /// Clear this scene.
                virtual void            Clear() = 0 ;
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
