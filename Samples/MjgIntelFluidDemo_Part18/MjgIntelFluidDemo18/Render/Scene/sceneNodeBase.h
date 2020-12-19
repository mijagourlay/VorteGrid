/** \file sceneNodeBase.h

    \brief Base class for a scene node.

    \author Copyright 2010-2012 MJG; All rights reserved.
*/
#ifndef SCENE_NODE_BASE_H
#define SCENE_NODE_BASE_H

#include "Core/Containers/slist.h"

#include "Core/Math/vec3.h"
#include "Core/Math/vec4.h"
#include "Core/Math/mat33.h"
#include "Core/Math/mat4.h"

#include "Render/Scene/iSceneNode.h"

// Macros ----------------------------------------------------------------------

#ifndef PRIVATE
#define PRIVATE private
#endif

#define ORIENTATION_AXIS_ANGLE  1   ///< Use axis-angle form to represent orientation
#define ORIENTATION_QUATERNION  2   ///< Use quaterion to represent orientation
#define ORIENTATION_MATRIX      3   ///< Use 3x3 matrix to represent orientation

#define RENDER_ORIENTATION_REPRESENTATION  ORIENTATION_MATRIX   ///< Which orientation representation to use

// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        typedef unsigned TypeId ;   ///< Type identifier, used for simple runtime type information.

        class ISceneManager ;

        /** Base class for a scene node.

            Scene nodes represent objects in a render scene.

            Specialized scene nodes include lights, cameras and models.

            Games can specialize SceneNodeBase to add custom scene nodes,
            such as particle systems and other visual effects, nodes to use for
            visualization and posable character models.
        */
        class SceneNodeBase : public ISceneNode
        {
            public:
                typedef SLIST< ISceneNode * >           SceneNodeContainerT ;
                typedef SceneNodeContainerT::Iterator   SceneNodeIteratorT  ;

                static const TypeId sTypeId = 'SNOD' ;

                SceneNodeBase( ISceneManager * sceneManager , TypeId typeId ) ;
                virtual ~SceneNodeBase() ;

                // ISceneNode methods
                virtual void            Update() ;

                /// Return parent of this node, or NULL if it is the root.
                virtual ISceneNode *    GetParent() { return mParent ; }

                virtual void            AdoptChild( ISceneNode * ) ;

                virtual void            OrphanChild( ISceneNode * ) ;

                virtual void            Visit( IVisitor * visitor ) ;
                virtual void            Clear( ClearDeletePolicy deleteNode = DELETE_NODES ) ;

                /// Do-nothing render method for base, which is used only as a container.
                virtual void            Render() {}

                /// Return type identifier for this node.
                const TypeId & GetTypeId() const { return mTypeId ; }

                void RenderChildren() ;

                /// Set world-space position that applies to this scene node as a translation.
                void SetPosition( const Vec3 & position )           { mPosition.x = position.x ; mPosition.y = position.y ; mPosition.z = position.z ; mPosition.w = 1.0f ; }

                /// Set world-space position that applies to this scene node as a translation.  w component must be 1.
                void SetPosition( const Vec4 & position )           { ASSERT( 1.0f == mPosition.w ) ; mPosition = position ; }

                /// Get world-space position that applies to this scene node as a translation.
                const Vec4 & GetPos4() const    { return mPosition ; }

                /// Get world-space position that applies to this scene node as a translation.
                const Vec3 & GetPosition() const    { return reinterpret_cast< const Vec3 & >( mPosition ) ; }

            #if RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_AXIS_ANGLE
                void SetOrientation( const Vec4 & orientation )   { mOrientation  = orientation ; }
                const Vec4 & GetOrientation() const { return mOrientation ; }
            #elif RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_MATRIX
                /// Set orientation matrix that applies to this scene node.
                void SetOrientation( const Mat33 & orientation )    { mOrientation  = orientation ; }

                /// Get orientation matrix that applies to this scene node.
                const Mat33 & GetOrientation() const { return mOrientation ; }
            #else
            #   error Invalid render orientation representation
            #endif

                /// Set scale that applies to this scene node. Only x, y, z components have meaning.
                void SetScale( const Vec3 & scale ) { mScale        = Vec4( scale.x , scale.y , scale.z , 1.0f ) ; }

                /// Return scale that applies to this scene node.
                const Vec3 & GetScale() const       { return reinterpret_cast< const Vec3 & >( mScale ) ; }

                /// Set local-to-world transform for this scene node position, orientation and scale..
                void SetLocalToWorld() ;

                /// Return scale that applies to this scene node.
                const Mat44 & GetLocalToWorld() const       { return mLocalToWorld ; }

            protected:
                /// Return SceneManager that manages this scene node.
                ISceneManager * GetSceneManager() const { return mSceneManager ; }

            PRIVATE:
                TypeId              mTypeId         ;   ///< Type identifier for this object

                ISceneManager *     mSceneManager   ;   ///< Manager that contains the scene with this node.  Non-owning cache.
                SceneNodeBase *     mParent         ;   ///< SceneNode that immediately contains this one.  Non-owning.
                SceneNodeContainerT mSceneNodes     ;   ///< Child nodes owned by this object.
                Vec4                mPosition       ;   ///< Position relative to parent node

            #if RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_AXIS_ANGLE
                Vec4                mOrientation    ;   ///< Orientation, in axis-angle form, relative to parent node
            #elif RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_MATRIX
                Mat33               mOrientation    ;   ///< Orientation, as a unitary transform, relative to parent node
            #else
            #   error Invalid render orientation representation
            #endif

                Vec4                mScale          ;   ///< Scale relative to parent node

                Mat44               mLocalToWorld   ;   ///< Local-to-world transform to place this node into world coordinates.  Assigned from mPosition, mOrientation and mScale.

            private:
                SceneNodeBase() ; // Disallow default construction.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
