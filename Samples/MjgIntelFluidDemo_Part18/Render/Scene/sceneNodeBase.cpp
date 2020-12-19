/** \file sceneNodeBase.cpp

    \brief Base class for a scene node.

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Scene/sceneNodeBase.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct base part of a scene node.

            \param sceneManager Scene manager that contains this node.

            \param typeId   Which type of node this is.
                Scene nodes can be assigned a "type" which can be used, for example, by
                scene node visitors (ISceneNode::IVisitor) to determine whether that
                visitor should operate on the node it visits.

        */
        SceneNodeBase::SceneNodeBase( ISceneManager * sceneManager , TypeId typeId )
            : mTypeId( typeId )
            , mSceneManager( sceneManager )
            , mParent( 0 )
            , mPosition( 0.0f , 0.0f , 0.0f , 1.0f )
#   if RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_AXIS_ANGLE
            , mOrientation( 0.0f , 0.0f , 0.0f , 0.0f )
#   elif RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_MATRIX
            , mOrientation( Mat33_xIdentity )
#   else
#       error Invalid render orientation representation
#   endif
            , mScale( 1.0f , 1.0f , 1.0f , 0.0f )
        {
            PERF_BLOCK( SceneNodeBase__SceneNodeBase ) ;
        }




        /** Destruct base part of a scene node.
        */
        SceneNodeBase::~SceneNodeBase()
        {
            PERF_BLOCK( SceneNodeBase__dtor ) ;

            Clear() ;
        }




        /** Update this node's absolute transformation based on its parent.

            \todo   This routine should not be here. Rather, an update-visitor should be passed into Visit.
        */
        void SceneNodeBase::Update()
        {
            PERF_BLOCK( SceneNodeBase__Update ) ;

            ASSERT( static_cast< SceneNodeBase * >( this )->GetTypeId() != sTypeId ) ; // Derived class must reassign type id.
        }




        /** Render children of this node.
        */
        void SceneNodeBase::RenderChildren()
        {
            PERF_BLOCK( SceneNodeBase__RenderChildren ) ;

            ASSERT( static_cast< SceneNodeBase * >( this )->GetTypeId() != sTypeId ) ; // Derived class must reassign type id.
            for( SceneNodeIteratorT iter = mSceneNodes.Begin() ; iter != mSceneNodes.End() ; ++ iter )
            {
                ISceneNode * & sceneNode = * iter ;
                ASSERT( static_cast< SceneNodeBase * >( sceneNode )->GetTypeId() != sTypeId ) ; // Derived class must reassign type id.
                sceneNode->Render() ;
            }
        }




        /** Add the given node as a child to this one, and take ownership of it.
        */
        /* virtual */ void SceneNodeBase::AdoptChild( ISceneNode * child )
        {
            PERF_BLOCK( SceneNodeBase__AdoptChild ) ;

            mSceneNodes.PushBack( child ) ;
        }




        /** Remove the given node, a child of this one, and relinquish ownership of it.
        */
        /* virtual */ void SceneNodeBase::OrphanChild( ISceneNode * child )
        {
            PERF_BLOCK( SceneNodeBase__OrphanChild ) ;

            /* SceneNodeContainerT::Iterator iter = */ mSceneNodes.Remove( child ) ;
            //ASSERT( iter != mSceneNodes.End() ) ;
        }




        /** Visit this node, then its children.
        */
        /* virtual */ void SceneNodeBase::Visit( IVisitor * visitor )
        {
            PERF_BLOCK( SceneNodeBase__Visit ) ;

            visitor->operator()( * this ) ;
            for( SceneNodeIteratorT iter = mSceneNodes.Begin() ; iter != mSceneNodes.End() ; ++ iter )
            {
                ISceneNode * & sceneNode = * iter ;
                ASSERT( static_cast< SceneNodeBase * >( sceneNode )->GetTypeId() != sTypeId ) ; // Derived class must reassign type id.
                visitor->operator()( * sceneNode ) ;
            }
        }




        /** Clear children from this node.

            \param deleteNode Whether to delete node or just remove it.
                Usually, a SceneNode owns its children, in the sense that the parent
                deletes its children.  But some SceneNodes, (for example the lights
                and cameras containers in the scene manager) are redundant auxiliary
                shallow containers that do not own their contents; they exist only
                to idenitfy certain kinds of nodes.  That is useful to optimize
                iterating over those kinds of nodes, and it is useful to use ISceneNode
                as the container since then ISceneNode::Visit can operate with it.
        */
        void SceneNodeBase::Clear( ClearDeletePolicy deleteNode )
        {
            PERF_BLOCK( SceneNodeBase__Clear ) ;

            for( SceneNodeIteratorT iter = mSceneNodes.Begin() ; iter != mSceneNodes.End() ; ++ iter )
            {
                ISceneNode * const & sceneNode = * iter ;
                ASSERT( static_cast< SceneNodeBase * >( sceneNode )->GetTypeId() != sTypeId ) ; // Derived class must reassign type id.
                sceneNode->Clear( deleteNode ) ;
                if( DELETE_NODES == deleteNode )
                {
                    delete sceneNode ;
                }
            }
            mSceneNodes.Clear() ;
        }




#if 0 // For reference -- an implementation of SetLocalToWorld that uses D3DXMath

        static SetLocalToWorld_D3D( SceneNodeBase & sceneNode )
        {
            D3DXMATRIX xLocalToWorldOrig ;
            HROK( g_pd3dDevice->GetTransform( D3DTS_WORLD, reinterpret_cast< D3DMATRIX *>( & xLocalToWorldOrig ) ) ) ; // save original matrix to restore it

            // Construct transform matrix for this object
            D3DXMATRIX xLocalToWorld ;
            D3DXMatrixIdentity( & xLocalToWorld ) ;

            // Apply translation.
            const Vec3 & vPosition = sceneNode.GetPosition() ;
            D3DXMATRIX xTrans ;
            D3DXMatrixTranslation( & xTrans , vPosition.x , vPosition.y , vPosition.z ) ;
            xLocalToWorld = xTrans * xLocalToWorld ;

            // Apply rotation.
#if RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_AXIS_ANGLE
            Vec4        axisAngle   = sceneNode.GetOrientation() ;
            const float angle       = axisAngle.Magnitudev3() ;
            {
                Vec4 vAxis( axisAngle ) ;
                vAxis.Normalizev3() ;
                D3DXMATRIX xRotate ;
                D3DXMatrixRotationAxis( & xRotate , (CONST D3DXVECTOR3 *) & vAxis , angle ) ;
                xLocalToWorld = xRotate * xLocalToWorld ;
            }
#elif RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_MATRIX
            {
                D3DXMATRIX xRotate ;
                ((Mat44&)xRotate).SetRotation( sceneNode.GetOrientation() ) ;
                xLocalToWorld = xRotate * xLocalToWorld ;
            }
#else
#   error Invalid render orientation representation
#endif

            // Apply scaling.
            {
                const Vec3 & vScale = sceneNode.GetScale() ;
                D3DXMATRIX xScale ;
                D3DXMatrixScaling( & xScale , vScale.x , vScale.y , vScale.z ) ;
                xLocalToWorld = xScale * xLocalToWorld ;
            }

        }

#endif




        /** Set the local-to-world matrix for this node, based on its position and orientation.
        */
        void SceneNodeBase::SetLocalToWorld()
        {
            PERF_BLOCK( SceneNodeBase__SetLocalToWorld ) ;

            // The order of these transformations matters.

            // Apply translation.
            const Vec3 & vPosition = GetPosition() ;
            mLocalToWorld.SetTranslation( vPosition ) ;

            // Apply rotation.
#if RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_AXIS_ANGLE
            Vec4        axisAngle   = GetOrientation() ;
            const float angle       = axisAngle.Magnitudev3() ;
            {
                Vec4 vAxis( axisAngle ) ;
                vAxis.Normalizev3() ;
                Mat44 xRotate ;
                xRotate.SetRotation( (Vec3&) vAxis , angle ) ;
                mLocalToWorld = xRotate * xLocalToWorld ;
            }
#elif RENDER_ORIENTATION_REPRESENTATION == ORIENTATION_MATRIX
            {
                Mat44 xRotate ;
                xRotate.SetRotation( GetOrientation() ) ;
                mLocalToWorld = xRotate * mLocalToWorld ;
            }
#else
#   error Invalid render orientation representation
#endif

            // Apply scaling.
            const Vec3 & vScale = GetScale() ;
            {
                Mat44 xScale ;
                xScale.SetScale( (Vec3&) vScale ) ;
                mLocalToWorld = xScale * mLocalToWorld ;
            }

            // TODO: Apply parent transformation to this one.
        }




#if defined( _DEBUG )




        class TestSceneNode : public PeGaSys::Render::SceneNodeBase
        {
        public:
            static const TypeId sTypeId = 'test' ; ///< Type identifier for this class

            explicit TestSceneNode( ISceneManager * sceneManager )
                : SceneNodeBase( sceneManager , sTypeId )
            {
            }

            ~TestSceneNode() {}

            virtual void Render()
            {
            }

        private:
        } ;

        void PeGaSys_Render_SceneNodeBase_UnitTest( void )
        {
            DebugPrintf( "SceneNodeBase::UnitTest ----------------------------------------------\n" ) ;

            {
                TestSceneNode testSceneNode( 0 ) ;

                // Update node.
                testSceneNode.Render() ;
            }

            DebugPrintf( "SceneNodeBase::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
