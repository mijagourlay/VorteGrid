/** \file camera.cpp

    \brief Scene node representing a camera

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Scene/camera.h"
#include "Render/Scene/iSceneManager.h"

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

        /** Construct scene node representing a camera.
        */
        Camera::Camera( ISceneManager * sceneManager )
            : SceneNodeBase( sceneManager , sTypeId )

            , mLookAt( 0.0f , 0.0f , 0.0f , 1.0f )
            , mUpApproximate( 0.0f , 0.0f , 1.0f , 0.0f )
            //, mView( Mat4_xIdentity )

            , mFieldOfViewVert( 90.0f )
            , mAspectRatio( 1.33f )
            , mNearClipDist( 1.0f )
            , mFarClipDist( 1000.0f )
            //, mProjection( Mat4_xIdentity )
        {
            PERF_BLOCK( Camera__Camera ) ;

            // Move camera away from default look-at point.
            SetPosition( Vec4( 2.0f , 0.0f , 0.0f , 1.0f ) ) ;

            // TODO: Implement culling frustum.
            //for( size_t idx = 0 ; idx < NUM_FRUSTUM_PLANES ; ++ idx )
            //{
            //    mFrustum[ idx ].x =
            //    mFrustum[ idx ].y =
            //    mFrustum[ idx ].z =
            //    mFrustum[ idx ].w = 0.0f ;
            //}
        }




        /** Destruct scene node representing a camera.
        */
        Camera::~Camera()
        {
            PERF_BLOCK( Camera__dtor ) ;
        }




        /** Set ratio of width to height.

            This should match the viewport aspect ratio, otherwise the image will be distorted.
            This should therefore be set by the Viewport just prior to calling Camera::RenderScene.
        */
        void    Camera::SetAspectRatio( float aspectRatio )
        {
            PERF_BLOCK( Camera__SetAspectRatio ) ;

            ASSERT( mAspectRatio >= 0.0f ) ;
            ASSERT( ! IsNan( mAspectRatio ) ) ;
            ASSERT( ! IsInf( mAspectRatio ) ) ;
            mAspectRatio = aspectRatio ;
        }




        /** Set camera perspective projection.
        */
        void Camera::SetPerspective( float verticalFov , float aspectRatio , float nearClipDistance , float farClipDistance )
        {
            PERF_BLOCK( Camera__SetPerspective ) ;

            SetFieldOfViewVert( verticalFov ) ;
            SetAspectRatio( aspectRatio ) ;
            mNearClipDist = nearClipDistance ;
            mFarClipDist = farClipDistance ;
        }


        /** Set camera position in spherical coordinates, relative to target.
        */
        void Camera::SetOrbit( float azimuth , float elevation , float radius )
        {
            PERF_BLOCK( Camera__SetOrbit ) ;

            // Avoid "gimbal lock" by limiting elevation angle to avoid the poles.
            static const float sAvoidPoles = 0.001f ;
            elevation = Clamp( elevation , sAvoidPoles , PI * ( 1.0f - sAvoidPoles ) ) ;

            Vec3 vRelativeEyePos = radius * Vec3(   sin( elevation ) * cos( azimuth )
                ,   sin( elevation ) * sin( azimuth )
                ,   cos( elevation ) ) ;
            SetEye( vRelativeEyePos + GetLookAt() ) ;
        }




        /** Get camera position in spherical coordinates, relative to target.

            \param azimuth      Angle, in radians, along horizontal direction, off positive x axis.

            \param elevation    Angle, in radians, off upward vertical axis.

            \param radius       Distance, in world units, of camera (eye) from target (look-at)
        */
        void            Camera::GetOrbit( float & azimuth , float & elevation , float & radius )
        {
            PERF_BLOCK( Camera__GetOrbit ) ;

            Vec3 vRelativeEyePos    = GetEye() - GetLookAt() ;
            radius      = vRelativeEyePos.Magnitude() ;
            azimuth     = atan2f( vRelativeEyePos.y , vRelativeEyePos.x ) ;
            elevation   = acosf( vRelativeEyePos.z / radius ) ;
        }




        /** Render a camera.

            Normally, rendering a camera does nothing, but for diagnosing a scene it
            could be useful to render geometry at the location of a camera, to visualize
            their locations.
        */
        void Camera::Render()
        {
            PERF_BLOCK( Camera__Render ) ;

            RenderChildren() ;
        }




        /** Render the scene which this camera sees.

            Caller (e.g. Viewport::TriggerCamera) should set Camera aspect ratio
            (by calling Camera::SetAspectRatio) just before calling this routine.
        */
        void Camera::RenderScene( const double & currentVirtualTimeInSeconds )
        {
            PERF_BLOCK( Camera__RenderScene ) ;

            GetSceneManager()->RenderScene( * this , currentVirtualTimeInSeconds ) ;
        }


    } ;
} ;




#if defined( _DEBUG )

void PeGaSys_Render_Camera_UnitTest( void )
{
    DebugPrintf( "Camera::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::Camera camera( 0 ) ;

        // Update camera
        camera.RenderScene( 0.0 ) ;
    }

    DebugPrintf( "Camera::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif
