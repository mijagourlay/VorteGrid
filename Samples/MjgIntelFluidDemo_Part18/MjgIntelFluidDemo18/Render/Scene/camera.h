/** \file camera.h

    \brief Scene node representing a camera

    \author Copyright 2010-2011 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_CAMERA_H
#define PEGASYS_RENDER_CAMERA_H

#include "Core/Containers/slist.h"

#include "Core/Math/vec4.h"
#include "Core/Math/mat4.h"

#include "Render/Scene/sceneNodeBase.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class ISceneManager ;

        /** Scene node representing a camera.
        */
        class Camera : public SceneNodeBase
        {
            public:
                static const TypeId sTypeId = 'camr' ; ///< Type identifier for this class

                explicit Camera( ISceneManager * sceneManager ) ;
                virtual ~Camera() ;

                virtual void        Render() ;

                void                RenderScene() ;

                void            SetEye( const Vec3 & eye )          { SetPosition( eye ) ; }

                /// Return World-space position of this camera.
                /// \see GetPosition.
                const Vec3 &    GetEye() const                      { return GetPosition() ; }
                const Vec4 &    GetEye4() const                     { ASSERT( 1.0f == GetPos4().w ) ; return GetPos4() ; }

                /// Set world-space position at which this camera looks.
                void            SetLookAt( const Vec3 & lookAt )    { mLookAt.x = lookAt.x ; mLookAt.y = lookAt.y ; mLookAt.z = lookAt.z ; mLookAt.w = 1.0f ; }

                /// Return world-space position at which this camera looks.
                const Vec3 &    GetLookAt() const                   { return reinterpret_cast< const Vec3 & >( mLookAt ) ; }

                /// Return World-space approximate direction toward which this camera top points.
                const Vec3 &    GetUpApproximate() const            { return reinterpret_cast< const Vec3 & >( mUpApproximate ) ; }

                void            SetFieldOfViewVert( float fovVert ) { mFieldOfViewVert = fovVert ; }
                /// Return Field of view angle (in degrees) along vertical direction.
                const float &   GetFieldOfViewVert() const          { return mFieldOfViewVert ; }

                void            SetAspectRatio( float aspectRatio ) ;

                /// Return Ratio of width to height.
                const float &   GetAspectRatio() const              { return mAspectRatio ; }

                /// Return World-space distance to near clip plane.
                const float &   GetNearClipDist() const             { return mNearClipDist ; }

                void SetClipDepths( float nearClipDistance , float farClipDistance ) ;

                /// Return World-space distance to far clip plane.
                const float &   GetFarClipDist() const              { return mFarClipDist ; }

                void SetPerspective( float verticalFov , float aspectRatio , float nearClipDepth , float farClipDepth ) ;

                /// Get camera position in spherical coordinates, relative to target.
                void            GetOrbit( float & azimuth , float & elevation , float & radius ) ;

                /// Set camera position in spherical coordinates, relative to target.
                void SetOrbit( float azimuth , float elevation , float radius ) ;

            private:
                static const size_t NUM_FRUSTUM_PLANES = 6 ;

                // View transformation parameters
                Vec4                mLookAt                         ;   ///< World-space position at which this camera looks
                Vec4                mUpApproximate                  ;   ///< World-space approximate direction toward which this camera top points
                //Mat44                mView                           ;   ///< View matrix cache

                // Projection transformation parameters
                float               mFieldOfViewVert                ;   ///< Field of view angle (in degrees) along vertical direction
                float               mAspectRatio                    ;   ///< Aspect ratio (height to width)
                float               mNearClipDist                   ;   ///< World-space distance to near clip plane
                float               mFarClipDist                    ;   ///< World-space distance to far clip plane
                //Mat44                mProjection                     ;   ///< Projection matrix cache

                //Vec4                mFrustum[ NUM_FRUSTUM_PLANES ]  ;   ///< Culling frustum planes. TODO: Implement culling frustum.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
