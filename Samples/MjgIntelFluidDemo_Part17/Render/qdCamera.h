/** \file qdCamera.h

    \brief Class to set a camera for rendering

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef QD_CAMERA_H
#define QD_CAMERA_H

#include "Core/Math/vec3.h"

/** Class to set a camera for rendering.
*/
class QdCamera
{
    public:
        QdCamera( int width , int height ) ;
        ~QdCamera() ;

        void SetCamera( void ) ;

        const int &     GetWidth( void ) const              { return mRenderWindowSize[0] ; }
        void            SetWidth( const int & width )       { mRenderWindowSize[0] = width ; }

        const int &     GetHeight( void ) const             { return mRenderWindowSize[1] ; }
        void            SetHeight( const int & height )     { mRenderWindowSize[1] = height ; }

        void            SetViewport( const int & width , const int & height )
        {
            SetWidth( width ) ;
            SetHeight( height ) ;
        }

        const Vec3 &    GetEye( void ) const                { return mEye ; }
        void            SetEye( const Vec3 & vEye )         { mEye = vEye ; }

        const Vec3 &    GetTarget( void ) const             { return mTarget ; }
        void            SetTarget( const Vec3 & vTarget )   { mTarget = vTarget ; }

        /** Get camera position in spherical coordinates, relative to target. */
        void            GetOrbit( float & azimuth , float & elevation , float & radius )
        {
            Vec3 vRelativeEyePos    = mEye - mTarget ;
            radius = vRelativeEyePos.Magnitude() ;
	        azimuth = atan2f( vRelativeEyePos.y , vRelativeEyePos.x ) ;
	        elevation = acosf( vRelativeEyePos.z / radius ) ;
        }


        /** Set camera position in spherical coordinates, relative to target. */
        void SetOrbit( float azimuth , float elevation , float radius )
        {
            Vec3 vRelativeEyePos = radius * Vec3(   sin( elevation ) * cos( azimuth )
                                                ,   sin( elevation ) * sin( azimuth )
                                                ,   cos( elevation ) ) ;
	        mEye = vRelativeEyePos + mTarget ;
        }

        void SetOrbitalTrajectory( const Vec3 & azElRadVel )
        {
            mOrbitalTrajectory = azElRadVel ;
        }

        void Update() ;

    private:
        QdCamera( const QdCamera & re) ;                // Disallow copy construction.
        QdCamera & operator=( const QdCamera & re ) ;   // Disallow assignment.

        Vec3    mEye                    ;   ///< Eye position in world space
        Vec3    mTarget                 ;   ///< Target position in world space
        Vec3    mUp                     ;   ///< Up direction in world space
        Vec3    mOrbitalTrajectory      ;   ///< Velocity of { azimuth , elevation , radius }
        int     mRenderWindowSize[2]    ;   ///< Render viewport width and height
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
