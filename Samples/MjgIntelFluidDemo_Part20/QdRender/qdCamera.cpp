/** \file qdCamera.h

    \brief Class to set a camera for rendering

    \see Accompanying articles for more information:

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "qdCamera.h"

#include "Core/Math/mat4.h"

#if defined( WIN32 )
    #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>

#include <math.h>

static const float sFieldOfViewVertical = 75.0f ;

extern void CheckGlError() ;





/** Construct object to set a camera for rendering.
*/
QdCamera::QdCamera( int width , int height )
    : mEye( 0.0f , -2.0f , 0.0f )
    , mTarget( 0.0f , 0.0f , 0.0f )
    , mUp( 0.0f , 0.0f , 1.0f )
    , mOrbitalTrajectory( 0.0f , 0.0f , 0.0f )
    , mFieldOfViewVertical( sFieldOfViewVertical )
    , mAspectRatio( 1.33f )
    , mNearClipDist( 0.1f )
    , mFarClipDist( 1000.0f )

{
    mRenderWindowSize[0] = width ;
    mRenderWindowSize[1] = height ;
}




/** Destruct object to set a camera for rendering.
*/
QdCamera::~QdCamera()
{
}




/** Set camera.
*/
void QdCamera::SetCamera()
{
    CheckGlError() ;

    glViewport( 0 , 0 , mRenderWindowSize[0] , mRenderWindowSize[1] ) ;
    glDepthMask( GL_TRUE ) ;  // Enable writing to the depth buffer

    glMatrixMode( GL_PROJECTION ) ;
    glLoadIdentity() ;
    mAspectRatio = float( mRenderWindowSize[0] ) / float( mRenderWindowSize[1] ) ;
    gluPerspective( mFieldOfViewVertical ,mAspectRatio , mNearClipDist , mFarClipDist ) ;

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(  mEye.x    , mEye.y    , mEye.z     ,
                mTarget.x , mTarget.y , mTarget.z  ,
                mUp.x     , mUp.y     , mUp.z      ) ;

    CheckGlError() ;
}




/** Update camera properties based on its trajectory.
*/
void QdCamera::Update()
{
    CheckGlError() ;

    float azimuth , elevation , radius ;
    // Obtain previous camera parameters.
    GetOrbit( azimuth , elevation , radius ) ;
    azimuth     += mOrbitalTrajectory.x ;
    elevation   += mOrbitalTrajectory.y ;
    radius      += mOrbitalTrajectory.z ;
    // Avoid gimbal lock by limiting elevation angle to avoid the poles.
    static const float sAvoidPoles = 0.001f ;
    elevation = Clamp( elevation , sAvoidPoles , PI * ( 1.0f - sAvoidPoles ) ) ;
    // Set new camera parameters based on how much mouse moved.
    SetOrbit( azimuth , elevation , radius ) ;

    CheckGlError() ;
}
