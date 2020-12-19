/*! \file qdCamera.h

    \brief Class to set a camera for rendering

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include <math.h>

#if defined( WIN32 )
    #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>

#include "Core/Math/Mat4.h"

#include "qdCamera.h"




/*! \brief Construct object to set a camera for rendering
*/
QdCamera::QdCamera( int width , int height )
    : mEye( 0.0f , -2.0f , 0.0f )
    , mTarget( 0.0f , 0.0f , 0.0f )
    , mUp( 0.0f , 0.0f , 1.0f )
{
    mRenderWindowSize[0] = width ;
    mRenderWindowSize[1] = height ;
}




/*! \brief Destruct object to set a camera for rendering
*/
QdCamera::~QdCamera()
{
}




/*! \brief Set camera
*/
void QdCamera::SetCamera( void )
{
    glViewport( 0 , 0 , mRenderWindowSize[0] , mRenderWindowSize[1] ) ;
    glDepthMask( GL_TRUE ) ;  // Enable writing to the depth buffer
    glClearColor( 0.0f , 0.0f , 0.25f , 0.0f ) ;
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ) ;

    glMatrixMode( GL_PROJECTION ) ;
    glLoadIdentity() ;
    static const float fieldOfView = 75.0f ;
    gluPerspective( /* field of view angle along y */ fieldOfView ,
                    /* aspect ratio */ float( mRenderWindowSize[0] ) / float( mRenderWindowSize[1] ),
                    /* near clip */ 0.1f ,
                    /* far clip */ 1000.0f ) ;

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(  mEye.x    , mEye.y    , mEye.z     ,
                mTarget.x , mTarget.y , mTarget.z  ,
                mUp.x     , mUp.y     , mUp.z      ) ;

}
