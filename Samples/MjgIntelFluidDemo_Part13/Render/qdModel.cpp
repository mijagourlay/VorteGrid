/*! \file qdModel.h

    \brief Model to render.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <math.h>

#if defined( WIN32 )
    #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include "External/glut.h"

#include "Core/Math/Mat4.h"

#include "qdLight.h"
#include "qdMaterial.h"
#include "qdModel.h"



/** Construct model to render.
*/
QdModel::QdModel( GLuint displayList , QdMaterial * material )
    : mMaterial( material )
    , mScale( 1.0f , 1.0f , 1.0f )
    , mRotation( Mat33_xIdentity )
    , mTranslation( 0.0f , 0.0f , 0.0f )
    , mDisplayList( displayList )
{
}




/** Destruct model to render.
*/
QdModel::~QdModel()
{
}




/** Set position and orientation of this model.
*/
void QdModel::SetScale( const Vec3 & scale )
{
    mScale = scale ;
}




/** Set position and orientation of this model.
*/
void QdModel::SetPositionOrientation( const Vec3 & position , const Mat33 & orientation )
{
    mTranslation = position ;
    mRotation    = orientation ;
}




/** Render model given lights.
*/
void QdModel::Render( Vector< QdLight > * lights , float timeNow )
{
    if( lights )
    {
        const size_t numLights = lights->size() ;
        for( size_t idxLight = 0 ; idxLight < numLights ; ++ idxLight )
        {
            (*lights)[ idxLight ].SetLight( static_cast< int >( idxLight ) , timeNow ) ;
        }
    }
    else
    {
        QdLight::DisableLights() ;
    }

    if( mMaterial )
    {
        mMaterial->UseMaterial() ;
    }

    {
        glPushMatrix() ;

        Mat4 xLocalToWorld( Mat4_xIdentity ) ;
        xLocalToWorld.SetTranslation( Vec4( mTranslation , 1.0f ) ) ;
        {
            Mat4 xRotate ;
            xRotate.SetRotation( mRotation ) ;
            xLocalToWorld = xRotate * xLocalToWorld ;
        }
        {
            Mat4 xScale ;
            xScale.SetScale( mScale ) ;
            xLocalToWorld = xScale * xLocalToWorld ;
        }
        glMultMatrixf( ( GLfloat * ) & xLocalToWorld ) ;

        glCallList( mDisplayList ) ;

        glPopMatrix() ;
    }
}




GLuint MakeUnitSphere()
{
	GLuint displayList = glGenLists( 1 ) ; // Create the id for the display list
	glNewList( displayList , GL_COMPILE ) ;
    glTexGeni( GL_S , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glTexGeni( GL_T , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glEnable( GL_TEXTURE_GEN_S ) ;
    glEnable( GL_TEXTURE_GEN_T ) ;
    glutSolidSphere( 1.0f , 32 , 32 ) ; // Radius is 1; diameter is 2.  So this is about twice as big (in span) as a unit cube.
    glDisable( GL_TEXTURE_GEN_S ) ;
    glDisable( GL_TEXTURE_GEN_T ) ;
    glEndList() ;
    return displayList ;
}




GLuint MakeUnitCube()
{
	GLuint displayList = glGenLists( 1 ) ; // Create the id for the display list
	glNewList( displayList , GL_COMPILE ) ;
    glTexGeni( GL_S , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glTexGeni( GL_T , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glEnable( GL_TEXTURE_GEN_S ) ;
    glEnable( GL_TEXTURE_GEN_T ) ;
#if 1
    glutSolidCube( 1.0f ) ;
#else // For diagnosing collision detection: wireframe with bounding sphere cage.  Sphere only works for cubical rigid bodies.
    glutWireCube( 1.0f ) ;
    const float boundingSphereRadius = 0.5 * fsqrtf( 3.0f ) ;
    glutWireSphere( boundingSphereRadius , 16 , 16 ) ;
#endif
    glDisable( GL_TEXTURE_GEN_S ) ;
    glDisable( GL_TEXTURE_GEN_T ) ;
    glEndList() ;
    return displayList ;
}
