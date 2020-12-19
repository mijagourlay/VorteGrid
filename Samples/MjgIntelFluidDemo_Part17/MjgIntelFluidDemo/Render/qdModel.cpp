/** \file qdModel.h

    \brief Model to render.

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "qdModel.h"

#include "qdLight.h"
#include "qdMaterial.h"

#include "Core/Math/mat4.h"

#include <GL/glut.h>

#include <math.h>




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
    // Since there is no way to copy a display list, there is no way to
    // implement copy constructor or assignment operator for an object that owns
    // an OpenGL display list.  Unfortunately, that means display lists leak.
    // A more modern approach would use vertex buffer objects instead, but
    // GLUT does not support those, and this is just quick-and-dirty diagnostic
    // rendering code (hence the QD prefix).
    //glDeleteLists( mDisplayList , 1 ) ;
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
void QdModel::Render( VECTOR< QdLight > * lights , float timeNow )
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
        glMatrixMode( GL_MODELVIEW ) ;
        glPushMatrix() ;

        Mat44 xLocalToWorld( Mat4_xIdentity ) ;
        xLocalToWorld.SetTranslation( mTranslation ) ;
        {
            Mat44 xRotate ;
            xRotate.SetRotation( mRotation ) ;
            xLocalToWorld = xRotate * xLocalToWorld ;
        }
        {
            Mat44 xScale ;
            xScale.SetScale( mScale ) ;
            xLocalToWorld = xScale * xLocalToWorld ;
        }
        glMultMatrixf( ( GLfloat * ) & xLocalToWorld ) ;

        glCallList( mDisplayList ) ;

        glPopMatrix() ;
    }
}




GLuint MakeSphere( float radius )
{
    GLuint displayList = glGenLists( 1 ) ; // Create the id for the display list
	glNewList( displayList , GL_COMPILE ) ;

    glPushAttrib( GL_TRANSFORM_BIT ) ; // Remember matrix mode.

    glEnable( GL_TEXTURE_GEN_S ) ;
    glEnable( GL_TEXTURE_GEN_T ) ;

    // Set up a reference plane pointing along +Z.
    GLfloat textureReferencePlane[] = { 0.0f , 0.0f , 0.49f , 0.0f } ;
    glTexGenfv( GL_T , GL_OBJECT_PLANE , textureReferencePlane ) ;
    glTexGeni( GL_S , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glTexGeni( GL_T , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;

    // Shift texture origin to be at bottom of sphere instead of in middle.
    glMatrixMode( GL_TEXTURE ) ;
    glPushMatrix ();
    glLoadIdentity ();
    glTranslatef( 0.0f , -0.5f , 0.0f ) ;

        // Uses many polygons, to proxy for more complicated geometry.
        glutSolidSphere( radius , 128 , 128 ) ; // Radius is 1; diameter is 2.  So this is about twice as big (in span) as a unit cube.

    glPopMatrix() ;

    glPopAttrib() ; // Restore matrix mode.

    glDisable( GL_TEXTURE_GEN_S ) ;
    glDisable( GL_TEXTURE_GEN_T ) ;
    glEndList() ;

    return displayList ;
}




GLuint MakeBox( const Vec3 & dimensions , float boundingSphereRadius )
{
	GLuint displayList = glGenLists( 1 ) ; // Create the id for the display list
	glNewList( displayList , GL_COMPILE ) ;
    glTexGeni( GL_S , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glTexGeni( GL_T , GL_TEXTURE_GEN_MODE , GL_OBJECT_LINEAR ) ;
    glEnable( GL_TEXTURE_GEN_S ) ;
    glEnable( GL_TEXTURE_GEN_T ) ;
    glPushMatrix() ;
    glScalef( dimensions.x , dimensions.y , dimensions.z ) ;
    glutSolidCube( 1.0f ) ;
    glPopMatrix() ;
#if defined( _DEBUG ) // For diagnosing collision detection: wireframe with bounding sphere cage.
    glutWireSphere( boundingSphereRadius , 16 , 16 ) ;
#endif
    (void) boundingSphereRadius ; // Avoid compiler warning about unreferenced parameter, for non-debug builds.
    glDisable( GL_TEXTURE_GEN_S ) ;
    glDisable( GL_TEXTURE_GEN_T ) ;
    glEndList() ;
    return displayList ;
}
