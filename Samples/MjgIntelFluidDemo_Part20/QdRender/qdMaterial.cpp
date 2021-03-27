/** \file qdMaterial.cpp

    \brief Rendering material.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2016 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#if defined( WIN32 )
    #include <windows.h>    // OpenGL headers on Windows use Windows defines.
#endif
#include <GL/glu.h> // Needed for stuff in gl_ext.h
#include <GL/gl_ext.h> // for GL_TEXTURE_CUBE_MAP

#include "Core/Math/vec4.h"

#include "Core/wrapperMacros.h"

#include "Image/image.h"

#include "qdMaterial.h"


/// Check whether OpenGL detected an error and, if so, report it.
void CheckGlError()
{
    bool hitAnyErrors = false ;
    GLenum glError ;
    while( ( glError = glGetError() ) != GL_NO_ERROR )
    {
        const GLubyte * glErrorString = gluErrorString( glError ) ;
        printf( "OpenGL: %s\n" , glErrorString ) ;
        hitAnyErrors = true ;
    }
    if( hitAnyErrors )
    {
        FAIL() ;
    }
}




/** Set render state for any parameter not explicitly set in UseMaterial.
*/
static void SetDefaultMaterialRenderState()
{
    CheckGlError() ;

    glEnable( GL_NORMALIZE )        ;   // Probably should use GL_RESCALE_NORMAL instead but this version of OpenGL does not seem to have that
    glEnable( GL_DEPTH_TEST )       ;   // Enable testing against depth buffer.

    glEnable( GL_ALPHA_TEST )       ;   // Enable testing against alpha channel.
    glAlphaFunc( GL_GEQUAL , 0.005f ) ;  // This should improve fill rate.

    glEnable( GL_CULL_FACE )        ;   // Enable culling faces based on which direction they face.
    glFrontFace( GL_CCW )           ;   // Establish the "winding order" for which face is the front.

    glShadeModel( GL_SMOOTH )       ;   // Enable smooth shading.

    CheckGlError() ;
}




/** Construct object to set render material.
*/
QdMaterial::QdMaterial()
    : mDepthWrite( true )
    , mBlendMode( BM_OPAQUE )
    , mCullFace( CF_BACK )
    , mLineWidth( 3.0f )
    //, mPointSize( 3.0f )
    , mColor( 1.0f , 1.0f , 1.0f , 1.0f )
    , mTexName( 0 )
    , mNumTexPages( 0 )
{
}




/** Destruct render material object.
*/
QdMaterial::~QdMaterial()
{
    CheckGlError() ;

    for( int iTex = 0 ; iTex < NUM_TEXTURES_MAX ; ++ iTex )
    {   // For each texture in this material...
        if( glIsTexture( mTexName ) )
        {   // OpenGL has a texture by this name.
            glDeleteTextures( 1 , & mTexName ) ;    // Tell OpenGL to free that texture.
        }
    }

    CheckGlError() ;
}




void QdMaterial::AssignTexture( const PeGaSys::Image & image )
{
    CheckGlError() ;

    // Make sure this material does not already have a texture,
    // otherwise the existing texture will "leak" video memory.
    ASSERT( 0 == mTexName ) ;

    // Make sure image is sane.
    ASSERT( image.GetWidth() > 0 ) ;
    ASSERT( image.GetHeight() > 0 ) ;
    ASSERT( 4 == image.GetNumChannels() ) ;
    ASSERT( image.GetNumPages() > 0 ) ;
    ASSERT( & image[ 0 ] ) ;

    mNumTexPages = image.GetNumPages() ;

    glDisable( GL_TEXTURE_CUBE_MAP ) ;
    glEnable( GL_TEXTURE_2D ) ;
    glGenTextures( 1 , & mTexName ) ;   // Generate a unique "name" for this texture.

    glBindTexture( GL_TEXTURE_2D , mTexName ) ;
    glPixelStorei( GL_UNPACK_ALIGNMENT , 1 ) ;  // texel data rows not padded

    // Generate MIP maps for all texture pages.
#if 0
    glGenerateMipmapEXT( GL_TEXTURE_2D ) ;
#elif 1
    gluBuild2DMipmaps( GL_TEXTURE_2D
                    , /* internal format */ GL_RGBA
                    , image.GetWidth()
                    , image.GetHeight() * image.GetNumPages() // In lieu of actual texture arrays, assume pages are layed out vertically.
                    , GL_RGBA
                    , GL_UNSIGNED_BYTE
                    , & image[ 0 ] ) ;
#endif

    CheckGlError() ;
}




/** Set up render device to use this material.
*/
void QdMaterial::UseMaterial() const
{
    CheckGlError() ;

    SetDefaultMaterialRenderState() ;

    glDepthMask( mDepthWrite ? GL_TRUE : GL_FALSE ) ;

    switch( mCullFace )
    {
        case CF_NONE            : glDisable( GL_CULL_FACE )                                     ; break ;
        case CF_FRONT           : glEnable( GL_CULL_FACE )  ; glCullFace( GL_FRONT )            ; break ;
        case CF_BACK            : glEnable( GL_CULL_FACE )  ; glCullFace( GL_BACK )             ; break ;
        case CF_FRONT_AND_BACK  : glEnable( GL_CULL_FACE )  ; glCullFace( GL_FRONT_AND_BACK )   ; break ;
        default: FAIL() ; break ;
    }

    switch( mBlendMode )
    {
        case BM_OPAQUE:
            glDisable( GL_BLEND ) ;
            glBlendFunc( GL_ONE , GL_ONE ) ;  // Ignored.
        break ;
        case BM_ALPHA:
            glEnable( GL_BLEND ) ;
            glBlendFunc( GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA ) ;  // Typical alpha blending
        break ;
        case BM_ADDITIVE:
            glEnable( GL_BLEND ) ;
            glBlendFunc( GL_SRC_ALPHA , GL_ONE ) ;  // Typical additive blending
        break ;
        default: FAIL() ; break ;
    }

    // Enable texturing.
    glEnable( GL_TEXTURE_2D ) ;
    //glDisable( GL_TEXTURE_2D ) ; // When rendering points, disable textures
    glBindTexture( GL_TEXTURE_2D , mTexName ) ;

    // Allow vertex color to modulate texture.
    glEnable( GL_COLOR_MATERIAL ) ;
    glTexEnvi( GL_TEXTURE_ENV , GL_TEXTURE_ENV_MODE , GL_MODULATE ) ;
    glColorMaterial( GL_FRONT_AND_BACK , GL_AMBIENT_AND_DIFFUSE ) ;

    // Set texture addressing mode.
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_WRAP_S     , GL_REPEAT );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_WRAP_T     , GL_REPEAT );

    // Set texture scaling filter modes.
#if 0   // Use MIPmaps
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MIN_FILTER , GL_NEAREST_MIPMAP_NEAREST );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MAG_FILTER , GL_NEAREST_MIPMAP_NEAREST );
#else   // Do not use MIPmaps
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MIN_FILTER , GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MAG_FILTER , GL_LINEAR );
#endif

    // Set material color.
    // If vertices have a color, they will override this.
    glColor4fv( (float*) & mColor ) ;
    glMaterialfv( GL_FRONT , GL_AMBIENT , (float*) & mColor ) ;
    glMaterialfv( GL_FRONT , GL_DIFFUSE , (float*) & mColor ) ;
    //glMaterialfv( GL_FRONT , GL_SPECULAR, (float*) & mColor ) ;

    CheckGlError() ;
}
