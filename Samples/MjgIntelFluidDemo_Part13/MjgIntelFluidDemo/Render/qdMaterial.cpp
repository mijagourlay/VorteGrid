/*! \file qdMaterial.cpp

    \brief Rendering material.

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

#if defined( WIN32 )
    #include <windows.h>    // OpenGL headers on Windows use Windows defines.
#endif
#include <GL/glu.h> // Needed for stuff in gl_ext.h
#include "External/gl_ext.h" // for GL_TEXTURE_CUBE_MAP

#include "Core/Math/vec4.h"

#include "Core/wrapperMacros.h"
#include "qdMaterial.h"


/** Set render state for any parameter not explicitly set in UseMaterial.
*/
static void SetDefaultMaterialRenderState()
{
    glEnable( GL_NORMALIZE )        ;   // Probably should use GL_RESCALE_NORMAL instead but this version of OpenGL does not seem to have that
    glEnable( GL_DEPTH_TEST )       ;   // Enable testing against depth buffer.

    glEnable( GL_ALPHA_TEST )       ;   // Enable testing against alpha channel.
    glAlphaFunc( GL_GEQUAL , 0.005f ) ;  // This should improve fill rate.

    glEnable( GL_CULL_FACE )        ;   // Enable culling faces based on which direction they face.
    glFrontFace( GL_CCW )           ;   // Establish the "winding order" for which face is the front.

    glShadeModel( GL_SMOOTH )       ;   // Enable smooth shading.
}




/** Construct object to set render material.
*/
QdMaterial::QdMaterial( void )
    : mDepthWrite( true )
    , mBlendMode( BM_OPAQUE )
    , mCullFace( GL_BACK )
    , mColor( 1.0f , 1.0f , 1.0f , 1.0f )
    , mTexName( 0 )
    , mNumTexPages( 0 )
{
}




/** Destruct render material object.
*/
QdMaterial::~QdMaterial()
{
    for( int iTex = 0 ; iTex < NUM_TEXTURES_MAX ; ++ iTex )
    {   // For each texture in this material...
        if( glIsTexture( mTexName ) )
        {   // OpenGL has a texture by this name.
            glDeleteTextures( 1 , & mTexName ) ;    // Tell OpenGL to free that texture.
        }
    }
}




/** Allocate texture.

    \param texWidth - width of entire texture in pixels

    \param texHeight - height of entire texture in pixels

    \param numTexPages - number of "pages" in thix texture.

        A single texture can be divided into multiple "pages"
        where each page (a region in U,V texture coordinate
        space) is used for a different purpose.  This effective
        allows the same geometry to use multiple different
        texture images, which in turn allows a single draw call
        to render a larger number of vertices.  Also useful
        for rendering different "species" of particles,
        such as hot and cold.

        Texture pages span the V coordinate.  So for example,
        if a texture has 1 page then V spans [0,1] as usual.
        If a texture has 2 pages then V spans [0,0.5]
        for the first page and [0.5,1] for the second page.
        The U coordinate always spans [0,1] as usual.

    \see AssignTexturePage for how to assign each page.

    \note This can only run after the device is initialized.
            Due to order-of-operations this cannot easily run
            in constructor.

    After calling this routine, caller must assign texture
    data by calling AssignTexturePage and FinalizeTexture.

*/
void QdMaterial::AllocateTexture( int texWidth , int texHeight , int numTexPages )
{
    // Make sure this material does not already have a texture,
    // otherwise the existing texture will "leak" video memory.

    glDisable( GL_TEXTURE_CUBE_MAP ) ;
    glEnable( GL_TEXTURE_2D ) ;
    glGenTextures( 1 , & mTexName ) ;   // Generate a unique "name" for this texture.

    mNumTexPages            = numTexPages   ;

    mTexImage.mWidth        = texWidth      ;
    mTexImage.mHeight       = texHeight     ;
    mTexImage.mNumChannels  = 4             ;

    glBindTexture( GL_TEXTURE_2D , mTexName ) ;
    glPixelStorei( GL_UNPACK_ALIGNMENT , 1 ) ;  // texel data rows not padded

    mTexImage.AllocateImageData() ;
}




void QdMaterial::AssignTexturePage( ITextureProc & textureProc , int texturePageIndex )
{
    // Make sure Initialize has been called.

    QdImage texturePage( mTexImage , texturePageIndex , mNumTexPages ) ;   // Make shallow copy to access only given page.

    textureProc( mTexImage , mNumTexPages , texturePageIndex ) ;
}




void QdMaterial::FinalizeTexture()
{
    // Generate MIP maps for all texture pages.
#if 0
    glGenerateMipmapEXT( GL_TEXTURE_2D ) ;
#elif 1
    gluBuild2DMipmaps( GL_TEXTURE_2D
                    , /* internal format */ GL_RGBA
                    , mTexImage.mWidth
                    , mTexImage.mHeight
                    , GL_RGBA
                    , GL_UNSIGNED_BYTE
                    , mTexImage.mImgData ) ;
#endif

#if 0 && defined( _DEBUG )
    {
        static int texFileCounter = 0 ;
        char textureImgFilename[ 64 ] ;
        sprintf( textureImgFilename , "tex%i.ppm" , texFileCounter ) ;
        imgWritePPM( mImgData , mWidth , mHeight , textureImgFilename ) ;
        sprintf( textureImgFilename , "tex%i.pgm" , texFileCounter ) ;
        imgWriteAlphaToPGM( mImgData , mWidth , mHeight , textureImgFilename ) ;
        ++ texFileCounter ;
    }
#endif

    mTexImage.FreeImageData() ;
}




/** Make a material with a simple noise texture.

    \param texWidth - see AllocateTexture

    \param texHeight - see AllocateTexture

    \param numTexPages - see AllocateTexture

    \param gamma - see AssignTexturePage

    \note This can only run after the device is initialized.
            Due to order-of-operations this cannot easily run
            in constructor.
*/
void QdMaterial::MakeSimpleNoiseTexture( int texWidth , int texHeight , int numTexPages , float gamma )
{
    const Vec4 blankColor( 1.0f , 1.0f , 1.0f , 1.0f ) ;
    NoiseTextureProc noiseBall( gamma , blankColor ) ;
    AllocateTexture( texWidth , texHeight , numTexPages ) ;
    AssignTexturePage( noiseBall , 0 ) ;
    FinalizeTexture() ;
}




/** Set up render device to use this material.
*/
void QdMaterial::UseMaterial() const
{
    SetDefaultMaterialRenderState() ;

    glDepthMask( mDepthWrite ? GL_TRUE : GL_FALSE ) ;
    glCullFace( mCullFace ) ;

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
    glColor3fv( (float*) & mColor ) ;
    glMaterialfv( GL_FRONT , GL_AMBIENT , (float*) & mColor ) ;
    glMaterialfv( GL_FRONT , GL_DIFFUSE , (float*) & mColor ) ;
    //glMaterialfv( GL_FRONT , GL_SPECULAR, (float*) & mColor ) ;
}
