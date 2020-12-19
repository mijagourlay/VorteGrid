/*! \file qdMaterial.cpp

    \brief Class to set a rendering material

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/

#if defined( WIN32 )
    #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include "gl_ext.h"

#include "Core/Math/vec4.h"

#include "wrapperMacros.h"
#include "qdMaterial.h"

#pragma warning( disable: 4244 )


static unsigned char blur3x3( int ix , int iy , int ic , const unsigned char * img , int nx , int ny , int nc )
{
    const unsigned ixOff = ix * nc + ic ;
    const unsigned nl    = nx * nc ;         // width (y) stride
    const unsigned iyOff = iy * nl ;
    const unsigned iym1  = ( iy - 1 ) * nl ;
    const unsigned iyp1  = ( iy + 1 ) * nl ;

    unsigned iCounter = 1 ;
    unsigned iOut     = img[ ixOff + iyOff ] ;

    if( ix > 0 )
    {
        const unsigned ixm1 = ( ix - 1 ) * nc + ic ;
        iOut += img[ ixm1 + iyOff ] ; iCounter ++ ;
        if( iy > 0 )
        {
            iOut += img[ ixm1 + iym1  ] ; iCounter ++ ;
        }
        if( iy < ( ny - 1 ) )
        {
            iOut += img[ ixm1 + iyp1  ] ; iCounter ++ ;
        }
    }
    if( ix < ( nx - 1 ) )
    {
        const unsigned ixp1 = ( ix + 1 ) * nc + ic ;
        iOut += img[ ixp1 + iyOff ] ; iCounter ++ ;
        if( iy > 0 )
        {
            iOut += img[ ixp1 + iym1  ] ; iCounter ++ ;
        }
        if( iy < ( ny - 1 ) )
        {
            iOut += img[ ixp1 + iyp1  ] ; iCounter ++ ;
        }
    }
        if( iy > 0 )
        {
            iOut += img[ ixOff + iym1 ] ; iCounter ++ ;
        }
        if( iy < ( ny - 1 ) )
        {
            iOut += img[ ixOff + iyp1 ] ; iCounter ++ ;
        }
    return MIN2( iOut / iCounter , 255 ) ;
}




// sTexImgDemo: data and routines used to create a demo texture for debugging -- obverts need for file IO.
static const int sTexImgDemoWidth        = 8 ;
static const int sTexImgDemoHeight       = 8 ;
static const int sTexImgDemoNumChannels  = 4;
static GLubyte sTexImgDemoData[ sTexImgDemoWidth * sTexImgDemoHeight * sTexImgDemoNumChannels ] ;




static void texImgSmooth( unsigned char * imgData , unsigned width , unsigned height , unsigned yBegin , unsigned numChannels , unsigned numSmoothingPasses )
{
    const unsigned  numBytes = width * height * numChannels ;
    GLubyte *       pImgTemp = (GLubyte*) malloc( numBytes ) ;
    const unsigned  yEnd     = yBegin + height ;
    memcpy( pImgTemp , imgData , numBytes ) ;
    for( unsigned iSmooth = 0 ; iSmooth < numSmoothingPasses ; ++ iSmooth )
    {
        for( unsigned iyDst = 0 ; iyDst < height ; ++ iyDst )
        {
            const unsigned iySrc = iyDst + yBegin ;
            for( unsigned ix = 0 ; ix < width ; ++ ix )
            {
                const unsigned offset = numChannels * ( ix + width * iyDst ) ;
                pImgTemp[ offset + 0 ] = blur3x3( ix , iySrc , 0 , imgData , width , yEnd , numChannels ) ;
                pImgTemp[ offset + 1 ] = blur3x3( ix , iySrc , 1 , imgData , width , yEnd , numChannels ) ;
                pImgTemp[ offset + 2 ] = blur3x3( ix , iySrc , 2 , imgData , width , yEnd , numChannels ) ;
                // Note the lack of smoothing in the 4th channel.  Assuming it's uniform.
            }
        }
        memcpy( imgData , pImgTemp , numBytes ) ;
    }
    free( pImgTemp ) ;
}




static void texImgMakeNoise( unsigned char * imgData , unsigned width , unsigned height , unsigned yBegin )
{
    const unsigned numChannels  = 4 ;
    const unsigned yEnd         = yBegin + height ;
    for( unsigned iy = yBegin ; iy < yEnd ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < width ; ++ ix )
        {
            const unsigned iNoise = rand() & 0xff ;
            const unsigned offset = numChannels * ( ix + width * iy ) ;
            imgData[ offset + 0 ] = iNoise ; // red
            imgData[ offset + 1 ] = iNoise ; // green
            imgData[ offset + 2 ] = iNoise ; // blue
            imgData[ offset + 3 ] = 0xff   ; // alpha (opacity)
        }
    }
}




static void texImgFadeAlphaRadially( unsigned char * imgData , unsigned width , unsigned height , unsigned yBegin , float power , float alphaMax )
{
    const unsigned  numChannels = 4 ;
    const float     almost256   = 256.0f * ( 1.0f - FLT_EPSILON ) ;
    const float     p           = power * 0.5f ;
    const unsigned  yEnd        = yBegin + height ;
    for( unsigned iy = yBegin ; iy < yEnd ; ++ iy )
    {
        const float y = 2.0f * ( float( iy - yBegin ) / float( height - 1 ) - 0.5f ) ; // value in [-1,1]
        for( unsigned ix = 0 ; ix < width ; ++ ix )
        {
            const float     x           = 2.0f * ( float( ix ) / float( width - 1 ) - 0.5f ) ; // value in [-1,1]
            const float     r2          = x * x + y * y ;
            const float     alpha0to1   = powf( CLAMP( 1.0f - r2 , 0.0f , 1.0f ) , p ) ;
            const float     alpha0to255 = alpha0to1 * almost256 * alphaMax ;
            const unsigned  offset      = numChannels * ( ix + width * iy ) ;
            const unsigned  iAlpha      = (unsigned) alpha0to255 ;
            imgData[ offset + 3 ] = iAlpha ; // alpha (opacity)
        }
    }
}




/*! \brief Apply brightness correction to the given image data.
*/
static void texImgGammaCorrect( unsigned char * imgData , unsigned width , unsigned height , unsigned yBegin , float gamma )
{
    const unsigned  numChannels = 4 ;
    const float     almost256   = 256.0f * ( 1.0f - FLT_EPSILON ) ;
    const unsigned  yEnd        = yBegin + height ;
    for( unsigned iy = yBegin ; iy < yEnd ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < width ; ++ ix )
        {
            const unsigned  offset  = numChannels * ( ix + width * iy ) ;
            imgData[ offset + 0 ] = powf( float( imgData[ offset + 0 ] ) / 255.0f , gamma ) * almost256 ;
            imgData[ offset + 1 ] = powf( float( imgData[ offset + 1 ] ) / 255.0f , gamma ) * almost256 ;
            imgData[ offset + 2 ] = powf( float( imgData[ offset + 2 ] ) / 255.0f , gamma ) * almost256 ;
        }
    }
}




/*! \brief Apply a tint to the given image data
*/
static void texImgTint( unsigned char * imgData , unsigned width , unsigned height , unsigned yBegin , const Vec4 & color )
{
    const unsigned  numChannels = 4 ;
    const float     almost256   = 256.0f * ( 1.0f - FLT_EPSILON ) ;
    const unsigned  yEnd        = yBegin + height ;
    for( unsigned iy = yBegin ; iy < yEnd ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < width ; ++ ix )
        {
            const unsigned  offset  = numChannels * ( ix + width * iy ) ;
            imgData[ offset + 0 ] = ( float( imgData[ offset + 0 ] ) / 255.0f  * color.x ) * almost256 ;
            imgData[ offset + 1 ] = ( float( imgData[ offset + 1 ] ) / 255.0f  * color.y ) * almost256 ;
            imgData[ offset + 2 ] = ( float( imgData[ offset + 2 ] ) / 255.0f  * color.z ) * almost256 ;
            imgData[ offset + 3 ] = ( float( imgData[ offset + 3 ] ) / 255.0f  * color.w ) * almost256 ;
        }
    }
}




/*! \brief Write out the alpha channel from a given image as a PGM file
*/
static void texImgWriteAlphaToPGM( unsigned char * imgData , unsigned width , unsigned height , const char * filename )
{
    FILE * fp = fopen( filename , "wb" ) ;
    fprintf( fp , "P5 %i %i 255\n" , width , height ) ;
    const unsigned  numChannels = 4 ;
    for( unsigned iy = 0 ; iy < height ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < width ; ++ ix )
        {
            const unsigned  offset  = numChannels * ( ix + width * iy ) ;
            // Write alpha value as a byte
            fwrite( & imgData[ offset + 3 ] , 1 , 1 , fp ) ;
        }
    }
    fclose( fp ) ;
}




/*! \brief Write out the given image as a PPM file
*/
static void texImgWritePPM( unsigned char * imgData , unsigned width , unsigned height , const char * filename )
{
    FILE * fp = fopen( filename , "wb" ) ;
    fprintf( fp , "P6 %i %i 255\n" , width , height ) ;
    const unsigned  numChannels = 4 ;
    for( unsigned iy = 0 ; iy < height ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < width ; ++ ix )
        {
            const unsigned  offset  = numChannels * ( ix + width * iy ) ;
            // Write red, green, blue values as bytes
            fwrite( & imgData[ offset + 0 ] , 1 , 3 , fp ) ;
        }
    }
    fclose( fp ) ;
}




static void SetDefaultMaterialRenderState( void )
{
    glEnable(GL_NORMALIZE); // Probably should use GL_RESCALE_NORMAL instead but this version of OpenGL does not seem to have that
    glEnable(GL_DEPTH_TEST) ; // Enable depth test
    glEnable(GL_BLEND) ;
    glBlendFunc( GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA ) ;  // Typical alpha blending
    glEnable(GL_ALPHA_TEST) ;
    glAlphaFunc( GL_GEQUAL , 0.02f ) ;  // This should improve fill rate
    glDepthMask( GL_TRUE ) ;
    glEnable( GL_CULL_FACE ) ;
    glCullFace( GL_BACK ) ;
    glFrontFace( GL_CCW ) ;
    glShadeModel( GL_SMOOTH ) ;
}




/*! \brief Construct object to set render material
*/
QdMaterial::QdMaterial( void )
    : mColor( 1.0f , 1.0f , 1.0f , 1.0f )
    , mDepthWrite( true )
    , mCullFace( GL_BACK )
    , mTexName( 0 )
    , mNumTexPages( 0 )
    , mTexWidth( 0 )
    , mTexHeight( 0 )
    , mImgData( 0 )
{
}




/*! \brief Allocate texture

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

    mNumTexPages    = numTexPages   ;
    mTexWidth       = texWidth      ;
    mTexHeight      = texHeight     ;

    glBindTexture( GL_TEXTURE_2D , mTexName ) ;
    glPixelStorei( GL_UNPACK_ALIGNMENT , 1 ) ;  // texel data rows not padded

    mImgData = (unsigned char *) malloc( mTexWidth * mTexHeight * NUM_CHANNELS ) ;
}




/*! \brief Initialize object to set render material

    \param textureProc - see AssignTexturePage

    \param texWidth - see AllocateTexture

    \param texHeight - see AllocateTexture

    \param gamma - see AssignTexturePage

    \param numTexPages - see AllocateTexture

    \note This can only run after the device is initialized.
            Due to order-of-operations this cannot easily run
            in constructor.
*/
void QdMaterial::Initialize( int texWidth , int texHeight , int numTexPages , TextureProc textureProc , float gamma )
{
    AllocateTexture( texWidth , texHeight , numTexPages ) ;
    const Vec4 blankColor( 1.0f , 1.0f , 1.0f , 1.0f ) ;
    AssignTexturePage( textureProc , gamma , 0 , blankColor ) ;
    FinalizeTexture() ;
}




void QdMaterial::AssignTexturePage( TextureProc textureProc , float gamma , int texturePage , const Vec4 & color )
{
    // Make sure Initialize has been called.

    const unsigned texHeight = mTexHeight / mNumTexPages ;
    const unsigned yBegin = texturePage * texHeight ;

    switch( textureProc )
    {
        case TP_NOISE_OPAQUE:
            texImgMakeNoise( mImgData , mTexWidth , texHeight , yBegin ) ;
            texImgSmooth( mImgData , mTexWidth , texHeight , yBegin , NUM_CHANNELS , /* num smoothing passes */ 1 ) ;
            break ;

        case TP_NOISE_BALL:
            texImgMakeNoise( mImgData , mTexWidth , texHeight , yBegin ) ;
            texImgSmooth( mImgData , mTexWidth , texHeight , yBegin , NUM_CHANNELS , /* num smoothing passes */ 1 ) ;
            texImgFadeAlphaRadially( mImgData , mTexWidth , texHeight , yBegin , 0.5f , 0.05f ) ;
            break ;
    }
    texImgGammaCorrect( mImgData , mTexWidth , texHeight , yBegin , gamma ) ;
    texImgTint( mImgData , mTexWidth , texHeight , yBegin , color ) ;
}




void QdMaterial::FinalizeTexture()
{
    // Generate MIP maps for all texture pages.
#if 0
    glGenerateMipmapEXT( GL_TEXTURE_2D ) ;
#elif 1
    gluBuild2DMipmaps( GL_TEXTURE_2D
                    , /* internal format */ GL_RGBA
                    , mTexWidth
                    , mTexHeight
                    , GL_RGBA
                    , GL_UNSIGNED_BYTE
                    , mImgData ) ;
#endif

#if 0 && defined( _DEBUG )
    {
        static int texFileCounter = 0 ;
        char textureImgFilename[ 64 ] ;
        sprintf( textureImgFilename , "tex%i.ppm" , texFileCounter ) ;
        texImgWritePPM( mImgData , mTexWidth , mTexHeight , textureImgFilename ) ;
        sprintf( textureImgFilename , "tex%i.pgm" , texFileCounter ) ;
        texImgWriteAlphaToPGM( mImgData , mTexWidth , mTexHeight , textureImgFilename ) ;
        ++ texFileCounter ;
    }
#endif

    free( mImgData ) ;
    mImgData = 0 ;
}




/*! \brief Destruct object to set render material
*/
QdMaterial::~QdMaterial()
{
    for( int iTex = 0 ; iTex < NUM_TEXTURES_MAX ; ++ iTex )
    {
        if( glIsTexture( mTexName ) )
        {
            glDeleteTextures( 1 , & mTexName ) ;
        }
    }
}




/*! \brief Setup renderer to use this material
*/
void QdMaterial::UseMaterial( void ) const
{
    SetDefaultMaterialRenderState() ;
    glDepthMask( mDepthWrite ? GL_TRUE : GL_FALSE ) ;
    glCullFace( mCullFace ) ;
    glEnable( GL_TEXTURE_2D ) ;
    //glDisable( GL_TEXTURE_2D ) ; // When rendering points, disable textures
    glBindTexture( GL_TEXTURE_2D , mTexName ) ;
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_WRAP_S     , GL_REPEAT );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_WRAP_T     , GL_REPEAT );
#if 0   // Use MIPmaps
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MIN_FILTER , GL_NEAREST_MIPMAP_NEAREST );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MAG_FILTER , GL_NEAREST_MIPMAP_NEAREST );
#else   // Do not use MIPmaps
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MIN_FILTER , GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MAG_FILTER , GL_LINEAR );
#endif
    glColor3fv( (float*) & mColor ) ;
    glMaterialfv( GL_FRONT , GL_AMBIENT , (float*) & mColor ) ;
    glMaterialfv( GL_FRONT , GL_DIFFUSE , (float*) & mColor ) ;
    //glMaterialfv( GL_FRONT , GL_SPECULAR, (float*) & mColor ) ;
}
