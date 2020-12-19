/*! \file qdMaterial.cpp

    \brief Class to set a rendering material

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
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




static void texImgSmooth( unsigned char * imgData , unsigned width , unsigned height , unsigned numChannels , unsigned numSmoothingPasses )
{
    const unsigned numBytes = width * height * numChannels ;
    GLubyte * pImgTemp = (GLubyte*) malloc( numBytes ) ;
    memcpy( pImgTemp , imgData , numBytes ) ;
    for( unsigned iSmooth = 0 ; iSmooth < numSmoothingPasses ; ++ iSmooth )
    {
        for( unsigned iy = 0 ; iy < height ; ++ iy )
        {
            for( unsigned ix = 0 ; ix < width ; ++ ix )
            {
                const unsigned offset = numChannels * ( ix + width * iy ) ;
                pImgTemp[ offset + 0 ] = blur3x3( ix , iy , 0 , imgData , width , height , numChannels ) ;
                pImgTemp[ offset + 1 ] = blur3x3( ix , iy , 1 , imgData , width , height , numChannels ) ;
                pImgTemp[ offset + 2 ] = blur3x3( ix , iy , 2 , imgData , width , height , numChannels ) ;
            }
        }
        memcpy( imgData , pImgTemp , numBytes ) ;
    }
    free( pImgTemp ) ;
}




static void texImgMakeNoise( unsigned char * imgData , unsigned width , unsigned height )
{
    static const unsigned numChannels = 4 ;
    for( unsigned iy = 0 ; iy < height ; ++ iy )
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




static void texImgFadeAlphaRadially( unsigned char * imgData , unsigned width , unsigned height , float power , float alphaMax )
{
    static const unsigned   numChannels = 4 ;
    static const float      almost256   = 256.0f * ( 1.0f - FLT_EPSILON ) ;
    const float             p           = power * 0.5f ;
    for( unsigned iy = 0 ; iy < height ; ++ iy )
    {
        const float y = 2.0f * ( float( iy ) / float( height - 1 ) - 0.5f ) ; // value in [-1,1]
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




static void texImgGammaCorrect( unsigned char * imgData , unsigned width , unsigned height , float gamma )
{
    static const unsigned   numChannels = 4 ;
    static const float      almost256   = 256.0f * ( 1.0f - FLT_EPSILON ) ;
    for( unsigned iy = 0 ; iy < height ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < width ; ++ ix )
        {
            const unsigned  offset      = numChannels * ( ix + width * iy ) ;
            imgData[ offset + 0 ] = powf( float( imgData[ offset + 0 ] ) / 255.0f , gamma ) * almost256 ;
            imgData[ offset + 1 ] = powf( float( imgData[ offset + 1 ] ) / 255.0f , gamma ) * almost256 ;
            imgData[ offset + 2 ] = powf( float( imgData[ offset + 2 ] ) / 255.0f , gamma ) * almost256 ;
        }
    }
}




static void SetDefaultMaterialRenderState( void )
{
    glEnable(GL_NORMALIZE); // Probably should use GL_RESCALE_NORMAL instead but this version of OpenGL does not seem to have that
    glEnable(GL_DEPTH_TEST) ; // Enable depth test
    glEnable(GL_BLEND) ;
    glBlendFunc( GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA ) ;  // Typical alpha blending
    glEnable(GL_ALPHA_TEST) ;
    glAlphaFunc( GL_GEQUAL , 0.02f ) ;  // This should improve fill rate
    glDepthMask(GL_FALSE) ;  // Disable writing to the depth buffer
    glEnable(GL_CULL_FACE) ;
    glCullFace(GL_BACK) ;
    glFrontFace(GL_CCW) ;
    glShadeModel( GL_SMOOTH ) ;
}




/*! \brief Construct object to set render material
*/
QdMaterial::QdMaterial( void )
    : mTexName( 0 )
{
    memset( mTextures , 0 , sizeof( mTextures ) ) ;
}




/*! \brief Initialize object to set render material

    \note This can only run after the device is initialized.
            Due to order-of-operations this cannot easily run
            in constructor.
*/
void QdMaterial::Initialize( void )
{
    glDisable( GL_TEXTURE_CUBE_MAP ) ;
    glEnable( GL_TEXTURE_2D ) ;
    glGenTextures( 1 , & mTexName ) ;           // Generate a unique "name" for this texture.
    glBindTexture( GL_TEXTURE_2D , mTexName ) ;
    glPixelStorei( GL_UNPACK_ALIGNMENT , 1 ) ; // texel data rows not padded
    static const unsigned numChannels   = 4 ;
    static const unsigned imgSize       = 16 ;
    unsigned char * pImgData = (unsigned char *) malloc( imgSize * imgSize * numChannels ) ;
    texImgMakeNoise( pImgData , imgSize , imgSize ) ;
    texImgSmooth( pImgData , imgSize , imgSize , numChannels , /* num smoothing passes */ 1 ) ;
    texImgFadeAlphaRadially( pImgData , imgSize , imgSize , 0.5f , 0.2f ) ;
    texImgGammaCorrect( pImgData , imgSize , imgSize , 0.5f ) ;

    gluBuild2DMipmaps( GL_TEXTURE_2D
                , /* internal format */ GL_RGBA
                , imgSize
                , imgSize
                , GL_RGBA
                , GL_UNSIGNED_BYTE
                , pImgData ) ;
    free( pImgData ) ;
}




/*! \brief Destruct object to set render material
*/
QdMaterial::~QdMaterial()
{
    for( int iTex = 0 ; iTex < NUM_TEXTURES_MAX ; ++ iTex )
    {
        if( glIsTexture( mTextures[ iTex ] ) )
        {
            GLuint texName = mTextures[ iTex ] ;
            glDeleteTextures( 1 , & texName ) ;
        }
        if( glIsTexture( mTexName ) )
        {
            glDeleteTextures( 1 , & mTexName ) ;
        }
    }
}




/*! \brief Set material
*/
void QdMaterial::SetMaterial( void )
{
    SetDefaultMaterialRenderState() ;
    glEnable( GL_TEXTURE_2D ) ;
    //glDisable( GL_TEXTURE_2D ) ; // When rendering points, disable textures
    glBindTexture( GL_TEXTURE_2D , mTexName ) ;
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_WRAP_S     , GL_REPEAT );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_WRAP_T     , GL_REPEAT );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MIN_FILTER , GL_NEAREST_MIPMAP_NEAREST );
    glTexParameteri(GL_TEXTURE_2D           , GL_TEXTURE_MAG_FILTER , GL_NEAREST_MIPMAP_NEAREST );
}
