/*! \file qdImage.cpp

    \brief Image representation and modification.

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "wrapperMacros.h"
#include "qdImage.h"

#pragma warning( disable: 4244 )


static const float  sAlmost256  = 256.0f * ( 1.0f - FLT_EPSILON ) ;
static const float  sOneOver255 = 1.0f / 255.0f ;

QdImage::QdImage()
    : mWidth( 0 )
    , mHeight( 0 )
    , mNumChannels( 0 )
    , mImgData( 0 )
    , mOwnImageData( true )
{}




QdImage::QdImage( const QdImage & that )
    : mWidth( 0 )
    , mHeight( 0 )
    , mNumChannels( 0 )
    , mImgData( 0 )
    , mOwnImageData( true )
{
    this->operator=( that ) ;
}




QdImage & QdImage::operator=( const QdImage & that )
{
    if( this == & that ) return * this ; // Do nothing for self-assignment.
    if( mImgData && mOwnImageData )
    {   // This image already has data.
        FreeImageData() ; // Free it before allocating more, to avoid a leak.
    }
    memcpy( this , & that , sizeof( * this ) ) ;            // Shallow-copy all members.
    mImgData = 0 ;                                          // Forget original data.
    AllocateImageData() ;                                   // Allocate new image memory.
    memcpy( mImgData , that.mImgData , SizeInBytes() ) ;    // Copy image data.
    return * this ;
}




/// Make a shallow copy of this image, to access only a single page within it.
QdImage::QdImage( const QdImage & multiPageImage , unsigned pageIndex , unsigned numPages )
    : mWidth( multiPageImage.mWidth )
    , mHeight( multiPageImage.mHeight / numPages )
    , mNumChannels( multiPageImage.mNumChannels )
    , mImgData( multiPageImage.mImgData + SizeInBytes() * pageIndex )
    , mOwnImageData( false )
{
}




QdImage::~QdImage()
{
    if( mOwnImageData && mImgData )
    {
        FreeImageData() ;
    }
}




void QdImage::AllocateImageData()
{
    mImgData = (unsigned char *) malloc( SizeInBytes() ) ;
}




void QdImage::FreeImageData()
{
    free( mImgData ) ;
    mImgData = 0 ;
}




/// Copy the shape from given image and allocate image data buffer.
void QdImage::CopyShape( const QdImage & that )
{
    if( this == & that ) return ; // Do nothing for self-assignment.
    if( mImgData && mOwnImageData )
    {   // This image already has data.
        FreeImageData() ; // Free it before allocating more, to avoid a leak.
    }
    memcpy( this , & that , sizeof( * this ) ) ;            // Shallow-copy all members.
    mImgData = 0 ;                                          // Forget original data.
    AllocateImageData() ;                                   // Allocate new image memory.
}




static inline float PixelFloat( const unsigned char & pixelValue )
{
    return float( pixelValue ) * sOneOver255 ;
}




static inline float PixelInt( const float & pixelValue )
{
    return pixelValue * sAlmost256 ;
}




/** Blur the given image pixel.
*/
static inline unsigned char blur3x3( int ix , int iy , int ic , const unsigned char * img , int nx , int ny , int nc )
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




/** Blur the given image.
*/
void QdImage::Blur( unsigned numSmoothingPasses )
{
    const size_t numBytesPerImage = SizeInBytes() ;

    // Create a temp space where blurred pixels are written.
    // This routine reads from the texture and writes to the temp space.
    // This is because the blur formula would otherwise mix blurred and non-blurred pixels.
    unsigned char * pImgTemp = (unsigned char *) malloc( numBytesPerImage ) ;
    // Copy texture to temp space.
    memcpy( pImgTemp , mImgData , numBytesPerImage ) ;

    for( unsigned iSmooth = 0 ; iSmooth < numSmoothingPasses ; ++ iSmooth )
    {
        for( unsigned iyDst = 0 ; iyDst < mHeight ; ++ iyDst )
        {
            const unsigned iySrc = iyDst ;
            for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
            {
                const unsigned offset = mNumChannels * ( ix + mWidth * iyDst ) ;
                pImgTemp[ offset + 0 ] = blur3x3( ix , iySrc , 0 , mImgData , mWidth , mHeight , mNumChannels ) ;
                pImgTemp[ offset + 1 ] = blur3x3( ix , iySrc , 1 , mImgData , mWidth , mHeight , mNumChannels ) ;
                pImgTemp[ offset + 2 ] = blur3x3( ix , iySrc , 2 , mImgData , mWidth , mHeight , mNumChannels ) ;
                // Note the lack of smoothing in the 4th channel.  Assuming it's uniform.
            }
        }
        // Transfer blurred image from temp space to texture.
        memcpy( mImgData , pImgTemp , numBytesPerImage ) ;
    }
    free( pImgTemp ) ;
}




/** Create an image of noise.
*/
void QdImage::MakeNoise()
{
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            const unsigned iNoise = rand() & 0xff ;
            const unsigned offset = mNumChannels * ( ix + mWidth * iy ) ;
            mImgData[ offset + 0 ] = iNoise ; // red
            mImgData[ offset + 1 ] = iNoise ; // green
            mImgData[ offset + 2 ] = iNoise ; // blue
            mImgData[ offset + 3 ] = 0xff   ; // alpha (opacity)
        }
    }
}




/** Modulate image opacity by a centered ellipse.
*/
void QdImage::FadeAlphaRadially( float power , float alphaMax )
{
    const float     p           = power * 0.5f ;
    const float     xSpacing    = 1.0f / float( mWidth  - 1 ) ;
    const float     ySpacing    = 1.0f / float( mHeight - 1 ) ;
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        const float y = 2.0f * ( float( iy ) * ySpacing - 0.5f ) ; // value in [-1,1]
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            const float     x           = 2.0f * ( float( ix ) * xSpacing - 0.5f ) ; // value in [-1,1]
            const float     r2          = x * x + y * y ;
            const float     alpha0to1   = powf( CLAMP( 1.0f - r2 , 0.0f , 1.0f ) , p ) ;
            const float     alpha0to255 = alpha0to1 * sAlmost256 * alphaMax ;
            const unsigned  offset      = mNumChannels * ( ix + mWidth * iy ) ;
            const unsigned  iAlpha      = (unsigned) alpha0to255 ;
            mImgData[ offset + 3 ] = iAlpha ; // alpha (opacity)
        }
    }
}




/** Apply brightness correction to the given image data.
*/
void QdImage::GammaCorrect( float gamma )
{
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            const unsigned  offset  = mNumChannels * ( ix + mWidth * iy ) ;
            mImgData[ offset + 0 ] = PixelInt( powf( PixelFloat( mImgData[ offset + 0 ] ) , gamma ) ) ;
            mImgData[ offset + 1 ] = PixelInt( powf( PixelFloat( mImgData[ offset + 1 ] ) , gamma ) ) ;
            mImgData[ offset + 2 ] = PixelInt( powf( PixelFloat( mImgData[ offset + 2 ] ) , gamma ) );
        }
    }
}




/** Apply a tint to the given image data.
*/
void QdImage::Tint( const Vec4 & color )
{
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            const unsigned  offset  = mNumChannels * ( ix + mWidth * iy ) ;
            mImgData[ offset + 0 ] = PixelInt( PixelFloat( mImgData[ offset + 0 ] ) * color.x ) ;
            mImgData[ offset + 1 ] = PixelInt( PixelFloat( mImgData[ offset + 1 ] ) * color.y ) ;
            mImgData[ offset + 2 ] = PixelInt( PixelFloat( mImgData[ offset + 2 ] ) * color.z ) ;
            mImgData[ offset + 3 ] = PixelInt( PixelFloat( mImgData[ offset + 3 ] ) * color.w ) ;
        }
    }
}




/** Draw a box along the perimeter of the image.
*/
void QdImage::DrawBox( unsigned thickness )
{
    const unsigned xEnd = mWidth  - thickness ;
    const unsigned yEnd = mHeight - thickness ;
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            int value = 0 ;
            if(     ( ( ix < thickness ) || ( ix >= xEnd ) )
                ||  ( ( iy < thickness ) || ( iy >= yEnd ) )    )
            {   // Pixel is on box perimeter.
                value = 255 ;
            }
            else
            {
                value = 0 ;
            }
            const unsigned  offset = mNumChannels * ( ix + mWidth * iy ) ;
            mImgData[ offset + 0 ] =
            mImgData[ offset + 1 ] =
            mImgData[ offset + 2 ] =
            mImgData[ offset + 3 ] = value ;
        }
    }
}




/** Draw a circle.

    \param radius       Radius in pixels.

    \param thickness    Thickness in pixels.
*/
void QdImage::DrawCircle( float radius , float thickness )
{
    const int   centerX    = mWidth  / 2 ;
    const int   centerY    = mHeight / 2 ;
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        const int distFromCenterY  = iy - centerY ;
        const int dist2FromCenterY = POW2( distFromCenterY ) ;
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            int value = 0 ;
            const int   distFromCenterX  = ix - centerX ;
            const int   dist2FromCenterX = POW2( distFromCenterX ) ;
            const int   dist2FromCenter  = dist2FromCenterX + dist2FromCenterY ;
            const float distFromRing     = fabsf( fsqrtf( float( dist2FromCenter ) ) - radius ) ;
            if( distFromRing <= thickness )
            {   // Pixel is on ring perimeter.
                value = 255 ;
            }
            else
            {
                value = 0 ;
            }
            const unsigned  offset = mNumChannels * ( ix + mWidth * iy ) ;
            mImgData[ offset + 0 ] =
            mImgData[ offset + 1 ] =
            mImgData[ offset + 2 ] =
            mImgData[ offset + 3 ] = value ;
        }
    }
}




/** Overlay this image over the given background image.

    \param destinationImage Can be either "this" or backgroundImage (or neither).

    \param backgroundImage  Image behind this one.

    Applies an alpha blend.
*/
void QdImage::Overlay( QdImage & destinationImage , const QdImage & backgroundImage )
{
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            const unsigned  offset                 = mNumChannels * ( ix + mWidth * iy ) ;
            float           overlayOpacity         = PixelFloat( mImgData[ offset + 3 ] ) ;
            float           oneMinusOverlayOpacity = 1.0f - overlayOpacity ;
            destinationImage.mImgData[ offset + 0 ] = PixelInt( PixelFloat( backgroundImage.mImgData[ offset + 0 ] ) * oneMinusOverlayOpacity + PixelFloat( mImgData[ offset + 0 ] ) * overlayOpacity ) ;
            destinationImage.mImgData[ offset + 1 ] = PixelInt( PixelFloat( backgroundImage.mImgData[ offset + 1 ] ) * oneMinusOverlayOpacity + PixelFloat( mImgData[ offset + 1 ] ) * overlayOpacity ) ;
            destinationImage.mImgData[ offset + 2 ] = PixelInt( PixelFloat( backgroundImage.mImgData[ offset + 2 ] ) * oneMinusOverlayOpacity + PixelFloat( mImgData[ offset + 2 ] ) * overlayOpacity ) ;
            destinationImage.mImgData[ offset + 3 ] = PixelInt( PixelFloat( backgroundImage.mImgData[ offset + 3 ] ) * oneMinusOverlayOpacity + PixelFloat( mImgData[ offset + 3 ] ) * overlayOpacity ) ;

            //printf("%i %i %i %i , " , destinationImage.mImgData[ offset + 0 ] , destinationImage.mImgData[ offset + 1 ] , destinationImage.mImgData[ offset + 2 ] , destinationImage.mImgData[ offset + 3 ] ) ;
        }
        //printf("\n") ;
    }
}




/** Write out the alpha channel from a given image as a PGM file.
*/
void QdImage::WriteAlphaToPGM( const char * filename )
{
    FILE * fp = fopen( filename , "wb" ) ;
    fprintf( fp , "P5 %i %i 255\n" , mWidth , mHeight ) ;
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            const unsigned  offset  = mNumChannels * ( ix + mWidth * iy ) ;
            // Write alpha value as a byte
            fwrite( & mImgData[ offset + 3 ] , 1 , 1 , fp ) ;
        }
    }
    fclose( fp ) ;
}




/** Write out the given image as a PPM file.
*/
void QdImage::WritePPM( const char * filename )
{
    FILE * fp = fopen( filename , "wb" ) ;
    fprintf( fp , "P6 %i %i 255\n" , mWidth , mHeight ) ;
    for( unsigned iy = 0 ; iy < mHeight ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < mWidth ; ++ ix )
        {
            const unsigned  offset  = mNumChannels * ( ix + mWidth * iy ) ;
            // Write red, green, blue values as bytes
            fwrite( & mImgData[ offset + 0 ] , 1 , 3 , fp ) ;
        }
    }
    fclose( fp ) ;
}
