/** \file image.cpp

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
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-14/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-15/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "image.h"

#pragma warning( disable: 4244 )


namespace PeGaSys
{

static const float  sAlmost256  = 256.0f * ( 1.0f - FLT_EPSILON ) ;
static const float  sOneOver255 = 1.0f / 255.0f ;

Image::Image()
    : mWidth( 0 )
    , mHeight( 0 )
    , mNumChannels( 0 )
    , mNumPages( 0 )
    , mImgData( 0 )
    , mOwnImageData( true )
{}




Image::Image( unsigned width , unsigned height , unsigned numChannels , unsigned numPages , bool ownImageData )
    : mWidth( width )
    , mHeight( height )
    , mNumChannels( numChannels )
    , mNumPages( numPages )
    , mImgData( 0 )
    , mOwnImageData( ownImageData )
{
    if( mOwnImageData )
    {
        AllocateImageData() ;
    }
}




Image::Image( const Image & that )
    : mWidth( 0 )
    , mHeight( 0 )
    , mNumChannels( 0 )
    , mImgData( 0 )
    , mNumPages( 0 )
    , mOwnImageData( true )
{
    this->operator=( that ) ;
}




Image & Image::operator=( const Image & that )
{
    if( this == & that ) return * this ; // Do nothing for self-assignment.
    ASSERT( that.mOwnImageData ) ; // Cannot deep-copy a shallow copy.
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




// TODO: FIXME: Remove this version.
/// Make this a shallow copy of the given multi-page image, to access only a single page within it.
Image::Image( const Image & multiPageImage , unsigned pageIndex , unsigned numPages )
    : mWidth( multiPageImage.mWidth )
    , mHeight( multiPageImage.mHeight / numPages )
    , mNumChannels( multiPageImage.mNumChannels )
    , mNumPages( 1 )
    , mImgData( multiPageImage.mImgData + SizeInBytes() * pageIndex )
    , mOwnImageData( false )
{
FAIL() ; // OBSOLETE
}




/// Make this a shallow copy of the given multi-page image, to access only a single page within it.
Image::Image( const Image & multiPageImage , unsigned pageIndex )
    : mWidth( multiPageImage.mWidth )
    , mHeight( multiPageImage.mHeight )
    , mNumChannels( multiPageImage.mNumChannels )
    , mNumPages( 1 )
    , mImgData( multiPageImage.mImgData + SizeInBytes() * pageIndex )
    , mOwnImageData( false )
{
    ASSERT( multiPageImage.mImgData ) ; // Make sure multiPageImage has already allocated image data, otherwise selecting specific pages means nothing.
}




Image::~Image()
{
    if( mOwnImageData && mImgData )
    {
        FreeImageData() ;
    }
}




void Image::AllocateImageData()
{
    ASSERT( SizeInBytes() > 0 ) ;
    ASSERT( 0 == mImgData ) ; // Otherwise memory leaks.
    ASSERT( mOwnImageData ) ;
    mImgData = (unsigned char *) malloc( SizeInBytes() ) ;
    ASSERT( mImgData != 0 ) ;
}




void Image::FreeImageData()
{
    ASSERT( mOwnImageData ) ;
    ASSERT( mImgData ) ;
    free( mImgData ) ;
    mImgData = 0 ;
}




/// Copy the shape from given image and allocate image data buffer.
void Image::CopyShape( const Image & that )
{
    if( this == & that ) return ; // Do nothing for self-assignment.
    if( mImgData && mOwnImageData )
    {   // This image already has data.
        FreeImageData() ; // Free it before allocating more, to avoid a leak.
    }
    memcpy( this , & that , sizeof( * this ) ) ;            // Shallow-copy all members.
    mImgData = 0 ;                                          // Forget original data.
    mOwnImageData = true ;                                  // This image owns its image data even if original didn't.
    AllocateImageData() ;                                   // Allocate new image memory.
}




#if 0
/** Write out the alpha channel from a given image as a PGM file.
*/
void Image::WriteAlphaToPGM( const char * filename )
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
void Image::WritePPM( const char * filename )
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
#endif

} ;