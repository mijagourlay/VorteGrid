/** \file image.cpp

    \brief Image representation and modification.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "Image/image.h"

#pragma warning( disable: 4244 )


namespace PeGaSys
{

Image::Image()
    : mNumChannels( 0 )
    , mWidth( 0 )
    , mHeight( 0 )
    , mNumPages( 0 )
    , mXStride( 0 )
    , mYStride( 0 )
    , mPageStride( 0 )
    , mImgData( 0 )
    , mOwnImageData( true )
{}




/** Create a multi-page image.

    \param  width   Image width in pixels.

    \param  height  Image height in pixels. If this is a multi-page image, this is the height of each image.

    \param  numPages    Number of image pages.  An Image object can hold multi-page images, in an image array.
                        Each element in the array is called a "page".

    \param  ownImageData    Whether this image object owns its image data.  If true, then this object allocates
                            image data on construction and when this object is destructed, it deletes the image
                            data.  If false, this object holds a shallow copy of the image data, so does not
                            delete it upon destruction.

*/
Image::Image( unsigned width , unsigned height , unsigned numChannels , unsigned numPages )
    : mNumChannels( numChannels )
    , mWidth( width )
    , mHeight( height )
    , mNumPages( numPages )
    , mXStride( numChannels )
    , mYStride( mXStride * width )
    , mPageStride( mYStride * height )
    , mImgData( 0 )
    , mOwnImageData( true )
{
    if( mOwnImageData )
    {
        AllocateImageData() ;
    }
}




/** Copy-construct an Image.
*/
Image::Image( const Image & that )
    : mNumChannels( 0 )
    , mWidth( 0 )
    , mHeight( 0 )
    , mNumPages( 0 )
    , mXStride( 0  )
    , mYStride( 0 )
    , mPageStride( 0 )
    , mImgData( 0 )
    , mOwnImageData( true )
{
    this->operator=( that ) ;
}




/** Copy image data from that to this.
*/
void Image::CopyImageData( const Image & that )
{
    if( this == & that ) return ; // Do nothing for self-assignment.

    ASSERT( that.mImgData ) ; // source image must have data.

    ASSERT( GetWidth() == that.GetWidth() ) ;
    ASSERT( GetHeight() == that.GetHeight() ) ;
    ASSERT( GetNumChannels() == that.GetNumChannels() ) ;
    ASSERT( GetNumPages() == that.GetNumPages() ) ;

    if(     ( GetYStride() == GetXStride() * GetWidth() )
        &&  ( GetPageStride() == GetYStride() * GetHeight() )
        &&  ( that.GetYStride() == that.GetXStride() * that.GetWidth() )
        &&  ( that.GetPageStride() == that.GetYStride() * that.GetHeight() )
        )
    {   // Rows are contiguous in both source and dst images.  Plain memcpy will work.
        ASSERT( SizeInBytes() == that.SizeInBytes() ) ; // Paranoid sanity check.
        memcpy( GetImageData() , that.GetImageData() , SizeInBytes() ) ;
    }
    else
    {   // Rows are NOT contiguous in either source or dst images.  Plain memcpy will NOT work.
        FAIL() ; // TODO: FIXME: Implement me.  Copy each row separately.
    }
}




/** Copy an image via assignment.
*/
Image & Image::operator=( const Image & that )
{
    CopyShape( that ) ;
    CopyImageData( that ) ;
    return * this ;
}




/** Make this a shallow copy of the given image, to access only a cropped region within it.

    Such an object is useful for accessing a cropped region of an image, such as for performing image manipulation
    operations on a cropped region.

    \note   This extracted image object accesses only the first page of the source image.  If you want to access another
            page, then first use one of the single-from-multiple page shallow constructors to extract the desired page.

    \param xFirst   Index into source image of first pixel along x direction.

    \param yFirst   Index into source image of first pixel along y direction.

    \param cropWidth Width in pixels of cropped region.

    \param cropHeight Height in pixels of cropped region.

    \note Since this is a shallow copy, the original image must remain viable until this object is destructed.

*/
Image::Image( const Image & sourceImage , CropRegion , unsigned xFirst , unsigned yFirst , unsigned cropWidth , unsigned cropHeight )
    : mNumChannels( sourceImage.mNumChannels )
    , mWidth( cropWidth )
    , mHeight( cropHeight )
    , mNumPages( 1 )
    , mXStride( sourceImage.mXStride )
    , mYStride( sourceImage.mYStride )
    , mPageStride( sourceImage.mPageStride )
    , mImgData( sourceImage.mImgData + xFirst * sourceImage.mXStride + yFirst * sourceImage.mYStride )
    , mOwnImageData( false )
{
FAIL() ; // Untested. TODO: Test me.
    ASSERT( xFirst <= sourceImage.GetWidth()  ) ;           // Crop region left   must be inside source image
    ASSERT( yFirst <= sourceImage.GetHeight() ) ;           // Crop region top    must be inside source image
    ASSERT( xFirst + mWidth  < sourceImage.GetWidth()  ) ;  // Crop region right  must be inside source image
    ASSERT( yFirst + mHeight <= sourceImage.GetHeight() ) ; // Crop region bottom must be inside source image
    ASSERT( sourceImage.mImgData ) ; // Make sure multiPageImage has already allocated image data, otherwise selecting specific pages means nothing.
}




/** Make this a shallow copy of the given multi-page image, to access only a single component within it.

    Such a single-component object is useful for accessing a single component in an Image object with multiple channels,
    since some manipulation routines only access the first channel of an image.

    \note   This extracted image object accesses only the first page of the source image.  If you want to access another
            page, then first use one of the single-from-multiple page shallow constructors to extract the desired page.

    \note Since this is a shallow copy, the original image must remain viable until this object is destructed.

*/
Image::Image( const Image & sourceImage , ExtractChannel , unsigned channelIndex )
    : mNumChannels( sourceImage.mNumChannels )
    , mWidth( sourceImage.mWidth )
    , mHeight( sourceImage.mHeight )
    , mXStride( sourceImage.mXStride )
    , mYStride( sourceImage.mYStride )
    , mPageStride( sourceImage.mPageStride )
    , mNumPages( 1 )
    , mImgData( sourceImage.mImgData + channelIndex )
    , mOwnImageData( false )
{
FAIL() ; // Untested. TODO: Test me.
    ASSERT( channelIndex < sourceImage.GetNumChannels() ) ;
    ASSERT( sourceImage.GetNumChannels() > 1 ) ; // It is nonsensical to use this constructor on a single-channel image.
    ASSERT( sourceImage.mImgData ) ; // Make sure multiPageImage has already allocated image data, otherwise selecting specific pages means nothing.
}




/** Make this a shallow copy of the given multi-page image, to access only a single page within it.

    Such a single-page object is useful for accessing a single page in a multi-page Image object,
    since most manipulation routines only access the first page of an image.

    \note Since this is a shallow copy, the original image must remain viable until this object is destructed.

*/
Image::Image( const Image & multiPageImage , ExtractPage , unsigned pageIndex )
    : mNumChannels( multiPageImage.mNumChannels )
    , mWidth( multiPageImage.mWidth )
    , mHeight( multiPageImage.mHeight )
    , mNumPages( 1 )
    , mXStride( multiPageImage.mXStride )
    , mYStride( multiPageImage.mYStride )
    , mPageStride( multiPageImage.mPageStride )
    , mImgData( multiPageImage.mImgData + multiPageImage.GetPageStride() * pageIndex )
    , mOwnImageData( false )
{
    ASSERT( multiPageImage.GetNumPages() > 1 ) ; // It is nonsensical to use this constructor on a single-page image.
    ASSERT( multiPageImage.mImgData ) ; // Make sure multiPageImage has already allocated image data, otherwise selecting specific pages means nothing.
}




/** Make this a shallow copy of the given multi-page image, to concatenate all pages as a vertical "film strip".

    Such a single-page object is useful for accessing the entire image array as a single image, where that image
    is numPages times as tall as each image in the original array.  This is useful for creating a single
    texture out of a multi-page image.

    \note Since this is a shallow copy, the original image must remain viable until this object is destructed.

*/
Image::Image( const Image & multiPageImage , ConcatenatePages )
    : mNumChannels( multiPageImage.mNumChannels )
    , mWidth( multiPageImage.mWidth )
    , mHeight( multiPageImage.mHeight * multiPageImage.mNumPages )
    , mNumPages( 1 )
    , mXStride( multiPageImage.mXStride )
    , mYStride( multiPageImage.mYStride )
    , mPageStride( mYStride * mHeight )
    , mImgData( multiPageImage.mImgData )
    , mOwnImageData( false )
{
FAIL() ; // Untested. TODO: Test me.
    ASSERT( multiPageImage.GetNumPages() > 1 ) ; // It is nonsensical to use this constructor on a single-page image.
    ASSERT( multiPageImage.mImgData ) ; // Make sure multiPageImage has already allocated image data, otherwise this shallow copy is useless.
}




Image::~Image()
{
    if( mOwnImageData && mImgData )
    {
        FreeImageData() ;
    }
}




/**
*/
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
    memcpy( this , & that , sizeof( * this ) ) ;    // Shallow-copy all members.
    mImgData = 0 ;                                  // Forget original data (which shallow copy copied).
    mOwnImageData = true ;                          // This image owns its image data even if original didn't.
    AllocateImageData() ;                           // Allocate new image memory.
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