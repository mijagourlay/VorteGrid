/** \file image.h

    \brief Image representation and modification.

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMAGE_H
#define PEGASYS_IMAGE_H

#include "Core/Utility/macros.h"

namespace PeGaSys
{
    /** Image.
    */
    class Image
    {
        public:
            enum CropRegion         { CROP_REGION       } ; // Token used to choose constructor for making a shallow-copy image that is a cropped region of the source image.
            enum ExtractChannel     { EXTRACT_CHANNEL   } ; // Token used to choose constructor for making a shallow-copy image that extracts a single channel from the source image.
            enum ExtractPage        { EXTRACT_PAGE      } ; // Token used to choose constructor for making a shallow-copy image that extracts a single page from the source image.
            enum ConcatenatePages   { CONCATENATE_PAGES } ; // Token used to choose constructor for making a shallow-copy image that has a single page with all pages from the source image concatenated vertically.

            Image() ;
            Image( unsigned width , unsigned height , unsigned numChannels , unsigned numPages ) ;
            explicit Image( const Image & that ) ;
            Image( const Image & multiPageImage , CropRegion , unsigned xFirst , unsigned yFirst , unsigned xLast , unsigned yLast ) ;
            Image( const Image & multiPageImage , ExtractChannel , unsigned channelIndex ) ;
            Image( const Image & multiPageImage , ExtractPage , unsigned pageIndex ) ;
            Image( const Image & multiPageImage , ConcatenatePages ) ;

            ~Image() ;

            Image & operator=( const Image & that ) ;

            /// Return image width in pixels.
            const unsigned & GetWidth() const
            {
                return mWidth ;
            }

            /// Return image height in pixels.
            const unsigned & GetHeight() const
            {
                return mHeight ;
            }

            /// Return number of channels in this image.
            const unsigned & GetNumChannels() const
            {
                return mNumChannels ;
            }

            /// Return number of pages in this image.
            const unsigned & GetNumPages() const
            {
                return mNumPages ;
            }

            /// Return number of bytes between each horizontally adjacent pixel in this image.
            unsigned    GetXStride() const
            {
                return mXStride ;
            }

            /// Return number of bytes between vertically adjacent pixels in this image.
            unsigned GetYStride() const
            {
                return mYStride ;
            }

            /// Return number of bytes between corresponding pixels in adjacent pages in this image.
            unsigned GetPageStride() const
            {
                return mPageStride ;
            }

            /// Return pixel at offset.
            unsigned char & operator[]( size_t offset )
            {
                ASSERT( offset < GetPageStride() ) ; // Approximate paranoid sanity check
                return mImgData[ offset ] ;
            }

            /// Return pixel at offset.
            const unsigned char & operator[]( size_t offset ) const
            {
                ASSERT( offset < GetPageStride() ) ; // Approximate paranoid sanity check
                return mImgData[ offset ] ;
            }

            const unsigned char * GetImageData() const { return mImgData ; }
                  unsigned char * GetImageData()       { return mImgData ; }

            void CopyShape( const Image & that ) ;
            void CopyImageData( const Image & that ) ;

        private:
            void    AllocateImageData() ;
            void    FreeImageData() ;

            /** Return number of bytes of underlying image data, including all pages.

                \note   This returns the size the image data would occupy if allocated.
                        It does not depend on whether the image data is actually allocated.

            */
            size_t SizeInBytes() const
            {
                return mPageStride * mNumPages ;
            }

            unsigned        mNumChannels    ;   ///< Number of channels in image, one byte per channel.
            unsigned        mWidth          ;   ///< Image width, in pixels.
            unsigned        mHeight         ;   ///< Image height, in pixels.
            unsigned        mNumPages       ;   ///< Number of pages in this image.  That is, the number of images in this image array.
            unsigned        mXStride        ;   ///< Number of bytes between horizontally adjacent pixels.
            unsigned        mYStride        ;   ///< Number of bytes between vertically adjacent pixels.
            unsigned        mPageStride     ;   ///< Number of bytes between corresponding pixel in adjacent pages.
            unsigned char * mImgData        ;   ///< Raw image data.
            bool            mOwnImageData   ;   ///< Whether this object owns the image data.
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
