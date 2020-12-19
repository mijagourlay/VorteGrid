/** \file image.h

    \brief Image.

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMAGE_H
#define PEGASYS_IMAGE_H

namespace PeGaSys
{
    /** Image.
    */
    class Image
    {
        public:
            Image() ;
            Image( unsigned width , unsigned height , unsigned numChannels , unsigned numPages , bool ownImageData = true ) ;
            explicit Image( const Image & that ) ;
            Image( const Image & multiPageImage , unsigned pageIndex , unsigned numPages ) ;
            Image( const Image & multiPageImage , unsigned pageIndex ) ;
            ~Image() ;

            Image & operator=( const Image & that ) ;

            /** Return number of bytes of image data, including all pages.

                \note   This returns the size the image data would occupy if allocated.
                        It does not depend on whether the image data is actually allocated.
            */
            size_t SizeInBytes() const { return mWidth * mHeight * mNumChannels * mNumPages ; }

            void    AllocateImageData() ;
            void    FreeImageData() ;
            void    CopyShape( const Image & that ) ;

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

            /// Return number of bytes between each pixel in this image.
            unsigned    GetStride() const
            {
                return mNumChannels ;
            }

            /// Return number of bytes between each row of pixels in this image.
            unsigned GetRowStride() const
            {
                return GetStride() * mWidth ;
            }

            /// Return pixel at offset.
            unsigned char & operator[]( size_t offset )
            {
                ASSERT( offset < SizeInBytes() ) ;
                return mImgData[ offset ] ;
            }

            /// Return pixel at offset.
            const unsigned char & operator[]( size_t offset ) const
            {
                ASSERT( offset < SizeInBytes() ) ;
                return mImgData[ offset ] ;
            }

            const unsigned char * GetImageData() const { return mImgData ; }

        private:
            unsigned        mWidth          ;   ///< Image width, in pixels.
            unsigned        mHeight         ;   ///< Image height, in pixels.
            unsigned        mNumChannels    ;   ///< Number of channels in image, one byte per channel.
            unsigned        mNumPages       ;   ///< Number of pages in this image.  That is, the number of images in this image array.
            unsigned char * mImgData        ;   ///< Raw image data.
            bool            mOwnImageData   ;   ///< Whether this object owns the image data.
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
