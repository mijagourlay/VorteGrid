/*! \file qdImage.h

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef QD_IMAGE_H
#define QD_IMAGE_H

/// Image representation and modification.
class QdImage
{
    public:
        QdImage() ;
        explicit QdImage( const QdImage & that ) ;
        QdImage( const QdImage & multiPageImage , size_t pageIndex , size_t numPages ) ;
        ~QdImage() ;

        QdImage & operator=( const QdImage & that ) ;

        /** Return number of bytes of image data.

            \note   This returns the size the image data would occupy if allocated.
                    It does not depend on whether the image data is actually allocated.
        */
        size_t SizeInBytes() { return mWidth * mHeight * mNumChannels ; }

        void    AllocateImageData() ;
        void    FreeImageData() ;
        void    CopyShape( const QdImage & that ) ;
        void    MakeNoise() ;
        void    Blur( unsigned numSmoothingPasses ) ;
        void    FadeAlphaRadially( float power , float alphaMax ) ;
        void    GammaCorrect( float gamma ) ;
        void    Tint( const Vec4 & color ) ;
        void    DrawBox( unsigned thickness ) ;
        void    DrawCircle( float radius , float thickness ) ;
        void    Overlay( QdImage & destinationImage , const QdImage & backgroundImage ) ;

        void    WriteAlphaToPGM( const char * filename ) ;
        void    WritePPM( const char * filename ) ;

        unsigned        mWidth          ;   ///< Image width, in pixels.
        unsigned        mHeight         ;   ///< Image height, in pixels.
        unsigned        mNumChannels    ;   ///< Number of channels in image, one byte per channel.
        unsigned char * mImgData        ;   ///< Raw image data.
        bool            mOwnImageData   ;   ///< Whether this object owns the image data.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
