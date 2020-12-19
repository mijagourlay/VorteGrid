/*! \file textureProc.h

    \brief Texture creation process.

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
#ifndef TEXTURE_PROC_H
#define TEXTURE_PROC_H

#include "qdImage.h"


/// Texture creation process abstract base class.
class ITextureProc
{
    public:
        virtual void operator()( QdImage & textureImage , size_t numPages , size_t pageIndex ) = 0 ;
} ;




/// Texture creation process to make a noise texture.
class NoiseTextureProc : public ITextureProc
{
    public:
        NoiseTextureProc( float gamma , const Vec4 & tint )
            : mGamma( gamma )
            , mTint( tint )
        {}

        virtual void operator()( QdImage & textureImage , size_t numPages , size_t pageIndex ) ;

        float   mGamma  ;
        Vec4    mTint   ;
} ;




/// Texture creation process to make a noisy ball texture.
class NoiseBallTextureProc : public NoiseTextureProc
{
    public:
        NoiseBallTextureProc( float gamma , const Vec4 tint )
            : NoiseTextureProc( gamma , tint )
        {}

        virtual void operator()( QdImage & textureImage , size_t numPages , size_t pageIndex ) ;
} ;




/// Texture creation process to make a decorated noisy ball texture.
class DecoratedNoiseBallTextureProc : public NoiseBallTextureProc
{
    public:
        DecoratedNoiseBallTextureProc( float gamma , const Vec4 tint )
            : NoiseBallTextureProc( gamma , tint )
        {}

        virtual void operator()( QdImage & textureImage , size_t numPages , size_t pageIndex ) ;
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
