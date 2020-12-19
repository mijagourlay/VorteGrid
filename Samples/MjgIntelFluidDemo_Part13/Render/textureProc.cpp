/*! \file textureProc.cpp

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

#include "Core/Math/vec4.h"

#include "Core/wrapperMacros.h"
#include "textureProc.h"



void NoiseTextureProc::operator()( QdImage & textureImage , size_t numPages , size_t pageIndex )
{
    QdImage texturePage( textureImage , pageIndex , numPages ) ;   // Make shallow copy to access only given page.

    const unsigned  numSmoothingPasses  = texturePage.mHeight / 8 ;

    texturePage.MakeNoise() ;
    texturePage.Blur( numSmoothingPasses ) ;
    texturePage.GammaCorrect( mGamma ) ;
    texturePage.Tint( mTint ) ;
}




void NoiseBallTextureProc::operator()( QdImage & textureImage , size_t numPages , size_t pageIndex )
{
    QdImage texturePage( textureImage , pageIndex , numPages ) ;   // Make shallow copy to access only given page.

    const unsigned  numSmoothingPasses  = texturePage.mHeight / 2 ;
    texturePage.MakeNoise() ;

    {   // Inscribe a ring inside the texture.
        // This simulates limn darkening.
        QdImage ring ;
        ring.CopyShape( texturePage ) ;
        ring.DrawCircle( float( ring.mWidth / 2 - 4 ) , 2.1f ) ;
        Vec4 limnTint( mTint.x * 0.0f , mTint.y * 0.0f , mTint.z * 0.0f , mTint.w ) ;
        ring.Tint( limnTint ) ;
        ring.Overlay( texturePage , texturePage ) ;
    }

    texturePage.Blur( numSmoothingPasses ) ;
    texturePage.FadeAlphaRadially( 0.5f , 1.0f ) ;
    texturePage.GammaCorrect( mGamma ) ;
    texturePage.Tint( mTint ) ;

}



void DecoratedNoiseBallTextureProc::operator()( QdImage & textureImage , size_t numPages , size_t pageIndex )
{
    NoiseBallTextureProc::operator()( textureImage , numPages , pageIndex ) ;

    QdImage texturePage( textureImage , pageIndex , numPages ) ;   // Make shallow copy to access only given page.

    {   // Draw a faint frame around the texture border.
        QdImage frame ;
        frame.CopyShape( texturePage ) ;
        frame.DrawBox( 16 ) ;
        frame.Tint( Vec4( 1.0f , 1.0f , 1.0f , 0.25f ) ) ;
        frame.Overlay( texturePage , texturePage ) ;
    }

    {   // Inscribe a ring inside the texture.
        // This demarks the vorton boundary.
        QdImage ring ;
        ring.CopyShape( texturePage ) ;
        ring.DrawCircle( float( ring.mWidth / 2 - 4 ) , 2.1f ) ;
        ring.Blur( 2 ) ;
        ring.Tint( Vec4( 1.0f , 1.0f , 1.0f , 0.5f ) ) ;
        ring.Overlay( texturePage , texturePage ) ;
    }

    {   // Draw a faint dot at the center of the texture.
        QdImage dot ;
        dot.CopyShape( texturePage ) ;
        dot.DrawCircle( 2 , 2.1f ) ;
        dot.Blur( 2 ) ;
        dot.Tint( Vec4( 1.0f , 1.0f , 1.0f , 0.25f ) ) ;
        dot.Overlay( texturePage , texturePage ) ;
    }
}
