/** \file imgOpRadialGradient.cpp

    \brief Operation to create a radial gradient in one component of an image.

    \author Copyright 2012-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
#   include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "ImgOpRadialGradient.h"

namespace PeGaSys
{

    /** Assign a radial gradient in one component of an image.

        \param power        Exponent used in formula pixelValue=max*(1-r^2)^power

        \param max          Maximum value that pixelValue takes.

        \param channelIndex Which image channel to assign. TODO: Omit this.  Use ExtractChannel constructor instead.
    */
    Image & ImgOpRadialGradient::RadialGradient( Image & image , float power , float max , int channelIndex )
    {
        ASSERT( ( max >= 0.0f ) && ( max <= 1.0f ) ) ;
        const float p           = power * 0.5f ;
        const float xSpacing    = 1.0f / float( image.GetWidth()  - 1 ) ;
        const float ySpacing    = 1.0f / float( image.GetHeight() - 1 ) ;

        const unsigned xStride = image.GetXStride() ;
        const unsigned yStride = image.GetYStride() ;
        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            const float y = 2.0f * ( float( iy ) * ySpacing - 0.5f ) ; // value in [-1,1]
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                const float     x           = 2.0f * ( float( ix ) * xSpacing - 0.5f ) ; // value in [-1,1]
                const float     r2          = x * x + y * y ;
                const float     alpha0to1   = powf( Clamp( 1.0f - r2 , 0.0f , 1.0f ) , p ) ;
                ASSERT( ( alpha0to1 >= 0.0f ) && ( alpha0to1 <= 1.0f ) ) ;
                const float     alpha0to255 = alpha0to1 * sAlmost256 * max ;
                ASSERT( ( alpha0to255 >= 0.0f ) && ( alpha0to255 < 256.0f ) ) ;
                const unsigned  offset  = xStride * ix + yStride * iy ;
                const unsigned  iGradient   = static_cast< unsigned >( alpha0to255 ) ;
                ASSERT( ( iGradient >= 0 ) && ( iGradient < 256 ) ) ;
                const unsigned char cGrad   = static_cast< unsigned char >( iGradient ) ;
                image[ offset + channelIndex ] = cGrad ;
            }
        }

        return image ;
    }

} ;