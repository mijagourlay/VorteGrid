/** \file imgOpLinearGradient.cpp

    \brief Operation to apply a linear gradient to an image.

    \author Copyright 2012-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
#   include <windows.h>
#endif

#include "imgOpLinearGradient.h"

namespace PeGaSys
{

    /** Apply a linear gradient to the given image.

        \param  direction   Direction vector (in uv space) of gradient.

        \param  colorMin    Color of start side of gradient.

        \param  colorMax    Color of finish side of gradient.
    */
    Image & ImgOpLinearGradient::Gradient( Image & image , const Vec2 & direction , const Vec4 & colorMin , const Vec4 & colorMax )
    {
        ASSERT( 4 == image.GetNumChannels() ) ; // TODO: FIXME: Current implementation assumes image has 4 channels.
        const float     oneOverWidth    = 1.0f / float( image.GetWidth()  ) ;
        const float     oneOverHeight   = 1.0f / float( image.GetHeight() ) ;
        const unsigned  xStride         = image.GetXStride() ;
        const unsigned  yStride         = image.GetYStride() ;
        const float     alongMax        = direction.x + direction.y ;
        const float     oneOverAlongMax = 1.0f / alongMax ;
        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            const float y = float( iy ) * oneOverHeight ;
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                const unsigned offset = xStride * ix + yStride * iy ;
                const float    x      = float( ix ) * oneOverWidth ;
                const float    along  = ( x * direction.x + y * direction.y ) * oneOverAlongMax ;
                ASSERT( along >= 0.0f ) ;
                ASSERT( along <= 1.0f ) ;
                // NOTE: Formula above assumes |direction|=1.
                // NOTE: FIXME: "along" can be outside [0,1] hence "color" can be outside [colorMin,colorMax].  To correct, either change oneOverWidth and oneOverHeight or change the values of x and y so that |<x,y>| is in [0,1].
                const Vec4     color  = along * colorMax + ( 1.0f - along ) * colorMin ;
                image[ offset + 0 ] = PixelInt( PixelFloat( image[ offset + 0 ] ) * color.x ) ;
                image[ offset + 1 ] = PixelInt( PixelFloat( image[ offset + 1 ] ) * color.y ) ;
                image[ offset + 2 ] = PixelInt( PixelFloat( image[ offset + 2 ] ) * color.z ) ;
                image[ offset + 3 ] = PixelInt( PixelFloat( image[ offset + 3 ] ) * color.w ) ;
            }
        }

        return image ;
    }
} ;
