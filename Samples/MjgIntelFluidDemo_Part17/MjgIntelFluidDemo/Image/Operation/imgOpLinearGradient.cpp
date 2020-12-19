/** \file imgOpLinearGradient.cpp

    \brief Operation to apply a linear gradient to an image.

    \author Copyright 2012-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
#include <windows.h>
#endif

#include "imgOpLinearGradient.h"

namespace PeGaSys
{

    /** Apply a linear gradient to the given image.
    */
    Image & ImgOpLinearGradient::Gradient( Image & image , const Vec2 & direction , const Vec4 & colorMin , const Vec4 & colorMax )
    {
        const Vec4      colorRange    = colorMax - colorMin ;
        const float     oneOverWidth  = 1.0f / float( image.GetWidth()  ) ;
        const float     oneOverHeight = 1.0f / float( image.GetHeight() ) ;
        const unsigned  rowStride     = image.GetRowStride() ;
        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            const float y = float( iy ) * oneOverHeight ;
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                const unsigned offset = image.GetStride() * ix + rowStride * iy ;
                const float    x      = float( ix ) * oneOverWidth ;
                const float    along  = x * direction.x + y * direction.y ;
                const Vec4     color  = along * colorRange + colorMin ;
                image[ offset + 0 ] = PixelInt( PixelFloat( image[ offset + 0 ] ) * color.x ) ;
                image[ offset + 1 ] = PixelInt( PixelFloat( image[ offset + 1 ] ) * color.y ) ;
                image[ offset + 2 ] = PixelInt( PixelFloat( image[ offset + 2 ] ) * color.z ) ;
                image[ offset + 3 ] = PixelInt( PixelFloat( image[ offset + 3 ] ) * color.w ) ;
            }
        }

        return image ;
    }
} ;
