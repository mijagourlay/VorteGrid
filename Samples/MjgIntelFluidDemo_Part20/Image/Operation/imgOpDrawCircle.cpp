/** \file imgOpDrawCircle.cpp

    \brief Image operation to draw a circle.

    \author Copyright 2012-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
#   include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "imgOpDrawCircle.h"

namespace PeGaSys
{

    /** Draw a circle in an image.

        \param  radius      Radius, in pixels, of circle.

        \param  thickness   Thickness, in pixels, of circle border.
    */
    Image & ImgOpDrawCircle::DrawCircle( Image & image , float radius , float thickness )
    {
        ASSERT( 4 == image.GetNumChannels() ) ; // TODO: FIXME: Current implementation assumes image has 4 channels.
        ASSERT( & image[0] != 0 ) ;
        const int       centerX     = image.GetWidth()  / 2 ;
        const int       centerY     = image.GetHeight() / 2 ;
        const unsigned  xStride     = image.GetXStride() ;
        const unsigned  yStride     = image.GetYStride() ;
        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            const int distFromCenterY  = iy - centerY ;
            const int dist2FromCenterY = POW2( distFromCenterY ) ;
            ASSERT( dist2FromCenterY >= 0 ) ;
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                unsigned char value = 0 ;
                const int   distFromCenterX  = ix - centerX ;
                const int   dist2FromCenterX = POW2( distFromCenterX ) ;
                ASSERT( dist2FromCenterX >= 0 ) ;
                const int   dist2FromCenter  = dist2FromCenterX + dist2FromCenterY ;
                ASSERT( dist2FromCenter >= 0 ) ;
                const float distFromRing     = fabsf( fsqrtf( float( dist2FromCenter ) ) - radius ) ;
                if( distFromRing <= thickness )
                {   // Pixel is on circle border.
                    value = 255 ;
                }
                else
                {
                    value = 0 ;
                }
                const unsigned  offset  = xStride * ix + yStride * iy ;
                image[ offset + 0 ] =
                image[ offset + 1 ] =
                image[ offset + 2 ] =
                image[ offset + 3 ] = value ;
            }
        }

        return image ;
    }

} ;