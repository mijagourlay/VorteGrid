/** \file imgOpDrawCircle.cpp

    \brief Image operation to draw a circle.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "imgOpDrawCircle.h"

namespace PeGaSys
{

/** Draw a circle in an image.
*/
Image & ImgOpDrawCircle::DrawCircle( Image & image , float radius , float thickness )
{
    ASSERT( & image[0] != 0 ) ;
    const int       centerX     = image.GetWidth()  / 2 ;
    const int       centerY     = image.GetHeight() / 2 ;
    const unsigned  rowStride   = image.GetRowStride() ;
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
            {   // Pixel is on ring perimeter.
                value = 255 ;
            }
            else
            {
                value = 0 ;
            }
            const unsigned  offset = image.GetStride() * ix + rowStride * iy ;
            image[ offset + 0 ] =
            image[ offset + 1 ] =
            image[ offset + 2 ] =
            image[ offset + 3 ] = value ;
        }
    }

    return image ;
}

} ;