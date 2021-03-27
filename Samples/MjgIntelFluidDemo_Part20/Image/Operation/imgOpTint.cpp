/** \file imgOpTint.cpp

    \brief Operation to tint an image.

    \author Copyright 2012-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "Image/Operation/imgOpTint.h"

#include "Core/Performance/perfBlock.h"

#if defined( WIN32 )
#   include <windows.h>
#endif

namespace PeGaSys
{

    /** Apply a tint to the given image.
    */
    Image & ImgOpTint::Tint( Image & image , const Vec4 & color )
    {
        PERF_BLOCK( ImgOpTint__Tint ) ;

        ASSERT( 4 == image.GetNumChannels() ) ; // TODO: FIXME: Current implementation assumes image has 4 channels.

        const unsigned xStride = image.GetXStride() ;
        const unsigned yStride = image.GetYStride() ;
        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                const unsigned  offset  = xStride * ix + yStride * iy ;
                image[ offset + 0 ] = PixelInt( PixelFloat( image[ offset + 0 ] ) * color.x ) ;
                image[ offset + 1 ] = PixelInt( PixelFloat( image[ offset + 1 ] ) * color.y ) ;
                image[ offset + 2 ] = PixelInt( PixelFloat( image[ offset + 2 ] ) * color.z ) ;
                image[ offset + 3 ] = PixelInt( PixelFloat( image[ offset + 3 ] ) * color.w ) ;
            }
        }

        return image ;
    }
} ;
