/** \file imgOpTint.cpp

    \brief Operation to tint an image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "imgOpTint.h"

namespace PeGaSys
{

    /** Apply a tint to the given image.
    */
    Image & ImgOpTint::Tint( Image & image , const Vec4 & color )
    {
        const unsigned rowStride = image.GetRowStride() ;
        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                const unsigned  offset  = image.GetStride() * ix + rowStride * iy ;
                image[ offset + 0 ] = PixelInt( PixelFloat( image[ offset + 0 ] ) * color.x ) ;
                image[ offset + 1 ] = PixelInt( PixelFloat( image[ offset + 1 ] ) * color.y ) ;
                image[ offset + 2 ] = PixelInt( PixelFloat( image[ offset + 2 ] ) * color.z ) ;
                image[ offset + 3 ] = PixelInt( PixelFloat( image[ offset + 3 ] ) * color.w ) ;
            }
        }

        return image ;
    }
} ;
