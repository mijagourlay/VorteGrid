/** \file imgOpApplyGamma.cpp

    \brief Operation to apply "gamma correction" to an image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "imgOpApplyGamma.h"

namespace PeGaSys
{

/** Apply "gamma" brightness correction to the given image data.
*/
Image & ImgOpApplyGamma::ApplyGamma( Image & image , float gamma )
{
    ASSERT( image.GetNumChannels() >= 3 ) ;
    const unsigned rowStride = image.GetRowStride() ;
    for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
        {
            const unsigned  offset  = image.GetStride() * ix + rowStride * iy ;
            image[ offset + 0 ] = PixelInt( powf( PixelFloat( image[ offset + 0 ] ) , gamma ) ) ;
            image[ offset + 1 ] = PixelInt( powf( PixelFloat( image[ offset + 1 ] ) , gamma ) ) ;
            image[ offset + 2 ] = PixelInt( powf( PixelFloat( image[ offset + 2 ] ) , gamma ) );
        }
    }

    return image ;
}

} ;