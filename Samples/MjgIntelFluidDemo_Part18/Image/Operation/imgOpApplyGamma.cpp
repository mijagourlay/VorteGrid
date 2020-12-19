/** \file imgOpApplyGamma.cpp

    \brief Operation to apply "gamma correction" to an image.

    \author Written and copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "Image/Operation/imgOpApplyGamma.h"

#include "Core/Performance/perfBlock.h"

#include <math.h>

#if defined( WIN32 )
#   include <windows.h>
#endif

namespace PeGaSys
{

    /** Apply "gamma" brightness correction to the given image data.
    */
    Image & ImgOpApplyGamma::ApplyGamma( Image & image , float gamma )
    {
        PERF_BLOCK( ImgOpApplyGamma__ApplyGamma ) ;

        ASSERT( image.GetNumChannels() >= 3 ) ; // Current implementation only supports images with at least 3 channels. TODO: FIXME.

        const unsigned xStride = image.GetXStride() ;
        const unsigned yStride = image.GetYStride() ;
        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                const unsigned  offset  = xStride * ix + yStride * iy ;
                image[ offset + 0 ] = PixelInt( powf( PixelFloat( image[ offset + 0 ] ) , gamma ) ) ;
                image[ offset + 1 ] = PixelInt( powf( PixelFloat( image[ offset + 1 ] ) , gamma ) ) ;
                image[ offset + 2 ] = PixelInt( powf( PixelFloat( image[ offset + 2 ] ) , gamma ) );
            }
        }

        return image ;
    }

} ;