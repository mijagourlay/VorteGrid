/** \file imgOpMakeNoise.cpp

    \brief Operation to make an image of noise.

    \author Written and copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "Image/Operation/imgOpMakeNoise.h"

#include "Core/Performance/perfBlock.h"

#if defined( WIN32 )
#   include <windows.h>
#endif

namespace PeGaSys
{

    /** Create an image of noise.
    */
    Image & ImgOpMakeNoise::MakeNoise( Image & image )
    {
        PERF_BLOCK( ImgOpMakeNoise__MakeNoise ) ;

        ASSERT( 4 == image.GetNumChannels() ) ; // TODO: FIXME: Current implementation assumes image has 4 channels.
        ASSERT( & image[ 0 ] ) ;

        const unsigned xStride = image.GetXStride() ;
        const unsigned yStride = image.GetYStride() ;

        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                const unsigned      offset = xStride * ix + yStride * iy ;
                const unsigned char iNoise = static_cast< unsigned char >( rand() & 0xff ) ;
                image[ offset + 0 ] = iNoise ; // red
                image[ offset + 1 ] = iNoise ; // green
                image[ offset + 2 ] = iNoise ; // blue
                image[ offset + 3 ] = 0xff   ; // alpha (opacity)
            }
        }

        return image ;
    }

} ;