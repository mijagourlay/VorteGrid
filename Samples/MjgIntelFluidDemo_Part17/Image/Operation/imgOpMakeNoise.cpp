/** \file imgOpMakeNoise.cpp

    \brief Operation to make an image of noise.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "imgOpMakeNoise.h"

namespace PeGaSys
{

/** Create an image of noise.
*/
Image & ImgOpMakeNoise::MakeNoise( Image & image )
{
    ASSERT( 4 == image.GetNumChannels() ) ;
    ASSERT( & image[ 0 ] ) ;
//if( ! image.mImgData )
//{   // Image data has not yet been allocated.
//    image.AllocateImageData() ;
//}

    const unsigned rowStride = image.GetRowStride() ;

    for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
        {
            const unsigned char iNoise = static_cast< unsigned char >( rand() & 0xff ) ;
            const unsigned offset = image.GetStride() * ix + rowStride * iy ;
            image[ offset + 0 ] = iNoise ; // red
            image[ offset + 1 ] = iNoise ; // green
            image[ offset + 2 ] = iNoise ; // blue
            image[ offset + 3 ] = 0xff   ; // alpha (opacity)
        }
    }

    return image ;
}

} ;