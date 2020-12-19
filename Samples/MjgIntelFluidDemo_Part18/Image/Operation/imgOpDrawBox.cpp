/** \file imgOpDrawBox.cpp

    \brief Image operation to draw a box.

    \author Copyright 2012-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
#   include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "imgOpDrawBox.h"

namespace PeGaSys
{

    /** Draw a box in an image.

        \param  thickness   Thickness, in pixels, of box border.
    */
    Image & ImgOpDrawBox::DrawBox( Image & image , unsigned thickness )
    {
        ASSERT( 4 == image.GetNumChannels() ) ; // TODO: FIXME: Current implementation assumes image has 4 channels.
        const unsigned xEnd     = image.GetWidth()  - thickness ;
        const unsigned yEnd     = image.GetHeight() - thickness ;
        const unsigned xStride  = image.GetXStride() ;
        const unsigned yStride  = image.GetYStride() ;
        for( unsigned iy = 0 ; iy < image.GetHeight() ; ++ iy )
        {
            for( unsigned ix = 0 ; ix < image.GetWidth() ; ++ ix )
            {
                unsigned char value = 0 ;
                if(     ( ( ix < thickness ) || ( ix >= xEnd ) )
                    ||  ( ( iy < thickness ) || ( iy >= yEnd ) )    )
                {   // Pixel is on box border.
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