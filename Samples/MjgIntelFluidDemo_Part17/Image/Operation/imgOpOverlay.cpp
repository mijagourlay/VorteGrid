/** \file imgOpOverlay.cpp

    \brief Operation to composite a foreground image over a background image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "ImgOpOverlay.h"

namespace PeGaSys
{

/** Composite a foreground image over a background image.

    \param dstImage Image to populate. Can be either "this" or backgroundImage (or neither).

    \param foregroundImage  Front image.

    \param backgroundImage  Image behind the front image.

    All images must have the same dimensions.

    Applies an alpha blend.
*/
Image & ImgOpOverlay::Overlay( Image & dstImage , const Image & foregroundImage , const Image & backgroundImage )
{
    ASSERT( dstImage.GetWidth()              == backgroundImage.GetWidth()  ) ;
    ASSERT( dstImage.GetHeight()             == backgroundImage.GetHeight() ) ;
    ASSERT( dstImage.GetNumChannels()        == backgroundImage.GetNumChannels() ) ;
    ASSERT( foregroundImage.GetWidth()       == backgroundImage.GetWidth()  ) ;
    ASSERT( foregroundImage.GetHeight()      == backgroundImage.GetHeight() ) ;
    ASSERT( foregroundImage.GetNumChannels() == backgroundImage.GetNumChannels() ) ;

    const unsigned rowStride = dstImage.GetRowStride() ;

    for( unsigned iy = 0 ; iy < dstImage.GetHeight() ; ++ iy )
    {
        for( unsigned ix = 0 ; ix < dstImage.GetWidth() ; ++ ix )
        {
            const unsigned  offset                 = dstImage.GetStride() * ix + rowStride * iy ;
            float           overlayOpacity         = PixelFloat( foregroundImage[ offset + 3 ] ) ;
            float           oneMinusOverlayOpacity = 1.0f - overlayOpacity ;
            ASSERT( ( overlayOpacity >= 0.0f ) && ( overlayOpacity <= 1.0f ) ) ;
            ASSERT( ( oneMinusOverlayOpacity >= 0.0f ) && ( oneMinusOverlayOpacity <= 1.0f ) ) ;
            dstImage[ offset + 0 ] = PixelInt( PixelFloat( backgroundImage[ offset + 0 ] ) * oneMinusOverlayOpacity + PixelFloat( foregroundImage[ offset + 0 ] ) * overlayOpacity ) ;
            dstImage[ offset + 1 ] = PixelInt( PixelFloat( backgroundImage[ offset + 1 ] ) * oneMinusOverlayOpacity + PixelFloat( foregroundImage[ offset + 1 ] ) * overlayOpacity ) ;
            dstImage[ offset + 2 ] = PixelInt( PixelFloat( backgroundImage[ offset + 2 ] ) * oneMinusOverlayOpacity + PixelFloat( foregroundImage[ offset + 2 ] ) * overlayOpacity ) ;
            dstImage[ offset + 3 ] = PixelInt( PixelFloat( backgroundImage[ offset + 3 ] ) * oneMinusOverlayOpacity + PixelFloat( foregroundImage[ offset + 3 ] ) * overlayOpacity ) ;

            //printf("%i %i %i %i , " , destinationImage[ offset + 0 ] , destinationImage[ offset + 1 ] , destinationImage[ offset + 2 ] , destinationImage[ offset + 3 ] ) ;
        }
        //printf("\n") ;
    }

    return dstImage ;
}

} ;