/** \file imgOpBlur.cpp

    \brief Operation to blur an image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#if defined( WIN32 )
    #include <windows.h>
#endif

#include "Core/Math/vec4.h"

#include "imgOpBlur.h"

namespace PeGaSys
{

/** Blur the given image pixel in the interior of an image.
*/
static inline unsigned char blurInterior3x3( int ix , int iy , int ic , const unsigned char * img , int nx , int ny , int nc )
{
    const unsigned ixOff =   ix       * nc + ic ;
    const unsigned ixm1  = ( ix - 1 ) * nc + ic ;
    const unsigned ixp1  = ( ix + 1 ) * nc + ic ;
    const unsigned nl    = nx * nc ;         // width (y) stride
    const unsigned iyOff =   iy       * nl ;
    const unsigned iym1  = ( iy - 1 ) * nl ;
    const unsigned iyp1  = ( iy + 1 ) * nl ;

    ASSERT( ( ix > 0 ) && ( ix < ( nx - 1 ) ) && ( iy > 0 ) && ( iy < ( ny - 1 ) ) ) ;
    (void) ny ; // Avoid compiler warning in non-debug builds.

    unsigned iOut   =
          img[ ixm1  + iym1  ]
        + img[ ixOff + iym1  ]
        + img[ ixp1  + iym1  ]
        + img[ ixm1  + iyOff ]
        + img[ ixOff + iyOff ]
        + img[ ixp1  + iyOff ]
        + img[ ixm1  + iyp1  ]
        + img[ ixOff + iyp1  ]
        + img[ ixp1  + iyp1  ] ;

    const unsigned      average = iOut / 9 ;
    ASSERT( average <= 255 ) ;
    const unsigned char result  = static_cast< unsigned char >( average ) ;
    return result ;
}




/** Blur the given image pixel, which could be at an edge or corner.
*/
static inline unsigned char blurEdge3x3( int ix , int iy , int ic , const unsigned char * img , int nx , int ny , int nc )
{
    const unsigned ixOff = ix * nc + ic ;
    const unsigned nl    = nx * nc ;         // width (y) stride
    const unsigned iyOff = iy * nl ;
    const unsigned iym1  = ( iy - 1 ) * nl ;
    const unsigned iyp1  = ( iy + 1 ) * nl ;

    unsigned iCounter = 1 ;
    unsigned iOut     = img[ ixOff + iyOff ] ;

    if( ix > 0 )
    {
        const unsigned ixm1 = ( ix - 1 ) * nc + ic ;
        iOut += img[ ixm1 + iyOff ] ; iCounter ++ ;
        if( iy > 0 )
        {
            iOut += img[ ixm1 + iym1  ] ; iCounter ++ ;
        }
        if( iy < ( ny - 1 ) )
        {
            iOut += img[ ixm1 + iyp1  ] ; iCounter ++ ;
        }
    }
    if( ix < ( nx - 1 ) )
    {
        const unsigned ixp1 = ( ix + 1 ) * nc + ic ;
        iOut += img[ ixp1 + iyOff ] ; iCounter ++ ;
        if( iy > 0 )
        {
            iOut += img[ ixp1 + iym1  ] ; iCounter ++ ;
        }
        if( iy < ( ny - 1 ) )
        {
            iOut += img[ ixp1 + iyp1  ] ; iCounter ++ ;
        }
    }
        if( iy > 0 )
        {
            iOut += img[ ixOff + iym1 ] ; iCounter ++ ;
        }
        if( iy < ( ny - 1 ) )
        {
            iOut += img[ ixOff + iyp1 ] ; iCounter ++ ;
        }
    const unsigned      average = iOut / iCounter ;
    const unsigned char result  = static_cast< unsigned char >( Min2( average , 255U ) ) ;
    return result ;
}




/** Blur the given image.
*/
Image & ImgOpBlur::Blur( Image & image , unsigned numSmoothingPasses )
{
    ASSERT( 1 == image.GetNumPages() ) ;
    const size_t numBytesPerImage = image.SizeInBytes() ;

    // Create a temp space to write blurred pixels.
    // This routine reads from the texture and writes to the temp space.
    // This is because the blur formula would otherwise mix blurred and non-blurred pixels.
    unsigned char * pImgTemp = (unsigned char *) malloc( numBytesPerImage ) ;
    // Copy texture to temp space.
    unsigned char * imgData = & image[ 0 ] ;
    memcpy( pImgTemp , imgData , numBytesPerImage ) ;

    const unsigned stride       = image.GetStride() ;
    const unsigned rowStride    = image.GetRowStride() ;
    const unsigned width        = image.GetWidth() ;
    const unsigned widthMinus1  = image.GetWidth() - 1 ;
    const unsigned height       = image.GetHeight() ;
    const unsigned heightMinus1 = image.GetHeight() - 1 ;
    const unsigned numChannels  = image.GetNumChannels() ;

    for( unsigned iSmooth = 0 ; iSmooth < numSmoothingPasses ; ++ iSmooth )
    {
        // Blur interior.
        for( unsigned iy = 1 ; iy < heightMinus1 ; ++ iy )
        {
            for( unsigned ix = 1 ; ix < widthMinus1 ; ++ ix )
            {
                const unsigned offset = stride * ix + rowStride * iy ;
                ASSERT( offset + 2 < numBytesPerImage ) ;
                pImgTemp[ offset + 0 ] = blurInterior3x3( ix , iy , 0 , imgData , width , height , numChannels ) ;
                pImgTemp[ offset + 1 ] = blurInterior3x3( ix , iy , 1 , imgData , width , height , numChannels ) ;
                pImgTemp[ offset + 2 ] = blurInterior3x3( ix , iy , 2 , imgData , width , height , numChannels ) ;
                // Note the lack of smoothing in the 4th channel.  Assuming it's uniform.
            }
        }
        // Blur edges.
        #define BLUR_EDGE( IX , IY )                                                                            \
        {                                                                                                       \
            const unsigned offset = stride * (IX) + rowStride * (IY) ;                                          \
            ASSERT( offset + 2 < numBytesPerImage ) ;                                                           \
            pImgTemp[ offset + 0 ] = blurEdge3x3( (IX) , (IY) , 0 , imgData , width , height , numChannels ) ;  \
            pImgTemp[ offset + 1 ] = blurEdge3x3( (IX) , (IY) , 1 , imgData , width , height , numChannels ) ;  \
            pImgTemp[ offset + 2 ] = blurEdge3x3( (IX) , (IY) , 2 , imgData , width , height , numChannels ) ;  \
        }
        for( unsigned ix = 1 ; ix < widthMinus1 ; ++ ix )
        {   // Blur top and bottom edges.
            BLUR_EDGE( ix , 0            ) ;
            BLUR_EDGE( ix , heightMinus1 ) ;
        }
        for( unsigned iy = 1 ; iy < heightMinus1 ; ++ iy )
        {   // Blur left and right edges
            BLUR_EDGE( 0           , iy ) ;
            BLUR_EDGE( widthMinus1 , iy ) ;
        }
        //// Blur corners.
        BLUR_EDGE( 0            , 0            ) ;
        BLUR_EDGE( widthMinus1  , 0            ) ;
        BLUR_EDGE( 0            , heightMinus1 ) ;
        BLUR_EDGE( widthMinus1  , heightMinus1 ) ;

        // Transfer blurred image from temp space to texture.
        memcpy( imgData , pImgTemp , numBytesPerImage ) ;
    }
    free( pImgTemp ) ;

    return image ;
}

}