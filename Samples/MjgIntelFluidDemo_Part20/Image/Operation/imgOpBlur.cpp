/** \file imgOpBlur.cpp

    \brief Operation to blur an image.

    \author Written and copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#pragma optimize( "" , on ) // This operation is ludicrously slow so optimize it even in non-opt builds.

// This operation is ludicrously slow so optionally disable asserts inside inner loops.
#if 0 && _DEBUG
#   define INNER_LOOP_ASSERT( cond ) ASSERT( cond )
#else
#   define INNER_LOOP_ASSERT( cond )
#endif

#include "Image/Operation/imgOpBlur.h"

#include "Core/Performance/perfBlock.h"

#if defined( WIN32 )
#   include <windows.h>
#endif

namespace PeGaSys
{

    /** Blur the given image pixel in the interior of an image.
    */
    static inline unsigned char blurInterior3x3( int ix , int iy , int ic , const unsigned char * img , int nx , int ny , int nc )
    {
        const unsigned ixOff =   ix       * nc + ic ;
        const unsigned ixm1  = ( ix - 1 ) * nc + ic ;
        const unsigned ixp1  = ( ix + 1 ) * nc + ic ;
        const unsigned nl    = nx * nc ;         // y stride, the number of bytes between vertically adjacent pixels. TODO: FIXME: pass this in as a parameter because for cropped images, yStride might not be nx*nc.
        const unsigned iyOff =   iy       * nl ;
        const unsigned iym1  = ( iy - 1 ) * nl ;
        const unsigned iyp1  = ( iy + 1 ) * nl ;

        INNER_LOOP_ASSERT( ( ix > 0 ) && ( ix < ( nx - 1 ) ) && ( iy > 0 ) && ( iy < ( ny - 1 ) ) ) ;
        UNUSED_PARAM( ny ) ; // Avoid compiler warning in builds where INNER_LOOP_ASSERT is defined as a do-nothing operation.

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
            INNER_LOOP_ASSERT( average <= 255 ) ;
            const unsigned char result  = static_cast< unsigned char >( average ) ;
            return result ;
    }




    /** Blur the given image pixel, which could be at an edge (including corners).
    */
    static inline unsigned char blurEdge3x3( int ix , int iy , int ic , const unsigned char * img , int nx , int ny , int nc )
    {
        const unsigned ixOff = ix * nc + ic ;
        const unsigned nl    = nx * nc ;         // y stride, the number of bytes between vertically adjacent pixels. TODO: FIXME: pass this in as a parameter because for cropped images, yStride might not be nx*nc.
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

        \todo TODO: FIXME: Replace numSmoothingPasses with a blur kernel size.
    */
    Image & ImgOpBlur::Blur( Image & image , unsigned numSmoothingPasses )
    {
        PERF_BLOCK( ImgOpBlur__Blur ) ;

        ASSERT( 1 == image.GetNumPages() ) ;

        ASSERT( image.GetNumChannels() >= 3 ) ; // TODO: FIXME: Current implementation assumes image has at least 3 channels.

        unsigned char * imgData = image.GetImageData() ;

        // Create a temp space to write blurred pixels.
        // This routine reads from the source image and writes to the temp space.
        // This is because the blur formula would otherwise mix blurred and non-blurred pixels.
        Image workspace( image ) ;

        const unsigned xStride      = image.GetXStride() ;
        const unsigned yStride      = image.GetYStride() ;
        const unsigned width        = image.GetWidth() ;
        const unsigned widthMinus1  = image.GetWidth() - 1 ;
        const unsigned height       = image.GetHeight() ;
        const unsigned heightMinus1 = image.GetHeight() - 1 ;
        const unsigned numChannels  = image.GetNumChannels() ;

        DEBUG_ONLY( const size_t offsetLimit = yStride * height  ; (void) offsetLimit ) ;

        for( unsigned iSmooth = 0 ; iSmooth < numSmoothingPasses ; ++ iSmooth )
        {   // For each smoothing pass...

            // Blur interior.
            for( unsigned iy = 1 ; iy < heightMinus1 ; ++ iy )
            {
                for( unsigned ix = 1 ; ix < widthMinus1 ; ++ ix )
                {
                    const unsigned  offset  = xStride * ix + yStride * iy ;
                    INNER_LOOP_ASSERT( offset + 2 < offsetLimit ) ;   // Paranoid sanity check; make sure pixel is inside image.
                    workspace[ offset + 0 ] = blurInterior3x3( ix , iy , 0 , imgData , width , height , numChannels ) ;
                    workspace[ offset + 1 ] = blurInterior3x3( ix , iy , 1 , imgData , width , height , numChannels ) ;
                    workspace[ offset + 2 ] = blurInterior3x3( ix , iy , 2 , imgData , width , height , numChannels ) ;
                    // Note the lack of smoothing in the 4th channel.  See TODO above.
                }
            }

            // Blur adjacent pixels with current one, checking to make sure adjacent pixels are inside image bounds.
#           define BLUR_EDGE_PIXEL( IX , IY )                                                                       \
            {                                                                                                       \
                const unsigned offset = xStride * (IX) + yStride * (IY) ;                                           \
                INNER_LOOP_ASSERT( offset + 2 < offsetLimit ) ;                                                     \
                workspace[ offset + 0 ] = blurEdge3x3( (IX) , (IY) , 0 , imgData , width , height , numChannels ) ; \
                workspace[ offset + 1 ] = blurEdge3x3( (IX) , (IY) , 1 , imgData , width , height , numChannels ) ; \
                workspace[ offset + 2 ] = blurEdge3x3( (IX) , (IY) , 2 , imgData , width , height , numChannels ) ; \
            }

            // Blur edges except corners.
            for( unsigned ix = 1 ; ix < widthMinus1 ; ++ ix )
            {   // Blur top and bottom edges except corners.
                BLUR_EDGE_PIXEL( ix , 0            ) ;
                BLUR_EDGE_PIXEL( ix , heightMinus1 ) ;
            }
            for( unsigned iy = 1 ; iy < heightMinus1 ; ++ iy )
            {   // Blur left and right edges except corners.
                BLUR_EDGE_PIXEL( 0           , iy ) ;
                BLUR_EDGE_PIXEL( widthMinus1 , iy ) ;
            }

            // Blur corners.
            BLUR_EDGE_PIXEL( 0            , 0            ) ;
            BLUR_EDGE_PIXEL( widthMinus1  , 0            ) ;
            BLUR_EDGE_PIXEL( 0            , heightMinus1 ) ;
            BLUR_EDGE_PIXEL( widthMinus1  , heightMinus1 ) ;

            // Transfer blurred image from temp workspace to texture.
            // TODO: FIXME: Avoid this copy or at least avoid doing it once per blur pass.  E.g. could swap workspace and original buffers every other pass rather than copy, then copy on last pass from workspace to original.
            image.CopyImageData( workspace ) ;
        }

        return image ;
    }

}