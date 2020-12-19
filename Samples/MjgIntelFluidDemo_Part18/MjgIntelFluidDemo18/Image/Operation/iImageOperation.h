/** \file iImageOperation.h

    \brief Image operation abstract base class.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
        - http://www.mijagourlay.com/
*/
#ifndef PEGASYS_I_IMAGE_OPERATION_H
#define PEGASYS_I_IMAGE_OPERATION_H

#include "float.h"

namespace PeGaSys
{

    /** Image operation abstract base class.
    */
    class IImageOperation
    {
        public:
            virtual ~IImageOperation() {}
            virtual Image & operator()( Image & image ) = 0 ;
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

    static const float  sAlmost256  = 256.0f * ( 1.0f - FLT_EPSILON ) ;
    static const float  sOneOver255 = 1.0f / 255.0f ;

    static inline float PixelFloat( const unsigned char pixelValue )
    {
        return static_cast< float >( pixelValue ) * sOneOver255 ;
    }




    static inline unsigned char PixelInt( const float & pixelValue )
    {
        float pixValF32 = pixelValue * sAlmost256 ;
        ASSERT( ( pixValF32 >= 0.0f ) && ( pixValF32 < 256.0f ) ) ;
        unsigned int  pixValInt32 = static_cast< unsigned int >( pixValF32 ) ;
        ASSERT( pixValInt32 < 256 ) ;
        unsigned char pixValInt8  = static_cast< unsigned char >( pixValInt32 ) ;
        return pixValInt8 ;
    }



}

#endif
