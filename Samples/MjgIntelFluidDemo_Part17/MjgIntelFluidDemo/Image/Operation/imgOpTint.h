/** \file imgOpTint.h

    \brief Operation to tint an image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_TINT_H
#define PEGASYS_IMG_OP_TINT_H

#include "Core/Math/Vec4.h"
#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Operation to tint an image.
    */
    class ImgOpTint : public IImageOperation
    {
        public:
            ImgOpTint( const Vec4 & color )
                : mColor( color )
            {}

            ImgOpTint( Image & image , const Vec4 & color )
                : mColor( color )
            {
                operator()( image ) ;
            }

            static Image & Tint( Image & image , const Vec4 & color ) ;

            virtual Image & operator()( Image & image )
            {
                return Tint( image , mColor ) ;
            }

        private:
            Vec4    mColor  ;   ///< Color to apply to image.
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
