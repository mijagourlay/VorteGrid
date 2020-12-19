/** \file imgOpLinearGradient.h

    \brief Operation to apply a linear gradient to an image.

    \author Copyright 2012-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_LINEAR_GRADIENT_H
#define PEGASYS_IMG_OP_LINEAR_GRADIENT_H

#include "Core/Math/Vec2.h"
#include "Core/Math/Vec4.h"

#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Operation to apply a linear gradient to an image.
    */
    class ImgOpLinearGradient : public IImageOperation
    {
        public:
            ImgOpLinearGradient( const Vec2 & direction , const Vec4 & colorMin , const Vec4 & colorMax )
                : mDirection( direction )
                , mColorMin( colorMin )
                , mColorMax( colorMax )
            {}

            ImgOpLinearGradient( Image & image , const Vec2 & direction , const Vec4 & colorMin , const Vec4 & colorMax )
                : mDirection( direction )
                , mColorMin( colorMin )
                , mColorMax( colorMax )
            {
                operator()( image ) ;
            }

            static Image & Gradient( Image & image , const Vec2 & direction , const Vec4 & colorMin , const Vec4 & colorMax ) ;

            virtual Image & operator()( Image & image )
            {
                return Gradient( image , mDirection , mColorMin , mColorMax ) ;
            }

        private:
            Vec2    mDirection  ;   ///< direction along which to apply gradient
            Vec4    mColorMin   ;   ///< Color at minimal end of gradient.
            Vec4    mColorMax   ;   ///< Color at maximal end of gradient.
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
