/** \file imgOpDrawCircle.h

    \brief Image operation to draw a circle.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_DRAW_CIRCLE_H
#define PEGASYS_IMG_OP_DRAW_CIRCLE_H

#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Image operation to draw a circle.
    */
    class ImgOpDrawCircle: public IImageOperation
    {
        public:
            ImgOpDrawCircle( float radius , float thickness )
                : mRadius( radius )
                , mThickness( thickness )
            {}

            ImgOpDrawCircle( Image & image , float radius , float thickness )
                : mRadius( radius )
                , mThickness( thickness )
            {
                operator()( image ) ;
            }

            static Image & DrawCircle( Image & image , float radius , float thickness ) ;

            virtual Image & operator()( Image & image )
            {
                return DrawCircle( image , mRadius , mThickness ) ;
            }

        private:
            float mRadius       ;   ///< Circle radius in pixels.
            float mThickness    ;   ///< Circle thickness in pixels.
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
