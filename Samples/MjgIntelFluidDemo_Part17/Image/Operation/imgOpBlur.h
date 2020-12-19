/** \file imgOpBlur.h

    \brief Operation to blur an image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_BLUR_H
#define PEGASYS_IMG_OP_BLUR_H

#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Operation to blur an image.
    */
    class ImgOpBlur : public IImageOperation
    {
        public:
            ImgOpBlur( unsigned numSmoothingPasses )
                : mNumSmoothingPasses( numSmoothingPasses )
            {}

            ImgOpBlur( Image & image , unsigned numSmoothingPasses )
                : mNumSmoothingPasses( numSmoothingPasses )
            {
                operator()( image ) ;
            }

            static Image & Blur( Image & image , unsigned numSmoothingPasses ) ;

            virtual Image & operator()( Image & image )
            {
                return Blur( image , mNumSmoothingPasses ) ;
            }

        private:
            unsigned    mNumSmoothingPasses ;
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
