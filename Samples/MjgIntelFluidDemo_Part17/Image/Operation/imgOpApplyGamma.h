/** \file imgOpApplyGamma.h

    \brief Operation to apply "gamma correction" to an image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_APPLY_GAMMA_H
#define PEGASYS_IMG_OP_APPLY_GAMMA_H

#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Operation to apply "gamma correction" to an image.
    */
    class ImgOpApplyGamma : public IImageOperation
    {
        public:
            ImgOpApplyGamma( float gamma )
                : mGamma( gamma )
            {}

            ImgOpApplyGamma( Image & image , float gamma )
                : mGamma( gamma )
            {
                operator()( image ) ;
            }

            static Image & ApplyGamma( Image & image , float gamma ) ;

            virtual Image & operator()( Image & image )
            {
                return ApplyGamma( image , mGamma ) ;
            }

        private:
            float   mGamma  ;   ///< Exponential curve power.
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

}

#endif
