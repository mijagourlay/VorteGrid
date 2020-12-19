/** \file imgOpMakeNoise.h

    \brief Operation to make an image of noise.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_MAKE_NOISE_H
#define PEGASYS_IMG_OP_MAKE_NOISE_H

#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Operation to make an image of noise.
    */
    class ImgOpMakeNoise: public IImageOperation
    {
        public:
            ImgOpMakeNoise()
            {}

            ImgOpMakeNoise( Image & image )
            {
                operator()( image ) ;
            }

            static Image & MakeNoise( Image & image ) ;

            virtual Image & operator()( Image & image )
            {
                return MakeNoise( image ) ;
            }
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
