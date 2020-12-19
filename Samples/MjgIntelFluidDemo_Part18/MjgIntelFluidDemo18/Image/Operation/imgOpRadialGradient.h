/** \file imgOpRadialGradient.h

    \brief Operation to create a radial gradient in one component of an image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_RADIAL_GRADIENT_H
#define PEGASYS_IMG_OP_RADIAL_GRADIENT_H

#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Operation to create a radial gradient in one component of an image.
    */
    class ImgOpRadialGradient: public IImageOperation
    {
        public:
            ImgOpRadialGradient( float power , float max , int componentIndex )
                : mPower( power )
                , mMax( max )
                , mComponentIndex( componentIndex )
            {}

            ImgOpRadialGradient( Image & image , float power , float max , int componentIndex )
                : mPower( power )
                , mMax( max )
                , mComponentIndex( componentIndex )
            {
                operator()( image ) ;
            }

            static Image & RadialGradient( Image & image , float power , float max , int componentIndex ) ;

            virtual Image & operator()( Image & image )
            {
                return RadialGradient( image , mPower , mMax , mComponentIndex ) ;
            }

        private:
            float   mPower          ;   ///< Exponent in radial gradient curve.
            float   mMax            ;   ///< Maximum value of gradient.
            int     mComponentIndex ;   ///< Index of component to assign.
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------
} ;

#endif
