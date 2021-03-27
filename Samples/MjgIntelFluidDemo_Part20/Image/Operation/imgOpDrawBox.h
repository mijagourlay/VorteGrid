/** \file imgOpDrawBox.h

    \brief Image operation to draw a box.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_DRAW_BOX_H
#define PEGASYS_IMG_OP_DRAW_BOX_H

#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Image operation to draw a box.
    */
    class ImgOpDrawBox : public IImageOperation
    {
        public:
            ImgOpDrawBox( unsigned thickness )
                : mThickness( thickness )
            {}

            ImgOpDrawBox( Image & image , unsigned thickness )
                : mThickness( thickness )
            {
                operator()( image ) ;
            }

            static Image & DrawBox( Image & image , unsigned thickness ) ;

            virtual Image & operator()( Image & image )
            {
                return DrawBox( image , mThickness ) ;
            }

        private:
            unsigned mThickness    ;   ///< Box thickness in pixels.
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
