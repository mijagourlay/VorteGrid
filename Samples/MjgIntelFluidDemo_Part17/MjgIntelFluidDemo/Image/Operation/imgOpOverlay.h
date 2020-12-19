/** \file imgOpOverlay.h

    \brief Operation to composite a foreground image over a background image.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_OVERLAY_H
#define PEGASYS_IMG_OP_OVERLAY_H

#include "../image.h"
#include "iImageOperation.h"

namespace PeGaSys
{

    /** Operation to composite a foreground image over a background image.
    */
    class ImgOpOverlay : public IImageOperation
    {
        public:
            ImgOpOverlay( const Image & foregroundImage , const Image & backgroundImage )
                : mForegroundImage( & foregroundImage )
                , mBackgroundImage( & backgroundImage )
            {}

            ImgOpOverlay( Image & image , const Image & foregroundImage , const Image & backgroundImage )
                : mForegroundImage( & foregroundImage )
                , mBackgroundImage( & backgroundImage )
            {
                operator()( image ) ;
            }

            static Image & Overlay( Image & image , const Image & foregroundImage , const Image & backgroundImage ) ;

            virtual Image & operator()( Image & image )
            {
                return Overlay( image , * mForegroundImage , * mBackgroundImage ) ;
            }

        private:
            const Image * mForegroundImage ;
            const Image * mBackgroundImage ;
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
