/** \file imgOpSequence.h

    \brief Sequence of image operations.

    \author Copyright 2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_SEQUENCE_H
#define PEGASYS_IMG_OP_SEQUENCE_H

namespace PeGaSys
{

    /** Sequence of image operations.
    */
    class ImgOpSequence : public IImageOperation
    {
        public:
            ImgOpSequence( size_t numOperations )
            {
                mImageOperations.reserve( numOperations ) ;
            }

            virtual Image & operator()( Image & image )
            {
                const size_t numImgOps = mImageOperations.size() ;
                for( size_t imgOpIdx = 0 ; imgOpIdx < numImgOps ; ++ imgOpIdx )
                {
                    IImageOperation * imgOp = mImageOperations[ imgOpIdx ] ;
                    imgOp->operator()( image ) ;
                }
                return image ;
            }

            void PushBack( IImageOperation * imgOp )
            {
                mImageOperations.push_back( imgOp ) ;
            }

        private:
            std::vector< IImageOperation * >    mImageOperations ;
    } ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

} ;

#endif
