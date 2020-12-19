/** \file imgOpSequences.h

    \brief Image operation sequences that have proven generically useful.

    \author Copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PEGASYS_IMG_OP_SEQUENCES_H
#define PEGASYS_IMG_OP_SEQUENCES_H

struct Vec4 ;

namespace PeGaSys
{
    class Image ;

    void ImgOpSeq_Noise( Image & noise , float gamma , const Vec4 & tint ) ;
    void ImgOpSeq_GradientNoise( Image & noise , float gamma , const Vec4 & tint ) ;
    void ImgOpSeq_NoiseBall( Image & noiseBall , float gamma , const Vec4 & tint ) ;
    void ImgOpSeq_DecoratedNoiseBall( Image & noiseBall , float gamma , const Vec4 & color ) ;
}

#endif
