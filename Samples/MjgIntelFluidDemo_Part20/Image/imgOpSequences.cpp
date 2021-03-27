/** \file imgOpSequences.cpp

    \brief Image operation sequences that have proven generically useful.

    \author Copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "Image/imgOpSequences.h"

#include "Image/Operation/imgOpApplyGamma.h"
#include "Image/Operation/imgOpBlur.h"
#include "Image/Operation/imgOpDrawBox.h"
#include "Image/Operation/imgOpDrawCircle.h"
#include "Image/Operation/imgOpMakeNoise.h"
#include "Image/Operation/imgOpOverlay.h"
#include "Image/Operation/imgOpLinearGradient.h"
#include "Image/Operation/imgOpRadialGradient.h"
#include "Image/Operation/imgOpTint.h"

#include "Core/Performance/PerfBlock.h"

namespace PeGaSys
{

    void ImgOpSeq_Noise( PeGaSys::Image & noise , float gamma , const Vec4 & tint )
    {
        PERF_BLOCK( ImgOpSeq_Noise ) ;

        using namespace PeGaSys ;
        const unsigned  numSmoothingPasses  = noise.GetHeight() / 16 ;
        ImgOpMakeNoise::MakeNoise( noise ) ;
        ImgOpBlur::Blur( noise , numSmoothingPasses ) ;
        ImgOpApplyGamma::ApplyGamma( noise , gamma ) ;
        ImgOpTint::Tint( noise , tint ) ;
    }




    void ImgOpSeq_GradientNoise( PeGaSys::Image & noise , float gamma , const Vec4 & tint )
    {
        PERF_BLOCK( ImgOpSeq_GradientNoise ) ;

        using namespace PeGaSys ;
        const unsigned  numSmoothingPasses  = noise.GetHeight() / 16 ;
        ImgOpMakeNoise::MakeNoise( noise ) ;
        ImgOpBlur::Blur( noise , numSmoothingPasses ) ;
        ImgOpApplyGamma::ApplyGamma( noise , gamma ) ;
        ImgOpTint::Tint( noise , tint ) ;
        ImgOpLinearGradient::Gradient( noise , Vec2( 0.0f , 1.0f ) , Vec4( 0.1f , 0.1f , 0.1f , 1.0f ) , Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) ) ;
    }




    void ImgOpSeq_NoiseBall( PeGaSys::Image & noiseBall , float gamma , const Vec4 & tint )
    {
        PERF_BLOCK( ImgOpSeq_NoiseBall ) ;

        using namespace PeGaSys ;
        ImgOpMakeNoise::MakeNoise( noiseBall ) ;

        {   // Inscribe a ring inside the texture.
            // This simulates limn darkening.
            Image ring ;
            ring.CopyShape( noiseBall ) ;
            const float radius = float( ring.GetWidth() / 2 - 4 ) ;
            static const float thickness = 2.1f ;
            ImgOpDrawCircle::DrawCircle( ring , radius , thickness ) ;
            const Vec4 limnTint( tint.x * 0.0f , tint.y * 0.0f , tint.z * 0.0f , tint.w ) ;
            ImgOpTint::Tint( ring , limnTint ) ;
            ImgOpOverlay::Overlay( noiseBall , ring , noiseBall ) ;
        }

        const unsigned  numSmoothingPasses  = noiseBall.GetHeight() / 2 ;

        ImgOpBlur::Blur( noiseBall , numSmoothingPasses ) ;
        ImgOpRadialGradient::RadialGradient( noiseBall , /* power */ 1.0f , /* max */ 1.0f , /* alpha component index */ 3 ) ;
        ImgOpApplyGamma::ApplyGamma( noiseBall , gamma ) ;
        ImgOpTint::Tint( noiseBall , tint ) ;
    }




    void ImgOpSeq_DecoratedNoiseBall( PeGaSys::Image & noiseBall , float gamma , const Vec4 & color )
    {
        PERF_BLOCK( ImgOpSeq_DecoratedNoiseBall ) ;

        using namespace PeGaSys ;
        ImgOpSeq_NoiseBall( noiseBall , gamma , color ) ;

        {   // Draw a faint frame around the texture border.
            Image frame ;
            frame.CopyShape( noiseBall ) ;
            ImgOpDrawBox::DrawBox( frame , 16 ) ;
            ImgOpTint::Tint( frame , Vec4( 1.0f , 1.0f , 1.0f , 0.25f ) ) ;
            ImgOpOverlay::Overlay( noiseBall , frame , noiseBall ) ;
        }

        {   // Inscribe a ring inside the texture.
            // This demarks the vorton boundary.
            Image ring ;
            ring.CopyShape( noiseBall ) ;
            ImgOpDrawCircle::DrawCircle( ring , /* radius */ float( ring.GetWidth() / 2 - 4 ) , /* thickness */ 2.1f ) ;
            ImgOpBlur::Blur( ring , /* num smoothing passes */ 2 ) ;
            ImgOpTint::Tint( ring , Vec4( 1.0f , 1.0f , 1.0f , 0.5f ) ) ;
            ImgOpOverlay::Overlay( noiseBall , ring , noiseBall ) ;
        }

        {   // Draw a faint dot at the center of the texture.
            Image dot ;
            dot.CopyShape( noiseBall ) ;
            ImgOpDrawCircle::DrawCircle( dot , /* radius */ 2 , /* thickness */ 2.1f ) ;
            ImgOpBlur::Blur( dot , /* num smoothing passes */ 2 ) ;
            ImgOpTint::Tint( dot , Vec4( 1.0f , 1.0f , 1.0f , 0.25f ) ) ;
            ImgOpOverlay::Overlay( noiseBall , dot , noiseBall ) ;
        }
    }

}
