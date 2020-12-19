/** \file materials.cpp

    \brief Routines to create render materials and textures for InteSiVis.

*/

#include "inteSiVis.h"

#include "Image/Operation/imgOpApplyGamma.h"
#include "Image/Operation/imgOpBlur.h"
#include "Image/Operation/imgOpDrawBox.h"
#include "Image/Operation/imgOpDrawCircle.h"
#include "Image/Operation/imgOpMakeNoise.h"
#include "Image/Operation/imgOpOverlay.h"
#include "Image/Operation/imgOpLinearGradient.h"
#include "Image/Operation/imgOpRadialGradient.h"
#include "Image/Operation/imgOpTint.h"




static void ImgOpSeq_Noise( PeGaSys::Image & noise , float gamma , const Vec4 & tint )
{
    using namespace PeGaSys ;
    const unsigned  numSmoothingPasses  = noise.GetHeight() / 16 ;
    ImgOpMakeNoise::MakeNoise( noise ) ;
    ImgOpBlur::Blur( noise , numSmoothingPasses ) ;
    ImgOpApplyGamma::ApplyGamma( noise , gamma ) ;
    ImgOpTint::Tint( noise , tint ) ;
}




static void ImgOpSeq_GradientNoise( PeGaSys::Image & noise , float gamma , const Vec4 & tint )
{
    using namespace PeGaSys ;
    const unsigned  numSmoothingPasses  = noise.GetHeight() / 16 ;
    ImgOpMakeNoise::MakeNoise( noise ) ;
    ImgOpBlur::Blur( noise , numSmoothingPasses ) ;
    ImgOpApplyGamma::ApplyGamma( noise , gamma ) ;
    ImgOpTint::Tint( noise , tint ) ;
    ImgOpLinearGradient::Gradient( noise , Vec2( 0.0f , 1.0f ) , Vec4( 0.1f , 0.1f , 0.1f , 1.0f ) , Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) ) ;
}




static void ImgOpSeq_NoiseBall( PeGaSys::Image & noiseBall , float gamma , const Vec4 & tint )
{
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




static void ImgOpSeq_DecoratedNoiseBall( PeGaSys::Image & noiseBall , float gamma , const Vec4 & color )
{
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




/** Make a material with a simple noise texture.

    \param texWidth - see AllocateTexture

    \param texHeight - see AllocateTexture

    \param numTexPages - see AllocateTexture

    \param gamma - see AssignTexturePage

    \note This can only run after the device is initialized.
            Due to order-of-operations this cannot easily run
            in constructor.
*/
static void MakeSimpleNoiseTexture( QdMaterial & material , int texWidth , int texHeight , int numTexPages , float gamma )
{
    const Vec4 blankColor( 1.0f , 1.0f , 1.0f , 1.0f ) ;

    PeGaSys::Image noise( texWidth , texHeight , 4 , numTexPages ) ;
    ImgOpSeq_Noise( noise , gamma , blankColor ) ;
    material.AssignTexture( noise ) ;
}




/** Make a material with a gradient noise texture.

    \param texWidth - see AllocateTexture

    \param texHeight - see AllocateTexture

    \param numTexPages - see AllocateTexture

    \param gamma - see AssignTexturePage

    \note This can only run after the device is initialized.
            Due to order-of-operations this cannot easily run
            in constructor.
*/
static void MakeGradientNoiseTexture( QdMaterial & material , int texWidth , int texHeight , int numTexPages , float gamma )
{
    const Vec4 blankColor( 1.0f , 1.0f , 1.0f , 1.0f ) ;

    PeGaSys::Image noise( texWidth , texHeight , 4 , numTexPages ) ;
    ImgOpSeq_GradientNoise( noise , gamma , blankColor ) ;
    material.AssignTexture( noise ) ;
}




/** Create materials for rendering particles.
*/
void InteSiVis::CreateParticleMaterials()
{
    ASSERT( ! mInitialized ) ; // This must be the first time this app has displayed anything.

    // Light & heavy diagnostic vorton materials.
    // This material is used to render vortons diagnostically, that is, for
    // diagnosing the algorithm.  This material is meant to reveal the vortons
    // and their state more clearly.
    {
        static const unsigned width         = 256 ;
        static const unsigned height        = 256 ;
        static const unsigned numChannels   = 4 ;
        static const unsigned numPages      = 2 ;

        const Vec4 translucentYellowish( 0.9f , 0.9f , 0.1f , 0.125f ) ;
        const Vec4 translucentCyanish  ( 0.1f , 0.9f , 0.9f , 0.25f ) ;

        PeGaSys::Image noiseBall( width , height , numChannels , numPages ) ;
        PeGaSys::Image noiseBallLight( noiseBall , 0 ) ;   // Page 0 of noiseBall
        PeGaSys::Image noiseBallHeavy( noiseBall , 1 ) ;   // Page 1 of noiseBall
        ImgOpSeq_DecoratedNoiseBall( noiseBallLight , 1.5f , translucentYellowish ) ;
        ImgOpSeq_DecoratedNoiseBall( noiseBallHeavy , 0.5f , translucentCyanish   ) ;
        mDiagnosticVortonMaterial.AssignTexture( noiseBall ) ;

        mDiagnosticVortonMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mDiagnosticVortonMaterial.SetDepthWrite( false ) ;
        mDiagnosticVortonMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        ASSERT( 1.0f == mDiagnosticVortonMaterial.GetScale() ) ;
        ASSERT( FLT_MAX == mDiagnosticVortonMaterial.GetDensityVisibility() ) ;
    }

    // Light & heavy simple vorton materials.
    // This material is used to render vortons for visual effects.  This
    // material lacks the obvious decorations, such as outlines and center dot.
    {
        static const unsigned width         = 256 ;
        static const unsigned height        = 256 ;
        static const unsigned numChannels   = 4 ;
        static const unsigned numPages      = 2 ;

        const Vec4 translucentYellowish( 0.9f , 0.9f , 0.1f , 0.125f ) ;
        const Vec4 translucentCyanish  ( 0.1f , 0.9f , 0.9f , 0.25f ) ;

        PeGaSys::Image noiseBall( width , height , numChannels , numPages ) ;
        PeGaSys::Image noiseBallLight( noiseBall , 0 ) ;   // Page 0 of noiseBall
        PeGaSys::Image noiseBallHeavy( noiseBall , 1 ) ;   // Page 1 of noiseBall
        ImgOpSeq_NoiseBall( noiseBallLight , 1.5f , translucentYellowish ) ;
        ImgOpSeq_NoiseBall( noiseBallHeavy , 0.5f , translucentCyanish   ) ;
        mSimpleVortonMaterial.AssignTexture( noiseBall ) ;

        mSimpleVortonMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mSimpleVortonMaterial.SetDepthWrite( false ) ;
        mSimpleVortonMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        ASSERT( 1.0f == mSimpleVortonMaterial.GetScale() ) ;
        ASSERT( FLT_MAX == mSimpleVortonMaterial.GetDensityVisibility() ) ;
    }

    // Light & heavy tracer dye materials
    {
        const Vec4 translucentRedish( 0.9f , 0.1f , 0.1f , 0.125f ) ; // redish
        const Vec4 translucentBluish( 0.1f , 0.1f , 0.9f , 0.125f ) ; // blueish

        PeGaSys::Image noiseBall( 128 , 128 , 4 , 2 ) ;
        PeGaSys::Image noiseBallLight( noiseBall , 0 ) ;   // Page 0 of noiseBall
        PeGaSys::Image noiseBallHeavy( noiseBall , 1 ) ;   // Page 1 of noiseBall
        ImgOpSeq_NoiseBall( noiseBallLight , 0.5f , translucentRedish ) ;
        ImgOpSeq_NoiseBall( noiseBallHeavy , 0.5f , translucentBluish   ) ;
        mDyeMaterial.AssignTexture( noiseBall ) ;

        mDyeMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mDyeMaterial.SetDepthWrite( false ) ;
        mDyeMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        mDyeMaterial.SetScale( 1.0123f ) ;
    #if ENABLE_FIRE
        mDyeMaterial.SetDensityVisibility( 255.0f * 1000.0f ) ;
    #else
        mDyeMaterial.SetDensityVisibility( FLT_MAX ) ;
    #endif
    }

    // Light & heavy tracer diagnostic materials
    {
        static const unsigned width         = 256 ;
        static const unsigned height        = 256 ;
        static const unsigned numChannels   = 4 ;
        static const unsigned numPages      = 2 ;

#if 1
        const Vec4 translucentRedish( 0.9f , 0.1f , 0.1f , 0.125f ) ; // redish
        const Vec4 translucentBluish( 0.1f , 0.1f , 0.9f , 0.125f ) ; // blueish
#else
        const Vec4 translucentRedish( 0.9f , 0.1f , 0.1f , 1.f ) ; // redish
        const Vec4 translucentBluish( 0.1f , 0.1f , 0.9f , 0.0f ) ; // blueish
#endif

        PeGaSys::Image noiseBall( width , height , numChannels , numPages ) ;
        PeGaSys::Image noiseBallLight( noiseBall , 0 ) ;   // Page 0 of noiseBall
        PeGaSys::Image noiseBallHeavy( noiseBall , 1 ) ;   // Page 1 of noiseBall
        ImgOpSeq_DecoratedNoiseBall( noiseBallLight , 1.5f , translucentRedish ) ;
        ImgOpSeq_DecoratedNoiseBall( noiseBallHeavy , 0.5f , translucentBluish   ) ;
        mDiagnosticTracerMaterial.AssignTexture( noiseBall ) ;

        mDiagnosticTracerMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mDiagnosticTracerMaterial.SetDepthWrite( false ) ;
        mDiagnosticTracerMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        ASSERT( 1.0f == mDiagnosticTracerMaterial.GetScale() ) ;
        ASSERT( FLT_MAX == mDiagnosticTracerMaterial.GetDensityVisibility() ) ;
    }

    // Smoke material
    {
        const Vec4 translucentGray( 0.7f , 0.7f , 0.75f , 0.125f ) ; // light gray with a hint of blue.

        PeGaSys::Image noiseBall( 128 , 128 , 4 , 1 ) ;
        ImgOpSeq_NoiseBall( noiseBall , 0.5f , translucentGray ) ;
        mSmokeMaterial.AssignTexture( noiseBall ) ;

        mSmokeMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mSmokeMaterial.SetDepthWrite( false ) ;
        mSmokeMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        mSmokeMaterial.SetScale( 4.0123f ) ;
    #if ENABLE_FIRE
        mSmokeMaterial.SetDensityVisibility( 100.0f ) ;
    #else
        mSmokeMaterial.SetDensityVisibility( 0.0f ) ;
    #endif
    }

    // Flame material
    {
        const Vec4 orange( 1.0f , 0.5f , 0.1f , 1.0f ) ; // orange.

        PeGaSys::Image noiseBall( 128 , 128 , 4 , 1 ) ;
        ImgOpSeq_NoiseBall( noiseBall , 0.5f , orange ) ;
        mFlameMaterial.AssignTexture( noiseBall ) ;

        mFlameMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mFlameMaterial.SetDepthWrite( false ) ;
        mFlameMaterial.SetBlendMode( QdMaterial::BM_ADDITIVE ) ;
        mFlameMaterial.SetScale( 4.0f ) ;
    #if ENABLE_FIRE
        mFlameMaterial.SetDensityVisibility( 255.0f * 100.0f ) ;
    #else
        mFlameMaterial.SetDensityVisibility( 0.0f ) ;
    #endif
    }

    // Fuel material.
    {
        const Vec4 translucentBluish( 0.1f , 0.2f , 0.9f , 0.125f ) ; // blueish

        PeGaSys::Image noiseBall( 128 , 128 , 4 , 1 ) ;
        ImgOpSeq_NoiseBall( noiseBall , 0.5f , translucentBluish ) ;
        mFuelMaterial.AssignTexture( noiseBall ) ;

        mFuelMaterial.SetColor( Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) );
        mFuelMaterial.SetDepthWrite( false ) ;
        mFuelMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        mFuelMaterial.SetScale( 2.0987f ) ;
    #if ENABLE_FIRE
        mFuelMaterial.SetDensityVisibility( 255.0f * 100.0f ) ;
    #else
        mFuelMaterial.SetDensityVisibility( FLT_MAX ) ;
    #endif
    }

}




/** Create rendering materials.
*/
void InteSiVis::CreateMaterials()
{
    ASSERT( ! mInitialized ) ; // This must be the first time this app has displayed anything.

    CreateParticleMaterials() ;

    MakeSimpleNoiseTexture( mPhysObjMaterial , 256 , 256 , 1 , 0.5f ) ;
    mPhysObjMaterial.SetColor( Vec4( 0.8f , 0.4f , 0.8f , 1.0f ) ) ;

    MakeSimpleNoiseTexture( mContainerMaterial , 256 , 256 , 1 , 0.2f ) ;
    mContainerMaterial.SetColor( Vec4( 0.8f , 0.8f , 0.8f , 0.1f ) ) ;
    mContainerMaterial.SetCullFace( QdMaterial::CF_NONE ) ;
    mContainerMaterial.SetDepthWrite( false ) ;
    mContainerMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;

    MakeSimpleNoiseTexture( mBoundingBoxMaterial , 8 , 8 , 1 , 0.0f ) ;
    mBoundingBoxMaterial.SetColor( Vec4( 0.7f , 0.7f , 0.7f , 0.1f ) ) ;
    mBoundingBoxMaterial.SetDepthWrite( false ) ;
    mBoundingBoxMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
    mBoundingBoxMaterial.SetCullFace( QdMaterial::CF_NONE ) ;

    MakeSimpleNoiseTexture( mPathlineMaterial , 8 , 8 , 1 , 0.0f ) ;
    mPathlineMaterial.SetColor( Vec4( 0.2f , 0.5f , 1.0f , 1.0f ) ) ;
    mPathlineMaterial.SetDepthWrite( false ) ;
    mPathlineMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;

    MakeGradientNoiseTexture( mSkyMaterial , 512 , 512 , 1 , 0.2f ) ;
    mSkyMaterial.SetColor( Vec4( 0.3f , 0.3f , 0.3f , 1.0f ) ) ;

    mInitialized = true ;
}
