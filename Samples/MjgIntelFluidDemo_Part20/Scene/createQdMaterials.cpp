/** \file materials.cpp

    \brief Routines to create render materials and textures for InteSiVis.

*/

#include "Scene/fluidScene.h" // For Make*Image declarations

#include "inteSiVis.h"

#include <Image/image.h>
#include <Image/imgOpSequences.h>

#include <Core/Performance/perfBlock.h>




/** Make an image with a simple noise texture.

    \param texWidth Texture width in pixels

    \param texHeight - see AllocateTexture

    \param numTexPages - see AllocateTexture

    \param gamma - see AssignTexturePage
*/
void MakeSimpleNoiseImage( PeGaSys::Image & noiseImage , int texWidth , int texHeight , int numTexPages , float gamma )
{
    PERF_BLOCK( MakeSimpleNoiseImage ) ;

    const Vec4 blankColor( 1.0f , 1.0f , 1.0f , 1.0f ) ;

    noiseImage.SetSize( texWidth , texHeight , 4 , numTexPages ) ;
    ImgOpSeq_Noise( noiseImage , gamma , blankColor ) ;
}




// Light & heavy diagnostic vorton images.
// This material is used to render vortons diagnostically, that is, for
// diagnosing the algorithm.  This material is meant to reveal the vortons
// and their state more clearly.
void MakeDiagnosticVortonImage( PeGaSys::Image & noiseBall )
{
    PERF_BLOCK( MakeDiagnosticVortonImage ) ;

    static const unsigned width         = 128 ;
    static const unsigned height        = 128 ;
    static const unsigned numChannels   = 4 ;
    static const unsigned numPages      = 2 ;

    const Vec4 translucentYellowish( 0.9f , 0.9f , 0.1f , 0.125f ) ;
    const Vec4 translucentCyanish  ( 0.1f , 0.9f , 0.9f , 0.25f ) ;

    noiseBall.SetSize( width , height , numChannels , numPages ) ;

    PeGaSys::Image noiseBallLight( noiseBall , PeGaSys::Image::EXTRACT_PAGE , 0 ) ;   // Page 0 of noiseBall
    PeGaSys::Image noiseBallHeavy( noiseBall , PeGaSys::Image::EXTRACT_PAGE , 1 ) ;   // Page 1 of noiseBall
    ImgOpSeq_DecoratedNoiseBall( noiseBallLight , 1.5f , translucentYellowish ) ;
    ImgOpSeq_DecoratedNoiseBall( noiseBallHeavy , 0.5f , translucentCyanish   ) ;
}




// Light & heavy simple vorton images.
// This material is used to render vortons for visual effects.  This
// material lacks the obvious decorations, such as outlines and center dot.
void MakeDiagnosticSimpleVortonImage( PeGaSys::Image & noiseBall )
{
    PERF_BLOCK( MakeDiagnosticSimpleVortonImage ) ;

    static const unsigned width         = 64 ;
    static const unsigned height        = 64 ;
    static const unsigned numChannels   = 4 ;
    static const unsigned numPages      = 2 ;

    const Vec4 translucentYellowish( 0.9f , 0.9f , 0.1f , 0.125f ) ;
    const Vec4 translucentCyanish  ( 0.1f , 0.9f , 0.9f , 0.25f ) ;

    noiseBall.SetSize( width , height , numChannels , numPages ) ;

    PeGaSys::Image noiseBallLight( noiseBall , PeGaSys::Image::EXTRACT_PAGE , 0 ) ;   // Page 0 of noiseBall
    PeGaSys::Image noiseBallHeavy( noiseBall , PeGaSys::Image::EXTRACT_PAGE , 1 ) ;   // Page 1 of noiseBall
    ImgOpSeq_NoiseBall( noiseBallLight , 1.5f , translucentYellowish ) ;
    ImgOpSeq_NoiseBall( noiseBallHeavy , 0.5f , translucentCyanish   ) ;
}




// Light & heavy tracer dye images
void MakeDyeImage( PeGaSys::Image & noiseBall )
{
    PERF_BLOCK( MakeDyeImage ) ;

    const Vec4 translucentRedish( 0.9f , 0.1f , 0.1f , 0.125f ) ; // redish
    const Vec4 translucentBluish( 0.1f , 0.1f , 0.9f , 0.125f ) ; // blueish

    noiseBall.SetSize( 64 , 64 , 4 , 2 ) ;

    PeGaSys::Image noiseBallLight( noiseBall , PeGaSys::Image::EXTRACT_PAGE , 0 ) ;   // Page 0 of noiseBall
    PeGaSys::Image noiseBallHeavy( noiseBall , PeGaSys::Image::EXTRACT_PAGE , 1 ) ;   // Page 1 of noiseBall
    ImgOpSeq_NoiseBall( noiseBallLight , 0.5f , translucentRedish ) ;
    ImgOpSeq_NoiseBall( noiseBallHeavy , 0.5f , translucentBluish ) ;
}




// Light & heavy tracer diagnostic images
void MakeDiagnosticTracerImage( PeGaSys::Image & noiseBall )
{
    PERF_BLOCK( MakeDiagnosticTracerImage ) ;
    static const unsigned width         = 128 ;
    static const unsigned height        = 128 ;
    static const unsigned numChannels   = 4 ;
    static const unsigned numPages      = 2 ;

#if 1
    const Vec4 translucentRedish( 0.9f , 0.1f , 0.1f , 0.125f ) ; // redish
    const Vec4 translucentBluish( 0.1f , 0.1f , 0.9f , 0.125f ) ; // blueish
#else
    const Vec4 translucentRedish( 0.9f , 0.1f , 0.1f , 1.f ) ; // redish
    const Vec4 translucentBluish( 0.1f , 0.1f , 0.9f , 0.0f ) ; // blueish
#endif

    noiseBall.SetSize( width , height , numChannels , numPages ) ;

    PeGaSys::Image noiseBallLight( noiseBall , PeGaSys::Image::EXTRACT_PAGE , 0 ) ;   // Page 0 of noiseBall
    PeGaSys::Image noiseBallHeavy( noiseBall , PeGaSys::Image::EXTRACT_PAGE , 1 ) ;   // Page 1 of noiseBall
    ImgOpSeq_DecoratedNoiseBall( noiseBallLight , 1.5f , translucentRedish ) ;
    ImgOpSeq_DecoratedNoiseBall( noiseBallHeavy , 0.5f , translucentBluish   ) ;
}




// Smoke image
void MakeSmokeImage( PeGaSys::Image & noiseBall )
{
    PERF_BLOCK( MakeSmokeImage ) ;

    const Vec4 translucentGray( 0.7f , 0.7f , 0.75f , 0.125f ) ; // light gray with a hint of blue.

    noiseBall.SetSize( 64 , 64 , 4 , 1 ) ;
    ImgOpSeq_NoiseBall( noiseBall , 0.5f , translucentGray ) ;
}




// Flame image
void MakeFlameImage( PeGaSys::Image & noiseBall )
{
    PERF_BLOCK( MakeFlameImage ) ;

    const Vec4 orange( 1.0f , 0.5f , 0.1f , 1.0f ) ; // orange.

    noiseBall.SetSize( 64 , 64 , 4 , 1 ) ;
    ImgOpSeq_NoiseBall( noiseBall , 0.5f , orange ) ;
}




// Fuel image
void MakeFuelImage( PeGaSys::Image & noiseBall )
{
    PERF_BLOCK( MakeFuelImage ) ;

    const Vec4 translucentBluish( 0.1f , 0.2f , 0.9f , 0.125f ) ; // blueish

    noiseBall.SetSize( 64 , 64 , 4 , 1 ) ;
    ImgOpSeq_NoiseBall( noiseBall , 0.5f , translucentBluish ) ;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/** Make a material with a simple noise texture.

    \param texWidth - see AllocateTexture

    \param texHeight - see AllocateTexture

    \param numTexPages - see AllocateTexture

    \param gamma - see AssignTexturePage

    \note   This can only run after the device is initialized.
            Due to order-of-operations this cannot easily run
            in constructor.
*/
static void MakeSimpleNoiseQdMaterial( QdMaterial & material , int texWidth , int texHeight , int numTexPages , float gamma )
{
    PERF_BLOCK( MakeSimpleNoiseQdMaterial ) ;

    PeGaSys::Image noise ;
    MakeSimpleNoiseImage( noise , texWidth , texHeight , numTexPages , gamma ) ;

    material.AssignTexture( noise ) ;
}




/** Create rendering materials.
*/
void InteSiVis::CreateQdMaterials()
{
    PERF_BLOCK( InteSiVis__CreateQdMaterials ) ;

    ASSERT( ! mInitialized ) ; // This must be the first time this app has displayed anything.

    {
        PERF_BLOCK( InteSiVis__CreateQdMaterials_BoundingBox ) ;
        MakeSimpleNoiseQdMaterial( mBoundingBoxMaterial , 8 , 8 , 1 , 0.0f ) ;
        mBoundingBoxMaterial.SetColor( Vec4( 0.7f , 0.7f , 0.7f , 0.1f ) ) ;
        mBoundingBoxMaterial.SetDepthWrite( false ) ;
        mBoundingBoxMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
        mBoundingBoxMaterial.SetCullFace( QdMaterial::CF_NONE ) ;
    }

    {
        PERF_BLOCK( InteSiVis__CreateQdMaterials_Pathline ) ;
        MakeSimpleNoiseQdMaterial( mPathlineMaterial , 8 , 8 , 1 , 0.0f ) ;
        mPathlineMaterial.SetColor( Vec4( 0.2f , 0.5f , 1.0f , 1.0f ) ) ;
        mPathlineMaterial.SetDepthWrite( false ) ;
        mPathlineMaterial.SetBlendMode( QdMaterial::BM_ALPHA ) ;
    }

    mInitialized = true ;
}
