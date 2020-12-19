/** \file D3D9_texture.cpp

    \brief Texture for Direct3D version 9

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/

#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#include <d3d9.h>

#include "Image/image.h"

#include "Render/Resource/textureSampler.h"

#include "Render/Platform/DirectX9/D3D9_api.h" // for HROK

#include "Render/Platform/DirectX9/D3D9_texture.h"

extern LPDIRECT3DDEVICE9 g_pd3dDevice ; // Direct3D rendering device

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {




static D3DTEXTUREFILTERTYPE SamplerState_FilterEnum( SamplerState::FilterE filter )
{

    switch( filter )
    {
    case SamplerState::FILTER_NEAREST   : return D3DTEXF_POINT  ; break ;
    case SamplerState::FILTER_LINEAR    : return D3DTEXF_LINEAR ; break ;
    default: FAIL() ; break ;
    }

    return D3DTEXF_POINT ;
}




static D3DTEXTUREADDRESS  SamplerState_AddressEnum( SamplerState::AddressE address )
{
    switch( address )
    {
    case SamplerState::ADDRESS_CLAMP    : return D3DTADDRESS_CLAMP  ; break ;
    case SamplerState::ADDRESS_REPEAT   : return D3DTADDRESS_WRAP   ; break ;
    default: FAIL() ; break ;
    }
    return D3DTADDRESS_CLAMP ;
}




static void SamplerState_Apply( const SamplerStateS & samplerState )
{
    {
        D3DTEXTUREFILTERTYPE minFilter = SamplerState_FilterEnum( samplerState.mMinFilter ) ;
        HROK( g_pd3dDevice->SetSamplerState( 0, D3DSAMP_MINFILTER, minFilter ) ) ;
    }
    {
        D3DTEXTUREFILTERTYPE magFilter = SamplerState_FilterEnum( samplerState.mMagFilter ) ;
        HROK( g_pd3dDevice->SetSamplerState( 0, D3DSAMP_MAGFILTER, magFilter ) ) ;
    }
    {
        D3DTEXTUREFILTERTYPE mipFilter = SamplerState_FilterEnum( samplerState.mMipFilter ) ;
        HROK( g_pd3dDevice->SetSamplerState( 0, D3DSAMP_MAGFILTER, mipFilter ) ) ;
    }
    {
        D3DTEXTUREADDRESS addressU = SamplerState_AddressEnum( samplerState.mAddressU ) ;
        HROK( g_pd3dDevice->SetSamplerState( 0, D3DSAMP_ADDRESSU  , addressU ) ) ;
    }
    {
        D3DTEXTUREADDRESS addressV = SamplerState_AddressEnum( samplerState.mAddressV ) ;
        HROK( g_pd3dDevice->SetSamplerState( 0, D3DSAMP_ADDRESSV  , addressV ) ) ;
    }
    {
        D3DTEXTUREADDRESS addressW = SamplerState_AddressEnum( samplerState.mAddressW ) ;
        HROK( g_pd3dDevice->SetSamplerState( 0, D3DSAMP_ADDRESSW  , addressW ) ) ;
    }

    {
        D3DTEXTUREOP comboOp = D3DTOP_MODULATE ;
        switch( samplerState.mCombineOperation )
        {
        case SamplerState::COMBO_OP_REPLACE : comboOp = D3DTOP_DISABLE  ; break ;
        case SamplerState::COMBO_OP_MODULATE: comboOp = D3DTOP_MODULATE ; break ;
        //case SamplerState::COMBO_OP_DECAL   : comboOp = D3DTOP_MODULATE ; break ;
        }
        HROK( g_pd3dDevice->SetTextureStageState( 0 , D3DTSS_COLOROP  , D3DTOP_MODULATE ) ) ;
        HROK( g_pd3dDevice->SetTextureStageState( 0 , D3DTSS_COLORARG1, D3DTA_TEXTURE   ) ) ;
        HROK( g_pd3dDevice->SetTextureStageState( 0 , D3DTSS_COLORARG2, D3DTA_DIFFUSE   ) ) ;
        HROK( g_pd3dDevice->SetTextureStageState( 0 , D3DTSS_ALPHAOP  , D3DTOP_DISABLE  ) ) ;
    }
}




/** Construct texture for Direct3D version 9.
*/
D3D9_Texture::D3D9_Texture()
    : TextureBase()
    , mTexture( 0 )
{
}




/** Destruct texture for Direct3D version 9.
*/
D3D9_Texture::~D3D9_Texture()
{
}




/* virtual */ void D3D9_Texture::Bind( ApiBase * /*renderApi*/ , const SamplerStateS & samplerState )
{
    g_pd3dDevice->SetTexture( 0 , mTexture ) ;

    SamplerState_Apply( samplerState ) ;
}




void D3D9_Texture::Create2DTextureFromImage( const Image * image )
{
    ASSERT( image != NULL ) ;

    // Make sure image is sane.
    ASSERT( image->GetWidth() > 0 ) ;
    ASSERT( image->GetHeight() > 0 ) ;
    ASSERT( image->GetNumChannels() > 0 ) ;
    ASSERT( image->GetNumPages() > 0 ) ;
    ASSERT( image->GetImageData() != 0 ) ;

    ASSERT( image->GetNumChannels() == 4 ) ; // For now, only support RGBA images
    ASSERT( image->GetNumPages() == 1 ) ; // For now, only support single-page images

    ASSERT( ( GetWidth() == static_cast< int >( image->GetWidth() ) ) || ( GetWidth() == 0 ) ) ;
    ASSERT( ( GetHeight() == static_cast< int >( image->GetHeight() ) ) || ( GetHeight() == 0 ) ) ;
    ASSERT( ( GetNumPlanes() == 1 ) || ( GetNumPlanes() == 0 ) ) ;
    ASSERT( ( GetShape() == TEX_SHAPE_2D ) || ( GetShape() == TEX_SHAPE_UNDEFINED ) ) ;
    ASSERT( ( GetShape() == TEX_SHAPE_2D ) || ( GetShape() == TEX_SHAPE_UNDEFINED ) ) ;

    ASSERT( ( GetFormat() == TEX_FORMAT_A8R8G8B8 ) || ( GetFormat() == TEX_FORMAT_UNDEFINED ) ) ; // For now, only support RGBA textures
    ASSERT( GetUsageFlags() == TEX_USAGE_DEFAULT ) ; // For now, only support default usage.

    // Make sure this material does not already have a texture,
    // otherwise the existing texture will "leak" video memory.
    ASSERT( 0 == mTexture ) ;

    //mNumTexPages = image->GetNumPages() ;

#if 0
    if( FAILED( D3DXCreateTextureFromFile( g_pd3dDevice , L"..\\banana.bmp", & mTexture ) ) )
    {
        FAIL() ;
    }
#else
    HROK( g_pd3dDevice->CreateTexture(
            image->GetWidth()
        ,   image->GetHeight()
        ,   1 // 0 // Generate all MIP levels
        ,   /*D3DUSAGE_AUTOGENMIPMAP | */D3DUSAGE_DYNAMIC
        ,   D3DFMT_A8R8G8B8
        ,   D3DPOOL_DEFAULT
        ,   & mTexture
        ,   NULL
        ) ) ;

    {
        D3DLOCKED_RECT lockedRectangle ;
        RECT rectangleToLock = { 0 } ;
        rectangleToLock.left = 0 ;
        rectangleToLock.top = 0 ;
        rectangleToLock.right = image->GetWidth() ;
        rectangleToLock.bottom = image->GetHeight() ;
        HRESULT hr = mTexture->LockRect(
              0 // level
            , & lockedRectangle
            , NULL // rectangle to lock.
            , 0 // lock flags
            ) ;
        ASSERT( SUCCEEDED( hr ) ) ; NON_DEBUG_ONLY( UNUSED_PARAM( hr ) ) ;
        // Populate texture image data.
        for( unsigned iy = 0 ; iy < image->GetHeight() ; ++ iy )
        {   // For each row in the image...
            BYTE * pData = static_cast< BYTE * >( lockedRectangle.pBits ) + lockedRectangle.Pitch * iy ;
            for( unsigned ix = 0 ; ix < image->GetWidth() ; ++ ix )
            {   // For each column in the image...
                unsigned offset = ix * image->GetXStride() + image->GetYStride() * iy ;
                // Assign color components for current texel from corresponding image pixel.
                pData[ ix * 4 + 0 ] = (*image)[ offset + 2 ] ;
                pData[ ix * 4 + 1 ] = (*image)[ offset + 1 ] ;
                pData[ ix * 4 + 2 ] = (*image)[ offset + 0 ] ;
                pData[ ix * 4 + 3 ] = (*image)[ offset + 3 ] ;
            }
        }
        HROK( mTexture->UnlockRect( 0 ) ) ;
    }
    // Generate MIP maps for all texture pages.
    //mTexture->GenerateMipSubLevels() ;
    SetNumMipLevels( mTexture->GetLevelCount() ) ;
#endif
}





/* virtual */ void D3D9_Texture::CreateFromImages( const Image * images , size_t numImages )
{
    if( 1 == numImages )
    {   // Source is a single image.
        // A single image can only be used to make 1D or 2D textures.
        if( ( GetShape() == TEX_SHAPE_2D ) || ( GetShape() == TEX_SHAPE_UNDEFINED ) )
        {   // Texture shape is either explicitly 2D or not yet defined.
            Create2DTextureFromImage( images ) ;
        }
        else
        {   // Texture shape is explicitly defined as something either incompatable or not yet implemented.
            FAIL() ;
        }
    }
    else
    {   // Multiple source images.
        FAIL() ; // Not yet supported.
    }
}




/* virtual */ void D3D9_Texture::CopyToImage( Image & /*image*/ )
{
}




    } ;
} ;


#if defined( _DEBUG )

void PeGaSys_Render_D3D9_Texture_UnitTest()
{
    DebugPrintf( "D3D9_Texture::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::D3D9_Texture d3d9_texture ;
    }

    DebugPrintf( "D3D9_Texture::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif