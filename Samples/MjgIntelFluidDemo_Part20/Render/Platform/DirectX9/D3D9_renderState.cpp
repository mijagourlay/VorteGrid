/** \file D3D9_renderState.cpp

    \brief Wrapper for routines specific to D3D9 render system API

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Core/Utility/macros.h"
#include "Core/File/debugPrint.h"

#include <windows.h>

#include <d3d9.h>
#include <d3d9Types.h>

#include "Render/Platform/DirectX9/D3D9_api.h" // for HROK

#include "Render/Platform/DirectX9/D3D9_renderState.h"


// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------

extern LPDIRECT3DDEVICE9   g_pd3dDevice ; // Direct3D rendering device

namespace PeGaSys {
    namespace Render {

// Private functions -----------------------------------------------------------

static void BlendState_Apply( const BlendStateS & blendState )
{
    HROK( g_pd3dDevice->SetRenderState( D3DRS_ALPHABLENDENABLE , blendState.mBlendEnabled ? TRUE : FALSE ) ) ;

    {
        D3DBLEND srcFactor = D3DBLEND_ONE ;

        switch( blendState.mBlendSrcAlpha )
        {
        case BlendState::BLEND_FACTOR_ZERO          : srcFactor = D3DBLEND_ZERO         ; break ;
        case BlendState::BLEND_FACTOR_ONE           : srcFactor = D3DBLEND_ONE          ; break ; // Default / additive
        case BlendState::BLEND_FACTOR_SRC_ALPHA     : srcFactor = D3DBLEND_SRCALPHA     ; break ; // Typical alpha blending
        case BlendState::BLEND_FACTOR_INV_SRC_ALPHA : srcFactor = D3DBLEND_INVSRCALPHA  ; break ;
            // TODO: FIXME: Support other blend factors
        default: FAIL() ; break ;
        }

        HROK( g_pd3dDevice->SetRenderState( D3DRS_SRCBLEND , srcFactor ) ) ;
    }

    {
        D3DBLEND dstFactor = D3DBLEND_ONE ;

        switch( blendState.mBlendDstAlpha )
        {
        case BlendState::BLEND_FACTOR_ZERO          : dstFactor = D3DBLEND_ZERO         ; break ;
        case BlendState::BLEND_FACTOR_ONE           : dstFactor = D3DBLEND_ONE          ; break ; // Default / additive
        case BlendState::BLEND_FACTOR_SRC_ALPHA     : dstFactor = D3DBLEND_SRCALPHA     ; break ;
        case BlendState::BLEND_FACTOR_INV_SRC_ALPHA : dstFactor = D3DBLEND_INVSRCALPHA  ; break ; // Typical alpha blending
            // TODO: FIXME: Support other blend factors
        default: FAIL() ; break ;
        }

        HROK( g_pd3dDevice->SetRenderState( D3DRS_DESTBLEND , dstFactor ) ) ;
    }

    // TODO: FIXME: Support other blend modes.
    ASSERT( blendState.mBlendSrcColor == blendState.mBlendSrcAlpha ) ;
    ASSERT( blendState.mBlendDstColor == blendState.mBlendDstAlpha ) ;
    ASSERT( blendState.mBlendOpColor  == blendState.mBlendOpAlpha ) ;
    ASSERT( BlendState::BLEND_OP_ADD  == blendState.mBlendOpColor ) ;
}




static D3DCMPFUNC compareFunction( CompareFuncE compareFunc )
{
    switch( compareFunc )
    {
    case CMP_FUNC_NEVER         : return D3DCMP_NEVER           ; break ;
    case CMP_FUNC_LESS          : return D3DCMP_LESS            ; break ;
    case CMP_FUNC_GREATER_EQUAL : return D3DCMP_GREATEREQUAL    ; break ;
    case CMP_FUNC_ALWAYS        : return D3DCMP_ALWAYS          ; break ;
        // TODO: FIXME: Support other comparison functions. See D3DCMPFUNC.
    default: FAIL() ; break ;
    }
    return D3DCMP_FORCE_DWORD ;
}




static void DepthState_Apply( const DepthStateS & depthState )
{
    D3DZBUFFERTYPE  zEnable = D3DZB_TRUE ;
    D3DCMPFUNC      zCmp    = compareFunction( depthState.mDepthFunc ) ;
    switch( depthState.mDepthFunc )
    {
    case CMP_FUNC_NEVER         :
    case CMP_FUNC_LESS          :
    case CMP_FUNC_GREATER_EQUAL :
        // TODO: FIXME: Support other depth modes. See D3DCMPFUNC.
        zEnable = D3DZB_TRUE  ;
        break ;
    case CMP_FUNC_ALWAYS:
        zEnable = D3DZB_FALSE ;
        break ;
    default: FAIL() ; break ;
    }
    HROK( g_pd3dDevice->SetRenderState( D3DRS_ZENABLE , zEnable  ) ) ;
    HROK( g_pd3dDevice->SetRenderState( D3DRS_ZFUNC   , zCmp     ) ) ;

    HROK( g_pd3dDevice->SetRenderState( D3DRS_ZWRITEENABLE , depthState.mDepthWriteEnabled ? TRUE : FALSE ) ) ;

    // TODO: FIXME: Support other depth mode states.
    ASSERT( 0 == depthState.mDepthBias ) ;
    ASSERT( depthState.mDepthClip ) ;
}




static void AlphaState_Apply( const AlphaStateS & alphaState )
{
    // Set alpha-test state.
    HROK( g_pd3dDevice->SetRenderState( D3DRS_ALPHATESTENABLE , alphaState.mAlphaTest ? TRUE : FALSE ) ) ;
    const D3DCMPFUNC compareFunc = compareFunction( alphaState.mAlphaFunc ) ;
    HROK( g_pd3dDevice->SetRenderState( D3DRS_ALPHAFUNC , compareFunc ) ) ;
    int alphaRef = int( alphaState.mAlphaRef * 255.0f + 0.5f ) ;
    ASSERT( ( alphaState.mAlphaRef >= 0 ) && ( alphaState.mAlphaRef <= 255 ) ) ;
    HROK( g_pd3dDevice->SetRenderState( D3DRS_ALPHAREF , alphaRef) ) ;

    RENDER_CHECK_ERROR( AlphaState_Apply ) ;
}




static void RasterState_Apply( const RasterStateS & rasterState )
{
    {
        D3DFILLMODE  fillMode = D3DFILL_SOLID         ;
        switch( rasterState.mFillMode )
        {
        case RasterState::FILL_MODE_POINT    : fillMode = D3DFILL_POINT      ; break ;
        case RasterState::FILL_MODE_WIREFRAME: fillMode = D3DFILL_WIREFRAME  ; break ;
        case RasterState::FILL_MODE_SOLID    : fillMode = D3DFILL_SOLID      ; break ;
        default: FAIL() ; break ;
        }
        HROK( g_pd3dDevice->SetRenderState( D3DRS_FILLMODE , fillMode ) ) ;
    }

    {
        D3DCULL cull = D3DCULL_CW ;
        switch( rasterState.mCullMode )
        {
        case RasterState::CULL_MODE_NONE : cull = D3DCULL_NONE ; break ;
        case RasterState::CULL_MODE_FRONT: cull = D3DCULL_CCW  ; break ; // Cull CCW faces. Assumes front faces are wound CCW.
        case RasterState::CULL_MODE_BACK : cull = D3DCULL_CW   ; break ; // Cull CW  faces. Assumes back  faces are wound CW.
        }
        HROK( g_pd3dDevice->SetRenderState( D3DRS_CULLMODE , cull ) ) ;
    }
}




static void ShadeMode_Apply( const ShadeModeE & shadeMode )
{

    D3DSHADEMODE shadeModel = D3DSHADE_GOURAUD ;
    switch( shadeMode )
    {
    case SHADE_MODE_FLAT  : shadeModel = D3DSHADE_FLAT    ; break ;
    case SHADE_MODE_SMOOTH: shadeModel = D3DSHADE_GOURAUD ; break ;
    default: FAIL() ; break ;
    }
    HROK( g_pd3dDevice->SetRenderState( D3DRS_SHADEMODE , D3DSHADE_GOURAUD ) ) ;
}




static void MaterialProperties_Apply( const MaterialPropertiesS & materialProperties )
{
    // Enable per-vertex color.
    // NOTE: Should only apply when providing per-vertex colors.
    HROK( g_pd3dDevice->SetRenderState( D3DRS_COLORVERTEX , TRUE ) ) ;
    HROK( g_pd3dDevice->SetRenderState( D3DRS_DIFFUSEMATERIALSOURCE , D3DMCS_COLOR1 ) ) ;
    HROK( g_pd3dDevice->SetRenderState( D3DRS_AMBIENTMATERIALSOURCE , D3DMCS_COLOR1 ) ) ;
    //g_pd3dDevice->SetRenderState( D3DRS_SPECULARMATERIALSOURCE , D3DMCS_COLOR1 ) ;

    // Set material colors.  These apply in the absence of per-vertex colors.
    D3DMATERIAL9 material ;
    ZeroMemory( & material , sizeof( D3DMATERIAL9 ) ) ;
#ifdef D3DXCOLOR // (old) D3D Extension library used with DXSDK
    material.Diffuse = D3DXCOLOR( materialProperties.mDiffuseColor.x , materialProperties.mDiffuseColor.y , materialProperties.mDiffuseColor.z , materialProperties.mDiffuseColor.w ) ;
    material.Ambient = D3DXCOLOR( materialProperties.mAmbientColor.x , materialProperties.mAmbientColor.y , materialProperties.mAmbientColor.z , materialProperties.mAmbientColor.w ) ;
    material.Specular = D3DXCOLOR( materialProperties.mSpecularColor.x , materialProperties.mSpecularColor.y , materialProperties.mSpecularColor.z , materialProperties.mSpecularColor.w ) ;
    material.Emissive = D3DXCOLOR( materialProperties.mEmissiveColor.x , materialProperties.mEmissiveColor.y , materialProperties.mEmissiveColor.z , materialProperties.mEmissiveColor.w ) ;
#else // WinSDK
    material.Diffuse  = reinterpret_cast< const D3DCOLORVALUE & >( materialProperties.mDiffuseColor  ) ;
    material.Ambient  = reinterpret_cast< const D3DCOLORVALUE & >( materialProperties.mAmbientColor  ) ;
    material.Specular = reinterpret_cast< const D3DCOLORVALUE & >( materialProperties.mSpecularColor ) ;
    material.Emissive = reinterpret_cast< const D3DCOLORVALUE & >( materialProperties.mEmissiveColor ) ;
#endif
    material.Power = materialProperties.mSpecularPower ;
    g_pd3dDevice->SetMaterial( & material );    // set the globably-used material to material
}

// Public functions ------------------------------------------------------------


/** Construct wrapper for routines specific to D3D9 render system API.
*/
D3D9_RenderStateCache::D3D9_RenderStateCache()
{
}




/** Destruct wrapper for routines specific to D3D9 render system API.
*/
D3D9_RenderStateCache::~D3D9_RenderStateCache()
{
}




/** Apply render state.
*/
void D3D9_RenderStateCache::Apply( const RenderStateS & renderState )
{
    RENDER_CHECK_ERROR( D3D9_RenderStateCache_Apply_before ) ;

    // TODO: FIXME: If render state has already been set, do not set it again.
    // TODO: FIXME: In debug mode, verify the underlying state matches the material.

    //if( ! memcmp( & mCurrentState.mBlendState , & renderState.mBlendState , sizeof( mCurrentState.mBlendState ) ) )
    {
        BlendState_Apply( renderState.mBlendState ) ;
    }

    //if( ! memcmp( & mCurrentState.mDepthState , & renderState.mDepthState , sizeof( mCurrentState.mDepthState ) ) )
    {
        DepthState_Apply( renderState.mDepthState ) ;
    }

    //if( ! memcmp( & mCurrentState.mAlphaState , & renderState.mAlphaState , sizeof( mCurrentState.mAlphaState ) ) )
    {
        AlphaState_Apply( renderState.mAlphaState ) ;
    }

    //if( ! memcmp( & mCurrentState.mRasterState , & renderState.mRasterState , sizeof( mCurrentState.mRasterState ) ) )
    {
        RasterState_Apply( renderState.mRasterState ) ;
    }

    if( ! memcmp( & mCurrentState.mShadeMode , & renderState.mShadeMode , sizeof( mCurrentState.mShadeMode ) ) )
    {
        ShadeMode_Apply( renderState.mShadeMode ) ;
    }

    if( ! memcmp( & mCurrentState.mMaterialProperties , & renderState.mMaterialProperties , sizeof( mCurrentState.mMaterialProperties ) ) )
    {
        MaterialProperties_Apply( renderState.mMaterialProperties ) ;
    }

    mCurrentState = renderState ;

    RENDER_CHECK_ERROR( D3D9_RenderStateCache_Apply_after ) ;
}




/** Apply render state.
*/
void D3D9_RenderStateCache::GetRenderState( RenderStateS & /*renderState*/ )
{
    RENDER_CHECK_ERROR( D3D9_RenderStateCache_Get_before ) ;

    //BlendState_Get( renderState.mBlendState ) ;
    //DepthState_Get( renderState.mDepthState ) ;
    //AlphaState_Get( renderState.mAlphaState ) ;
    //RasterState_Get( renderState.mRasterState ) ;
    //ShadeMode_Get( renderState.mShadeMode ) ;
    //MaterialProperties_Get( renderState.mMaterialProperties ) ;

    RENDER_CHECK_ERROR( D3D9_RenderStateCache_Get_after ) ;
}




    } ;
} ;




#if defined( _DEBUG )

void D3D9_RenderState_UnitTest()
{
    DebugPrintf( "D3D9_RenderStateCache::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::D3D9_RenderStateCache renderState ;
    }

    DebugPrintf( "D3D9_RenderStateCache::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif

