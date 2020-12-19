/** \file renderState.h

    \brief Render state

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_RENDER_STATE_H
#define PEGASYS_RENDER_RENDER_STATE_H

#include "Core/Math/vec4.h"
#include "Core/Math/mat4.h"

#include "Core/Containers/intrusivePtr.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /**
        */
        struct BlendStateS
        {
            enum BlendFactorE
            {
                BLEND_FACTOR_ZERO           ,
                BLEND_FACTOR_ONE            ,
                BLEND_FACTOR_SRC_ALPHA      ,
                BLEND_FACTOR_SRC_COLOR      ,
                BLEND_FACTOR_INV_SRC_ALPHA  ,
                BLEND_FACTOR_INV_SRC_COLOR  ,
                BLEND_FACTOR_DST_ALPHA      ,
                BLEND_FACTOR_DST_COLOR      ,
                BLEND_FACTOR_INV_DST_ALPHA  ,
                BLEND_FACTOR_INV_DST_COLOR  ,
            } ;

            enum BlendOpE
            {
                BLEND_OP_ADD        ,
                BLEND_OP_SUBTRACT   ,
                BLEND_OP_MIN        ,
                BLEND_OP_MAX        ,
            } ;

            bool            mBlendEnabled   ;   ///< Whether to perform alpha blending.  Default is false.
            BlendFactorE    mBlendSrcColor  ;   ///< Blend factor for source term.  Default is one.
            BlendFactorE    mBlendDstColor  ;   ///< Blend factor for destination term.  Default is zero.
            BlendOpE        mBlendOpColor   ;   ///< Blend operation to perform to combine src and dst terms.  Default is add.
            BlendFactorE    mBlendSrcAlpha  ;   ///< Blend factor for source term.  Default is one.
            BlendFactorE    mBlendDstAlpha  ;   ///< Blend factor for destination term.  Default is zero
            BlendOpE        mBlendOpAlpha   ;   ///< Blend operation to perform to combine src and dst terms.  Default is add.
        } ;

        class BlendState : public BlendStateS
        {
        public:
            BlendState()
            {
                mBlendEnabled  = false              ;
                mBlendSrcColor = BLEND_FACTOR_ONE   ;
                mBlendDstColor = BLEND_FACTOR_ZERO  ;
                mBlendOpColor  = BLEND_OP_ADD       ;
                mBlendSrcAlpha = BLEND_FACTOR_ONE   ;
                mBlendDstAlpha = BLEND_FACTOR_ZERO  ;
                mBlendOpAlpha  = BLEND_OP_ADD       ;
            }

            operator BlendStateS & ()
            {
                return * this ;
            }

            void SetOpaque()
            {
                mBlendEnabled  = false              ;
                mBlendSrcColor = BLEND_FACTOR_ONE   ;
                mBlendDstColor = BLEND_FACTOR_ZERO  ;
                mBlendOpColor  = BLEND_OP_ADD       ;
                mBlendSrcAlpha = BLEND_FACTOR_ONE   ;
                mBlendDstAlpha = BLEND_FACTOR_ZERO  ;
                mBlendOpAlpha  = BLEND_OP_ADD       ;
            }

            void SetAlpha()
            {
                mBlendEnabled  = true                       ;
                mBlendSrcColor = BLEND_FACTOR_SRC_ALPHA     ;
                mBlendDstColor = BLEND_FACTOR_INV_SRC_ALPHA ;
                mBlendOpColor  = BLEND_OP_ADD               ;
                mBlendSrcAlpha = BLEND_FACTOR_SRC_ALPHA     ;
                mBlendDstAlpha = BLEND_FACTOR_INV_SRC_ALPHA ;
                mBlendOpAlpha  = BLEND_OP_ADD               ;
            }

            void SetAdditive()
            {
                mBlendEnabled  = true                   ;
                mBlendSrcColor = BLEND_FACTOR_SRC_ALPHA ;
                mBlendDstColor = BLEND_FACTOR_ONE       ;
                mBlendOpColor  = BLEND_OP_ADD           ;
                mBlendSrcAlpha = BLEND_FACTOR_SRC_ALPHA ;
                mBlendDstAlpha = BLEND_FACTOR_ONE       ;
                mBlendOpAlpha  = BLEND_OP_ADD           ;
            }

        } ;

        enum CompareFuncE
        {
            CMP_FUNC_NEVER          ,   ///< Never pass the comparision.
            CMP_FUNC_LESS           ,   ///< If current/source is less than new/destination, comparison passes.
            CMP_FUNC_GREATER_EQUAL  ,   ///< If current/source is greater than or equal to than new/destination, comparison passes.
            CMP_FUNC_ALWAYS         ,   ///< Always pass the comparision.
        } ;

        struct DepthStateS
        {
            CompareFuncE    mDepthFunc          ;   ///< Whether to enable depth test, and if so, which test to use.  Default is LESS.
            bool            mDepthWriteEnabled  ;   ///< Whether to write to the depth buffer. Default is true.
            int             mDepthBias          ;   ///< Depth value added to each pixel. Default is 0.
            bool            mDepthClip          ;   ///< Whether to enable depth-clipping. Default is true.
        } ;

        class DepthState : public DepthStateS
        {
        public:
            DepthState()
            {
                mDepthFunc          = CMP_FUNC_LESS ;
                mDepthWriteEnabled  = true ;
                mDepthBias          = 0 ;
                mDepthClip          = true ;
            }
        } ;

        struct StencilStateS
        {
            bool    mStencilEnabled ;   ///< Whether to perform stencil testing. Default is false.
            // TODO: FIXME: add other stencil state parameters.
        } ;

        class StencilState : public StencilStateS
        {
        public:
            StencilState()
            {
                mStencilEnabled = false ;
            }
        } ;

        struct AlphaStateS
        {
            bool            mAlphaTest  ;   ///< Whether to enable alpha-test.  Default is false.
            CompareFuncE    mAlphaFunc  ;   ///< Test to use to accept or reject pixel, based on its alpha value.  Default is GREATEREQUAL.
            float           mAlphaRef   ;   ///< Value to use for the current/source alpha during alpha-test comparisons.  Default is 0.
        } ;

        class AlphaState : public AlphaStateS
        {
        public:
            AlphaState()
            {
                mAlphaTest  = false ;
                mAlphaFunc  = CMP_FUNC_GREATER_EQUAL ;
                mAlphaRef   = 0.0f ;
            }
        } ;

        struct RasterStateS
        {
            enum FillModeE
            {
                FILL_MODE_POINT     ,   ///< Draw vertices as points
                FILL_MODE_WIREFRAME ,   ///< Draw triangle edges
                FILL_MODE_SOLID     ,   ///< Draw triangle faces.
            } ;

            enum CullModeE
            {
                CULL_MODE_NONE  ,   ///< Draw all triangles. Disable culling.
                CULL_MODE_FRONT ,   ///< Do not draw front-facing (counter-clockwise) triangles
                CULL_MODE_BACK  ,   ///< Do not draw back-facing (clockwise) triangles.
            } ;

            FillModeE       mFillMode           ;   ///< How to draw primitives. Default is FILL_MODE_SOLID.
            CullModeE       mCullMode           ;   ///< Which faces, if any, to cull.  Default is CULL_MODE_BACK.
            bool            mMultiSampleEnabled ;   ///< Whether to enable multi-sample anti-aliasing.  Default is false.
            bool            mSmoothLines        ;   ///< Whether to use smooth-line rendering.  Default is false.
        } ;

        class RasterState : public RasterStateS
        {
        public:
            RasterState()
            {
                mFillMode           = FILL_MODE_SOLID ;
                mCullMode           = CULL_MODE_BACK ;
                mMultiSampleEnabled = false ;
                mSmoothLines        = false ;
            }
        } ;


        enum ShadeModeE
        {
            SHADE_MODE_FLAT     ,
            SHADE_MODE_SMOOTH   ,
        } ;


        struct ShaderS
        {
            void * mDummy ;
            // TODO: FIXME: Add shader info.  Maybe this will end up being abstract.
        } ;


        class Shader : public ShaderS
        {
        public:
            Shader()
            {
                mDummy = 0 ;
            }
        } ;

        struct MaterialPropertiesS
        {
            Vec4    mDiffuseColor   ;
            Vec4    mAmbientColor   ;
            Vec4    mSpecularColor  ;
            Vec4    mEmissiveColor  ;
            float   mSpecularPower  ;
        } ;

        class MaterialProperties : public MaterialPropertiesS
        {
        public:
            MaterialProperties()
            {
                mDiffuseColor  = Vec4( 0.8f , 0.8f , 0.8f , 1.0f ) ;
                mAmbientColor  = Vec4( 0.2f , 0.2f , 0.2f , 1.0f ) ;
                mSpecularColor = Vec4( 0.0f , 0.0f , 0.0f , 1.0f ) ;
                mEmissiveColor = Vec4( 0.0f , 0.0f , 0.0f , 1.0f ) ;
                mSpecularPower = 0.0f ;
            }
        } ;


        struct TransformsS
        {
            Mat44   mViewMatrix ;
        } ;

        class Transforms : public TransformsS
        {
        public:
            Transforms()
            {
                mViewMatrix = Mat4_xIdentity ;
            }
        } ;

        /** Render state.
        */
        struct RenderStateS
        {
            BlendState          mBlendState         ;
            DepthState          mDepthState         ;
            StencilState        mStencilState       ;
            AlphaState          mAlphaState         ;
            RasterState         mRasterState        ;
            ShadeModeE          mShadeMode          ;
            Shader              mShader             ;
            MaterialProperties  mMaterialProperties ;
            Transforms          mTransforms         ;
        } ;


        class RenderState : public RenderStateS , public RefCountedMixin
        {
        public:
            RenderState() ;
            virtual ~RenderState() {}

            void ReleaseReference() ; // Used by IntrusivePtr (indirectly, through global ReleaseReference)

            #if defined( _DEBUG )
                static void UnitTest() ;
            #endif
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

        /// Add reference to a RenderState, for use by IntrusivePtr.
        inline void AddReference( RenderState * renderState )     { renderState->AddReference() ; }

        /// Release reference to a RenderState, for use by IntrusivePtr.
        inline void ReleaseReference( RenderState * renderState ) { renderState->ReleaseReference() ; }

    } ;
} ;

#endif
