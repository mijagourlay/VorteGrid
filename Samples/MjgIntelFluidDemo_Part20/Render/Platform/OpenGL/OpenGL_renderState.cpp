/** \file OpenGL_renderState.cpp

    \brief Wrapper for routines specific to OpenGL render system API

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Platform/OpenGL/OpenGL_renderState.h"

#include "Render/Platform/OpenGL/OpenGL_api.h" // for RENDER_CHECK_ERROR

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#if defined( WIN32 )
#   include <windows.h>
#endif

#include <GL/glut.h>

// Macros -----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        // Private variables -----------------------------------------------------------
        // Public variables ------------------------------------------------------------
        // Private functions -----------------------------------------------------------

        static void BlendState_Apply( const BlendStateS & blendState )
        {
            PERF_BLOCK( Render__BlendState_Apply ) ;

            if( blendState.mBlendEnabled )
            {
                glEnable( GL_BLEND ) ;
            }
            else
            {
                glDisable( GL_BLEND ) ;
            }

            GLenum srcFactor = GL_ONE ;

            switch( blendState.mBlendSrcAlpha )
            {
            case BlendState::BLEND_FACTOR_ZERO          : srcFactor = GL_ZERO                ; break ;
            case BlendState::BLEND_FACTOR_ONE           : srcFactor = GL_ONE                 ; break ; // Default / additive
            case BlendState::BLEND_FACTOR_SRC_ALPHA     : srcFactor = GL_SRC_ALPHA           ; break ; // Typical alpha blending
            case BlendState::BLEND_FACTOR_INV_SRC_ALPHA : srcFactor = GL_ONE_MINUS_SRC_ALPHA ; break ;
                // TODO: FIXME: Support other blend factors
            default: FAIL() ; break ;
            }

            GLenum dstFactor = GL_ONE ;

            switch( blendState.mBlendDstAlpha )
            {
            case BlendState::BLEND_FACTOR_ZERO          : dstFactor = GL_ZERO                ; break ;
            case BlendState::BLEND_FACTOR_ONE           : dstFactor = GL_ONE                 ; break ; // Default / additive
            case BlendState::BLEND_FACTOR_SRC_ALPHA     : dstFactor = GL_SRC_ALPHA           ; break ;
            case BlendState::BLEND_FACTOR_INV_SRC_ALPHA : dstFactor = GL_ONE_MINUS_SRC_ALPHA ; break ; // Typical alpha blending
                // TODO: FIXME: Support other blend factors
            default: FAIL() ; break ;
            }

            glBlendFunc( srcFactor , dstFactor ) ;

            // TODO: FIXME: Support other blend modes.
            ASSERT( blendState.mBlendSrcColor == blendState.mBlendSrcAlpha ) ;
            ASSERT( blendState.mBlendDstColor == blendState.mBlendDstAlpha ) ;
            ASSERT( blendState.mBlendOpColor  == blendState.mBlendOpAlpha ) ;
            ASSERT( BlendState::BLEND_OP_ADD  == blendState.mBlendOpColor ) ;

            RENDER_CHECK_ERROR( BlendState_Apply ) ;
        }




        static GLenum compareFunction( CompareFuncE compareFunc )
        {
            PERF_BLOCK( Render__compareFunction ) ;

            switch( compareFunc )
            {
            case CMP_FUNC_NEVER         : return GL_NEVER  ; break ;
            case CMP_FUNC_LESS          : return GL_LESS   ; break ;
            case CMP_FUNC_GREATER_EQUAL : return GL_GEQUAL ; break ;
            case CMP_FUNC_ALWAYS        : return GL_ALWAYS ; break ;
                // TODO: FIXME: Support other comparison functions. See glDepthFunc, glAlphaFunc.
            default: FAIL() ; break ;
            }
            return GL_NEVER ;
        }




        static void DepthState_Apply( const DepthStateS & depthState )
        {
            PERF_BLOCK( Render__DepthState_Apply ) ;

            switch( depthState.mDepthFunc )
            {
            case CMP_FUNC_NEVER         :
            case CMP_FUNC_LESS          :
            case CMP_FUNC_GREATER_EQUAL :
                // TODO: FIXME: Support other depth modes. See glDepthFunc.
                glEnable( GL_DEPTH_TEST ) ;
                break ;
            case CMP_FUNC_ALWAYS        :
                glDisable( GL_DEPTH_TEST ) ;
                break ;
            default: FAIL() ; break ;
            }

            const GLenum depthFunc = compareFunction( depthState.mDepthFunc ) ;
            glDepthFunc( depthFunc  ) ; 

            glDepthMask( depthState.mDepthWriteEnabled ? GL_TRUE : GL_FALSE ) ;

            // TODO: FIXME: Support other depth mode states.
            ASSERT( 0 == depthState.mDepthBias ) ;
            ASSERT( depthState.mDepthClip ) ;

            RENDER_CHECK_ERROR( DepthState_Apply ) ;
        }




        static void AlphaState_Apply( const AlphaStateS & alphaState )
        {
            PERF_BLOCK( Render__AlphaState_Apply ) ;

            // Set alpha-test state.
            if( alphaState.mAlphaTest )
            {   // Enable testing against alpha channel.
                glEnable( GL_ALPHA_TEST ) ;
            }
            else
            {   // Disable testing against alpha channel.
                glDisable( GL_ALPHA_TEST ) ;
            }

            const GLenum alphaFunc = compareFunction( alphaState.mAlphaFunc ) ;

            glAlphaFunc( alphaFunc , alphaState.mAlphaRef ) ;

            RENDER_CHECK_ERROR( AlphaState_Apply ) ;
        }




        static void RasterState_Apply( const RasterStateS & rasterState )
        {
            PERF_BLOCK( Render__RasterState_Apply ) ;

            {
                GLenum polygonMode = GL_FILL ;
                switch( rasterState.mFillMode )
                {
                case RasterState::FILL_MODE_POINT       : polygonMode = GL_POINT    ; break ;
                case RasterState::FILL_MODE_WIREFRAME   : polygonMode = GL_LINE     ; break ;
                case RasterState::FILL_MODE_SOLID       : polygonMode = GL_FILL     ; break ;
                default: FAIL() ; break ;
                }
                glPolygonMode( GL_FRONT_AND_BACK , polygonMode ) ;
            }

            {
                if( RasterState::CULL_MODE_NONE == rasterState.mCullMode )
                {
                    glDisable( GL_CULL_FACE ) ;
                }
                else
                {
                    glEnable( GL_CULL_FACE ) ;
                    GLenum cullFace = GL_BACK ;
                    switch( rasterState.mCullMode )
                    {
                    case RasterState::CULL_MODE_FRONT: cullFace = GL_FRONT  ; break ;
                    case RasterState::CULL_MODE_BACK : cullFace = GL_BACK   ; break ;
                    default: FAIL() ; break ;
                    }
                    glCullFace( cullFace ) ;
                    glFrontFace( GL_CCW ) ; // PeGaSys defines a front face as being wound CCW.
                }
            }

            glEnable( GL_NORMALIZE )        ;   // Probably should use GL_RESCALE_NORMAL instead but this version of OpenGL does not seem to have that

            RENDER_CHECK_ERROR( RasterState_Apply ) ;
        }




        static void ShadeMode_Apply( const ShadeModeE & shadeMode )
        {
            PERF_BLOCK( Render__ShadeMode_Apply ) ;

            GLenum shadeModel = GL_SMOOTH ;
            switch( shadeMode )
            {
            case SHADE_MODE_FLAT  : shadeModel = GL_FLAT   ; break ;
            case SHADE_MODE_SMOOTH: shadeModel = GL_SMOOTH ; break ;
            default: FAIL() ; break ;
            }
            glShadeModel( shadeModel ) ;

            RENDER_CHECK_ERROR( ShadeMode_Apply ) ;
        }




        static void MaterialProperties_Apply( const MaterialPropertiesS & materialProperties )
        {
            PERF_BLOCK( Render__MaterialProperties_Apply ) ;

            // Enable per-vertex color.
            // NOTE: Should only apply when providing per-vertex colors.
            // NOTE: glColorMaterial (re)sets the material, so any set-material done before this is overwritten.
            // see http://www.opengl.org/archives/resources/features/KilgardTechniques/oglpitfall/
            // Always be careful to set glColorMaterial before you enable GL_COLOR_MATERIAL.
            glColorMaterial( GL_FRONT_AND_BACK , GL_AMBIENT_AND_DIFFUSE ) ;
            glEnable( GL_COLOR_MATERIAL ) ;
            //glDisable( GL_COLOR_MATERIAL ) ;

            // Set material color.
            // If vertices have a color, they will override this.
            // This is only meaningful when the vertex format lacks color.
            glColor4fv( (float*) & materialProperties.mDiffuseColor  ) ;
            glMaterialfv( GL_FRONT , GL_AMBIENT   , (float*) & materialProperties.mAmbientColor  ) ;
            glMaterialfv( GL_FRONT , GL_DIFFUSE   , (float*) & materialProperties.mDiffuseColor  ) ;
            glMaterialfv( GL_FRONT , GL_SPECULAR  , (float*) & materialProperties.mSpecularColor ) ;
            glMateriali ( GL_FRONT , GL_SHININESS , static_cast< int >( materialProperties.mSpecularPower ) ) ;
            glMaterialfv( GL_FRONT , GL_EMISSION  , (float*) & materialProperties.mEmissiveColor ) ;

            RENDER_CHECK_ERROR( MaterialProperties_Apply ) ;
        }




        static void Transforms_Get( const TransformsS & transforms )
        {
            PERF_BLOCK( Render__Transforms_Get ) ;

            Mat44 viewMatrix ;
            glGetFloatv( GL_MODELVIEW_MATRIX , (GLfloat *) & transforms.mViewMatrix ) ;

            RENDER_CHECK_ERROR( Transforms_Get ) ;
        }


        // Public functions ------------------------------------------------------------


        /** Construct wrapper for routines specific to OpenGL render system API.
        */
        OpenGL_RenderStateCache::OpenGL_RenderStateCache()
        {
            PERF_BLOCK( OpenGL_RenderStateCache__OpenGL_RenderStateCache ) ;
        }




        /** Destruct wrapper for routines specific to OpenGL render system API.
        */
        OpenGL_RenderStateCache::~OpenGL_RenderStateCache()
        {
            PERF_BLOCK( OpenGL_RenderStateCache__dtor ) ;
        }




        /** Apply render state.
        */
        void OpenGL_RenderStateCache::Apply( const RenderStateS & renderState )
        {
            PERF_BLOCK( OpenGL_RenderStateCache__Apply ) ;

            RENDER_CHECK_ERROR( OpenGL_RenderStateCache_Apply_before ) ;

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

            //if( ! memcmp( & mCurrentState.mShadeMode , & renderState.mShadeMode , sizeof( mCurrentState.mShadeMode ) ) )
            {
                ShadeMode_Apply( renderState.mShadeMode ) ;
            }

            //if( ! memcmp( & mCurrentState.mMaterialProperties , & renderState.mMaterialProperties , sizeof( mCurrentState.mMaterialProperties ) ) )
            {
                MaterialProperties_Apply( renderState.mMaterialProperties ) ;
            }

            mCurrentState = renderState ;

            RENDER_CHECK_ERROR( OpenGL_RenderStateCache_Apply_after ) ;
        }





        /** Get render state from underlying render system.
        */
        void OpenGL_RenderStateCache::GetRenderState( RenderStateS & renderState )
        {
            PERF_BLOCK( OpenGL_RenderStateCache__GetRenderState ) ;

            RENDER_CHECK_ERROR( OpenGL_RenderStateCache_Get_before ) ;

            //BlendState_Get( renderState.mBlendState ) ;
            //DepthState_Get( renderState.mDepthState ) ;
            //AlphaState_Get( renderState.mAlphaState ) ;
            //RasterState_Get( renderState.mRasterState ) ;
            //ShadeMode_Get( renderState.mShadeMode ) ;
            //MaterialProperties_Get( renderState.mMaterialProperties ) ;
            Transforms_Get( renderState.mTransforms ) ;

            RENDER_CHECK_ERROR( OpenGL_RenderStateCache_Get_after ) ;
        }





    } ;
} ;




#if defined( _DEBUG )

void PeGaSys_Render_OpenGL_RenderState_UnitTest()
{
    DebugPrintf( "OpenGL_RenderStateCache::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::OpenGL_RenderStateCache renderState ;
    }

    DebugPrintf( "OpenGL_RenderStateCache::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif

