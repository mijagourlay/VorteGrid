/** \file OpenGL_texture.cpp

    \brief Texture for OpenGL

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Platform/OpenGL/OpenGL_Texture.h"

#include "Render/Resource/textureSampler.h"

#include "Render/Platform/OpenGL/OpenGL_api.h" // For GL_CHECK_ERROR

#include <Image/image.h>

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#if defined( WIN32 )
#   include <windows.h>
#endif

#include <GL/gl.h>  // On Windows machines, this requires #include windows.h
#include <GL/glu.h>
#include "glExt.h"

#include <limits.h> // for INT_MAX

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        static const GLuint INVALID_TEXTURE_NAME = 0 ;

        static GLenum SamplerState_FilterEnum( SamplerState::FilterE filter , SamplerState::FilterE mipFilter )
        {   // TODO: FIXME: This belongs in TextureSampler.
            PERF_BLOCK( Render__SamplerState_FilterEnum ) ;

            if( SamplerState::FILTER_NO_MIPMAP == mipFilter )
            {
                switch( filter )
                {
                case SamplerState::FILTER_NEAREST   : return GL_NEAREST ; break ;
                case SamplerState::FILTER_LINEAR    : return GL_LINEAR  ; break ;
                default: FAIL() ; break ;
                }
            }
            else if( SamplerState::FILTER_NEAREST == mipFilter )
            {
                switch( filter )
                {
                case SamplerState::FILTER_NEAREST   : return GL_NEAREST_MIPMAP_NEAREST  ; break ;
                case SamplerState::FILTER_LINEAR    : return GL_NEAREST_MIPMAP_LINEAR   ; break ;
                default: FAIL() ; break ;
                }
            }
            else if( SamplerState::FILTER_LINEAR == mipFilter )
            {
                switch( filter )
                {
                case SamplerState::FILTER_NEAREST   : return GL_LINEAR_MIPMAP_NEAREST  ; break ;
                case SamplerState::FILTER_LINEAR    : return GL_LINEAR_MIPMAP_LINEAR   ; break ;
                default: FAIL() ; break ;
                }
            }
            return GL_NEAREST ;
        }




        static GLenum SamplerState_AddressEnum( SamplerState::AddressE address )
        {   // TODO: FIXME: This belongs in TextureSampler.
            PERF_BLOCK( Render__SamplerState_AddressEnum ) ;

            switch( address )
            {
            case SamplerState::ADDRESS_CLAMP    : return GL_CLAMP   ; break ;
            case SamplerState::ADDRESS_REPEAT   : return GL_REPEAT  ; break ;
            default: FAIL() ; break ;
            }
            return GL_CLAMP ;
        }




        static void SamplerState_Apply( const SamplerStateS & samplerState )
        {   // TODO: FIXME: This belongs in TextureSampler.
            PERF_BLOCK( Render__SamplerState_Apply ) ;

            RENDER_CHECK_ERROR( SamplerState_Apply_before ) ;
            {
                GLenum minFilter = SamplerState_FilterEnum( samplerState.mMinFilter , samplerState.mMipFilter ) ;
                glTexParameteri( GL_TEXTURE_2D , GL_TEXTURE_MIN_FILTER , minFilter ) ;
            }
            RENDER_CHECK_ERROR( SamplerState_Apply_min ) ;
            {
                GLenum magFilter = SamplerState_FilterEnum( samplerState.mMagFilter , SamplerState::FILTER_NO_MIPMAP ) ;
                glTexParameteri( GL_TEXTURE_2D , GL_TEXTURE_MAG_FILTER , magFilter ) ;
            }
            RENDER_CHECK_ERROR( SamplerState_Apply_mag ) ;
            {
                GLenum addressU = SamplerState_AddressEnum( samplerState.mAddressU ) ;
                glTexParameteri( GL_TEXTURE_2D  , GL_TEXTURE_WRAP_S , addressU ) ;
            }
            RENDER_CHECK_ERROR( SamplerState_Apply_addrU ) ;
            {
                GLenum addressV = SamplerState_AddressEnum( samplerState.mAddressV ) ;
                glTexParameteri( GL_TEXTURE_2D  , GL_TEXTURE_WRAP_T , addressV ) ;
            }
            RENDER_CHECK_ERROR( SamplerState_Apply_addrV ) ;
            {
                GLenum comboOp = GL_MODULATE ;
                switch( samplerState.mCombineOperation )
                {
                case SamplerState::COMBO_OP_REPLACE : comboOp = GL_REPLACE  ; break ;
                case SamplerState::COMBO_OP_MODULATE: comboOp = GL_MODULATE ; break ;
                    //case SamplerState::COMBO_OP_DECAL   : comboOp = GL_DECAL    ; break ;
                }
                glTexEnvi( GL_TEXTURE_ENV , GL_TEXTURE_ENV_MODE , comboOp ) ;
            }
            RENDER_CHECK_ERROR( SamplerState_Apply_op ) ;
        }




        /** Construct texture for OpenGL.
        */
        OpenGL_Texture::OpenGL_Texture()
            : TextureBase()
            , mTextureName( INVALID_TEXTURE_NAME )
        {
            PERF_BLOCK( OpenGL_Texture__OpenGL_Texture ) ;
        }




        /** Destruct texture for OpenGL.
        */
        OpenGL_Texture::~OpenGL_Texture()
        {
            PERF_BLOCK( OpenGL_Texture__dtor ) ;
        }




        /* virtual */ void OpenGL_Texture::Bind( ApiBase * /*renderApi*/ , const SamplerStateS & samplerState )
        {
            PERF_BLOCK( OpenGL_Texture__Bind ) ;

            // Enable texturing.
            glEnable( GL_TEXTURE_2D ) ;
            //glDisable( GL_TEXTURE_2D ) ; // When rendering points, disable textures
            glBindTexture( GL_TEXTURE_2D , mTextureName ) ;

            SamplerState_Apply( samplerState ) ;

            RENDER_CHECK_ERROR( OpenGL_Texture__Bind ) ;
        }




        void OpenGL_Texture::Create2DTextureFromImage( const Image * image )
        {
            PERF_BLOCK( OpenGL_Texture__Create2DTextureFromImage ) ;

            ASSERT( image != NULL ) ;

            // Make sure image is sane.
            ASSERT( image->GetWidth() > 0 ) ;
            ASSERT( image->GetHeight() > 0 ) ;
            ASSERT( image->GetNumChannels() > 0 ) ;
            ASSERT( image->GetNumPages() > 0 ) ;
            ASSERT( image->GetImageData() != 0 ) ;

            ASSERT( image->GetNumChannels() == 4 ) ; // For now, only support RGBA images

            ASSERT( ( GetWidth()  == static_cast< int >( image->GetWidth()  ) ) || ( GetWidth()  == 0 ) ) ;
            ASSERT( ( GetHeight() == static_cast< int >( image->GetHeight() ) ) || ( GetHeight() == 0 ) ) ;
            ASSERT( ( GetNumPlanes() == 1 ) || ( GetNumPlanes() == 0 ) ) ;
            ASSERT( ( GetShape() == TEX_SHAPE_2D ) || ( GetShape() == TEX_SHAPE_UNDEFINED ) ) ;
            ASSERT( ( GetShape() == TEX_SHAPE_2D ) || ( GetShape() == TEX_SHAPE_UNDEFINED ) ) ;

            ASSERT( ( GetFormat() == TEX_FORMAT_A8R8G8B8 ) || ( GetFormat() == TEX_FORMAT_UNDEFINED ) ) ; // For now, only support RGBA textures
            ASSERT( GetUsageFlags() == TEX_USAGE_DEFAULT ) ; // For now, only support default usage.

            // Make sure this material does not already have a texture,
            // otherwise the existing texture will "leak" video memory.
            ASSERT( INVALID_TEXTURE_NAME == mTextureName ) ;

            //mNumTexPages = image->GetNumPages() ;

            glDisable( GL_TEXTURE_CUBE_MAP ) ;
            glEnable( GL_TEXTURE_2D ) ;
            glGenTextures( 1 , & mTextureName ) ;   // Generate a unique "name" for this texture.
            ASSERT( mTextureName != INVALID_TEXTURE_NAME ) ;

            glBindTexture( GL_TEXTURE_2D , mTextureName ) ;
            glPixelStorei( GL_UNPACK_ALIGNMENT , 1 ) ;  // texel data rows not padded

            // Generate MIP maps for all texture pages.
#if 0
            glGenerateMipmapEXT( GL_TEXTURE_2D ) ;
#elif 1
            const int gluError = gluBuild2DMipmaps( GL_TEXTURE_2D
                , /* internal format */ GL_RGBA
                , image->GetWidth()
                , image->GetHeight() * image->GetNumPages() // In lieu of actual texture arrays, assume pages are layed out vertically.
                , GL_RGBA
                , GL_UNSIGNED_BYTE
                , & (*image)[ 0 ] ) ;
            ASSERT( 0 == gluError ) ; NON_DEBUG_ONLY( UNUSED_PARAM( gluError ) ) ;
#endif

            {
                int width    = image->GetWidth() ;
                int height   = image->GetHeight() ;
                int mipLevel = 0 ;
                do
                {
                    glGetTexLevelParameteriv( GL_TEXTURE_2D , mipLevel , GL_TEXTURE_WIDTH  , & width ) ;
                    glGetTexLevelParameteriv( GL_TEXTURE_2D , mipLevel , GL_TEXTURE_HEIGHT , & height ) ;
                    ++ mipLevel ;
                } while( width * height > 1 ) ;
                const int numMipLevels = mipLevel ;
                SetNumMipLevels( numMipLevels ) ;
            }


            RENDER_CHECK_ERROR( OpenGL_Texture__Create2DTextureFromImage ) ;
        }





        /* virtual */ void OpenGL_Texture::CreateFromImages( const Image * images , size_t numImages )
        {
            PERF_BLOCK( OpenGL_Texture__CreateFromImages ) ;

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

            RENDER_CHECK_ERROR( OpenGL_Texture__CreateFromImages ) ;
        }





        /* virtual */ void OpenGL_Texture::CopyToImage( Image & /*image*/ )
        {
            PERF_BLOCK( OpenGL_Texture__CopyToImage ) ;

            RENDER_CHECK_ERROR( OpenGL_Texture__CopyToImage ) ;
        }




    } ;
} ;




#if defined( _DEBUG )

void PeGaSys_Render_OpenGL_Texture_UnitTest()
{
    DebugPrintf( "OpenGL_Texture::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::OpenGL_Texture openGL_texture ;
    }

    DebugPrintf( "OpenGL_Texture::UnitTest: THE END ----------------------------------------------\n" ) ;
}

#endif