/** \file OpenGL_Texture.h

    \brief Texture for OpenGL

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_TEXTURE_H
#define PEGASYS_RENDER_OPENGL_TEXTURE_H

#include "Render/Resource/texture.h"

#if defined( WIN32 )
#   include <windows.h>
#endif

#include <GL/gl.h>  // On Windows machines, this requires #include windows.h

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Texture for OpenGL.
        */
        class OpenGL_Texture : public TextureBase
        {
            public:
                OpenGL_Texture() ;
                virtual ~OpenGL_Texture() ;

                virtual void Bind( ApiBase * renderApi , const SamplerStateS & samplerState ) ;
                virtual void CreateFromImages( const Image * images , size_t numImages ) ;
                virtual void CopyToImage( Image & image ) ;

            private:
                void Create2DTextureFromImage( const Image * image ) ;

                GLuint mTextureName    ;   ///< OpenGL texture identifier
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
