/** \file D3D9_texture.h

    \brief Texture for Direct3D version 9

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_TEXTURE_H
#define PEGASYS_RENDER_OPENGL_TEXTURE_H

#include "Render/Resource/texture.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Texture for Direct3D version 9.
        */
        class D3D9_Texture : public TextureBase
        {
            public:
                D3D9_Texture() ;
                virtual ~D3D9_Texture() ;

                virtual void Bind( ApiBase * renderApi , const SamplerStateS & samplerState ) ;
                virtual void CreateFromImages( const Image * images , size_t numImages ) ;
                virtual void CopyToImage( Image & image ) ;

            private:
                void Create2DTextureFromImage( const Image * image ) ;

                IDirect3DTexture9 * mTexture ;
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
