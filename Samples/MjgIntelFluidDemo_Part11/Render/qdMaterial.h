/*! \file qdMaterial.h

    \brief Rendering material.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef QD_MATERIAL_H
#define QD_MATERIAL_H

#if defined( WIN32 )
    #include <windows.h>    // OpenGL headers on Windows use Windows defines.
#endif
#include <GL/gl.h> // For GLuint

#include "textureProc.h"

/** Rendering material base class.
*/
class QdMaterial
{
    public:

        enum BlendModeE
        {
            BM_OPAQUE   ,
            BM_ALPHA    ,
            BM_ADDITIVE ,
            NUM_BLEND_MODES
        } ;

        QdMaterial() ;
        ~QdMaterial() ;

        void                SetDepthWrite( bool depthWrite )        { mDepthWrite = depthWrite ; }
        void                SetBlendMode( BlendModeE blendMode )    { mBlendMode = blendMode ; }
        const BlendModeE &  GetBlendMode() const                    { return mBlendMode ; }

        const Vec4 &        GetColor() const                        { return mColor ; }
        void                SetColor( const Vec4 & color )          { mColor = color ; }
        const unsigned &    GetNumTexPages() const                  { return mNumTexPages ; }

        void AllocateTexture( int texWidth , int texHeight , int numTexPages ) ;
        void AssignTexturePage( ITextureProc & textureProc , int texturePageIndex ) ;
        void FinalizeTexture() ;

        void MakeSimpleNoiseTexture( int texWidth , int texHeight , int numTexPages , float gamma ) ;

        void UseMaterial( void ) const ;

    private:
        QdMaterial( const QdMaterial & re) ;                ///< Disallow copy construction.
        QdMaterial & operator=( const QdMaterial & re ) ;   ///< Disallow assignment.

        static const int        NUM_TEXTURES_MAX    = 1 ;   ///< Maximum number of textures per material.

        typedef GLuint TextureT ;

        // Render state members.
        bool            mDepthWrite     ;   ///< Whether to write to the depth buffer.
        BlendModeE      mBlendMode      ;   ///< Mode used to blend translucent pixels.
        int             mCullFace       ;   ///< Which face to cull (front or back)

        // Texture members.
        Vec4            mColor          ;   ///< Material color.  Modulates texture.
        GLuint          mTexName        ;   ///< Texture "name" (identifier)
        unsigned        mNumTexPages    ;   ///< Number of "pages" in a texture.  A page is a partition of a texture, in (u,v) coordinates.
        QdImage         mTexImage       ;   ///< Texture image (all pages).
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
