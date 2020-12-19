/** \file qdMaterial.h

    \brief Rendering material.

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef QD_MATERIAL_H
#define QD_MATERIAL_H

#include "Core/Math/vec4.h"

#if defined( WIN32 )
    #include <windows.h>    // OpenGL headers on Windows use Windows defines.
#endif
#include <GL/gl.h> // For GLuint

namespace PeGaSys
{
    class Image ; // Forward declaration
} ;


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

        enum CullFaceE
        {
            CF_NONE             ,
            CF_FRONT            ,
            CF_BACK             ,
            CF_FRONT_AND_BACK   ,
            NUM_CULL_FACE_OPTIONS
        } ;

        QdMaterial() ;
        ~QdMaterial() ;

        void                SetDepthWrite( bool depthWrite )        { mDepthWrite = depthWrite ; }
        void                SetBlendMode( BlendModeE blendMode )    { mBlendMode = blendMode ; }
        const BlendModeE &  GetBlendMode() const                    { return mBlendMode ; }

        const Vec4 &        GetColor() const                        { return mColor ; }
        void                SetColor( const Vec4 & color )          { mColor = color ; }

        const CullFaceE &   GetCullFace() const                     { return mCullFace ; }
        void                SetCullFace( CullFaceE cullFace )       { mCullFace = cullFace ; }

        const unsigned &    GetNumTexPages() const                  { return mNumTexPages ; }

        void AssignTexture( const PeGaSys::Image & image ) ;

        void UseMaterial() const ;

    private:
        QdMaterial( const QdMaterial & ) ;              ///< Disallow copy construction.
        QdMaterial & operator=( const QdMaterial & ) ;  ///< Disallow assignment.

        static const int    NUM_TEXTURES_MAX    = 1 ;   ///< Maximum number of textures per material.

        typedef GLuint TextureT ;

        // Render state members.
        bool            mDepthWrite     ;   ///< Whether to write to the depth buffer.
        BlendModeE      mBlendMode      ;   ///< Mode used to blend translucent pixels.
        CullFaceE       mCullFace       ;   ///< Which face to cull (front, back, both or neither).

        // Texture members.
        Vec4            mColor          ;   ///< Material color.  Modulates texture.
        GLuint          mTexName        ;   ///< Texture "name" (identifier)

        unsigned        mNumTexPages    ;   ///< Number of "pages" in a texture.  A page is a partition of a texture, in (u,v) coordinates. This is used in place of a texture array.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

extern void CheckGlError() ;

#endif
