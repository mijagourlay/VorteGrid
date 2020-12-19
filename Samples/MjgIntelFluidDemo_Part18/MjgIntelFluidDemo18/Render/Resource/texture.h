/** \file texture.h

    \brief Texture base class.

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_TEXTURE_H
#define PEGASYS_RENDER_TEXTURE_H

#include "Core/Containers/intrusivePtr.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    // Forward declaration.
    class Image ;

    namespace Render
    {
        // Forward declaration
        class ApiBase ;
        struct SamplerStateS ;

        /** Texture base class.
        */
        class TextureBase : public RefCountedMixin
        {
            public:
                /** Format of texture data.
                */
                enum FormatE
                {
                    TEX_FORMAT_R8G8B8   ,
                    TEX_FORMAT_A8R8G8B8 ,
                    TEX_FORMAT_D16      ,
                    TEX_FORMAT_D32      ,
                    TEX_FORMAT_D24S8    ,
                    TEX_FORMAT_UNDEFINED    ///< Texture format not assigned yet.
                } ;


                /** Texture memory usage flags. Combine with bitwise-OR.
                */
                enum UsageFlagsE
                {
                    TEX_USAGE_DEFAULT       = 0x0 ,
                    TEX_USAGE_DYNAMIC       = 0x1 , ///< Texture is writable.  Default is read-only.
                    TEX_USAGE_RENDER_TARGET = 0x2 , ///< Texture will be used as a render target.
                } ;


                /** Topological configuration of texture.
                */
                enum ShapeE
                {
                    TEX_SHAPE_1D        ,   ///< Line texture.
                    TEX_SHAPE_2D        ,   ///< Planar texture.
                    TEX_SHAPE_ARRAY     ,   ///< Array of planar textures.
                    TEX_SHAPE_3D        ,   ///< Volume texture.
                    TEX_SHAPE_CUBEMAP   ,   ///< 6 square planar faces.
                    TEX_SHAPE_UNDEFINED ,   ///< Shape not assigned yet.
                } ;

                TextureBase() ;
                TextureBase( int width , int height , int numPlanes , int numMipLevels , FormatE format , UsageFlagsE usageFlags , ShapeE shape ) ;

                virtual ~TextureBase() ;

            #if defined( _DEBUG )
                static void UnitTest() ;
            #endif

                void ReleaseReference() ; // Used by IntrusivePtr (indirectly, through global ReleaseReference)

                void SetWidth( int width ) ;
                void SetHeight( int height ) ;
                void SetNumPlanes( int numPlanes ) ;
                void SetNumMipLevels( int numMipLevels ) ;
                void SetFormat( FormatE format) ;
                void SetUsageFlags( UsageFlagsE usageFlags ) ;
                void SetShape( ShapeE shape ) ;

                const int &         GetWidth() const        { return mWidth ; }
                const int &         GetHeight() const       { return mHeight ; }
                const int &         GetNumPlanes() const    { return mNumPlanes ; }
                const int &         GetNumMipLevels() const { return mNumMipLevels ; }
                const FormatE &     GetFormat() const       { return mFormat ; }
                const UsageFlagsE & GetUsageFlags() const   { return mUsageFlags ; }
                const ShapeE &      GetShape() const        { return mShape ; }

                virtual void Bind( ApiBase * renderApi , const SamplerStateS & samplerState ) = 0 ;
                virtual void CreateFromImages( const Image * images , size_t numImages ) = 0 ;
                virtual void CopyToImage( Image & image ) = 0 ;

            protected:
                int         mWidth          ;   ///< Texture width, in pixels.
                int         mHeight         ;   ///< Texture height, in pixels.
                int         mNumPlanes      ;   ///< Number of planes for a texture array or volume texture.
                int         mNumMipLevels   ;   ///< Number of MIPmap levels.
                FormatE     mFormat         ;   ///< Format of texture data.
                UsageFlagsE mUsageFlags     ;   ///< Memory usage of texture data.
                ShapeE      mShape          ;   ///< Topological configuration of texture.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

        /// Add reference to a MeshBase, for use by IntrusivePtr.
        inline void AddReference( TextureBase * textureBase )     { textureBase->AddReference() ; }

        /// Release reference to a MeshBase, for use by IntrusivePtr.
        inline void ReleaseReference( TextureBase * textureBase ) { textureBase->ReleaseReference() ; }

    } ;
} ;

#endif
