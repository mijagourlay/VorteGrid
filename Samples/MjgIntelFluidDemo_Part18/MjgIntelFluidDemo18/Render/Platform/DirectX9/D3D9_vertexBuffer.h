/** \file D3D9_VertexBuffer.h

    \brief Vertex buffer for Direct3D version 9

    \author Copyright 2010-2014 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_D3D9_VERTEX_BUFFER_H
#define PEGASYS_RENDER_D3D9_VERTEX_BUFFER_H

#include <d3d9.h>
#include <d3d9types.h>

#include "Render/Resource/vertexBuffer.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Vertex buffer for Direct3D version 9.
        */
        class D3D9_VertexBuffer : public VertexBufferBase
        {
            public:

                typedef D3DCOLOR                COLOR_UBYTES  ;
                typedef LPDIRECT3DVERTEXBUFFER9 VERTEX_BUFFER_POINTER_TYPE ;

                struct VertexFormatPosition
                {   // Custom vertex format for position
                    float px, py, pz ;  // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_Position = D3DFVF_XYZ ;

                ////////////////////////////////////////////////////////
                // Position plus one other component
                ////////////////////////////////////////////////////////

                struct VertexFormatPositionColor
                {   // Custom vertex format for position+color
                    float px, py, pz ;  // untransformed (world-space) position
                    COLOR_UBYTES color; // color in RGBA form packed into 4 unsigned bytes. TODO: Structure this analogous to OpenGL.
                } ;
                static const unsigned sVertexFormatFlags_PositionColor = D3DFVF_XYZ | D3DFVF_DIFFUSE ;

                struct VertexFormatPositionNormal
                {   // Custom vertex format for position+normal
                    float px, py, pz ;  // untransformed (world-space) position
                    float nx, ny, nz ;  // surface normal unit vector
                } ;
                static const unsigned sVertexFormatFlags_PositionNormal  = D3DFVF_XYZ | D3DFVF_NORMAL ;

                struct VertexFormatPositionTexture
                {   // Custom vertex format for position+texCoord
                    float px, py, pz ;  // untransformed (world-space) position
                    float ts, tt ;      // 2D texture coordinates
                } ;
                // Note that D3DFVF_TEXCOORDSIZE2(0) is default, but we provide it explicitly anyway for clarity of intention.
                static const unsigned sVertexFormatFlags_PositionTexture = D3DFVF_XYZ | D3DFVF_TEX1 | D3DFVF_TEXCOORDSIZE2(0) ;

                ////////////////////////////////////////////////////////
                // Position plus two other components
                ////////////////////////////////////////////////////////

                struct VertexFormatPositionNormalColor
                {   // Custom vertex format for position+color+normal
                    float px, py, pz ;  // untransformed (world-space) position
                    float nx, ny, nz ;  // surface normal unit vector
                    COLOR_UBYTES diffuse ; // Diffuse color in RGBA form packed into 4 unsigned bytes. TODO: Structure this analogous to OpenGL, as an array of 4 unsigned char.
                #define INCLUDE_SPECULAR
                #ifdef INCLUDE_SPECULAR
                    COLOR_UBYTES specular ; // Specular color in RGBA form packed into 4 unsigned bytes. TODO: Structure this analogous to OpenGL, as an array of 4 unsigned char.
                #endif
                } ;
            #ifdef INCLUDE_SPECULAR
                static const unsigned sVertexFormatFlags_PositionNormalColor = D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_DIFFUSE | D3DFVF_SPECULAR ;
            #else
                static const unsigned sVertexFormatFlags_PositionNormalColor = D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_DIFFUSE ;
            #endif

                struct VertexFormatPositionNormalTexture
                {   // Custom vertex format for position+normal
                    float px, py, pz ;  // untransformed (world-space) position
                    float nx, ny, nz ;  // surface normal unit vector
                    float ts, tt     ;  // 2D texture coordinates
                } ;
                // Note that D3DFVF_TEXCOORDSIZE2(0) is default, but we provide it explicitly anyway for clarity of intention.
                static const unsigned sVertexFormatFlags_PositionNormalTexture = D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_TEX1 | D3DFVF_TEXCOORDSIZE2(0) ;

                struct VertexFormatPositionColorTexture
                {   // Custom vertex format for position+color+texture
                    float           px, py, pz  ;   // untransformed (world-space) position
                    COLOR_UBYTES    color_argb  ;   // color in ARGB form packed into 4 unsigned bytes. TODO: Structure this analogous to OpenGL, as an array of 4 unsigned char.
                    float           ts, tt      ;   // 2D texture coordinates
                } ;
                // Note that D3DFVF_TEXCOORDSIZE2(0) is default, but we provide it explicitly anyway for clarity of intention.
                static const unsigned sVertexFormatFlags_PositionColorTexture = D3DFVF_XYZ | D3DFVF_DIFFUSE | D3DFVF_TEX1 | D3DFVF_TEXCOORDSIZE2(0) ;

                ////////////////////////////////////////////////////////
                // Position plus three other components
                ////////////////////////////////////////////////////////

                struct VertexFormatPositionNormalColorTexture
                {   // Custom vertex format for position+normal+color+texture
                    float px, py, pz        ;   // untransformed (world-space) position
                    float nx, ny, nz        ;   // surface normal unit vector
                    COLOR_UBYTES color_argb ;   // color in ARGB form packed into 4 unsigned bytes. TODO: Structure this analogous to OpenGL, as an array of 4 unsigned char.
                    float ts, tt, tu, tv    ;   // 4D texture coordinates
                } ;
                static const unsigned sVertexFormatFlags_PositionNormalColorTexture = D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_DIFFUSE | D3DFVF_TEX1 | D3DFVF_TEXCOORDSIZE4(0) ;

            public:
                D3D9_VertexBuffer() ;
                virtual ~D3D9_VertexBuffer() ;

                virtual void TranslateFromGeneric( const VertexDeclaration & targetVertexDeclaration ) ;
                virtual void Clear() {}

                static const unsigned sTypeId = 'vbd9' ;

            private:
                virtual bool Allocate( size_t numVertices ) ;
                virtual void Deallocate() ;
                virtual void DeclareVertexFormat( const VertexDeclaration & vertexDeclaration ) ;

                /// Get vertex format layout identifier
                const unsigned &             GetVertexFormat() const    { return mVertexFormat ; }

                virtual void * LockVertexData() ;
                virtual void UnlockVertexData() ;
                virtual void * GetElementStart( void * vertexData , VertexDeclaration::VertexElement::SemanticE semantic , size_t which ) ;

                void    Swap( D3D9_VertexBuffer & that ) ;
                bool    CreateVertexBuffer( const VertexDeclaration & vertexDeclaration , size_t numVertices ) ;
                void    CopyVerticesFromGenericToPlatformSpecific( const D3D9_VertexBuffer & genericVertexBuffer ) ;

                friend class D3D9_Mesh ; // Grant access to GetInternalBuffer.

                // Only D3D9_Mesh should access GetInternalBuffer.
                VERTEX_BUFFER_POINTER_TYPE & GetInternalBuffer()        { return mInternalVertexBuffer ; }

                unsigned                    mVertexFormat           ;   ///< Flexible vertex format flags
                VERTEX_BUFFER_POINTER_TYPE  mInternalVertexBuffer   ;   ///< Internal D3D vertex buffer object

                size_t  mOffsetPx                   ;   ///< Offset, relative to start of vertex, of position
                size_t  mOffsetTu                   ;   ///< Offset, relative to start of vertex, of texture coordinate
                size_t  mOffsetCr                   ;   ///< Offset, relative to start of vertex, of color
                size_t  mOffsetNx                   ;   ///< Offset, relative to start of vertex, of normal
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
