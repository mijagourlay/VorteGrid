/** \file OpenGL_VertexBuffer.h

    \brief Vertex buffer for OpenGL

    \author Copyright 2010-2014 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_VERTEX_BUFFER_H
#define PEGASYS_RENDER_OPENGL_VERTEX_BUFFER_H

#include "Render/Resource/vertexBuffer.h"

#if defined( WIN32 )
#   include <windows.h> // for WINGDIAPI, APIENTRY, CALLBACK, used by gl.h.
#endif

#include <GL/gl.h>

// Macros ----------------------------------------------------------------------

/** Enable use of Vertex Buffer Objects, an OpenGL extension.

    Vertex buffer objects facilitate directly filling vertex buffers into GPU-owned memory
    bypassing an intermediate CPU-owned buffer.  VBO's are therefore much faster.
    But not all versions of OpenGL support VBO's.
    Support for VBO is detected and selected at runtime.
*/
#define USE_VERTEX_BUFFER_OBJECT 1

// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Vertex buffer for OpenGL.
        */
        class OpenGL_VertexBuffer : public VertexBufferBase
        {
            public:

                typedef unsigned char *         VERTEX_BUFFER_POINTER_TYPE ;
                typedef const unsigned char *   CONST_VERTEX_BUFFER_POINTER_TYPE ;

                struct VertexFormatPosition
                {   // Custom vertex format for position only
                    float px, py, pz ;  // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_Position = GL_V3F ;

                ////////////////////////////////////////////////////////
                // Position plus one other component
                ////////////////////////////////////////////////////////

                struct VertexFormatPositionNormal
                {   // Custom vertex format for position+normal
                    float nx, ny, nz ;  // surface normal unit vector
                    float px, py, pz ;  // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_PositionNormal = GL_N3F_V3F ;

                struct VertexFormatPositionColor
                {   // Custom vertex format for position+color
                    unsigned char color_rgba[4]; // color in RGBA form packed into 4 unsigned bytes
                    float px, py, pz ;  // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_PositionColor = GL_C4UB_V3F ;

                struct VertexFormatPositionTexture
                {   // Custom vertex format for position+texCoord
                    float ts, tt ;      // 2D texture coordinates
                    float px, py, pz ;  // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_PositionTexture = GL_T2F_V3F ;

                ////////////////////////////////////////////////////////
                // Position plus two other components
                ////////////////////////////////////////////////////////

                struct VertexFormatPositionNormalColor
                {   // Custom vertex format for position+normal
                    float cr, cg, cb, ca ;  // color in RGBA form
                    float nx, ny, nz ;      // surface normal unit vector
                    float px, py, pz ;      // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_PositionNormalColor = GL_C4F_N3F_V3F ;

                struct VertexFormatPositionNormalTexture
                {   // Custom vertex format for position+normal
                    float ts, tt     ;  // 2D texture coordinates
                    float nx, ny, nz ;  // surface normal unit vector
                    float px, py, pz ;  // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_PositionNormalTexture = GL_T2F_N3F_V3F ;

#if 0
// TODO: Deprecate this format because it doesn't match D3D very well, it lacks alpha, it takes more memory and the extra precision in color doesn't yield anything useful.
struct VertexFormatPositionColorTexture
{   // Custom vertex format for position+texCoord
    float ts, tt            ;   // 2D texture coordinates
    float cr, cg, cb        ;   // color in RGB form
    float px, py, pz        ;   // untransformed (world-space) position
} ;
static const unsigned sVertexFormatFlags_PositionColor3Texture2 = GL_T2F_C3F_V3F ;
#endif

// TODO: After removing above, Rename to VertexFormatPositionColorTexture
                struct VertexFormatPositionColor4Texture2
                {   // Custom vertex format for position+texCoord
                    float           ts, tt      ;   // 2D texture coordinates
                    unsigned char   crgba[4]    ;   // color in RGBA form
                    float           px, py, pz  ;   // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_PositionColor4Texture2 = GL_T2F_C4UB_V3F ;

                ////////////////////////////////////////////////////////
                // Position plus three other components
                ////////////////////////////////////////////////////////

                struct VertexFormatPositionNormalColorTexture
                {   // Custom vertex format for position+normal
                    float ts, tt, tu, tv    ;   // 4D texture coordinates
                    float cr, cg, cb, ca    ;   // color in RGBA form
                    float nx, ny, nz        ;   // surface normal unit vector
                    float px, py, pz, pw    ;   // untransformed (world-space) position
                } ;
                static const unsigned sVertexFormatFlags_PositionNormalColorTexture = GL_T4F_C4F_N3F_V4F ;

            public:
                static const unsigned sTypeId = 'vbog' ;

                OpenGL_VertexBuffer() ;
                virtual ~OpenGL_VertexBuffer() ;

                virtual void TranslateFromGeneric( const VertexDeclaration & targetVertexDeclaration ) ;
                virtual void Clear() {}

                /// Tell OpenGL where the vertex data is, that future operations will use.
                void BindVertexData() ;

            private:
                virtual bool Allocate( size_t numVertices ) ;
                virtual void Deallocate() ;
                virtual void DeclareVertexFormat( const VertexDeclaration & vertexDeclaration ) ;

                /// Get vertex format layout identifier
                const unsigned &    GetVertexFormat() const { return mVertexFormat ; }

                virtual void * LockVertexData() ;
                virtual void UnlockVertexData() ;
                virtual void * GetElementStart( void * vertexData , VertexDeclaration::VertexElement::SemanticE semantic , size_t which ) ;

                /// Get identifier (name) of OpenGL Vertex Buffer Object, or 0 if there is none.
                const GLuint        GetVboName() const      { return mVboName ; }

            private:
                void    CheckBinding() ;
                void    Swap( OpenGL_VertexBuffer & that ) ;
                bool    CreateVertexBuffer( const VertexDeclaration & vertexDeclaration , size_t numVertices ) ;
                void    CopyVerticesFromGenericToPlatformSpecific( const OpenGL_VertexBuffer & genericVertexBuffer ) ;
                bool    HasElementWithSemantic( PeGaSys::Render::VertexDeclaration::VertexElement::SemanticE semantic ) ;

                unsigned    mVertexFormat               ;   ///< One of the OpenGL intrinsically supported vertex formats
                GLuint      mVboName                    ;   ///< Identifer for OpenGL vertex buffer object.  Mutually exclusive with mOglVertexArrayData.
                VERTEX_BUFFER_POINTER_TYPE mOglVertexArrayData  ;   /// Old-style OpenGL vertex array. Mutually exclusive with mVboName.

                GLubyte *   mOffsetPx                   ;   ///< Offset, relative to start of vertex, of position
                GLubyte *   mOffsetTu                   ;   ///< Offset, relative to start of vertex, of texture coordinate
                GLubyte *   mOffsetCr                   ;   ///< Offset, relative to start of vertex, of color
                GLubyte *   mOffsetNx                   ;   ///< Offset, relative to start of vertex, of normal

                GLint       mNumPosCoordsPerVert        ;   ///< Number of position coordinates per vertex
                GLint       mNumTexCoordsPerVert        ;   ///< Number of texture coordinates per vertex
                GLint       mNumColorComponentsPertVert ;   ///< Number of color components per vertex

                GLenum      mPosType                    ;   /// Type of position data
                GLenum      mTexCoordType               ;   /// Type of texture coordinate data
                GLenum      mNormalType                 ;   /// Type of normal data
                GLenum      mColorType                  ;   /// Type of color data

        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
