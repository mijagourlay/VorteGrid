/** \file OpenGL_VertexBuffer.cpp

    \brief Vertex buffer for OpenGL

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Platform/OpenGL/OpenGL_VertexBuffer.h"

#include "Render/Platform/OpenGL/OpenGL_Api.h"
#include "Render/Platform/OpenGL/OpenGL_Extensions.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#include <memory.h>
#include <float.h>

#if defined( WIN32 )
#   include <windows.h>
#endif

#include <GL/gl.h>  // On Windows machines, this requires #include windows.h

// Types -----------------------------------------------------------------------

typedef unsigned char * VERTEX_BUFFER_POINTER_TYPE  ;

// Macros ----------------------------------------------------------------------

#if defined( _DEBUG )
#   define CHECK_BINDING() CheckBinding()
#else
#   define CHECK_BINDING()
#endif

// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct vertex buffer for OpenGL.
        */
        OpenGL_VertexBuffer::OpenGL_VertexBuffer()
            : VertexBufferBase()
            , mVertexFormat( 0 )
            , mVboName( 0 )
            , mOglVertexArrayData( NULLPTR )

            , mOffsetPx( 0 )
            , mOffsetTu( 0 )
            , mOffsetCr( 0 )
            , mOffsetNx( 0 )

            , mNumPosCoordsPerVert( 0 )
            , mNumTexCoordsPerVert( 0 )
            , mNumColorComponentsPertVert( 0 )

            , mPosType( 0 )
            , mTexCoordType( 0 )
            , mNormalType( 0 )
            , mColorType( 0 )

        {
            PERF_BLOCK( OpenGL_VertexBuffer__OpenGL_VertexBuffer ) ;

            mTypeId = sTypeId ;
        }




        /** Destruct vertex buffer for OpenGL.
        */
        OpenGL_VertexBuffer::~OpenGL_VertexBuffer()
        {
            PERF_BLOCK( OpenGL_VertexBuffer__dtor ) ;

            Deallocate() ;
        }




        /** Make sure vertex buffer bound is the one associated with this object.
        */
        void OpenGL_VertexBuffer::CheckBinding()
        {
            PERF_BLOCK( OpenGL_VertexBuffer__CheckBinding ) ;

            if( mVboName != 0 )
            {   // This VB uses vertex buffer object.
                GLint arrayBufferBinding = 0 ;
                glGetIntegerv( GL_ARRAY_BUFFER_BINDING , & arrayBufferBinding ) ;
                ASSERT( static_cast< GLint >( mVboName ) == arrayBufferBinding ) ;
            }
        }




        /** Swap all members between that and this.
        */
        void OpenGL_VertexBuffer::Swap( OpenGL_VertexBuffer & that )
        {
            PERF_BLOCK( OpenGL_VertexBuffer__Swap ) ;

            OpenGL_VertexBuffer temp ;
            memcpy( & temp ,   this , sizeof( that ) ) ;
            memcpy(   this , & that , sizeof( that ) ) ;
            memcpy( & that , & temp , sizeof( that ) ) ;
        }




        /** Get address of vertex data that this buffer object manages, and hold a lock on it (for the CPU, to prevent GPU from accessing it).
        */
        void * OpenGL_VertexBuffer::LockVertexData()
        {
            PERF_BLOCK( OpenGL_VertexBuffer__LockVertexData ) ;

            if( mVboName != 0 )
            {   // Using Vertex Buffer Objects.  Give CPU access to GPU data (and deny GPU access).
                // Inform renderer of the vertex format and location of vertex data.
                BindVertexData() ;
                CHECK_BINDING() ;
                VERTEX_BUFFER_POINTER_TYPE pVertexData = (VERTEX_BUFFER_POINTER_TYPE) glMapBuffer( GL_ARRAY_BUFFER , GL_WRITE_ONLY ) ;
                RENDER_CHECK_ERROR( OpenGL_VertexBuffer__LockVertexData ) ;
                ASSERT( pVertexData != 0 ) ;
                ASSERT( IsAligned( pVertexData , 64 ) ) ; // Check whether buffer is cache-aligned.  Technically it does not have to be but it can reduce performance if it isn't.  Feel free to comment out this line if it triggers, but I would like to be notified.
                return pVertexData ;
            }
            else
            {   // Using old-style vertex arrays.  No such thing as locking that data.  Just return CPU-owned buffer.
                return mOglVertexArrayData ;
            }
        }




        /** Release CPU's hold on vertex data to allow GPU to access it.

        \note   This does NOT release the resource in the sense of releasing a refcount, i.e. this will NOT
        induce deallocating the vertex *buffer*.  This only release the lock on the vertex *data*.
        */
        void OpenGL_VertexBuffer::UnlockVertexData()
        {
            PERF_BLOCK( OpenGL_VertexBuffer__UnlockVertexData ) ;

            ASSERT( NULLPTR == GetGenericVertexData() ) ; // Mutually exclusive with generic VB.

            if( mVboName != 0 )
            {   // Using Vertex Buffer Objects.  Presumably, CPU holds a lock on the vertex data.
                ASSERT( NULLPTR == mOglVertexArrayData ) ; // Mutually exclusive with VBO.
                // Release lock held by CPU (if it indeed held it, e.g. from when it was filling the vertex buffer).
                CHECK_BINDING() ;
                DEBUG_ONLY( GLboolean unmapped = ) glUnmapBuffer( GL_ARRAY_BUFFER ) ;
                RENDER_CHECK_ERROR( OpenGL_VertexBuffer__UnlockVertexData ) ;
                ASSERT( unmapped ) ;
            }
            else
            {   // Using old-style vertex arrays.  No such thing as locking that data.  Do nothing.
                ASSERT( mOglVertexArrayData != NULLPTR ) ;
            }
        }




        bool OpenGL_VertexBuffer::HasElementWithSemantic( PeGaSys::Render::VertexDeclaration::VertexElement::SemanticE semantic )
        {
            PERF_BLOCK( OpenGL_VertexBuffer__HasElementWithSemantic ) ;

            bool hasElementWithSemantic = false ;
            switch( semantic )
            {
            case VertexDeclaration::VertexElement::POSITION     : hasElementWithSemantic = true                                             ; break ;
            case VertexDeclaration::VertexElement::NORMAL       : hasElementWithSemantic = GetVertexDeclaration().HasNormals()              ; break ;
            case VertexDeclaration::VertexElement::COLOR_AMBIENT: hasElementWithSemantic = GetVertexDeclaration().HasColors()               ; break ;
            case VertexDeclaration::VertexElement::TEXTURE_0    : hasElementWithSemantic = GetVertexDeclaration().HasTextureCoordinates()   ; break ;
            }
            return hasElementWithSemantic ;
        }




        /** Get address of vertex element with the given semantic.
        */
        void * OpenGL_VertexBuffer::GetElementStart( void * vertexData , PeGaSys::Render::VertexDeclaration::VertexElement::SemanticE semantic , size_t which )
        {
            PERF_BLOCK( OpenGL_VertexBuffer__GetElementStart ) ;

            ASSERT( 0 == which ) ; // Multiple elements with the same semantic not yet supported.
            NON_DEBUG_ONLY( UNUSED_PARAM( which ) ) ;

            if( ! HasElementWithSemantic( semantic ) )
            {
                FAIL() ;
                return NULLPTR ;
            }

            GLubyte * vertexDataBytes = static_cast< GLubyte * >( vertexData ) ;

            switch( semantic )
            {
            case VertexDeclaration::VertexElement::POSITION     : return vertexDataBytes + reinterpret_cast< size_t >( mOffsetPx ) ; break ;
            case VertexDeclaration::VertexElement::NORMAL       : return vertexDataBytes + reinterpret_cast< size_t >( mOffsetNx ) ; break ;
            case VertexDeclaration::VertexElement::COLOR_AMBIENT: return vertexDataBytes + reinterpret_cast< size_t >( mOffsetCr ) ; break ;
            case VertexDeclaration::VertexElement::TEXTURE_0    : return vertexDataBytes + reinterpret_cast< size_t >( mOffsetTu ) ; break ;
            }

            FAIL() ; // Unsupported semantic.
            return NULLPTR ;
        }




        /** Allocate memory for vertex buffer.

            \return true if succeeded, false otherwise
        */
        bool OpenGL_VertexBuffer::Allocate( size_t numVertices )
        {
            PERF_BLOCK( OpenGL_VertexBuffer__Allocate ) ;

            RENDER_CHECK_ERROR( OpenGL_VertexBuffer__Allocate ) ;

            ASSERT( 0 == mVboName ) ;   // Not allowed to allocate over existing VBO.
            ASSERT( NULLPTR == mOglVertexArrayData ) ;  // Not allowed to allocate over VB array.
            ASSERT( NULLPTR == GetGenericVertexData() ) ; // Not allowed to allocate over generic VB.
            ASSERT( numVertices != 0 ) ;
            ASSERT( GetPopulation() == 0 ) ;
            ASSERT( GetVertexSizeInBytes() != 0 ) ;    // Vertex format must have been specified beforehand.

            static bool forciblyUseOldStyleArrays = false ; // Used to test code path for old-style arrays on displays that support VBO's.

            if( glGenBuffers && ! forciblyUseOldStyleArrays )
            {   // This display supports Vertex Buffer Objects.
                // Reserve VBO buffer identifier
                glGenBuffers( 1 , & mVboName ) ;
                RENDER_CHECK_ERROR( OpenGL_VertexBuffer__CreateVertexBuffer_glGenBuffers ) ;

                // Create vertex buffer.  This allocates enough memory to hold all the vertex data.
                glBindBuffer( GL_ARRAY_BUFFER , mVboName ) ;
                RENDER_CHECK_ERROR( OpenGL_VertexBuffer__Allocate_glBindBuffer ) ;
                CHECK_BINDING() ;

                const size_t uVboSize = GetVertexSizeInBytes() * numVertices ;
                glBufferData( GL_ARRAY_BUFFER , uVboSize , NULL , GL_STREAM_DRAW ) ;
                RENDER_CHECK_ERROR( OpenGL_VertexBuffer__Allocate_glBufferData ) ;

                glBindBuffer( GL_ARRAY_BUFFER , mVboName ) ;
                RENDER_CHECK_ERROR( OpenGL_VertexBuffer_Allocate_glBindBuffer_2 ) ;
            }
            else
            {   // This display does NOT support Vertex Buffer Objects.  Use old-style vertex arrays instead.
                const size_t numBytes = GetVertexSizeInBytes() * numVertices ;
                mOglVertexArrayData = NEW unsigned char[ numBytes ] ;
                ASSERT( mOglVertexArrayData != NULLPTR ) ;
                ASSERT( IsAligned( mOglVertexArrayData , 64 ) ) ; // Check whether buffer is cache-aligned.  Technically it does not have to be but it can reduce performance if it isn't.  Feel free to comment out this line if it triggers, but I would like to be notified.
                ASSERT( 0 == mVboName ) ;   // Old-style vertex array is mutually exclusive with VBO.
            }

            ASSERT( ( 0 == mVboName ) || ( NULLPTR == mOglVertexArrayData ) ) ; // Can not have both VBO and CPU-owned vertex buffer.
            ASSERT( ( 0 != mVboName ) || ( NULLPTR != mOglVertexArrayData ) ) ; // Must have either VBO or CPU-owned vertex buffer.

            if( ( mVboName != 0 ) || ( mOglVertexArrayData != NULLPTR ) )
            {   // Allocation succeeded.
                CHECK_BINDING() ;
                SetCapacity( numVertices ) ;
                return true ; // Inform caller allocation succeeded.
            }
            FAIL() ;
            return false ; // Inform caller allocation failed.
        }




        /** Deallocate vertex buffer allocated by Allocate.
        */
        void OpenGL_VertexBuffer::Deallocate()
        {
            PERF_BLOCK( OpenGL_VertexBuffer__Deallocate ) ;

            RENDER_CHECK_ERROR( OpenGL_VertexBuffer__Deallocate_enter ) ;

#       if USE_VERTEX_BUFFER_OBJECT
            if( glDeleteBuffers && ( mVboName != 0 ) )
            {   // Used new-style vertex buffer objects.
                ASSERT( NULLPTR == mOglVertexArrayData ) ;
                // Tell OpenGL to delete vertex buffer object allocated in "GPU domain"
                glDeleteBuffers( 1 , & mVboName ) ;
                mVboName = 0 ;
            }
            else
#       endif
            {   // Used old-style vertex arrays (not vertex buffer objects).
                ASSERT( 0 == mVboName ) ;
                delete mOglVertexArrayData ;    // Delete buffer allocated by CPU
            }

            SetPopulation( 0 ) ;
            SetCapacity( 0 ) ;

            RENDER_CHECK_ERROR( OpenGL_VertexBuffer__Deallocate_exit ) ;
        }




        /** Set vertex buffer format.
        */
        void OpenGL_VertexBuffer::DeclareVertexFormat( const VertexDeclaration & vertexDeclaration )
        {
            PERF_BLOCK( OpenGL_VertexBuffer__DeclareVertexFormat ) ;

            ASSERT( GetVertexDeclaration().mVertexFormat == VertexDeclaration::VERTEX_FORMAT_NONE ) ;
            ASSERT( GetVertexSizeInBytes() == 0 ) ;

            ASSERT( 0 == mOffsetPx ) ;
            ASSERT( 0 == mOffsetTu ) ;
            ASSERT( 0 == mOffsetCr ) ;
            ASSERT( 0 == mOffsetNx ) ;

            ASSERT( 0 == mNumPosCoordsPerVert ) ;
            ASSERT( 0 == mNumTexCoordsPerVert ) ;
            ASSERT( 0 == mNumColorComponentsPertVert ) ;

            ASSERT( 0 == mPosType ) ;
            ASSERT( 0 == mTexCoordType ) ;
            ASSERT( 0 == mNormalType ) ;
            ASSERT( 0 == mColorType ) ;

            // Uniformly assign position info, since it's the same across most vertex formats.
            mNumPosCoordsPerVert = 3 ;
            mPosType = GL_FLOAT ;

            mNormalType = GL_FLOAT ;    // When there vertex has a normal, its type is always float (in current implementation).

            // Establish format of vertex buffer, based on vertex declaration.
            const VertexDeclaration::VertexFormatE & tgtVertFmt = vertexDeclaration.mVertexFormat ;
            size_t vertexSize = 0 ;
            switch( tgtVertFmt )
            {
            case VertexDeclaration::VERTEX_FORMAT_NONE:
                FAIL() ;
                break ;

            case VertexDeclaration::POSITION:
                vertexSize      = sizeof( VertexFormatPosition ) ;
                mVertexFormat   = sVertexFormatFlags_Position ;
                mOffsetPx       = (GLubyte*) offsetof( VertexFormatPosition , px  ) ;
                break ;

            case VertexDeclaration::POSITION_NORMAL                :
                vertexSize      = sizeof( VertexFormatPositionNormal ) ;
                mVertexFormat   = sVertexFormatFlags_PositionNormal ;

                mOffsetPx       = (GLubyte*) offsetof( VertexFormatPositionNormal , px  ) ;
                mOffsetNx       = (GLubyte*) offsetof( VertexFormatPositionNormal , nx  ) ;
                break ;

            case VertexDeclaration::POSITION_COLOR                 :    // Unusual: color is bytes
                vertexSize                  = sizeof( VertexFormatPositionColor ) ;
                mVertexFormat               = sVertexFormatFlags_PositionColor ;

                mOffsetPx                   = (GLubyte*) offsetof( VertexFormatPositionColor , px    ) ;
                mOffsetCr                   = (GLubyte*) offsetof( VertexFormatPositionColor , color_rgba ) ;    // Only vertex format where the color member is named "color_rgba".

                mNumColorComponentsPertVert = 4 ;
                mColorType                  = GL_UNSIGNED_BYTE ;    // Only vertex format where the color type is unsigned byte.
                break ;

            case VertexDeclaration::POSITION_TEXTURE               :
                vertexSize = sizeof( VertexFormatPositionTexture ) ;
                mVertexFormat = sVertexFormatFlags_PositionTexture ;

                mOffsetPx               = (GLubyte*) offsetof( VertexFormatPositionTexture , px ) ;
                mOffsetTu               = (GLubyte*) offsetof( VertexFormatPositionTexture , ts ) ;
                mNumTexCoordsPerVert    = 2 ;
                mTexCoordType           = GL_FLOAT ;
                break ;

            case VertexDeclaration::POSITION_NORMAL_COLOR           :
                vertexSize = sizeof( VertexFormatPositionNormalColor ) ;
                mVertexFormat = sVertexFormatFlags_PositionNormalColor ;

                mOffsetPx   = (GLubyte*) offsetof( VertexFormatPositionNormalColor , px ) ;
                mOffsetCr   = (GLubyte*) offsetof( VertexFormatPositionNormalColor , cr ) ;
                mOffsetNx   = (GLubyte*) offsetof( VertexFormatPositionNormalColor , nx ) ;

                mNumColorComponentsPertVert = 4 ;
                mColorType                  = GL_FLOAT ;
                break ;

            case VertexDeclaration::POSITION_NORMAL_TEXTURE        :
                vertexSize = sizeof( VertexFormatPositionNormalTexture ) ;
                mVertexFormat = sVertexFormatFlags_PositionNormalTexture ;

                mOffsetPx               = (GLubyte*) offsetof( VertexFormatPositionNormalTexture , px ) ;
                mOffsetTu               = (GLubyte*) offsetof( VertexFormatPositionNormalTexture , ts ) ;
                mOffsetNx               = (GLubyte*) offsetof( VertexFormatPositionNormalTexture , nx ) ;

                mNumTexCoordsPerVert    = 2 ;

                mTexCoordType           = GL_FLOAT ;
                break ;

            case VertexDeclaration::POSITION_COLOR_TEXTURE         : // Unusual: 3 color components
                vertexSize    = sizeof( VertexFormatPositionColor4Texture2 ) ;
                mVertexFormat = sVertexFormatFlags_PositionColor4Texture2 ;

                mOffsetPx                   = (GLubyte*) offsetof( VertexFormatPositionColor4Texture2 , px    ) ;
                mOffsetCr                   = (GLubyte*) offsetof( VertexFormatPositionColor4Texture2 , crgba ) ;    // Only vertex format where the color member is named "crgba".
                mOffsetTu                   = (GLubyte*) offsetof( VertexFormatPositionColor4Texture2 , ts    ) ;

                mNumPosCoordsPerVert        = 3 ;
                mNumColorComponentsPertVert = 4 ;
                mNumTexCoordsPerVert        = 2 ;

                mColorType                  = GL_UNSIGNED_BYTE ;
                mTexCoordType               = GL_FLOAT ;
                break ;

            case VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE  : // Unusual: 4 position coordinates, 4 texture coordinates
                vertexSize = sizeof( VertexFormatPositionNormalColorTexture ) ;
                mVertexFormat = sVertexFormatFlags_PositionNormalColorTexture ;

                mOffsetPx               = (GLubyte*) offsetof( VertexFormatPositionNormalColorTexture , px ) ;
                mOffsetNx               = (GLubyte*) offsetof( VertexFormatPositionNormalColorTexture , nx ) ;
                mOffsetCr               = (GLubyte*) offsetof( VertexFormatPositionNormalColorTexture , cr ) ;
                mOffsetTu               = (GLubyte*) offsetof( VertexFormatPositionNormalColorTexture , ts ) ;

                mNumPosCoordsPerVert        = 4 ;   // Only vertex format with 4 position coords.
                mNumColorComponentsPertVert = 4 ;
                mNumTexCoordsPerVert        = 4 ;   // Only vertex format with 4 tex coords.

                mColorType              = GL_FLOAT ;
                mTexCoordType           = GL_FLOAT ;
                break ;

            case VertexDeclaration::GENERIC  :
                vertexSize    = sizeof( GenericVertex ) ;
                mVertexFormat = 0 ;
                break ;

            case VertexDeclaration::NUM_FORMATS:
                FAIL() ;
                mVertexFormat = 0 ;
                break ;

            default:
                FAIL() ;
                mVertexFormat = 0 ;
                break ;
            }

            SetVertexSize( vertexSize ) ;
            SetVertexDeclaration( vertexDeclaration ) ;
        }




        /** Format and allocate vertex buffer.

        \return true if succeeded, false otherwise
        */
        bool OpenGL_VertexBuffer::CreateVertexBuffer( const VertexDeclaration & vertexDeclaration , size_t numVertices )
        {
            PERF_BLOCK( OpenGL_VertexBuffer__CreateVertexBuffer ) ;

            DeclareVertexFormat( vertexDeclaration ) ;
            return Allocate( numVertices ) ;
        }




        static inline char ColorByteFromFloat( float fColor0to1 )
        {
            PERF_BLOCK( Render__ColorByteFromFloat ) ;

            static const float almost256 = 256.0f * ( 1.0f - FLT_EPSILON ) ;
            unsigned int iColor0to255 = static_cast< unsigned int >( fColor0to1 * almost256 ) ;
            ASSERT( ( iColor0to255 >= 0 ) && ( iColor0to255 <= 255 ) ) ;
            return static_cast< unsigned char >( iColor0to255 ) ;
        }




        static inline unsigned ColorIntFromFloats( float fRed , float fGrn , float fBlu , float fAlp )
        {
            PERF_BLOCK( Render__ColorIntFromFloats ) ;

            unsigned int iRed = ColorByteFromFloat( fRed ) ;
            unsigned int iGrn = ColorByteFromFloat( fGrn ) ;
            unsigned int iBlu = ColorByteFromFloat( fBlu ) ;
            unsigned int iAlp = ColorByteFromFloat( fAlp ) ;
#       if 1   // Big-endian
            const unsigned color = ( iRed       ) | ( iGrn <<  8 ) | ( iBlu << 16 ) | ( iAlp << 24 ) ;
#       else   // Little-endian
            const unsigned color = ( iRed << 24 ) | ( iGrn << 16 ) | ( iBlu <<  8 ) | ( iAlp       ) ;
#       endif
            return color ;
        }




        /** Copy vertices from that (which must be generic) to this.
        */
        void    OpenGL_VertexBuffer::CopyVerticesFromGenericToPlatformSpecific( const OpenGL_VertexBuffer & genericVertexBuffer )
        {
            PERF_BLOCK( OpenGL_VertexBuffer__CopyVerticesFromGenericToPlatformSpecific ) ;

            ASSERT( NULLPTR == GetGenericVertexData() ) ; // VB can't have both D3D and generic data.
            ASSERT( GetVertexDeclaration().mVertexFormat != VertexDeclaration::VERTEX_FORMAT_NONE ) ;
            ASSERT( GetCapacity() >= genericVertexBuffer.GetPopulation() ) ;
            ASSERT( GetVertexSizeInBytes() > 0 ) ;
            ASSERT( genericVertexBuffer.GetGenericVertexData() != NULLPTR ) ; // incoming VB must be generic.
            ASSERT( genericVertexBuffer.GetVertexDeclaration().mVertexFormat == VertexDeclaration::GENERIC ) ;
            ASSERT( genericVertexBuffer.GetVertexSizeInBytes() > 0 ) ;

            // Obtain lock (for CPU) on vertex buffer contents so GPU does not try to access it while this routine changes its contents.
            void *                  dstVertexData   = (VERTEX_BUFFER_POINTER_TYPE) LockVertexData() ;

            ASSERT( dstVertexData != NULLPTR ) ;  // Make sure we obtained lock.

            const size_t            numVerts        = genericVertexBuffer.GetPopulation() ;
            const GenericVertex *   src             = genericVertexBuffer.GetGenericVertexData() ;
            const VertexDeclaration::VertexFormatE & tgtVertFmt = GetVertexDeclaration().mVertexFormat ;

            switch( tgtVertFmt )
            {
            case VertexDeclaration::VERTEX_FORMAT_NONE:
                FAIL() ;
                break ;

            case VertexDeclaration::POSITION:
                {
                    VertexFormatPosition * dst = static_cast< VertexFormatPosition * >( dstVertexData ) ;
                    for( size_t idx = 0 ; idx < numVerts ; ++ idx )
                    {
                        dst[ idx ].px = src[ idx ].px ;
                        dst[ idx ].py = src[ idx ].py ;
                        dst[ idx ].pz = src[ idx ].pz ;
                    }
                }
                break ;

            case VertexDeclaration::POSITION_COLOR                 :
                {
                    VertexFormatPositionColor * dst = static_cast< VertexFormatPositionColor * >( dstVertexData ) ;
                    for( size_t idx = 0 ; idx < numVerts ; ++ idx )
                    {
                        dst[ idx ].px = src[ idx ].px ;
                        dst[ idx ].py = src[ idx ].py ;
                        dst[ idx ].pz = src[ idx ].pz ;
                        dst[ idx ].color_rgba[0] = ColorByteFromFloat( src[ idx ].cr ) ;
                        dst[ idx ].color_rgba[1] = ColorByteFromFloat( src[ idx ].cg ) ;
                        dst[ idx ].color_rgba[2] = ColorByteFromFloat( src[ idx ].cb ) ;
                        dst[ idx ].color_rgba[3] = ColorByteFromFloat( src[ idx ].ca ) ;
                    }
                }
                break ;

            case VertexDeclaration::POSITION_NORMAL                :
                {
                    VertexFormatPositionNormal * dst = static_cast< VertexFormatPositionNormal * >( dstVertexData ) ;
                    for( size_t idx = 0 ; idx < numVerts ; ++ idx )
                    {
                        dst[ idx ].px = src[ idx ].px ;
                        dst[ idx ].py = src[ idx ].py ;
                        dst[ idx ].pz = src[ idx ].pz ;
                        dst[ idx ].nx = src[ idx ].nx ;
                        dst[ idx ].ny = src[ idx ].ny ;
                        dst[ idx ].nz = src[ idx ].nz ;
                    }
                }
                break ;

            case VertexDeclaration::POSITION_TEXTURE               :
                {
                    VertexFormatPositionTexture * dst = static_cast< VertexFormatPositionTexture * >( dstVertexData ) ;
                    for( size_t idx = 0 ; idx < numVerts ; ++ idx )
                    {
                        dst[ idx ].px = src[ idx ].px ;
                        dst[ idx ].py = src[ idx ].py ;
                        dst[ idx ].pz = src[ idx ].pz ;
                        dst[ idx ].ts = src[ idx ].ts ;
                        dst[ idx ].tt = src[ idx ].tt ;
                    }
                }
                break ;

            case VertexDeclaration::POSITION_NORMAL_COLOR          :
                {
                    VertexFormatPositionNormalColor * dst = static_cast< VertexFormatPositionNormalColor * >( dstVertexData ) ;
                    for( size_t idx = 0 ; idx < numVerts ; ++ idx )
                    {
                        dst[ idx ].px = src[ idx ].px ;
                        dst[ idx ].py = src[ idx ].py ;
                        dst[ idx ].pz = src[ idx ].pz ;
                        dst[ idx ].nx = src[ idx ].nx ;
                        dst[ idx ].ny = src[ idx ].ny ;
                        dst[ idx ].nz = src[ idx ].nz ;
                        dst[ idx ].cr = src[ idx ].cr ;
                        dst[ idx ].cg = src[ idx ].cg ;
                        dst[ idx ].cb = src[ idx ].cb ;
                        dst[ idx ].ca = src[ idx ].ca ;
                    }
                }
                break ;

            case VertexDeclaration::POSITION_NORMAL_TEXTURE        :
                {
                    VertexFormatPositionNormalTexture * dst = static_cast< VertexFormatPositionNormalTexture * >( dstVertexData ) ;
                    for( size_t idx = 0 ; idx < numVerts ; ++ idx )
                    {
                        dst[ idx ].px = src[ idx ].px ;
                        dst[ idx ].py = src[ idx ].py ;
                        dst[ idx ].pz = src[ idx ].pz ;
                        dst[ idx ].nx = src[ idx ].nx ;
                        dst[ idx ].ny = src[ idx ].ny ;
                        dst[ idx ].nz = src[ idx ].nz ;
                        dst[ idx ].ts = src[ idx ].ts ;
                        dst[ idx ].tt = src[ idx ].tt ;
                    }
                }
                break ;

            case VertexDeclaration::POSITION_COLOR_TEXTURE         :
                {
                    VertexFormatPositionColor4Texture2 * dst = static_cast< VertexFormatPositionColor4Texture2 * >( dstVertexData ) ;
                    for( size_t idx = 0 ; idx < numVerts ; ++ idx )
                    {
                        dst[ idx ].px       = src[ idx ].px ;
                        dst[ idx ].py       = src[ idx ].py ;
                        dst[ idx ].pz       = src[ idx ].pz ;
                        dst[ idx ].crgba[0] = ColorByteFromFloat( src[ idx ].cr ) ;
                        dst[ idx ].crgba[1] = ColorByteFromFloat( src[ idx ].cg ) ;
                        dst[ idx ].crgba[2] = ColorByteFromFloat( src[ idx ].cb ) ;
                        dst[ idx ].crgba[3] = ColorByteFromFloat( src[ idx ].ca ) ;
                        dst[ idx ].ts       = src[ idx ].ts ;
                        dst[ idx ].tt       = src[ idx ].tt ;
                    }
                }
                break ;

            case VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE  :
                {
                    VertexFormatPositionNormalColorTexture * dst = static_cast< VertexFormatPositionNormalColorTexture * >( dstVertexData ) ;
                    for( size_t idx = 0 ; idx < numVerts ; ++ idx )
                    {
                        dst[ idx ].px = src[ idx ].px ;
                        dst[ idx ].py = src[ idx ].py ;
                        dst[ idx ].pz = src[ idx ].pz ;
                        dst[ idx ].pw = src[ idx ].pw ;
                        dst[ idx ].nx = src[ idx ].nx ;
                        dst[ idx ].ny = src[ idx ].ny ;
                        dst[ idx ].nz = src[ idx ].nz ;
                        dst[ idx ].cr = src[ idx ].cr ;
                        dst[ idx ].cg = src[ idx ].cg ;
                        dst[ idx ].cb = src[ idx ].cb ;
                        dst[ idx ].ca = src[ idx ].ca ;
                        dst[ idx ].ts = src[ idx ].ts ;
                        dst[ idx ].tt = src[ idx ].tt ;
                        dst[ idx ].tu = src[ idx ].tu ;
                        dst[ idx ].tv = src[ idx ].tv ;
                    }
                }
                break ;

            case VertexDeclaration::NUM_FORMATS:
                FAIL() ;
                break ;

            default:
                FAIL() ;
                break ;
            }

            SetPopulation( numVerts ) ;

            UnlockVertexData() ;
        }




        /* virtual */ void OpenGL_VertexBuffer::TranslateFromGeneric( const VertexDeclaration & targetVertexDeclaration )
        {
            PERF_BLOCK( OpenGL_VertexBuffer__TranslateFromGeneric ) ;

            ASSERT( GetGenericVertexData() != NULLPTR ) ; // Incoming VB must be generic.
            ASSERT( GetVertexDeclaration().mVertexFormat == VertexDeclaration::GENERIC ) ;
            ASSERT( GetPopulation() > 0 ) ;
            ASSERT( GetVertexSizeInBytes() > 0 ) ;

            // Save off original data because members of this will get overwritten.
            OpenGL_VertexBuffer genericSource ;
            genericSource.Swap( * this ) ;

            if( CreateVertexBuffer( targetVertexDeclaration , genericSource.GetPopulation() ) )
            {   // Successfully allocated new vertex buffer.
                CopyVerticesFromGenericToPlatformSpecific( genericSource ) ;
            }
            else
            {
                FAIL() ;
            }

            // On scope close, genericSource will be destructed and its resources released.
        }




        /** Set OpenGL state so future OpenGL operations (e.g. rendering) use this vertex buffer's data.
        */
        void OpenGL_VertexBuffer::BindVertexData()
        {
            PERF_BLOCK( OpenGL_VertexBuffer__BindVertexData ) ;

            RENDER_CHECK_ERROR( OpenGL_VertexBuffer__BindVertexData_entry ) ;

            const GLsizei   vertexSize      = static_cast< GLsizei >( GetVertexSizeInBytes() ) ;
            ASSERT( vertexSize != 0 ) ;
            ASSERT( mVertexFormat != 0 ) ;

            if( GetVboName() != 0 )
            {   // Mesh uses Vertex Buffer Objects

                // Tell OpenGL which buffer is bound, i.e. to which buffer the following operations pertain to.
                glBindBuffer( GL_ARRAY_BUFFER , mVboName ) ;
                CHECK_BINDING() ;

                switch( GetVertexFormat() )
                {
                case sVertexFormatFlags_Position:
                    glVertexPointer  ( mNumPosCoordsPerVert , mPosType , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glDisableClientState( GL_NORMAL_ARRAY ) ;
                    glDisableClientState( GL_COLOR_ARRAY ) ;
                    glDisableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
                case sVertexFormatFlags_PositionNormal:
                    glNormalPointer  (                        mNormalType , vertexSize , mOffsetNx ) ;
                    glVertexPointer  ( mNumPosCoordsPerVert , mPosType    , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glEnableClientState( GL_NORMAL_ARRAY ) ;
                    glDisableClientState( GL_COLOR_ARRAY ) ;
                    glDisableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
                case sVertexFormatFlags_PositionColor:
                    glColorPointer   ( mNumColorComponentsPertVert , mColorType , vertexSize , mOffsetCr ) ;
                    glVertexPointer  ( mNumPosCoordsPerVert        , mPosType   , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glDisableClientState( GL_NORMAL_ARRAY ) ;
                    glEnableClientState( GL_COLOR_ARRAY ) ;
                    glDisableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
                case sVertexFormatFlags_PositionTexture:
                    glTexCoordPointer( mNumTexCoordsPerVert , mTexCoordType , vertexSize , mOffsetTu ) ;
                    glVertexPointer  ( mNumPosCoordsPerVert , mPosType      , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glDisableClientState( GL_NORMAL_ARRAY ) ;
                    glDisableClientState( GL_COLOR_ARRAY ) ;
                    glEnableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
                case sVertexFormatFlags_PositionNormalColor:
                    glNormalPointer  (                               mNormalType , vertexSize , mOffsetNx ) ;
                    glColorPointer   ( mNumColorComponentsPertVert , mColorType  , vertexSize , mOffsetCr ) ;
                    glVertexPointer  ( mNumPosCoordsPerVert        , mPosType    , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glEnableClientState( GL_NORMAL_ARRAY ) ;
                    glEnableClientState( GL_COLOR_ARRAY ) ;
                    glDisableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
                case sVertexFormatFlags_PositionNormalTexture:
                    glNormalPointer  (                        mNormalType   , vertexSize , mOffsetNx ) ;
                    glTexCoordPointer( mNumTexCoordsPerVert , mTexCoordType , vertexSize , mOffsetTu ) ;
                    glVertexPointer  ( mNumPosCoordsPerVert , mPosType      , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glEnableClientState( GL_NORMAL_ARRAY ) ;
                    glDisableClientState( GL_COLOR_ARRAY ) ;
                    glEnableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
#if 0
                case sVertexFormatFlags_PositionColor3Texture2:
                    glColorPointer   ( mNumColorComponentsPertVert , mColorType    , vertexSize , mOffsetCr ) ;
                    glTexCoordPointer( mNumTexCoordsPerVert        , mTexCoordType , vertexSize , mOffsetTu ) ;
                    glVertexPointer  ( mNumPosCoordsPerVert        , mPosType      , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glDisableClientState( GL_NORMAL_ARRAY ) ;
                    glEnableClientState( GL_COLOR_ARRAY ) ;
                    glEnableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
#endif
                case sVertexFormatFlags_PositionColor4Texture2:
                    glColorPointer   ( mNumColorComponentsPertVert , mColorType    , vertexSize , mOffsetCr ) ;
                    glTexCoordPointer( mNumTexCoordsPerVert        , mTexCoordType , vertexSize , mOffsetTu ) ;
                    glVertexPointer  ( mNumPosCoordsPerVert        , mPosType      , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glDisableClientState( GL_NORMAL_ARRAY ) ;
                    glEnableClientState( GL_COLOR_ARRAY ) ;
                    glEnableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
                case sVertexFormatFlags_PositionNormalColorTexture:
                    glNormalPointer  (                               mNormalType   , vertexSize , mOffsetNx ) ;
                    glColorPointer   ( mNumColorComponentsPertVert , mColorType    , vertexSize , mOffsetCr ) ;
                    glTexCoordPointer( mNumTexCoordsPerVert        , mTexCoordType , vertexSize , mOffsetTu ) ;
                    glVertexPointer  ( mNumPosCoordsPerVert        , mPosType      , vertexSize , mOffsetPx ) ;

                    glEnableClientState( GL_VERTEX_ARRAY ) ;
                    glEnableClientState( GL_NORMAL_ARRAY ) ;
                    glEnableClientState( GL_COLOR_ARRAY ) ;
                    glEnableClientState( GL_TEXTURE_COORD_ARRAY ) ;
                    break ;
                default:
                    FAIL() ;
                    break ;
                }
            }
            else
            {
                OpenGL_VertexBuffer::CONST_VERTEX_BUFFER_POINTER_TYPE    vertexData      = static_cast< OpenGL_VertexBuffer::CONST_VERTEX_BUFFER_POINTER_TYPE >( LockVertexData() ) ;
                glInterleavedArrays( mVertexFormat , vertexSize , vertexData ) ;
                RENDER_CHECK_ERROR( OpenGL_VertexBuffer__BindVertexData_IA ) ;
            }

            RENDER_CHECK_ERROR( OpenGL_VertexBuffer__BindVertexData_exit ) ;
        }

    } ;
} ;


#if defined( _DEBUG )

void PeGaSys_Render_OpenGL_VertexBuffer_UnitTest()
{
    DebugPrintf( "OpenGL_VertexBuffer::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::OpenGL_VertexBuffer openGL_VertexBuffer ;
        PeGaSys::Render::VertexDeclaration targetVertexDeclaration ;
        openGL_VertexBuffer.TranslateFromGeneric( targetVertexDeclaration ) ;
#ifdef Clear
#   undef Clear
#endif
        openGL_VertexBuffer.Clear() ;
    }

    DebugPrintf( "OpenGL_VertexBuffer::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif