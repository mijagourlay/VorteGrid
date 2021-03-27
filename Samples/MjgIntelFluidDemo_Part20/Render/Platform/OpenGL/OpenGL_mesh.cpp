/** \file OpenGL_Mesh.cpp

    \brief Geometry mesh for OpenGL

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Platform/OpenGL/OpenGL_Mesh.h"
#include "Render/Platform/OpenGL/OpenGL_VertexBuffer.h"
#include "Render/Platform/OpenGL/OpenGL_IndexBuffer.h"

#include "Render/Platform/OpenGL/OpenGL_api.h" // For GL_CHECK_ERROR
#include "Render/Platform/OpenGL/OpenGL_Extensions.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#if defined( WIN32 )
#   include <windows.h>
#endif

#include <GL/gl.h>  // On Windows machines, this requires #include windows.h
#include <GL/glu.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct geometry mesh for OpenGL.
        */
        OpenGL_Mesh::OpenGL_Mesh( ModelData * owningModelData )
            : MeshBase( owningModelData )
        {
            PERF_BLOCK( OpenGL_Mesh__OpenGL_Mesh ) ;
        }




        /** Destruct geometry mesh for OpenGL.
        */
        OpenGL_Mesh::~OpenGL_Mesh()
        {
            PERF_BLOCK( OpenGL_Mesh__dtor ) ;
        }




        /** Modify render state based on vertex buffer properties.
        */
        static void ModifyRenderState( OpenGL_VertexBuffer * vertexBuffer )
        {
            PERF_BLOCK( ModifyRenderState ) ;

            if( vertexBuffer->GetVertexDeclaration().HasNormals() )
            {   // Vertices contain normals, which is only meaningful when there is lighting.
                glEnable( GL_LIGHTING ) ;
            }
            else
            {   // Vertices lacks normals, which means enabling lighting would result in nonsense.
                glDisable( GL_LIGHTING ) ;
            }

            if( vertexBuffer->GetVertexDeclaration().HasTextureCoordinates() )
            {   // Vertices contain texture coordinates, which is only meaningful when there is a texture bound.
                // NOTE: enabling/disabling is meant to be per texture unit, so this should be interleaved with glActiveTexture.
                ASSERT( glIsEnabled( GL_TEXTURE_1D ) || glIsEnabled( GL_TEXTURE_2D ) ) ;
                GLint boundTexture ;
                glGetIntegerv( GL_TEXTURE_BINDING_2D , & boundTexture ) ;
                ASSERT( glIsTexture( boundTexture ) ) ;
                // TODO: Could also check active texture of unit:
                //GLint activeTexture ;
                //glGetIntegerv( GL_ACTIVE_TEXTURE , & activeTexture ) ;
            }
            else
            {   // Vertices lack texture coordinates, which means enabling texturing would result in nonsense.
                // NOTE: enabling/disabling is meant to be per texture unit, so this should be interleaved with glActiveTexture.
                ASSERT( ! glIsEnabled( GL_TEXTURE_1D ) ) ;
                ASSERT( ! glIsEnabled( GL_TEXTURE_2D ) ) ;
                //ASSERT( ! glIsEnabled( GL_TEXTURE_3D ) ) ;
                //ASSERT( ! glIsEnabled( GL_TEXTURE_CUBE_MAP ) ) ;
                //glDisable( GL_TEXTURE_1D ) ;
                //glDisable( GL_TEXTURE_2D ) ;
                //glDisable( GL_TEXTURE_3D ) ;
                //glDisable( GL_TEXTURE_CUBE_MAP ) ;
            }
            RENDER_CHECK_ERROR( OpenGL_ModifyRenderState_exit ) ;
        }




        void OpenGL_Mesh::Render()
        {
            PERF_BLOCK( OpenGL_Mesh__Render ) ;

            RENDER_CHECK_ERROR( OpenGL_Mesh__Render_entry ) ;

            OpenGL_VertexBuffer *   vertexBuffer    = static_cast< OpenGL_VertexBuffer * >( GetVertexBuffer() ) ;
            ASSERT( vertexBuffer != NULLPTR ) ;
            ASSERT( vertexBuffer->GetTypeId() == OpenGL_VertexBuffer::sTypeId ) ;

            // Set render state (lighting, texturing, etc.) appropriate for this vertex buffer.
            ModifyRenderState( vertexBuffer ) ;

            // Inform renderer of the vertex format and location of vertex data.
            vertexBuffer->BindVertexData() ;

            typedef const OpenGL_IndexBuffer::INDEX_BUFFER_POINTER_TYPE ibPtr ;

            const OpenGL_IndexBuffer *  indexBuffer = static_cast< const OpenGL_IndexBuffer * >( GetIndexBuffer() ) ;

            // Draw primitives.

            GLenum primitiveType = 0 ; // Initialize to an invalid value to catch missed cases below.

            if( indexBuffer != NULL )
            {   // Index buffer exists.
                RENDER_CHECK_ERROR( OpenGL_Mesh_Render_IB ) ;

                ASSERT( indexBuffer->GetTypeId() == OpenGL_IndexBuffer::sTypeId ) ;
                const GLsizei numIndices = static_cast< GLsizei >( indexBuffer->GetNumIndices() ) ;

                ASSERT( numIndices > 0 ) ;

                switch( GetPrimitiveType() )
                {
                case PRIMITIVE_TRIANGLES:
                    {
                        DEBUG_ONLY( const GLsizei numTriangles = numIndices  / 3 ) ;
                        DEBUG_ONLY( const GLsizei numTriVerts  = numTriangles * 3 ) ;
                        ASSERT( numIndices == numTriVerts ) ;
                        primitiveType = GL_TRIANGLES ;
                    }
                    break ;

                case PRIMITIVE_QUADS:
                    {
                        DEBUG_ONLY( const GLsizei numQuads      = numIndices / 4 ) ;
                        DEBUG_ONLY( const GLsizei numQuadVerts  = numQuads   * 4 ) ;
                        ASSERT( numIndices == numQuadVerts ) ;
                        primitiveType = GL_QUADS ;
                    }
                    break ;

                default:
                    FAIL() ;
                    break ;
                }

                const void *    indexData   = 0 ;
                GLenum          indexType   = 0 ;
                if( indexBuffer->GetIndexType() == IndexBufferBase::INDEX_TYPE_16 )
                {   // Index buffer uses 16-bit integers.
                    indexData   = indexBuffer->GetIndicesWord() ;
                    indexType   = GL_UNSIGNED_SHORT ;
                }
                else
                {   // Index buffer uses 32-bit integers.
                    ASSERT( indexBuffer->GetIndexType() == IndexBufferBase::INDEX_TYPE_32 );
                    indexData   = indexBuffer->GetIndicesInt() ;
                    indexType   = GL_UNSIGNED_INT ;
                }

                ASSERT( indexData != 0 ) ;

                glDrawElements( primitiveType
                    , numIndices
                    , indexType
                    , indexData ) ;

                RENDER_CHECK_ERROR( OpenGL_Mesh_Render_DrawElements ) ;
            }
            else
            {   // no index buffer; render vertex buffer directly
                const GLsizei numVertices = static_cast< GLsizei >( vertexBuffer->GetPopulation() ) ;

                switch( GetPrimitiveType() )
                {
                case PRIMITIVE_TRIANGLES:
                    {
                        DEBUG_ONLY( const GLsizei numTriangles = numVertices  / 3 ) ;
                        DEBUG_ONLY( const GLsizei numTriVerts  = numTriangles * 3 ) ;
                        ASSERT( numVertices == numTriVerts ) ;
                        primitiveType = GL_TRIANGLES ;
                    }
                    break ;

                case PRIMITIVE_QUADS:
                    {
                        DEBUG_ONLY( const GLsizei numQuads     = numVertices  / 4 ) ;
                        DEBUG_ONLY( const GLsizei numQuadVerts = numQuads     * 4 ) ;
                        ASSERT( numVertices == numQuadVerts ) ;
                        primitiveType = GL_QUADS ;
                    }
                    break ;

                default:
                    FAIL() ;
                    break ;
                }

                glDrawArrays( primitiveType , 0 , numVertices ) ;

                RENDER_CHECK_ERROR( OpenGL_Mesh_Render_DrawArrays ) ;
            }

            RENDER_CHECK_ERROR( OpenGL_Mesh_Render_exit ) ;
        }



    } ;
} ;


#if defined( _DEBUG )

void PeGaSys_Render_OpenGL_Mesh_UnitTest()
{
    DebugPrintf( "OpenGL_Mesh::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::OpenGL_Mesh openGL_mesh( NULLPTR ) ;
        openGL_mesh.Render() ;
    }

    DebugPrintf( "OpenGL_Mesh::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif