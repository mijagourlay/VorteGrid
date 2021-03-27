/** \file D3D9_Mesh.cpp

    \brief Geometry mesh for Direct3D version 9

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#include "Render/Platform/DirectX9/D3D9_VertexBuffer.h"
#include "Render/Platform/DirectX9/D3D9_IndexBuffer.h"
#include "Render/Platform/DirectX9/D3D9_api.h" // for HROK

#include "Render/Platform/DirectX9/D3D9_Mesh.h"

extern LPDIRECT3DDEVICE9 g_pd3dDevice ; // Direct3D rendering device

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

/** Construct geometry mesh for Direct3D version 9.
*/
D3D9_Mesh::D3D9_Mesh( ModelData * owningModelData )
    : MeshBase( owningModelData )
{
}




/** Destruct geometry mesh for Direct3D version 9.
*/
D3D9_Mesh::~D3D9_Mesh()
{
}




static void ModifyRenderState( D3D9_VertexBuffer * vertexBuffer )
{
    if( vertexBuffer->GetVertexDeclaration().HasNormals() )
    {   // Vertices contain normals, which is only meaningful when there is lighting.
        HROK( g_pd3dDevice->SetRenderState( D3DRS_LIGHTING , TRUE ) ) ;
    }
    else
    {   // Vertices lacks normals, which means enabling lighting would result in nonsense.
        HROK( g_pd3dDevice->SetRenderState( D3DRS_LIGHTING , FALSE ) ) ;
    }

    DWORD value ;
    HROK( g_pd3dDevice->GetTextureStageState( 0 , D3DTSS_COLOROP , & value ) ) ;

    if( vertexBuffer->GetVertexDeclaration().HasTextureCoordinates() )
    {   // Vertices contain texture coordinates, which is only meaningful when there is a texture bound.
        ASSERT( value != D3DTOP_DISABLE ) ;

        IDirect3DBaseTexture9 * pTexture ;
        HROK( g_pd3dDevice->GetTexture( 0 , & pTexture ) ) ;
        ASSERT( pTexture != NULL ) ;
        HROK( pTexture->Release() ) ;
    }
    else
    {   // Vertices lacks texture coordinates, which means enabling texturing would result in nonsense.
        ASSERT( D3DTOP_DISABLE == value ) ;
    }
}




void D3D9_Mesh::Render()
{
    D3D9_VertexBuffer * vertexBuffer    = static_cast< D3D9_VertexBuffer * >( GetVertexBuffer() ) ;
    ASSERT( vertexBuffer != NULLPTR ) ;
    ASSERT( vertexBuffer->GetTypeId() == D3D9_VertexBuffer::sTypeId ) ;

    // Set render state (lighting, texturing, etc.) appropriate for this vertex buffer.
    ModifyRenderState( vertexBuffer ) ;

    // Inform renderer of the vertex format and location of vertex data.
    const unsigned &    vertexFormat    = vertexBuffer->GetVertexFormat() ;
    const UINT          vertexSize      = static_cast< UINT >( vertexBuffer->GetVertexSizeInBytes() ) ;
    ASSERT( vertexFormat != 0 ) ;
    if( vertexBuffer->GetInternalBuffer() == NULLPTR )
    {   // Vertex data has not yet been allocated; nothing to render yet.  This can happen for particle systems before particles are emitted, since the VB is allocated on demand.
        return ;
    }
	HROK( g_pd3dDevice->SetStreamSource( 0 , vertexBuffer->GetInternalBuffer() , 0 , vertexSize ) ) ;
	HROK( g_pd3dDevice->SetFVF( vertexFormat ) ) ;

    const D3D9_IndexBuffer * indexBuffer = static_cast< const D3D9_IndexBuffer * >( GetIndexBuffer() ) ;

    UINT numPrims = 0 ;

    // Draw primitives

    D3DPRIMITIVETYPE primitiveType = D3DPT_FORCE_DWORD ; // Initialize to an invalid value to catch missed cases below.

    if( indexBuffer != NULL )
    {   // Index buffer exists.
        ASSERT( indexBuffer->GetTypeId() == D3D9_IndexBuffer::sTypeId ) ;
        ASSERT( indexBuffer->GetInternalBuffer() != 0 ) ;
        size_t numIndices = indexBuffer->GetNumIndices() ;

	    HROK( g_pd3dDevice->SetIndices( indexBuffer->GetInternalBuffer() ) ) ;

        switch( GetPrimitiveType() )
        {
        case PRIMITIVE_TRIANGLES:
            {
                numPrims = static_cast< UINT >( numIndices / 3 ) ;
                DEBUG_ONLY( const size_t numTriVerts  = numPrims * 3 ) ;
                ASSERT( numIndices == numTriVerts ) ;
                primitiveType = D3DPT_TRIANGLELIST ;
            }
            break ;

        case PRIMITIVE_QUADS: // NOTE: ALERT: Quads not supported by most flavors of D3D.  This is just a workaround to show geometry.
            {
                numPrims = static_cast< UINT >( indexBuffer->GetNumIndices() ) ; // NOTE: ALERT: For quads this would be /4
                DEBUG_ONLY( const size_t numQuads      = numIndices / 4 ) ;
                DEBUG_ONLY( const size_t numQuadVerts  = numQuads   * 4 ) ;
                ASSERT( numIndices == numQuadVerts ) ;
                primitiveType = D3DPT_POINTLIST ;   // NOTE: ALERT: Not quad!
                FAIL() ; // D3D does not support POINTLIST for indexed primitives.
            }
            break ;

        default:
            FAIL() ;
            break ;
        }

		HROK( g_pd3dDevice->DrawIndexedPrimitive(
                  primitiveType
                , 0
                , 0
                , static_cast< UINT >( vertexBuffer->GetPopulation() )
                , 0
                , numPrims
                ) ) ;
    }
    else
    {   // no index buffer; render vertex buffer directly
        const size_t numVertices = vertexBuffer->GetPopulation() ;

        switch( GetPrimitiveType() )
        {
        case PRIMITIVE_TRIANGLES:
            {
                numPrims = static_cast< UINT >( numVertices / 3 ) ;
                DEBUG_ONLY( const size_t numTriangles = numVertices  / 3 ) ;
                DEBUG_ONLY( const size_t numTriVerts  = numTriangles * 3 ) ;
                ASSERT( numVertices == numTriVerts ) ;
                primitiveType = D3DPT_TRIANGLELIST ;
            }
            break ;

        case PRIMITIVE_QUADS: // NOTE: ALERT: Quads not supported by most flavors of D3D.  This is just a workaround to show geometry.
            {
                numPrims = static_cast< UINT >( numVertices ) ; // NOTE: ALERT: For quads this would be /4
                DEBUG_ONLY( const size_t numQuads = numVertices / 4 ) ;
                DEBUG_ONLY( const size_t numQuadVerts  = numQuads * 4 ) ;
                ASSERT( numVertices == numQuadVerts ) ;
                primitiveType = D3DPT_POINTLIST ;   // NOTE: ALERT: Not quad!
            }
            break ;

        default:
            FAIL() ;
            break ;
        }

        HROK( g_pd3dDevice->DrawPrimitive( primitiveType , 0 , numPrims ) ) ;
    }
}



    } ;
} ;


#if defined( _DEBUG )

void PeGaSys_Render_D3D9_Mesh_UnitTest( void )
{
    DebugPrintf( "D3D9_Mesh::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::D3D9_Mesh d3d9_mesh( NULLPTR ) ;
        d3d9_mesh.Render() ;
    }

    DebugPrintf( "D3D9_Mesh::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif