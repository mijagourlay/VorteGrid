/** \file D3D9_IndexBuffer.cpp

    \brief Index buffer for Direct3D version 9

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#include "Render/Platform/DirectX9/D3D9_api.h" // for HROK

#include "Render/Platform/DirectX9/D3D9_IndexBuffer.h"

extern LPDIRECT3DDEVICE9   g_pd3dDevice ; // Direct3D rendering device

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

/** Construct index buffer for Direct3D version 9.
*/
D3D9_IndexBuffer::D3D9_IndexBuffer()
    : IndexBufferBase( sTypeId )
    , mInternalIndexBuffer( NULLPTR )
{
    ASSERT( GetTypeId() == D3D9_IndexBuffer::sTypeId ) ;
}




/** Destruct index buffer for Direct3D version 9.
*/
D3D9_IndexBuffer::~D3D9_IndexBuffer()
{
    Clear() ;
}




/** Allocate indices.
*/
/* virtual */ void D3D9_IndexBuffer::Allocate( size_t numIndices , IndexTypeE indexType )
{
    ASSERT( NULLPTR == mInternalIndexBuffer ) ;
    ASSERT( 0 == mNumIndices ) ;
    ASSERT( numIndices > 0 ) ;
    ASSERT( ( INDEX_TYPE_16 == indexType ) || ( INDEX_TYPE_32 == indexType ) ) ;

    const D3DFORMAT d3dFormat = ( INDEX_TYPE_16 == indexType ) ? D3DFMT_INDEX16 : D3DFMT_INDEX32 ;
    const size_t    indexSize = ( INDEX_TYPE_16 == indexType ) ? sizeof( WORD ) : sizeof( int ) ;

    mNumIndices = numIndices ;
    mIndexType = indexType ;

    const UINT   indexBufferSize = static_cast< UINT >( numIndices * indexSize ) ;

    if( FAILED( g_pd3dDevice->CreateIndexBuffer( indexBufferSize
                                                , D3DUSAGE_WRITEONLY
                                                , d3dFormat
                                                , D3DPOOL_DEFAULT
                                                , & mInternalIndexBuffer
                                                , NULL              ) ) )

    {   // creation failed
        FAIL() ; // crash will happen below
    }

}




void D3D9_IndexBuffer::Clear()
{
    if( mInternalIndexBuffer )
    {
        mInternalIndexBuffer->Release() ;
        mInternalIndexBuffer = NULLPTR ;
    }
}




/* virtual */ WORD *  D3D9_IndexBuffer::GetIndicesWord()
{
    ASSERT( GetIndexType() == INDEX_TYPE_16 ) ;

    // Get address of actual index data.
    WORD * pIndices = NULL ;
    if( FAILED( mInternalIndexBuffer->Lock( 0
        , 0
        , (void**) & pIndices
        , 0 // D3DLOCK flags
        ) ) )
    {
        FAIL() ;
    }

    ASSERT( pIndices != 0 ) ;

    return pIndices ;
}




/* virtual */ int *   D3D9_IndexBuffer::GetIndicesInt()
{
    ASSERT( GetIndexType() == INDEX_TYPE_32 ) ;

    // Get address of actual index data.
    int * pIndices = NULL ;
    if( FAILED( mInternalIndexBuffer->Lock( 0
        , 0
        , (void**) & pIndices
        , 0 // D3DLOCK flags
        ) ) )
    {
        FAIL() ;
    }

    ASSERT( pIndices != 0 ) ;

    return pIndices ;
}




/* virtual */ void D3D9_IndexBuffer::Unlock()
{
    HROK( mInternalIndexBuffer->Unlock() ) ;
}




    } ;
} ;


#if defined( _DEBUG )

void PeGaSys_Render_D3D9_IndexBuffer_UnitTest( void )
{
    DebugPrintf( "D3D9_IndexBuffer::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::D3D9_IndexBuffer d3D9_IndexBuffer ;
        d3D9_IndexBuffer.Allocate( 1 , PeGaSys::Render::IndexBufferBase::INDEX_TYPE_32 ) ;
        d3D9_IndexBuffer.Clear() ;
    }

    DebugPrintf( "D3D9_IndexBuffer::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif