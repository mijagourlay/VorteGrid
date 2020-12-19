/** \file D3D9_VertexBuffer.cpp

    \brief Vertex buffer for Direct3D version 9

    \author Copyright 2010 MJG; All rights reserved.
*/

#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#include "Render/Platform/DirectX9/D3D9_api.h" // for HROK

#include "Render/Platform/DirectX9/D3D9_VertexBuffer.h"

extern LPDIRECT3DDEVICE9   g_pd3dDevice ; // Direct3D rendering device

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

/** Construct vertex buffer for Direct3D version 9.
*/
D3D9_VertexBuffer::D3D9_VertexBuffer()
    : VertexBufferBase()
    , mVertexFormat( 0 )
    , mInternalVertexBuffer( NULLPTR )
    , mOffsetPx( 0 )
    , mOffsetTu( 0 )
    , mOffsetCr( 0 )
    , mOffsetNx( 0 )
{
    mTypeId = sTypeId ;
}




/** Destruct vertex buffer for Direct3D version 9.
*/
D3D9_VertexBuffer::~D3D9_VertexBuffer()
{
    Deallocate() ;
}




/** Swap all members between that and this.
*/
void D3D9_VertexBuffer::Swap( D3D9_VertexBuffer & that )
{
    D3D9_VertexBuffer temp ;
    memcpy( & temp ,   this , sizeof( that ) ) ;
    memcpy(   this , & that , sizeof( that ) ) ;
    memcpy( & that , & temp , sizeof( that ) ) ;
}




/** Get address of vertex data that this buffer object manages, and hold a lock on it (for the CPU, to prevent GPU from accessing it).
*/
void * D3D9_VertexBuffer::LockVertexData()
{
    if( mInternalVertexBuffer )
    {
        void * vertexData = NULLPTR ;
        HROK( mInternalVertexBuffer->Lock( 0 , 0 , & vertexData , 0 ) ) ;
        return vertexData ;
    }
    return NULLPTR ;
}




/** Unlock CPU's hold on vertex data to allow GPU to access it.
*/
void D3D9_VertexBuffer::UnlockVertexData()
{
    ASSERT( mInternalVertexBuffer ) ;
    HROK( mInternalVertexBuffer->Unlock() ) ;
}




        /** Get address of vertex element with the given semantic.
        */
        void * D3D9_VertexBuffer::GetElementStart( void * vertexData , PeGaSys::Render::VertexDeclaration::VertexElement::SemanticE semantic , size_t which )
        {
            unsigned char * vertexDataBytes = static_cast< unsigned char * >( vertexData ) ;
            ASSERT( 0 == which ) ; // Multiple elements with the same semantic not yet supported.
            NON_DEBUG_ONLY( UNUSED_PARAM( which ) ) ;

            FAIL() ; // Not yet implemented.  Need to assign offsets in DeclareVertexFormat before code below will yield correct value.

            switch( semantic )
            {
            case VertexDeclaration::VertexElement::POSITION     : return vertexDataBytes + mOffsetPx ; break ;
            case VertexDeclaration::VertexElement::NORMAL       : return vertexDataBytes + mOffsetNx ; break ;
            case VertexDeclaration::VertexElement::COLOR_AMBIENT: return vertexDataBytes + mOffsetCr ; break ;
            case VertexDeclaration::VertexElement::TEXTURE_0    : return vertexDataBytes + mOffsetTu ; break ;
            }
            FAIL() ; // Unsupported semantic.
            return NULLPTR ;
        }





/** Allocate memory for vertex buffer.

    \return true if succeeded, false otherwise
*/
bool D3D9_VertexBuffer::Allocate( size_t numVertices )
{
    ASSERT( NULLPTR == GetGenericVertexData() ) ;   // Not allowed to allocate over a generic vertex buffer.
    ASSERT( NULLPTR == mInternalVertexBuffer ) ;    // Not allowed to allocate over existing D3D vertex buffer.
    ASSERT( numVertices != 0 ) ;
    ASSERT( GetPopulation() == 0 ) ;
    ASSERT( GetVertexSizeInBytes() != 0 ) ;

    mInternalVertexBuffer = NULLPTR ;

    if( SUCCEEDED( g_pd3dDevice->CreateVertexBuffer(  static_cast< UINT >( GetVertexSizeInBytes() * numVertices )
                                                    , D3DUSAGE_WRITEONLY
                                                    , mVertexFormat
                                                    , D3DPOOL_DEFAULT
                                                    , & mInternalVertexBuffer
                                                    , NULL                  ) ) )
    {   // Allocation succeeded.
        SetCapacity( numVertices ) ;
        return true ; // Inform caller allocation succeeded.
    }
    FAIL() ;
    return false ; // Inform caller allocation failed.
}




/** Deallocate vertex buffer allocated by Allocate.
*/
void D3D9_VertexBuffer::Deallocate()
{
    if( mInternalVertexBuffer )
    {   // This object allocated vertex data via a vertex buffer which it also allocated.
        ASSERT( NULLPTR == GetGenericVertexData() ) ; // VB can't have both D3D and generic data.

        // Release the vertex buffer, which also releases the vertex data it contains.
        mInternalVertexBuffer->Release() ;

        mInternalVertexBuffer = NULLPTR ;
    }

    SetPopulation( 0 ) ;
    SetCapacity( 0 ) ;
}




/** Set vertex buffer format.
*/
void D3D9_VertexBuffer::DeclareVertexFormat( const VertexDeclaration & vertexDeclaration )
{
    ASSERT( GetVertexDeclaration().mVertexFormat == VertexDeclaration::VERTEX_FORMAT_NONE ) ;
    ASSERT( GetVertexSizeInBytes() == 0 ) ;

    // TODO: Assign vertex element offsets.

    // Establish format of vertex buffer, based on vertex declaration.
    const VertexDeclaration::VertexFormatE & tgtVertFmt = vertexDeclaration.mVertexFormat ;
    size_t vertexSize = 0 ;
    switch( tgtVertFmt )
    {
        case VertexDeclaration::VERTEX_FORMAT_NONE:
            FAIL() ;
        break ;

        case VertexDeclaration::POSITION:
            vertexSize = sizeof( VertexFormatPosition ) ;
            mVertexFormat = sVertexFormatFlags_Position ;
        break ;
            
        case VertexDeclaration::POSITION_NORMAL:
            vertexSize = sizeof( VertexFormatPositionNormal ) ;
            mVertexFormat = sVertexFormatFlags_PositionNormal ;
        break ;

        case VertexDeclaration::POSITION_COLOR:
            vertexSize = sizeof( VertexFormatPositionColor ) ;
            mVertexFormat = sVertexFormatFlags_PositionColor ;
        break ;

        case VertexDeclaration::POSITION_TEXTURE:
            vertexSize = sizeof( VertexFormatPositionTexture ) ;
            mVertexFormat = sVertexFormatFlags_PositionTexture ;
        break ;

        case VertexDeclaration::POSITION_NORMAL_COLOR:
            vertexSize = sizeof( VertexFormatPositionNormalColor ) ;
            mVertexFormat = sVertexFormatFlags_PositionNormalColor ;
        break ;

        case VertexDeclaration::POSITION_NORMAL_TEXTURE:
            vertexSize = sizeof( VertexFormatPositionNormalTexture ) ;
            mVertexFormat = sVertexFormatFlags_PositionNormalTexture ;
        break ;

        case VertexDeclaration::POSITION_COLOR_TEXTURE:
            vertexSize = sizeof( VertexFormatPositionColorTexture ) ;
            mVertexFormat = sVertexFormatFlags_PositionColorTexture ;
        break ;

        case VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE  :
            vertexSize = sizeof( VertexFormatPositionNormalColorTexture ) ;
            mVertexFormat = sVertexFormatFlags_PositionNormalColorTexture ;
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
bool D3D9_VertexBuffer::CreateVertexBuffer( const VertexDeclaration & vertexDeclaration , size_t numVertices )
{
    DeclareVertexFormat( vertexDeclaration ) ;
    return Allocate( numVertices ) ;
}




static inline char ColorByteFromFloat( float fColor0to1 )
{
    static const float almost256 = 256.0f * ( 1.0f - FLT_EPSILON ) ;
    unsigned int iColor0to255 = static_cast< unsigned int >( fColor0to1 * almost256 ) ;
    ASSERT( ( iColor0to255 >= 0 ) && ( iColor0to255 <= 255 ) ) ;
    return static_cast< unsigned char >( iColor0to255 ) ;
}




static inline unsigned ColorIntFromFloats( float fRed , float fGrn , float fBlu , float fAlp )
{
    unsigned int iRed = ColorByteFromFloat( fRed ) ;
    unsigned int iGrn = ColorByteFromFloat( fGrn ) ;
    unsigned int iBlu = ColorByteFromFloat( fBlu ) ;
    unsigned int iAlp = ColorByteFromFloat( fAlp ) ;
    const unsigned color = D3DCOLOR_RGBA( iRed , iGrn , iBlu , iAlp ) ;
    return color ;
}




/** Copy vertices from that (which must be generic) to this.
*/
void    D3D9_VertexBuffer::CopyVerticesFromGenericToPlatformSpecific( const D3D9_VertexBuffer & genericVertexBuffer )
{
    ASSERT( NULLPTR == GetGenericVertexData() ) ; // VB can't have both D3D and generic data.
    ASSERT( GetVertexDeclaration().mVertexFormat != VertexDeclaration::VERTEX_FORMAT_NONE ) ;
    ASSERT( GetCapacity() >= genericVertexBuffer.GetPopulation() ) ;
    ASSERT( GetVertexSizeInBytes() > 0 ) ;
    ASSERT( genericVertexBuffer.GetGenericVertexData() != NULLPTR ) ; // incoming VB must be generic.
    ASSERT( genericVertexBuffer.GetVertexDeclaration().mVertexFormat == VertexDeclaration::GENERIC ) ;
    ASSERT( genericVertexBuffer.GetVertexSizeInBytes() > 0 ) ;

    // Obtain lock (for CPU) on vertex buffer contents so GPU does not try to access it while this routine changes its contents.
    void *                  dstVertexData   = LockVertexData() ;

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
                dst[ idx ].color = ColorIntFromFloats( src[ idx ].cr , src[ idx ].cg , src[ idx ].cb , src[ idx ].ca ) ;
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

        case VertexDeclaration::POSITION_NORMAL_COLOR:
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
#           ifdef INCLUDE_SPECULAR
                dst[ idx ].specular =
#           endif
                dst[ idx ].diffuse  = ColorIntFromFloats( src[ idx ].cr , src[ idx ].cg , src[ idx ].cb , src[ idx ].ca ) ;
                //memset( & dst[idx] , 0 , sizeof( dst[idx] ) ) ;
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
            VertexFormatPositionColorTexture * dst = static_cast< VertexFormatPositionColorTexture * >( dstVertexData ) ;
           for( size_t idx = 0 ; idx < numVerts ; ++ idx )
            {
                dst[ idx ].px = src[ idx ].px ;
                dst[ idx ].py = src[ idx ].py ;
                dst[ idx ].pz = src[ idx ].pz ;
                dst[ idx ].color_argb = ColorIntFromFloats( src[ idx ].cr , src[ idx ].cg , src[ idx ].cb , src[ idx ].ca ) ;
                dst[ idx ].ts = src[ idx ].ts ;
                dst[ idx ].tt = src[ idx ].tt ;
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
                dst[ idx ].nx = src[ idx ].nx ;
                dst[ idx ].ny = src[ idx ].ny ;
                dst[ idx ].nz = src[ idx ].nz ;
                dst[ idx ].color_argb = ColorIntFromFloats( src[ idx ].cr , src[ idx ].cg , src[ idx ].cb , src[ idx ].ca ) ;
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




/* virtual */ void D3D9_VertexBuffer::TranslateFromGeneric( const VertexDeclaration & targetVertexDeclaration )
{
    ASSERT( GetGenericVertexData() != NULLPTR ) ; // Incoming VB must be generic.
    ASSERT( GetVertexDeclaration().mVertexFormat == VertexDeclaration::GENERIC ) ;
    ASSERT( GetPopulation() > 0 ) ;
    ASSERT( GetVertexSizeInBytes() > 0 ) ;

    // Save off original data because members of this will get overwritten.
    D3D9_VertexBuffer genericSource ;
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






    } ;
} ;


#if defined( _DEBUG )

void PeGaSys_Render_D3D9_VertexBuffer_UnitTest( void )
{
    DebugPrintf( "D3D9_VertexBuffer::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::D3D9_VertexBuffer d3D9_VertexBuffer ;
        PeGaSys::Render::VertexDeclaration targetVertexDeclaration ;
        d3D9_VertexBuffer.TranslateFromGeneric( targetVertexDeclaration ) ;
        d3D9_VertexBuffer.Clear() ;
    }

    DebugPrintf( "D3D9_VertexBuffer::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif