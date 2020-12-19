/** \file indexBuffer.cpp

    \brief Index buffer base class

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/indexBuffer.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct index buffer base class members.
        */
        IndexBufferBase::IndexBufferBase( TypeId typeId )
            : mTypeId( typeId )
            , mNumIndices( 0 )
            , mIndexType( INDEX_TYPE_NONE )
        {
            PERF_BLOCK( IndexBufferBase__IndexBufferBase ) ;
        }




        /** Destruct index buffer base class members.
        */
        IndexBufferBase::~IndexBufferBase()
        {
            PERF_BLOCK( IndexBufferBase__dtor ) ;
        }




    } ;
} ;


#if defined( _DEBUG )

class TestIndexBuffer : public PeGaSys::Render::IndexBufferBase
{
public:
    static const TypeId sTypeId = 'TsIb' ;
    TestIndexBuffer()
        : IndexBufferBase( sTypeId )
    {
        ASSERT( GetTypeId() == TestIndexBuffer::sTypeId ) ;
    }

    virtual void    Allocate( size_t /*numIndices*/ , PeGaSys::Render::IndexBufferBase::IndexTypeE /*indexType*/ ) {}
    virtual WORD *  GetIndicesWord() { return 0 ; }
    virtual int *   GetIndicesInt() { return 0 ; }
    virtual void    Unlock() {}
    virtual void    Clear() {}
} ;

void PeGaSys_Render_IndexBuffer_UnitTest( void )
{
    DebugPrintf( "IndexBufferBase::UnitTest ----------------------------------------------\n" ) ;

    {
        TestIndexBuffer indexBuffer ;
        indexBuffer.Allocate( 1 , PeGaSys::Render::IndexBufferBase::INDEX_TYPE_32 ) ;
        indexBuffer.Clear() ;
    }

    DebugPrintf( "IndexBufferBase::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif