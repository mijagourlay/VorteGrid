/** \file OpenGL_IndexBuffer.cpp

    \brief Index buffer for OpenGL

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Platform/OpenGL/OpenGL_IndexBuffer.h"

#include <Core/Performance/perfBLock.h>
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

        /** Construct index buffer for OpenGL.
        */
        OpenGL_IndexBuffer::OpenGL_IndexBuffer()
            : IndexBufferBase( sTypeId )
            , mIndexData( NULLPTR )
        {
            PERF_BLOCK( OpenGL_IndexBuffer__OpenGL_IndexBuffer ) ;

            ASSERT( GetTypeId() == OpenGL_IndexBuffer::sTypeId ) ;
        }




        /** Destruct index buffer for OpenGL.
        */
        OpenGL_IndexBuffer::~OpenGL_IndexBuffer()
        {
            PERF_BLOCK( OpenGL_IndexBuffer__dtor ) ;

            Clear() ;
        }




        /** Allocate indices.
        */
        /* virtual */ void OpenGL_IndexBuffer::Allocate( size_t numIndices , IndexTypeE indexType )
        {
            PERF_BLOCK( OpenGL_IndexBuffer__Allocate ) ;

            ASSERT( NULLPTR == mIndexData ) ;
            ASSERT( 0 == mNumIndices ) ;
            ASSERT( numIndices > 0 ) ;
            ASSERT( ( INDEX_TYPE_16 == indexType ) || ( INDEX_TYPE_32 == indexType ) ) ;

            mNumIndices = numIndices ;
            mIndexType = indexType ;

            if( INDEX_TYPE_16 == indexType )
            {
                WORD * pIndices = NEW WORD[ numIndices ] ;
                ASSERT( pIndices != 0 ) ;
                mIndexData = pIndices ;
            }
            else
            {
                ASSERT( INDEX_TYPE_32 == indexType ) ;
                int * pIndices = NEW int[ numIndices ] ;
                ASSERT( pIndices != 0 ) ;
                mIndexData = pIndices ;
            }
        }




        void    OpenGL_IndexBuffer::Clear()
        {
            PERF_BLOCK( OpenGL_IndexBuffer__Clear ) ;

            delete GetIndicesWord() ;
            mIndexData = NULLPTR ;
        }




        /* virtual */ WORD *  OpenGL_IndexBuffer::GetIndicesWord()
        {
            PERF_BLOCK( OpenGL_IndexBuffer__GetIndicesWord ) ;

            ASSERT( GetIndexType() == INDEX_TYPE_16 ) ;
            return static_cast< WORD * >( mIndexData ) ;
        }




        /* virtual */ int *   OpenGL_IndexBuffer::GetIndicesInt()
        {
            PERF_BLOCK( OpenGL_IndexBuffer__GetIndicesInt ) ;

            ASSERT( GetIndexType() == INDEX_TYPE_32 ) ;
            return static_cast< int * >( mIndexData ) ;
        }




        /* virtual */ void OpenGL_IndexBuffer::Unlock()
        {
            PERF_BLOCK( OpenGL_IndexBuffer__Unlock ) ;
        }


    } ;
} ;


#if defined( _DEBUG )

void PeGaSys_Render_OpenGL_IndexBuffer_UnitTest()
{
    DebugPrintf( "OpenGL_IndexBuffer::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::OpenGL_IndexBuffer openGL_IndexBuffer ;
        openGL_IndexBuffer.Allocate( 1 , PeGaSys::Render::IndexBufferBase::INDEX_TYPE_32 ) ;
        openGL_IndexBuffer.Clear() ;
    }

    DebugPrintf( "OpenGL_IndexBuffer::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif