/** \file vertexBuffer.cpp

    \brief Vertex buffer base class

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/vertexBuffer.h"

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

        /** Construct vertex buffer base class members.
        */
        VertexBufferBase::VertexBufferBase()
            : mTypeId( sTypeId )
            , mGenericVertexData( NULLPTR )
            , mVertexDeclaration()
            , mVertexSize( 0 )
            , mPopulation( 0 )
            , mCapacity( 0 )
        {
            PERF_BLOCK( VertexBufferBase__VertexBufferBase ) ;
        }




        /** Destruct vertex buffer base class members.
        */
        VertexBufferBase::~VertexBufferBase()
        {
            PERF_BLOCK( VertexBufferBase__dtor ) ;

            delete mGenericVertexData ;
        }




        /** Allocate vertices with generic intermediate format.
        */
        GenericVertex * VertexBufferBase::AllocateGeneric( size_t numVertices )
        {
            PERF_BLOCK( VertexBufferBase__AllocateGeneric ) ;

            ASSERT( NULLPTR == mGenericVertexData ) ;
            ASSERT( VertexDeclaration::VERTEX_FORMAT_NONE == mVertexDeclaration.mVertexFormat ) ;
            ASSERT( 0 == GetVertexSizeInBytes() ) ;
            ASSERT( 0 == GetPopulation() ) ;
            ASSERT( 0 == GetCapacity() ) ;
            ASSERT( numVertices > 0 ) ;

            SetCapacity( numVertices ) ;

            mGenericVertexData = NEW GenericVertex[ numVertices ] ;
            ASSERT( mGenericVertexData != 0 ) ;

            SetVertexDeclaration( VertexDeclaration( VertexDeclaration::GENERIC ) ) ;
            SetVertexSize( sizeof( GenericVertex ) ) ;

            return mGenericVertexData ;
        }




        /** Change capacity of this vertex buffer.

            This empties the current contents of the vertex buffer and reallocates an empty buffer of the given capacity.

            \note   This can block if the GPU currently holds a lock on the resources associated with this vertex buffer.
        */
        bool VertexBufferBase::ChangeCapacityAndReallocate( size_t numVertices )
        {
            PERF_BLOCK( VertexBufferBase__ChangeCapacityAndReallocate ) ;

            ASSERT( numVertices > GetCapacity() ) ; // No good reason to call this if capacity is not increasing.  But technically this is not a required precondition.
            Deallocate() ;
            return Allocate( numVertices ) ;
        }




        /// Set object that declares format for vertices in this buffer.
        void    VertexBufferBase::SetVertexDeclaration( const VertexDeclaration & vertexDeclaration )
        {
            PERF_BLOCK( VertexBufferBase__SetVertexDeclaration ) ;

            ASSERT( mVertexDeclaration.IsInvalid() ) ;  // Not allowed to change vertex format after it was set.
            mVertexDeclaration = vertexDeclaration ;
        }




        /// Set size, in bytes, each vertex occupies.
        void    VertexBufferBase::SetVertexSize( size_t vertexSize )
        {
            PERF_BLOCK( VertexBufferBase__SetVertexSize ) ;

            ASSERT( ( 0 == mVertexSize ) || ( 0 == vertexSize ) ) ;    // Not allowed to change vertex size; it is determined by vertex declaration anyway.
            mVertexSize = vertexSize ;
        }




        /// Set number of vertices in this buffer.
        void    VertexBufferBase::SetPopulation( size_t numVertices )
        {
            PERF_BLOCK( VertexBufferBase__SetPopulation ) ;

            ASSERT( numVertices <= GetCapacity() ) ;
            mPopulation = numVertices ;
        }




        /// Set number of vertices this buffer can hold.
        void    VertexBufferBase::SetCapacity( size_t capacity )
        {
            PERF_BLOCK( VertexBufferBase__SetCapacity ) ;

            ASSERT( ( 0 == mCapacity ) || ( 0 == capacity ) ) ;  // Not allowed to change capacity without reallocating vertex buffer.
            mCapacity = capacity ;
        }


    } ;
} ;


#if defined( _DEBUG )

class TestVertexBuffer : public PeGaSys::Render::VertexBufferBase
{
public:
    virtual void    TranslateFromGeneric( const PeGaSys::Render::VertexDeclaration & /*targetVertexDeclaration*/ ) {}
    virtual void    Clear() {}
    virtual bool    Allocate( size_t ) { return false ; }
    virtual void    Deallocate() {}
    virtual void *  LockVertexData() { return NULLPTR ; }
    virtual void    UnlockVertexData() {}
    virtual void *  GetElementStart( void * vertexData , PeGaSys::Render::VertexDeclaration::VertexElement::SemanticE /*semantic*/ , size_t /*which*/ ) { return vertexData ; }
    virtual void    DeclareVertexFormat( const PeGaSys::Render::VertexDeclaration & ) {}
} ;

void PeGaSys_Render_VertexBuffer_UnitTest( void )
{
    DebugPrintf( "VertexBufferBase::UnitTest ----------------------------------------------\n" ) ;

    {
        TestVertexBuffer vertexBuffer ;
        PeGaSys::Render::VertexDeclaration targetVertexDeclaration ;
        vertexBuffer.TranslateFromGeneric( targetVertexDeclaration ) ;
        vertexBuffer.Clear() ;
        vertexBuffer.Allocate( 0 ) ;
        vertexBuffer.Deallocate() ;
        vertexBuffer.DeclareVertexFormat( targetVertexDeclaration ) ;
        vertexBuffer.LockVertexData() ;
        vertexBuffer.UnlockVertexData() ;
        vertexBuffer.GetElementStart( NULLPTR , PeGaSys::Render::VertexDeclaration::VertexElement::POSITION , 0 ) ;
    }

    DebugPrintf( "VertexBufferBase::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif