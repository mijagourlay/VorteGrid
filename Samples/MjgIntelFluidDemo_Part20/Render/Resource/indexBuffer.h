/** \file indexBuffer.h

    \brief Index buffer base class

    \author Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_INDEX_BUFFER_BASE_H
#define PEGASYS_RENDER_INDEX_BUFFER_BASE_H

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Index buffer base class.

            Each platform should specialize this class
        */
        class IndexBufferBase
        {
            public:
                enum IndexTypeE
                {
                        INDEX_TYPE_NONE ///< Unassigned
                    ,   INDEX_TYPE_16   ///< 16-bit integer (a.k.a "word" or "short")
                    ,   INDEX_TYPE_32   ///< 32-bit integer
                } ;

                typedef unsigned short  WORD ;  ///< Abbreviation for unsigned short.

                typedef unsigned TypeId ;   ///< Identifier for simple run-time type checking

                IndexBufferBase( TypeId typeId ) ;
                virtual ~IndexBufferBase() ;

                /// Return type of the underlying implementation for this index buffer object.
                const TypeId & GetTypeId() const { return mTypeId ; }

                /// Return type of index values.
                IndexTypeE GetIndexType() const { return mIndexType ; }

                /// Return address of index data, as short integers.  Must call UnlockIndices afterwards.
                virtual WORD *  GetIndicesWord() = 0 ;

                /// Return address of index data, as integers.  Must call UnlockIndices afterwards.
                virtual int *   GetIndicesInt() = 0 ;

                /// Unlock indices obtained by GetIndices*()
                virtual void Unlock() = 0 ;

                /// Return number of elements this index buffer represents.
                const size_t &  GetNumIndices() const { return mNumIndices ; }

                /// Platform-specific routine to allocate index buffer.
                virtual void Allocate( size_t numIndices , IndexTypeE indexType ) = 0 ;

                /// Platform-specific routine to empty and release index buffer.
                virtual void Clear() = 0 ;

            protected:
                static const unsigned sTypeId = 'IXBF' ;    ///< Type identifier for index buffer

                IndexTypeE  mIndexType      ;   ///< Type of index data
                size_t      mNumIndices     ;   ///< Number of indices, i.e. number of elements in index buffer.

            private:
                TypeId      mTypeId         ;   ///< Type identifier
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
