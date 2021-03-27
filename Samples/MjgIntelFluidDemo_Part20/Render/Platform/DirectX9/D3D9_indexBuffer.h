/** \file D3D9_IndexBuffer.h

    \brief Index buffer for Direct3D version 9

    \author Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_D3D9_INDEX_BUFFER_H
#define PEGASYS_RENDER_D3D9_INDEX_BUFFER_H

#include <d3d9.h>
#include <d3d9types.h>

#include "Render/Resource/indexBuffer.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Index buffer for Direct3D version 9.
        */
        class D3D9_IndexBuffer : public IndexBufferBase
        {
            private:

                typedef LPDIRECT3DINDEXBUFFER9  INDEX_BUFFER_POINTER_TYPE ;

            public:
                static const unsigned sTypeId = 'ibd9' ;

                D3D9_IndexBuffer() ;
                virtual ~D3D9_IndexBuffer() ;

                virtual void    Allocate( size_t numIndices , IndexTypeE indexType ) ;
                virtual WORD *  GetIndicesWord() ;
                virtual int *   GetIndicesInt() ;
                virtual void    Unlock() ;
                virtual void    Clear() ;

            private:
                friend class D3D9_Mesh ; // Grant access to GetInternalBuffer.

                // Only D3D9_Mesh should access GetInternalBuffer.
                      INDEX_BUFFER_POINTER_TYPE & GetInternalBuffer()       { return mInternalIndexBuffer ; }
                const INDEX_BUFFER_POINTER_TYPE & GetInternalBuffer() const { return mInternalIndexBuffer ; }

                INDEX_BUFFER_POINTER_TYPE mInternalIndexBuffer ;
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
