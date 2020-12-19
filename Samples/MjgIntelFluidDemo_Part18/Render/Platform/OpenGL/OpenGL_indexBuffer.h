/** \file OpenGL_IndexBuffer.h

    \brief Index buffer for OpenGL

    \author Copyright 2010-2014 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_INDEX_BUFFER_H
#define PEGASYS_RENDER_OPENGL_INDEX_BUFFER_H

#include "Render/Resource/indexBuffer.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Index buffer for OpenGL.
        */
        class OpenGL_IndexBuffer : public IndexBufferBase
        {
            public:
                typedef WORD *  INDEX_BUFFER_POINTER_TYPE ;

                OpenGL_IndexBuffer() ;
                virtual ~OpenGL_IndexBuffer() ;

                virtual void    Allocate( size_t numIndices , IndexTypeE indexType ) ;
                virtual WORD *  GetIndicesWord() ;
                virtual int *   GetIndicesInt() ;
                const WORD *    GetIndicesWord() const { return static_cast< const WORD * >( mIndexData ) ; }
                const int *     GetIndicesInt() const { return static_cast< const int * >( mIndexData ) ; }
                virtual void    Unlock() ;
                virtual void    Clear() ;

                static const unsigned sTypeId = 'ibog' ;

            private:
                void *      mIndexData      ;   ///< Address of index data
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
