/** \file mesh.h

    \brief Geometry mesh base class.

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_MESH_H
#define PEGASYS_RENDER_MESH_H

#include "Core/Containers/intrusivePtr.h"

#include "Render/Resource/vertexBuffer.h"
#include "Render/Resource/technique.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

struct Vec3 ;

namespace PeGaSys
{

    namespace Render
    {
        // Forward declarations
        class   ApiBase             ;
        class   ModelData           ;
        class   Material            ;
        class   VertexBufferBase    ;
        class   IndexBufferBase     ;


        enum PrimitiveE
        {
            PRIMITIVE_NONE              ,
            PRIMITIVE_POINTS            ,
            PRIMITIVE_LINES             ,
            PRIMITIVE_TRIANGLES         ,
            PRIMITIVE_TRIANGLE_STRIP    ,
            PRIMITIVE_QUADS             ,
            PRIMITIVE_NUM
        } ;


        /** Geometry mesh base class.
        */
        class MeshBase : public RefCountedMixin
        {
            public:
                MeshBase( ModelData * owningModelData ) ;
                virtual ~MeshBase() ;

                /// Platform-specific method to render geometry mesh.
                virtual void Render() = 0 ;

                /// Set render technique (material) used to render this mesh.
                void SetTechnique( Technique * technique )
                {
                    mTechnique = technique ;
                }

                      Technique * GetTechnique()       { return mTechnique.Get() ; }
                const Technique * GetTechnique() const { return mTechnique.Get() ; }

                void ReleaseReference() ; // Used by IntrusivePtr (indirectly, through global ReleaseReference)

                void SetPrimitiveType( PrimitiveE primitiveType ) ;
                PrimitiveE GetPrimitiveType() const { return mPrimitiveType ; }

                VertexBufferBase * NewVertexBuffer( ApiBase * renderApi ) ;

                /// Return address of buffer for vertices that define this mesh.
                VertexBufferBase *      GetVertexBuffer()       { return mVertexBuffer ; }
                const VertexBufferBase *GetVertexBuffer() const { return mVertexBuffer ; }

                /// Return address of buffer for indices indicating edges between vertices in this mesh.
                IndexBufferBase *       GetIndexBuffer()        { return mIndexBuffer  ; }
                const IndexBufferBase * GetIndexBuffer() const  { return mIndexBuffer  ; }

                void MakeBox( ApiBase * renderApi , const Vec3 & dimensions , VertexDeclaration::VertexFormatE vertexFormat , PrimitiveE primitiveType = PRIMITIVE_TRIANGLES , bool useIndices = false ) ;
                void MakeSphere( ApiBase * renderApi , float radius , int numLatitudinalSegment , int numLongitudinalSegments , VertexDeclaration::VertexFormatE vertexFormat ) ;

            private:
                typedef IntrusivePtr< Technique > TechniquePtr ;

                ModelData           *   mOwningModelData    ;   ///< ModelData object that owns this mesh.
                TechniquePtr            mTechnique          ;   ///< Technique used to render this mesh.
                VertexBufferBase    *   mVertexBuffer       ;   ///< Vertex data.
                IndexBufferBase     *   mIndexBuffer        ;   ///< Index data.
                PrimitiveE              mPrimitiveType      ;   ///< Type of primitive shape this mesh uses.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

        /// Add reference to a MeshBase, for use by IntrusivePtr.
        inline void AddReference( MeshBase * meshBase )     { meshBase->AddReference() ; }

        /// Release reference to a MeshBase, for use by IntrusivePtr.
        inline void ReleaseReference( MeshBase * meshBase ) { meshBase->ReleaseReference() ; }

    } ;
} ;

#endif
