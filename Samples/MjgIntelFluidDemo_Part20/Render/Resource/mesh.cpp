/** \file mesh.cpp

    \brief Base class for geometry mesh

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/mesh.h"

#include "Render/Resource/material.h"
#include "Render/Resource/vertexBuffer.h"
#include "Render/Resource/indexBuffer.h"

#include "Render/Device/api.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>
#include <Core/Math/vec3.h>

namespace PeGaSys {
    namespace Render {

        // Types -----------------------------------------------------------------------

        // Which index type to use for MakeBox, MakeSphere.

#define USE_INDEX_16_FOR_SIMPLE_SHAPES 1

#if USE_INDEX_16_FOR_SIMPLE_SHAPES   // Use 16-bit integers for index buffer
        static const IndexBufferBase::IndexTypeE sIndexType = IndexBufferBase::INDEX_TYPE_16 ;
        typedef WORD IndexType ;
#   define GET_INDICES GetIndicesWord
#else   // Use 32-bit integers for index buffer
        static const IndexBufferBase::IndexTypeE sIndexType = IndexBufferBase::INDEX_TYPE_32 ;
        typedef int IndexType ;
#   define GET_INDICES GetIndicesInt
#endif

        // Private variables -----------------------------------------------------------
        // Public variables ------------------------------------------------------------
        // Private functions -----------------------------------------------------------
        // Public functions ------------------------------------------------------------

        /** Construct base part of geometry mesh.
        */
        MeshBase::MeshBase( ModelData * owningModelData )
            : mOwningModelData( owningModelData )
            , mTechnique( NULLPTR )
            , mVertexBuffer( NULLPTR )
            , mIndexBuffer( NULLPTR )
            , mPrimitiveType( PRIMITIVE_NONE )
        {
            PERF_BLOCK( MeshBase__MeshBase ) ;
        }




        /** Destruct base part of geometry mesh.
        */
        MeshBase::~MeshBase()
        {
            PERF_BLOCK( MeshBase__dtor ) ;

            delete mVertexBuffer ;
            delete mIndexBuffer ;
        }




        /** Release reference to this object and delete it if that was the last reference.
        */
        void MeshBase::ReleaseReference()
        {
            PERF_BLOCK( MeshBase__ReleaseReference ) ;

            if( _ReleaseReference() )
            {   // That was the last reference to this object.
                delete this ;
            }
        }




        /** Create a new vertex buffer for this mesh.
        */
        VertexBufferBase * MeshBase::NewVertexBuffer( ApiBase * renderApi )
        {
            PERF_BLOCK( MeshBase__NewVertexBuffer ) ;

            ASSERT( renderApi ) ;
            ASSERT( 0 == mVertexBuffer ) ;

            // Create vertex buffer
            mVertexBuffer = renderApi->NewVertexBuffer() ;

            return mVertexBuffer ;
        }




        void MeshBase::SetPrimitiveType( PrimitiveE primitiveType )
        {
            PERF_BLOCK( MeshBase__SetPrimitiveType ) ;

            ASSERT( ( PRIMITIVE_NONE == mPrimitiveType ) || ( primitiveType == mPrimitiveType ) ) ; // Not allowed to change format once set.
            mPrimitiveType = primitiveType ;
        }




        void ConvertGeometryWithIndicesToWithout( GenericVertex * dstVertBuf , const GenericVertex * srcVertBuf , const IndexType * indices , size_t numIndices )
        {
            PERF_BLOCK( Render__ConvertGeometryWithIndicesToWithout ) ;

            for( size_t dstIdx = 0 ; dstIdx < numIndices ; ++ dstIdx )
            {   // For each destination vertex (which corresponds to each source index)...
                dstVertBuf[ dstIdx ] = srcVertBuf[ indices[ dstIdx ] ] ;
            }
        }




        /** Make a box-shaped polygon mesh.
        */
        void MeshBase::MakeBox( ApiBase * renderApi , const Vec3 & dimensions , VertexDeclaration::VertexFormatE vertexFormat , PrimitiveE primitiveType , bool useIndices )
        {
            PERF_BLOCK( MeshBase__MakeBox ) ;

            ASSERT( renderApi ) ;
            ASSERT( dimensions.x >= 0.0f ) ;
            ASSERT( dimensions.y >= 0.0f ) ;
            ASSERT( dimensions.z >= 0.0f ) ;
            ASSERT( ! IsNan( dimensions ) ) ;
            ASSERT( ! IsInf( dimensions ) ) ;
            ASSERT( NULLPTR == mVertexBuffer ) ;
            ASSERT( NULLPTR == mIndexBuffer ) ;
            ASSERT( ( PRIMITIVE_TRIANGLES == primitiveType ) || ( PRIMITIVE_QUADS == primitiveType ) ) ;

            mPrimitiveType = primitiveType ;

            // Create and populate index buffer.
            // ---------------------------------

            static const size_t numIndices = ( PRIMITIVE_TRIANGLES == mPrimitiveType ) ? ( 12 * 3 ) : ( 6 * 4 ) ;
            mIndexBuffer = renderApi->NewIndexBuffer() ;
            mIndexBuffer->Allocate( numIndices , sIndexType ) ;

            // TODO: FIXME: Lock index buffer.
            IndexType * indices = mIndexBuffer->GET_INDICES() ;

            if( PRIMITIVE_TRIANGLES == mPrimitiveType )
            {   // Assign triangles.
                indices[  0 ] = 0 ; indices[  1 ] = 2 ; indices[  2 ] = 1 ;
                indices[  3 ] = 1 ; indices[  4 ] = 2 ; indices[  5 ] = 3 ;

                indices[  6 ] = 2 ; indices[  7 ] = 6 ; indices[  8 ] = 3 ;
                indices[  9 ] = 3 ; indices[ 10 ] = 6 ; indices[ 11 ] = 7 ;

                indices[ 12 ] = 6 ; indices[ 13 ] = 4 ; indices[ 14 ] = 7 ;
                indices[ 15 ] = 4 ; indices[ 16 ] = 5 ; indices[ 17 ] = 7 ;

                indices[ 18 ] = 0 ; indices[ 19 ] = 5 ; indices[ 20 ] = 4 ;
                indices[ 21 ] = 0 ; indices[ 22 ] = 1 ; indices[ 23 ] = 5 ;

                indices[ 24 ] = 0 ; indices[ 25 ] = 4 ; indices[ 26 ] = 2 ;
                indices[ 27 ] = 2 ; indices[ 28 ] = 4 ; indices[ 29 ] = 6 ;

                indices[ 30 ] = 1 ; indices[ 31 ] = 3 ; indices[ 32 ] = 5 ;
                indices[ 33 ] = 3 ; indices[ 34 ] = 7 ; indices[ 35 ] = 5 ;
            }
            else
            {   // Assign quads
                ASSERT( PRIMITIVE_QUADS == mPrimitiveType ) ;

                indices[  0 ] = 0 ; indices[  1 ] = 4 ; indices[  2 ] = 6 ; indices[  3 ] = 2 ;
                indices[  4 ] = 1 ; indices[  5 ] = 3 ; indices[  6 ] = 7 ; indices[  7 ] = 5 ;

                indices[  8 ] = 0 ; indices[  9 ] = 1 ; indices[ 10 ] = 5 ; indices[ 11 ] = 4 ;
                indices[ 12 ] = 2 ; indices[ 13 ] = 6 ; indices[ 14 ] = 7 ; indices[ 15 ] = 3 ;

                indices[ 16 ] = 0 ; indices[ 17 ] = 2 ; indices[ 18 ] = 3 ; indices[ 19 ] = 1 ;
                indices[ 20 ] = 4 ; indices[ 21 ] = 5 ; indices[ 22 ] = 7 ; indices[ 23 ] = 6 ;
            }

#       undef GET_INDICES

            mIndexBuffer->Unlock() ;

            // Create and populate vertices.
            // -----------------------------

            NewVertexBuffer( renderApi ) ;

            // When useIndices, original vertex buffer is the final one.
            // Otherwise, original vertex buffer is over-allocated so that the last 8 entries are the "source" vertices, and the first numIndices elements will get filled in later by ConvertGeometryWithIndicesToWithout.  (The last 8 will go unused.)
            const size_t numSrcVerts = 8 ;
            const size_t numDstVerts = numSrcVerts + ( useIndices ? 0 : numIndices ) ;
            GenericVertex * genericVertexBuffer = mVertexBuffer->AllocateGeneric( numDstVerts ) ;
            GenericVertex * verts = useIndices ? genericVertexBuffer : & genericVertexBuffer[ numIndices ] ;

            const float & dimx = dimensions.x ;
            const float & dimy = dimensions.y ;
            const float & dimz = dimensions.z ;

            {   // Assign vertex positions.
                verts[ 0 ].px = 0.0f ; verts[ 0 ].py = 0.0f ; verts[ 0 ].pz = 0.0f ; verts[ 0 ].pw = 1.0f ;
                verts[ 1 ].px = dimx ; verts[ 1 ].py = 0.0f ; verts[ 1 ].pz = 0.0f ; verts[ 1 ].pw = 1.0f ;
                verts[ 2 ].px = 0.0f ; verts[ 2 ].py = dimy ; verts[ 2 ].pz = 0.0f ; verts[ 2 ].pw = 1.0f ;
                verts[ 3 ].px = dimx ; verts[ 3 ].py = dimy ; verts[ 3 ].pz = 0.0f ; verts[ 3 ].pw = 1.0f ;

                verts[ 4 ].px = 0.0f ; verts[ 4 ].py = 0.0f ; verts[ 4 ].pz = dimz ; verts[ 4 ].pw = 1.0f ;
                verts[ 5 ].px = dimx ; verts[ 5 ].py = 0.0f ; verts[ 5 ].pz = dimz ; verts[ 5 ].pw = 1.0f ;
                verts[ 6 ].px = 0.0f ; verts[ 6 ].py = dimy ; verts[ 6 ].pz = dimz ; verts[ 6 ].pw = 1.0f ;
                verts[ 7 ].px = dimx ; verts[ 7 ].py = dimy ; verts[ 7 ].pz = dimz ; verts[ 7 ].pw = 1.0f ;

                // The 4D texture coordinates are 3D (ts,tt,tu) plus the homogeneous coordinate (tv), which is set to 1, as for positions.
                // The 3rd dimension here (tu) is unused and its value does not matter.
                verts[ 0 ].ts = 0.00f ; verts[ 0 ].tt = 0.0f ; verts[ 0 ].tu = 0.0f ; verts[ 0 ].tv = 1.0f ;
                verts[ 1 ].ts = 0.25f ; verts[ 1 ].tt = 0.0f ; verts[ 1 ].tu = 0.0f ; verts[ 1 ].tv = 1.0f ;
                verts[ 2 ].ts = 0.50f ; verts[ 2 ].tt = 0.0f ; verts[ 2 ].tu = 0.0f ; verts[ 2 ].tv = 1.0f ;
                verts[ 3 ].ts = 0.75f ; verts[ 3 ].tt = 0.0f ; verts[ 3 ].tu = 0.0f ; verts[ 3 ].tv = 1.0f ;

                verts[ 4 ].ts = 0.00f ; verts[ 4 ].tt = 1.0f ; verts[ 4 ].tu = 0.0f ; verts[ 4 ].tv = 1.0f ;
                verts[ 5 ].ts = 0.25f ; verts[ 5 ].tt = 1.0f ; verts[ 5 ].tu = 0.0f ; verts[ 5 ].tv = 1.0f ;
                verts[ 6 ].ts = 0.50f ; verts[ 6 ].tt = 1.0f ; verts[ 6 ].tu = 0.0f ; verts[ 6 ].tv = 1.0f ;
                verts[ 7 ].ts = 0.75f ; verts[ 7 ].tt = 1.0f ; verts[ 7 ].tu = 0.0f ; verts[ 7 ].tv = 1.0f ;
            }

            for( size_t idx = 0 ; idx < numSrcVerts ; ++ idx )
            {   // For each vertex...
                // Assign color based on position.
                verts[ idx ].cr = verts[ idx ].px ? 1.0f : 0.0f ;
                verts[ idx ].cg = verts[ idx ].py ? 1.0f : 0.0f ;
                verts[ idx ].cb = verts[ idx ].pz ? 1.0f : 0.0f ;
                verts[ idx ].ca = 1.0f ;
                // Translate box to be centered about origin
                verts[ idx ].px -= 0.5f * dimx ;
                verts[ idx ].py -= 0.5f * dimy ;
                verts[ idx ].pz -= 0.5f * dimz ;
                // Assign normal based on position.
                Vec3 normal = Vec3( verts[ idx ].px , verts[ idx ].py , verts[ idx ].pz ) ;
                normal.Normalize() ;
                verts[ idx ].nx = normal.x ;
                verts[ idx ].ny = normal.y ;
                verts[ idx ].nz = normal.z ;
            }

            if( useIndices )
            {   // Use index buffer for final output.
                mVertexBuffer->SetPopulation( numSrcVerts ) ;
            }
            else
            {   // Do not use index buffer for final output.
                ConvertGeometryWithIndicesToWithout( genericVertexBuffer , verts , indices , numIndices ) ;
                mVertexBuffer->SetPopulation( numIndices ) ;
                mIndexBuffer->Clear() ;
                renderApi->DeleteIndexBuffer( mIndexBuffer ) ;
                mIndexBuffer = NULLPTR ;
            }
            mVertexBuffer->TranslateFromGeneric( VertexDeclaration( vertexFormat ) ) ;
        }




        /** Make a sphere-shaped polygon mesh.
        */
        void MeshBase::MakeSphere( ApiBase * renderApi , float radius , int numLongitudinalSegments , int numLatitudinalSegment , VertexDeclaration::VertexFormatE vertexFormat )
        {
            /** An alternative procedure to make a spherical mesh could be to make any
                convex polyhedron, center it on the origin, then project its vertices to the unit sphere.
                With such a procedure, beginning with a tesselated cube, it could make texture-mapping more straightforward.
            */

            PERF_BLOCK( MeshBase__MakeSphere ) ;

            mPrimitiveType = PRIMITIVE_TRIANGLES ;

            // Create and populate vertices.

            const int & width   = numLatitudinalSegment ;
            const int & height  = numLongitudinalSegments ;

            const int   numVerts    = (height-2)* width+2;

            mVertexBuffer   = renderApi->NewVertexBuffer() ;
            GenericVertex * verts = mVertexBuffer->AllocateGeneric( numVerts ) ;
            mVertexBuffer->SetPopulation( numVerts ) ;

            const float oneOverPi    = 1.0f / PI ;
            const float oneOverTwoPi = 1.0f / TWO_PI ;
            int vertIdx , j ;
            for( vertIdx = 0 , j = 1 ; j < height - 1 ; j ++ )
            {
                const float theta     = float(j) / (height-1) * PI   ;
                const float rSinTheta = radius * sinf( theta ) ;
                const float rCosTheta = radius * cosf( theta ) ;
                for( int i = 0 ; i < width ; i ++ )
                {
                    const float phi = float(i) / (width -1) * TWO_PI ;
                    verts[ vertIdx ].px =   rSinTheta * cosf(phi) ;
                    verts[ vertIdx ].py =   rCosTheta ;
                    verts[ vertIdx ].pz = - rSinTheta * sinf(phi) ;
                    verts[ vertIdx ].pw = 1.0f ;

                    // The 4D texture coordinates are 3D (ts,tt,tu) plus the homogeneous coordinate (tv), which is set to 1, as for positions.
                    // The 3rd dimension here (tu) is unused and its value does not matter.
                    verts[ vertIdx ].ts = phi * oneOverTwoPi ;
                    verts[ vertIdx ].tt = theta * oneOverPi ;
                    ASSERT( verts[ vertIdx ].ts >= 0.0f ) ;
                    ASSERT( verts[ vertIdx ].ts <= 1.0f ) ;
                    ASSERT( verts[ vertIdx ].tt >= 0.0f ) ;
                    ASSERT( verts[ vertIdx ].tt <= 1.0f ) ;
                    verts[ vertIdx ].tu = 0.0f ;
                    verts[ vertIdx ].tv = 1.0f ;

                    ++ vertIdx ;
                }
            }
            // Assign polar vertices.
            verts[ vertIdx ].px =  0;
            verts[ vertIdx ].py =  radius ;
            verts[ vertIdx ].pz =  0;
            verts[ vertIdx ].pw = 1.0f ;

            verts[ vertIdx ].ts = 0.5f ;   // Choose middle of texture since this pole will neighbor all adjacent vertices; this will yield symmetric interpolation.
            verts[ vertIdx ].tt = 0.005f ; // Avoid 0.0 as a hack to avoid sampling outside texture.
            verts[ vertIdx ].tu = 0.0f ;
            verts[ vertIdx ].tv = 1.0f ;

            ++ vertIdx ;

            verts[ vertIdx ].px =  0;
            verts[ vertIdx ].py = -radius ;
            verts[ vertIdx ].pz =  0;
            verts[ vertIdx ].pw = 1.0f ;

            verts[ vertIdx ].ts = 0.5f ;   // Choose middle of texture since this pole will neighbor all adjacent vertices; this will yield symmetric interpolation.
            verts[ vertIdx ].tt = 0.995f ; // Avoid 1.0 as a hack to avoid sampling outside texture.
            verts[ vertIdx ].tu = 0.0f ;
            verts[ vertIdx ].tv = 1.0f ;

            ASSERT( vertIdx == numVerts - 1 ) ;

            for( int idx = 0 ; idx < numVerts ; ++ idx )
            {   // For each vertex...
                // Assign color based on position.
                verts[ idx ].cr = ( verts[ idx ].px + radius ) * 0.5f / radius ;
                verts[ idx ].cg = ( verts[ idx ].py + radius ) * 0.5f / radius ;
                verts[ idx ].cb = ( verts[ idx ].pz + radius ) * 0.5f / radius ;
                verts[ idx ].ca = 1.0f ;
                ASSERT( verts[ idx ].cr >= 0.0f && verts[ idx ].cr <= 1.0f ) ;
                ASSERT( verts[ idx ].cg >= 0.0f && verts[ idx ].cg <= 1.0f ) ;
                ASSERT( verts[ idx ].cb >= 0.0f && verts[ idx ].cb <= 1.0f ) ;
                // Assign normal based on position.
                Vec3 normal = Vec3( verts[ idx ].px , verts[ idx ].py , verts[ idx ].pz ) ;
                normal.Normalize() ;
                verts[ idx ].nx = normal.x ;
                verts[ idx ].ny = normal.y ;
                verts[ idx ].nz = normal.z ;
            }

            mVertexBuffer->TranslateFromGeneric( VertexDeclaration( vertexFormat ) ) ;

            // Create and populate index buffer (triangles).
            const int   numTriangles    = (height-2)*(width-1)*2;

#if 1   // Use 16-bit integers for index buffer
            static const IndexBufferBase::IndexTypeE indexType = IndexBufferBase::INDEX_TYPE_16 ;
            typedef WORD IndexType ;
#   define GET_INDICES GetIndicesWord
            ASSERT( numTriangles * 3 < 65535 ) ;
#else   // Use 32-bit integers for index buffer
            static const IndexBufferBase::IndexTypeE indexType = IndexBufferBase::INDEX_TYPE_32 ;
            typedef int IndexType ;
#   define GET_INDICES GetIndicesInt
#endif

            mIndexBuffer = renderApi->NewIndexBuffer() ;
            const size_t numIndices = numTriangles * 3 ; // 3 indices per triangle
            mIndexBuffer->Allocate( numIndices , indexType ) ;

            // TODO: FIXME: Lock index buffer.
            IndexType * indices = mIndexBuffer->GET_INDICES() ;

            size_t idxIdx = 0 ;
            for( int j=0; j<height-3; j++ )
            {   // For each triangle not near the poles...
                for( int i=0; i<width-1; i++ )
                {
                    indices[ idxIdx ++ ] = static_cast< IndexType >( (j  )*width + i   ) ;
                    indices[ idxIdx ++ ] = static_cast< IndexType >( (j+1)*width + i+1 ) ;
                    indices[ idxIdx ++ ] = static_cast< IndexType >( (j  )*width + i+1 ) ;
                    indices[ idxIdx ++ ] = static_cast< IndexType >( (j  )*width + i   ) ;
                    indices[ idxIdx ++ ] = static_cast< IndexType >( (j+1)*width + i   ) ;
                    indices[ idxIdx ++ ] = static_cast< IndexType >( (j+1)*width + i+1 ) ;
                }
            }
            for( int i = 0 ; i < width - 1 ; i ++ )
            {   // For each triangle near the poles...
                indices[ idxIdx ++ ] = static_cast< IndexType >( (height-2)*width        ) ;
                indices[ idxIdx ++ ] = static_cast< IndexType >( i                       ) ;
                indices[ idxIdx ++ ] = static_cast< IndexType >( i+1                     ) ;
                indices[ idxIdx ++ ] = static_cast< IndexType >( (height-2)*width+1      ) ;
                indices[ idxIdx ++ ] = static_cast< IndexType >( (height-3)*width + i+1  ) ;
                indices[ idxIdx ++ ] = static_cast< IndexType >( (height-3)*width + i    ) ;
            }

            ASSERT( idxIdx == numIndices ) ;

            for( int triIdx = 0 ; triIdx < numTriangles ; ++ triIdx )
            {
                ASSERT( indices[ triIdx * 3 + 0 ] < numVerts ) ;
                ASSERT( indices[ triIdx * 3 + 1 ] < numVerts ) ;
                ASSERT( indices[ triIdx * 3 + 2 ] < numVerts ) ;
            }

            mIndexBuffer->Unlock() ;
        }



    } ;
} ;


#if defined( _DEBUG )

class TestMesh : public PeGaSys::Render::MeshBase
{
public:
    TestMesh( PeGaSys::Render::ModelData * owningModelData )
        : MeshBase( owningModelData )
    {
    }

    ~TestMesh() {}

    virtual bool Update()
    {
        return true ;
    }

    virtual void Render() {}

private:
} ;

void PeGaSys_Render_Mesh_UnitTest()
{
    DebugPrintf( "MeshBase::UnitTest ----------------------------------------------\n" ) ;

    {
        TestMesh mesh( NULLPTR ) ;

        mesh.MakeBox( 0 , Vec3( 2.0f , 3.0f , 5.0f ) , PeGaSys::Render::VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE ) ;
        mesh.Render() ;
    }
    {
        TestMesh mesh( NULLPTR ) ;

        mesh.MakeSphere( 0 , 1.0f , 8 , 16 , PeGaSys::Render::VertexDeclaration::POSITION_NORMAL_COLOR_TEXTURE ) ;
        mesh.Render() ;
    }

    DebugPrintf( "MeshBase::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif
