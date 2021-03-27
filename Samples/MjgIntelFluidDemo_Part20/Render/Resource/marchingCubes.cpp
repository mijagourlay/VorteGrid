//#pragma optimize( "" , off )
/** \file marchingCubes.cpp

    \brief Class to generate mesh from grid of values

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/marchingCubes.h"

#include "Render/Resource/material.h"
#include "Render/Resource/vertexBuffer.h"
#include "Render/Resource/indexBuffer.h"
#include "Render/Resource/mesh.h"

#include "Render/Device/api.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>
#include <Core/Math/vec3.h>

#if USE_TBB
static TbbAtomicBool sDebugPrintfLock ;
#endif

namespace PeGaSys {
    namespace Render {

// Macros ----------------------------------------------------------------------

#ifndef ABS
#   define ABS(x) ( ((x) >= 0) ? (x) : (-(x)) )
#endif

// Types -----------------------------------------------------------------------

        // TODO: Remove GridCell.  It is too generic and filling this struct probably wastes time due to redundancy and additional memory copies.
        typedef struct {
            Vec3  p[8]   ; /// Position of each grid point on the corners of a cell.
            float val[8] ; /// Values at each grid point.
        } GridCell;


        /** Class to allocate blocks of vertices from within a "mother" vertex buffer, which is used as a memory pool.

            This class wraps an existing vertex buffer whose *memory* was already allocated
            (by the runtime system) to have a given capacity.  It is meant to facilitate
            multiple threads filling that pool with minimal contention.

            The use of the term "allocate" here is has two different meanings, depending on
            context:

                -   Using the runtime system, i.e. the underlying systems API allocates memory
                    for the pool (i.e. the mother vertex buffer). That happens elsewhere, and
                    must have already happened before instantiating an object of this class.

                -   Using this per-block allocator, which allocates fixed-size blocks from the
                    pool, then allocated individual vertices within those blocks
                    (actually, 3 contiguous vertices at a time, i.e. 1 triangle at a time).

            The mother vertex buffer is used as a linear pool, meaning blocks are allocated linearly,
            from the start, and never deallocated.  The intention is that each thread will have its
            own block, at any given moment.  The allocation of blocks is the only potential point
            of contention, and is synchronized using a single atomic integer, shared across all threads.
            That integer is the number of vertices "allocated" in the sense that the are associated
            with a block.

            For each block, there is another counter, which is the number of vertices allocated
            within the block.  That counter is local to each thread, therefore requires no synchronization.

            Since vertices are allocated in blocks, it is possible (and likely) that at the end of the
            procedure, each thread will have allocated a block but not allocated all vertices within the block.
            This implies the mother vertex buffer will have "bubbles" of unallocated vertices in it.
            These are harmless, as long as they are neither referenced by any index buffer, nor belong
            to any triangle with non-zero size.

            The main problem with such unused vertices is that they waste memory.

            Ideally, the block size (i.e. number of vertices per block) should be tuned to balance that
            waste with speed of allocation, which in turn implies reducing contention.  On one extreme,
            a block could be a single triangle (i.e. 3 vertices). But that would impose the maximum amount
            of contention since each vertex allocation would require an atomic operation on the shared vertex
            counter in the pool.  The other extreme would be to have a block size equal the pool
            capacity divided by the number of threads.  Then there would be only one atomic access per thread,
            total.  But this would likely yield some threads to have a lot of waste and lead other threads to
            exhast the capacity of their block.

            Another important factor is L1 cache friendliness.  Each thread will likely run on its own core,
            which will have its own L1 cache.  If the vertices belonging to different cores shared overlapped
            in cache lines, then each line would have to be marked as shared, so writing to it would require
            invalidating the corresponding L1 cache line for other cores, even though the individual
            vertices are not shared across cores.  This could incur enormous performance issues.  The block
            size should therefore be sized such that each block aligns with an integer number of cache lines.
            Typically that is 64 bytes, so the block size (in bytes) should be a multiple of 64 bytes.
            For that to be useful, the vertex buffer start should also be aligned to a cache line, but
            that is beyond the control of this class.

        */
        class VertexLinearPoolAllocator
        {
            static const size_t INVALID_INDEX               = ~ size_t( 0 ) ;   /// Special value that mIndexOfCurrentBlock could never legitimately have.
            static const size_t NUM_VERTICES_PER_TRIANGLE   = 3             ;   /// Determined by basic geometry :)
            static const size_t NUM_TRIANGLES_PER_BLOCK     = 64            ;   /// Size is an attempt to arrange for disjoint cache lines for each block.

        public:

            static const size_t NUM_VERTICES_PER_BLOCK      = NUM_TRIANGLES_PER_BLOCK * NUM_VERTICES_PER_TRIANGLE ;


            /** Construct a vertex allocator.

                \param vertexCounter    Address of the number (an atomic size_t) of vertices "allocated" from a vertex buffer.

                \param capacityInVertices   Total capacity, in vertices, of the vertex buffer.
            */
            VertexLinearPoolAllocator( TbbAtomicSizeT * vertexCounter , size_t capacityInVertices )
                : mVertexCounter( vertexCounter )
                , mIndexOfCurrentBlock( INVALID_INDEX )
                , mNumVertsAllocedInCurrentBlock( NUM_VERTICES_PER_BLOCK )
                , mTotalCapacityInVertices( capacityInVertices )
            {
            }




            /** Allocate 3 contiguous vertices.

                \return Index of the triangle allocated.  This is 1/3 of the index of the first vertex of the triangle.

            */
            size_t AllocateTriangle()
            {
                if(     ( INVALID_INDEX == mIndexOfCurrentBlock )
                    ||  ( mNumVertsAllocedInCurrentBlock + NUM_VERTICES_PER_TRIANGLE > NUM_VERTICES_PER_BLOCK ) )
                {   // There is no current block yet (i.e. no allocations occurred yet) or no room remains in this block.
                    // Try to allocate a block.
                    // NOTE: fetch_and_add returns the *original* value, and increments the variable.
                    mIndexOfCurrentBlock = mVertexCounter->fetch_and_add( NUM_VERTICES_PER_BLOCK ) ;
#if 0 && defined( _DEBUG )
{
TBB_SCOPED_SPIN_LOCK( sDebugPrintfLock ) ;
DebugPrintf( "%i,%i,0,BLOCK\n" , GetCurrentThreadId() , mIndexOfCurrentBlock ) ;
}
#endif
                    mNumVertsAllocedInCurrentBlock = 0 ;
                    if( mIndexOfCurrentBlock + NUM_VERTICES_PER_BLOCK >= mTotalCapacityInVertices )
                    {   // Exceeded pool capacity.
                        FAIL() ;
                        mIndexOfCurrentBlock = INVALID_INDEX ;  // Try to make failure obvious.
                        // Ideally, caller would respond to this failure by reallocating the vertex buffer to have larger capacity.
                    }
                }
                // Room remains within this block.
                size_t vertexIndex = mIndexOfCurrentBlock + mNumVertsAllocedInCurrentBlock ;   // Calculate offset of next vertex within this block.
                mNumVertsAllocedInCurrentBlock += NUM_VERTICES_PER_TRIANGLE ; // Allocate vertices for that triangle.
                size_t triangleIndex = vertexIndex / NUM_VERTICES_PER_TRIANGLE ;
                ASSERT( triangleIndex * NUM_VERTICES_PER_TRIANGLE == vertexIndex ) ;
#if 0 && defined( _DEBUG )
{
TBB_SCOPED_SPIN_LOCK( sDebugPrintfLock ) ;
DebugPrintf( "%i,%i,%i,t\n" , GetCurrentThreadId() , mIndexOfCurrentBlock , vertexIndex ) ;
}
#endif
                return triangleIndex ;
            }




            /** Fill the unallocated vertices of the current block with triangles that have zero area.

                \param positionsStart   Address of first component of first position of first vertex in vertex buffer.
                
                \param vertexStrideInBytes  Number of bytes between adjacent vertices in buffer.

                \note   This routine assumes vertex positions are stored as 3 contiguous floats,
                        the x,y,z components of position.

                Calling this function allows the vertex buffer to be rendered even with some unallocated vertices
                interleaved between allocated vertices.

                This assumes the vertex buffer is rendered without an index buffer.  If the VB was rendered
                with an index buffer then calling this would be superflouos, assuming the index buffer does
                not reference any unallocated vertices.

            */
            void FillBlockRemainderWithDegenerateTriangles( float * positionsStart , size_t vertexStrideInBytes )
            {

#if 0 && defined( _DEBUG )
{
TBB_SCOPED_SPIN_LOCK( sDebugPrintfLock ) ;
DebugPrintf( "%i,%i,FillBlockRemainderWithDegenerateTriangles\n" , GetCurrentThreadId() , mIndexOfCurrentBlock ) ;
}
#endif

                unsigned char * positionsAsBytes = reinterpret_cast< unsigned char * >( positionsStart ) ;
                for( size_t idxVertWithinBlock = mNumVertsAllocedInCurrentBlock ; idxVertWithinBlock < NUM_VERTICES_PER_BLOCK ; ++ idxVertWithinBlock )
                {   // For each unallocated vertex within current block...
                    const size_t    idxVertWithinMotherBuffer   = idxVertWithinBlock + mIndexOfCurrentBlock ;
                    const size_t    offsetInBytes               = idxVertWithinMotherBuffer * vertexStrideInBytes ;
                    float *         vertPosAsFloats             = reinterpret_cast< float * >( & positionsAsBytes[ offsetInBytes ] ) ;
                    // The value here does not matter, as long as all 3 vertices in the triangle have the same coordinate,
                    // or more precisely, as long as the triangle has zero area.  I chose to use an extreme value here to make
                    // it more obvious, when using a memory watch, which vertices are for padding.
                    vertPosAsFloats[ 0 ] =
                    vertPosAsFloats[ 1 ] =
                    vertPosAsFloats[ 2 ] = FLT_MIN ;
                }
            }

        private:
            TbbAtomicSizeT * mVertexCounter         ; // Currently "allocated" number of vertices within the pool.
            size_t mIndexOfCurrentBlock             ; // Index, with pool, of current block used to allocate vertices.
            size_t mNumVertsAllocedInCurrentBlock   ; // Number of vertices allocated within the current block.
            size_t mTotalCapacityInVertices         ; // Total capacity of VB; used to check for overflow.
        } ;

// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------


        /*
            Linearly interpolate the position where an isosurface cuts
            an edge between two vertices, each with their own scalar value
        */
        Vec3 VertexInterp( float isolevel , Vec3 p1 , Vec3 p2 , float valp1 , float valp2 )
        {
            PERF_BLOCK( VertexInterp ) ;

            float mu ;
            Vec3 p;

            // Instead of using these conditional branches, clamp mu to [0,1]; select is faster than conditional branches.
            if ( ABS( isolevel - valp1 ) < 0.00001) return p1 ;
            if ( ABS( isolevel - valp2 ) < 0.00001) return p2 ;
            if ( ABS( valp1    - valp2 ) < 0.00001) return p1 ;

            mu = (isolevel - valp1) / (valp2 - valp1) ; // mu in [0,1] corresponds to zero-crossing in [p1,p2]
            p.x = p1.x + mu * ( p2.x - p1.x ) ;
            p.y = p1.y + mu * ( p2.y - p1.y ) ;
            p.z = p1.z + mu * ( p2.z - p1.z ) ;

            return p ;
        }




        /*
            \see VertexInterp.
        */
        Vec3 InterpolateVertexPositionAndNormal( float isolevel , Vec3 p1 , Vec3 p2 , float valp1 , float valp2 , Vec3 & normalAtVertex , const Vec3 & normal1 , const Vec3 & normal2 )
        {
            PERF_BLOCK( InterpolateVertexPositionAndNormal ) ;

            float   tween   = Clamp( (isolevel - valp1) / (valp2 - valp1) , 0.0f , 1.0f ) ; // mu in [0,1] corresponds to zero-crossing in [p1,p2]
            Vec3    vertPos = p1      + tween * ( p2      - p1      ) ;
            normalAtVertex  = normal1 + tween * ( normal2 - normal1 ) ;

            return vertPos ;
        }




        int edgeTable[256]=
        {
            0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
            0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
            0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
            0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
            0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
            0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
            0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
            0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
            0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
            0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
            0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
            0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
            0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
            0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
            0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
            0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
            0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
            0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
            0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
            0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
            0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
            0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
            0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
            0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
            0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
            0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
            0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
            0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
            0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
            0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
            0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
            0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
        };

        int triTable[256][16] =
        {
            {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
            {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
            {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
            {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
            {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
            {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
            {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
            {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
            {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
            {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
            {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
            {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
            {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
            {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
            {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
            {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
            {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
            {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
            {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
            {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
            {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
            {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
            {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
            {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
            {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
            {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
            {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
            {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
            {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
            {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
            {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
            {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
            {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
            {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
            {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
            {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
            {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
            {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
            {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
            {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
            {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
            {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
            {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
            {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
            {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
            {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
            {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
            {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
            {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
            {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
            {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
            {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
            {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
            {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
            {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
            {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
            {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
            {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
            {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
            {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
            {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
            {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
            {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
            {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
            {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
            {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
            {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
            {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
            {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
            {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
            {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
            {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
            {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
            {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
            {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
            {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
            {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
            {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
            {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
            {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
            {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
            {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
            {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
            {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
            {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
            {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
            {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
            {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
            {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
            {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
            {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
            {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
            {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
            {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
            {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
            {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
            {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
            {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
            {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
            {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
            {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
            {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
            {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
            {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
            {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
            {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
            {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
            {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
            {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
            {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
            {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
            {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
            {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
            {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
            {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
            {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
            {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
            {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
            {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
            {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
            {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
            {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
            {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
            {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
            {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
            {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
            {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
            {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
            {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
            {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
            {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
            {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
            {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
            {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
            {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
            {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
            {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
            {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
            {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
            {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
            {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
            {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
            {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
            {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
            {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
            {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
            {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
            {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
            {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
            {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
            {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
            {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
            {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
            {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
            {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
            {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
            {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
            {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
            {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
            {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
            {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
            {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
            {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
            {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
            {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
            {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
            {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
            {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
            {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
            {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
            {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
            {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
            {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
            {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
            {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
            {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
            {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
            {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
            {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
            {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
            {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
        };




        /** Given a grid cell and an isolevel, calculate the triangular
            facets required to represent the isosurface through the cell
        */
        void PolygoniseCellWithNormal( const GridCell & cell , float isolevel , Vec3 * firstTriangle , size_t vertexStrideInBytes , Vec3 * firstNormal , size_t normalStrideInBytes , VertexLinearPoolAllocator & vertexAllocator )
        {
            PERF_BLOCK( PolygoniseCellWithNormal ) ;

            ASSERT( firstTriangle != firstNormal ) ; // positions are at same address for normals, which is nonsense.
            ASSERT( firstNormal != NULLPTR ) ; // missing normals.

            int i ;
            int cubeindex;
            Vec3 vertList[12];

            // Determine the index into the edge table which tells us which vertices are inside of the surface
            cubeindex = 0;
            if (cell.val[0] < isolevel) cubeindex |=   1;
            if (cell.val[1] < isolevel) cubeindex |=   2;
            if (cell.val[2] < isolevel) cubeindex |=   4;
            if (cell.val[3] < isolevel) cubeindex |=   8;
            if (cell.val[4] < isolevel) cubeindex |=  16;
            if (cell.val[5] < isolevel) cubeindex |=  32;
            if (cell.val[6] < isolevel) cubeindex |=  64;
            if (cell.val[7] < isolevel) cubeindex |= 128;

            // Cube is entirely in/out of the surface
            if (edgeTable[cubeindex] == 0)
                return ;

            /* Find vertices where the surface intersects the cube.

                    7----e6-----6
                   /|          /|
                e7/ |e11    e5/ |e10
                 /  |        /  |
                4----e4-----5   |
                |   |       |   |
                |   3---e2--|---2
                |  /        |  /
              e8| /e3     e9| /e1
                |/          |/
                0----e0-----1

            */

            if (edgeTable[cubeindex] &    1) vertList[ 0] = VertexInterp( isolevel , cell.p[0] , cell.p[1] , cell.val[0] , cell.val[1] );
            if (edgeTable[cubeindex] &    2) vertList[ 1] = VertexInterp( isolevel , cell.p[1] , cell.p[2] , cell.val[1] , cell.val[2] );
            if (edgeTable[cubeindex] &    4) vertList[ 2] = VertexInterp( isolevel , cell.p[2] , cell.p[3] , cell.val[2] , cell.val[3] );
            if (edgeTable[cubeindex] &    8) vertList[ 3] = VertexInterp( isolevel , cell.p[3] , cell.p[0] , cell.val[3] , cell.val[0] );

            if (edgeTable[cubeindex] &   16) vertList[ 4] = VertexInterp( isolevel , cell.p[4] , cell.p[5] , cell.val[4] , cell.val[5] );
            if (edgeTable[cubeindex] &   32) vertList[ 5] = VertexInterp( isolevel , cell.p[5] , cell.p[6] , cell.val[5] , cell.val[6] );
            if (edgeTable[cubeindex] &   64) vertList[ 6] = VertexInterp( isolevel , cell.p[6] , cell.p[7] , cell.val[6] , cell.val[7] );
            if (edgeTable[cubeindex] &  128) vertList[ 7] = VertexInterp( isolevel , cell.p[7] , cell.p[4] , cell.val[7] , cell.val[4] );

            if (edgeTable[cubeindex] &  256) vertList[ 8] = VertexInterp( isolevel , cell.p[0] , cell.p[4] , cell.val[0] , cell.val[4] );
            if (edgeTable[cubeindex] &  512) vertList[ 9] = VertexInterp( isolevel , cell.p[1] , cell.p[5] , cell.val[1] , cell.val[5] );
            if (edgeTable[cubeindex] & 1024) vertList[10] = VertexInterp( isolevel , cell.p[2] , cell.p[6] , cell.val[2] , cell.val[6] );
            if (edgeTable[cubeindex] & 2048) vertList[11] = VertexInterp( isolevel , cell.p[3] , cell.p[7] , cell.val[3] , cell.val[7] );

            // Create the triangle.
            char * triangleVertexPositionsBytes = reinterpret_cast< char * >( firstTriangle ) ;
            char * triangleVertexNormalsBytes   = reinterpret_cast< char * >( firstNormal   ) ;

#           define TRI_POS(  offset ) ( * reinterpret_cast< Vec3 * >( & triangleVertexPositionsBytes[ offset ] ) )
#           define TRI_NORM( offset ) ( * reinterpret_cast< Vec3 * >( & triangleVertexNormalsBytes  [ offset ] ) )

            for ( i=0 ; triTable[cubeindex][i] != -1 ; i += 3 )
            {
                // TODO: Detect and respond to vertex buffer allocator exceeding capacity.
                const size_t triangleIdx    = vertexAllocator.AllocateTriangle() ;
                const size_t vertexIdx      = triangleIdx * 3 ;
                const size_t offsetInBytes  = vertexIdx * vertexStrideInBytes ;

                const Vec3 p0 = TRI_POS( offsetInBytes                           ) = vertList[ triTable[ cubeindex ][ i+1 ] ] ; // Triangle first  vertex position
                const Vec3 p1 = TRI_POS( offsetInBytes + vertexStrideInBytes     ) = vertList[ triTable[ cubeindex ][ i+0 ] ] ; // Triangle second vertex position
                const Vec3 p2 = TRI_POS( offsetInBytes + vertexStrideInBytes * 2 ) = vertList[ triTable[ cubeindex ][ i+2 ] ] ; // Triangle third  vertex position

                // Compute triangle face normal.
                const Vec3 p01           = p1 - p0 ;
                const Vec3 p02           = p2 - p0 ;
                const Vec3 faceNormal    = p01 ^ p02 ;
                const Vec3 faceNormalDir = faceNormal.GetDir() ;

                // Assign triangle normal.
                TRI_NORM( offsetInBytes                           ) = faceNormalDir ;
                TRI_NORM( offsetInBytes + normalStrideInBytes     ) = faceNormalDir ;
                TRI_NORM( offsetInBytes + normalStrideInBytes * 2 ) = faceNormalDir ;
            }

#           undef TRI_NORM
#           undef TRI_POS

            return ;
        }




        /// Output an ASCII PLY file header.
        void VertexBufferWrapper_OutputPlyAsciiHeader( const VertexBufferWrapper * vertexBufferWrapper , FILE * filePtr )
        {
            PERF_BLOCK( VertexBufferWrapper_OutputPlyAsciiHeader ) ;

            fprintf( filePtr , "ply\n" ) ;
            fprintf( filePtr , "format ascii 1.0\n" ) ;
            fprintf( filePtr , "comment isosurface\n" ) ;

            fprintf( filePtr , "element vertex %i\n" , vertexBufferWrapper->count ) ;
            fprintf( filePtr , "property float x\n" ) ;
            fprintf( filePtr , "property float y\n" ) ;
            fprintf( filePtr , "property float z\n" ) ;

            const size_t numTriangles = vertexBufferWrapper->count / 3 ;
            ASSERT( 3 * numTriangles == vertexBufferWrapper->count ) ;
            fprintf( filePtr , "element face %i\n" , numTriangles ) ;
            fprintf( filePtr , "property list uchar int vertex_indices\n" ) ;

            fprintf( filePtr , "end_header\n" ) ;

            fflush( filePtr ) ;
        }




        /// Output a vertex list for a PLY file.
        void VertexBufferWrapper_OutputPlyVertexList( const VertexBufferWrapper * vertexBufferWrapper , FILE * filePtr )
        {
            PERF_BLOCK( VertexBufferWrapper_OutputPlyVertexList ) ;

            char * triangleVertexPositionsBytes = reinterpret_cast< char * >( vertexBufferWrapper->positions ) ;

            for( size_t vtxIdx = 0 ; vtxIdx < vertexBufferWrapper->count ; ++ vtxIdx )
            {
                const size_t offset = vtxIdx * vertexBufferWrapper->stride ;
                const Vec3 * pos    = reinterpret_cast< Vec3 * >( & triangleVertexPositionsBytes[ offset ] ) ;
                fprintf( filePtr , "%g %g %g\n" , pos->x , pos->y , pos->z ) ;
            }
            fflush( filePtr ) ;
        }




        /// Output a face list for a PLY file.
        void VertexBufferWrapper_OutputPlyFaceList( const VertexBufferWrapper * vertexBufferWrapper , FILE * filePtr )
        {
            PERF_BLOCK( VertexBufferWrapper_OutputPlyFaceList ) ;

            for( size_t vtxIdx = 0 ; vtxIdx < vertexBufferWrapper->count ; vtxIdx += 3 )
            {
                fprintf( filePtr , "3 %i %i %i\n"
                    , vtxIdx + 0
                    , vtxIdx + 1
                    , vtxIdx + 2
                    ) ;
            }
            fflush( filePtr ) ;
        }




        /// Output a vertex buffer as a PLY file.
        void VertexBufferWrapper_OutputPly( const VertexBufferWrapper * vertexBufferWrapper , const char * plyFilename )
        {
            PERF_BLOCK( VertexBufferWrapper_OutputPly ) ;

            FILE * filePtr = fopen( plyFilename , "w" ) ;
            ASSERT( filePtr ) ;

            VertexBufferWrapper_OutputPlyAsciiHeader( vertexBufferWrapper , filePtr ) ;
            VertexBufferWrapper_OutputPlyVertexList( vertexBufferWrapper , filePtr ) ;
            VertexBufferWrapper_OutputPlyFaceList( vertexBufferWrapper , filePtr ) ;

            fclose( filePtr ) ;
        }




        /** Extract isosurface triangles for a slice of a grid of values.

            \param vertexBufferWrapper Data structure used to access vertex data.

            \param isoLevel    Value of isosurface to extract.

            \param valGrid     Grid of (scalar) values whose iso-level to extract.

            \todo TODO: Detect and handle when the vertex allocator fails due to exceeding capacity.
        */
        ResultCodeE ExtractIsoLevel_Slice( VertexBufferWrapper * vertexBufferWrapper , float isoLevel , const GridWrapper * valGrid , size_t zStart , size_t zEnd )
        {
            PERF_BLOCK( ExtractIsoLevel ) ;

            ASSERT( vertexBufferWrapper->normals != vertexBufferWrapper->positions ) ; // VB has same address for positions and normals, which is nonsense.

            VertexLinearPoolAllocator vertexAllocator( & vertexBufferWrapper->count , vertexBufferWrapper->capacity ) ;

            GridCell cell ;

            const char *    gridValuesBytes = reinterpret_cast< const char * >( valGrid->values ) ;

#           define GRID_VALUES( offset ) ( * reinterpret_cast< const float * >( & gridValuesBytes[ offset ] ) )

            // For each cell, extract points for cell, run PolygoniseCell, and place triangles into vertex buffer.
            // NOTE: This is inefficient, due to repeated calculations and copying data, both of which can readily be avoided by incorporating much of this code into the polygonize routine, or otherwise making it easier to reuse that info.
            for( size_t iz = zStart ; iz < zEnd /* valGrid->number[2] - 1 */ ; iz ++ )
            {
                const size_t offsetZ = iz * valGrid->strides[ 2 ] ; // z contribution to offset into grid.
                const float fz = float( iz ) ;
                for( size_t iy = 0 ; iy < valGrid->number[1] - 1 ; iy ++ )
                {
                    const size_t offsetYZ = iy * valGrid->strides[ 1 ] + offsetZ ;  // y and z contributions into offset into grid.
                    const float fy = float( iy ) ;
                    for( size_t ix = 0 ; ix < valGrid->number[0] - 1 ; ix ++ )
                    {
                        const size_t offsetXYZ = ix * valGrid->strides[ 0 ] + offsetYZ ;    // Offset of first voxel of current grid cell.

                        // TODO: Inline PolygoniseCellWithNormal here. Using cell as an intermediary is (probably) grossly inefficient.

                        cell.val[0] = GRID_VALUES( offsetXYZ                                                                   ) ;
                        cell.val[1] = GRID_VALUES( offsetXYZ + valGrid->strides[0]                                             ) ;
                        cell.val[2] = GRID_VALUES( offsetXYZ + valGrid->strides[0] + valGrid->strides[1]                       ) ;
                        cell.val[3] = GRID_VALUES( offsetXYZ                       + valGrid->strides[1]                       ) ;
                        cell.val[4] = GRID_VALUES( offsetXYZ                                             + valGrid->strides[2] ) ;
                        cell.val[5] = GRID_VALUES( offsetXYZ + valGrid->strides[0]                       + valGrid->strides[2] ) ;
                        cell.val[6] = GRID_VALUES( offsetXYZ + valGrid->strides[0] + valGrid->strides[1] + valGrid->strides[2] ) ;
                        cell.val[7] = GRID_VALUES( offsetXYZ                       + valGrid->strides[1] + valGrid->strides[2] ) ;

                        const float fx = float( ix ) ;

                        const Vec3 gridPtPos = fx * valGrid->directions[0] + fy * valGrid->directions[1] + fz * valGrid->directions[2] + valGrid->minPos ;

                        // Memory layout for cell.p must match that of .val, above.  Furthermore, both must match what the table assumes.
                        cell.p[0] = gridPtPos                              ;
                        cell.p[1] = gridPtPos   + valGrid->directions[ 0 ] ; // +X
                        cell.p[2] = cell.p[ 1 ] + valGrid->directions[ 1 ] ; // +X +Y
                        cell.p[3] = gridPtPos   + valGrid->directions[ 1 ] ; //    +Y
                        // +Z:
                        cell.p[4] = cell.p[ 0 ] + valGrid->directions[ 2 ] ;
                        cell.p[5] = cell.p[ 1 ] + valGrid->directions[ 2 ] ;
                        cell.p[6] = cell.p[ 2 ] + valGrid->directions[ 2 ] ;
                        cell.p[7] = cell.p[ 3 ] + valGrid->directions[ 2 ] ;

                        // TODO: Detect and respond to vertex buffer allocator exceeding capacity.
                        PolygoniseCellWithNormal( cell , isoLevel
                            , reinterpret_cast< Vec3 * >( vertexBufferWrapper->positions )
                            , vertexBufferWrapper->stride
                            , reinterpret_cast< Vec3 * >( vertexBufferWrapper->normals ) , vertexBufferWrapper->stride , vertexAllocator ) ;
                    }
                }
            }

            // Fill remainder of block with degenerate triangles.
            vertexAllocator.FillBlockRemainderWithDegenerateTriangles( vertexBufferWrapper->positions , vertexBufferWrapper->stride ) ;

            return RESULT_OKAY ;
        }




#if USE_TBB
        /** Function object to extract an isosurface from a grid of values.
        */
        class ExtractIsoLevel_TBB
        {
            VertexBufferWrapper * mVertexBufferWrapper ;
            float mIsoLevel ;
            const GridWrapper * mValGrid ;
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Extract isosurface for subset of grid.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                ExtractIsoLevel_Slice( mVertexBufferWrapper , mIsoLevel , mValGrid , r.begin() , r.end() ) ;
            }

            ExtractIsoLevel_TBB( VertexBufferWrapper * vertexBufferWrapper , float isoLevel , const GridWrapper * valGrid )
                : mVertexBufferWrapper( vertexBufferWrapper )
                , mIsoLevel( isoLevel )
                , mValGrid( valGrid )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
        } ;
#endif




        ResultCodeE ExtractIsoLevel( VertexBufferWrapper * vertexBufferWrapper , float isoLevel , const GridWrapper * valGrid )
        {
            const size_t numZMinus1 = valGrid->number[2] - 1 ;
            ResultCodeE  resultCode = RESULT_OKAY ;

#       if USE_TBB
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize =  Max2( size_t( 1 ) , numZMinus1 / 2 /*/ GetNumberOfProcessors() */) ;
            // Extract isosurface using threading building blocks
            parallel_for( tbb::blocked_range<size_t>( 0 , numZMinus1 , grainSize ) , ExtractIsoLevel_TBB( vertexBufferWrapper , isoLevel , valGrid ) ) ;
            if( vertexBufferWrapper->count + VertexLinearPoolAllocator::NUM_VERTICES_PER_BLOCK > vertexBufferWrapper->capacity )
            {   // Vertex buffer probably had insufficient capacity.
                resultCode = RESULT_INSUFFICIENT_CAPACITY ;
            }
#       else
            resultCode = ExtractIsoLevel_Slice( vertexBufferWrapper , isoLevel , valGrid , /* zStart */ 0 , numZMinus1 ) ;
#       endif

#       if 0 && defined( _DEBUG )
            {
                char vbFilename[ 128 ] ;
                static counter = 0 ;
                sprintf( vbFilename , "vb-%i.ply" , counter ) ;
                ++ counter ;
                VertexBufferWrapper_OutputPly( vertexBufferWrapper , vbFilename ) ;
            }
#       endif

            return resultCode ;
        }




        /** Initialize a GridWrapper object.
        */
        void GridWrapper_Init( GridWrapper * grid )
        {
            PERF_BLOCK( GridWrapper_Init ) ;

            memset( grid , 0 , sizeof( * grid ) ) ;
        }




        /** Free memory for a grid allocated and wrapped by GridWrapper.

            \note   Normally the grid memory would be supplied by another data structure.
                    This routine is provided for tests and demonstrations.

            \see Grid_Allocate.
        */
        void GridWrapper_Free( GridWrapper * grid )
        {
            PERF_BLOCK( GridWrapper_Free ) ;

            delete grid->values ;
            grid->values = NULLPTR ;
        }




        /** Allocate memory for a grid wrapped by GridWrapper.

            \note   Normally the grid memory would be supplied by another data structure.
                    This routine is provided for tests and demonstrations.

            \see GridWrapper_Free.
        */
        void Grid_Allocate( GridWrapper * grid , size_t elementSizeInBytes , size_t numX , size_t numY , size_t numZ )
        {
            PERF_BLOCK( Grid_Allocate ) ;

            ASSERT( NULLPTR == grid->values ) ;
            ASSERT( elementSizeInBytes > 0 ) ;
            ASSERT( numX > 1 ) ;
            ASSERT( numY > 1 ) ;
            ASSERT( numZ > 1 ) ;

            const size_t numGridPoints = numX * numY * numZ ;
            const size_t numBytes      = elementSizeInBytes * numGridPoints ;
            grid->values = reinterpret_cast< float * >( NEW char[ numBytes ] ) ;
            ASSERT( grid->values != NULLPTR ) ;
            ASSERT( IsAligned( grid->values , sizeof( float ) ) ) ;

            DEBUG_ONLY( memset( grid->values , 0xba , numBytes ) ) ;

            grid->number[ 0 ] = numX ;
            grid->number[ 1 ] = numY ;
            grid->number[ 2 ] = numZ ;

            grid->strides[ 0 ] = elementSizeInBytes ;
            grid->strides[ 1 ] = grid->strides[ 0 ] * numX ;
            grid->strides[ 2 ] = grid->strides[ 1 ] * numY ;
        }




        /** Make a spherical distribution of SDF values discretized to a grid of given dimensions.

            \note   Normally grid data would be supplied by another data structure.
                    This routine is provided for tests and demonstrations.
        */
        void GridWrapper_MakeSphere( GridWrapper * sdfGrid , GridWrapper * gradGrid , size_t gridDim , float radius )
        {
            PERF_BLOCK( GridWrapper_MakeSphere ) ;

            Grid_Allocate( sdfGrid  , sizeof( float ) , gridDim , gridDim , gridDim ) ;
            Grid_Allocate( gradGrid , sizeof( Vec3 )  , gridDim , gridDim , gridDim ) ;

            char * sdfGridValuesBytes  = reinterpret_cast< char * >(  sdfGrid->values ) ;
            char * gradGridValuesBytes = reinterpret_cast< char * >( gradGrid->values ) ;

            // Why -3?
            // -1 to ensure that the "rightmost" (farthest-along-positive-direction) gridpoint SDF values (along each axis) would exactly hit the values +radius.
            // Phrased differently, the -1 ensures symmetry: the rightmost gridpoints have the same SDF as their leftmost counterparts.
            // But to extract an isosurface with isolevel==radius, there would need to be gridpoints outside +-radius, hence another -1 in each direction.
            const float diameter = 2.0f * radius ;
            const float perX     = diameter / float( sdfGrid->number[ 0 ] - 3 ) ;
            const float perY     = diameter / float( sdfGrid->number[ 1 ] - 3 ) ;
            const float perZ     = diameter / float( sdfGrid->number[ 2 ] - 3 ) ;

            // Center sphere within grid domain.
            // The -per[XYZ] term takes into account the padding added to make sure all points along the surface lie within a grid cell. (See above.)
            gradGrid->minPos = sdfGrid->minPos = Vec3( - radius - perX , - radius - perY , - radius - perZ ) ;

            gradGrid->directions[ 0 ] = sdfGrid->directions[ 0 ] = Vec3( perX , 0.0f , 0.0f ) ;
            gradGrid->directions[ 1 ] = sdfGrid->directions[ 1 ] = Vec3( 0.0f , perY , 0.0f ) ;
            gradGrid->directions[ 2 ] = sdfGrid->directions[ 2 ] = Vec3( 0.0f , 0.0f , perZ ) ;

            DEBUG_ONLY( float valMin =   FLT_MAX ) ;
            DEBUG_ONLY( float valMax = - FLT_MAX ) ;

            for( size_t iz = 0 ; iz < sdfGrid->number[ 2 ] ; iz ++ )
            {   // For each grid point along z...
                const size_t offsetZ = iz * sdfGrid->strides[ 2 ] ;
                const float  fz      = float( iz ) * perZ  ; // distance along z from min position, in [0,diameter]
                const float  dz      = fz + sdfGrid->minPos.z ; // shifted distance along z from center, in [-radius,radius]
                const float  dz2     = dz * dz ;
                for( size_t iy = 0 ; iy < sdfGrid->number[ 1 ] ; iy ++ )
                {   // For each grid point along y...
                    const size_t offsetYZ = iy * sdfGrid->strides[ 1 ] + offsetZ ;
                    const float  fy       = float( iy ) * perY ;
                    const float  dy       = fy + sdfGrid->minPos.y ;
                    const float  dy2      = dy * dy ;
                    const float  dy2_dz2  = dy2 + dz2 ;
                    for( size_t ix = 0 ; ix < sdfGrid->number[ 0 ] ; ix ++ )
                    {   // For each grid point along x...
                        const size_t offsetXYZ      = ix * sdfGrid->strides[ 0 ] + offsetYZ ;
                        const float  fx             = float( ix ) * perX ;
                        const float  dx             = fx + sdfGrid->minPos.x ;
                        const float  dx2            = dx * dx ;
                        const float  dx2_dy2_dz2    = dx2 + dy2_dz2 ;
                        const float  distFromCenter = fsqrtf( dx2_dy2_dz2 ) ;

                        float *      sdfGridValue   = reinterpret_cast< float * >( & sdfGridValuesBytes[ offsetXYZ ] ) ;
                        ASSERT( IsAligned( sdfGridValue , sizeof( float ) ) ) ;

                        * sdfGridValue = distFromCenter ;

                        DEBUG_ONLY( valMin = Min2( valMin , * sdfGridValue ) ) ;
                        DEBUG_ONLY( valMax = Max2( valMax , * sdfGridValue ) ) ;

                        const Vec3   dispFromCenter = Vec3( dx , dy , dz ) ;
                        const Vec3   dirFromCenter  = dispFromCenter.GetDir() ;

                        Vec3 *  gradGridValue  = reinterpret_cast< Vec3 * >( & gradGridValuesBytes[ offsetXYZ ] ) ;
                        ASSERT( IsAligned( gradGridValue , sizeof( float ) ) ) ;

                        * gradGridValue = dirFromCenter ;
                    }
                }
            }
        }




        /** Update the mesh of an intrisic isosurface extracted from the given wrapped grid.

            \see Mesh_MakeFromVolume.
        */
        ResultCodeE Mesh_UpdateFromVolume( MeshBase * mesh , float isoLevel , const GridWrapper * valGrid )
        {
            PERF_BLOCK( Mesh_UpdateFromVolume ) ;

            ASSERT( mesh->GetVertexBuffer() != NULLPTR ) ;
            ASSERT( NULLPTR == mesh->GetIndexBuffer() ) ;

            ASSERT( valGrid->number[ 0 ] > 1 ) ;    // Grid must have at least 2 points (i.e. 1 cell) in each direction.
            ASSERT( valGrid->number[ 1 ] > 1 ) ;
            ASSERT( valGrid->number[ 2 ] > 1 ) ;

            DEBUG_ONLY( const size_t numGridCells = ( valGrid->number[ 0 ] - 1 ) * ( valGrid->number[ 1 ] - 1 ) * ( valGrid->number[ 2 ] - 1 ) ) ;

            ASSERT( PRIMITIVE_TRIANGLES == mesh->GetPrimitiveType() ) ;

            VertexBufferBase * meshVertBuf = mesh->GetVertexBuffer() ;
            ASSERT( meshVertBuf->GetCapacity() >= numGridCells ) ;

            VertexBufferWrapper vertBufWrapper ;
            vertBufWrapper.capacity = mesh->GetVertexBuffer()->GetCapacity() ;
            vertBufWrapper.count = 0 ;
            vertBufWrapper.stride = mesh->GetVertexBuffer()->GetVertexSizeInBytes() ;
            void * vertexData = meshVertBuf->LockVertexData() ;
            vertBufWrapper.positions = static_cast< float * >( meshVertBuf->GetElementStart( vertexData , PeGaSys::Render::VertexDeclaration::VertexElement::POSITION , 0 ) ) ;
            vertBufWrapper.normals = static_cast< float * >( meshVertBuf->GetElementStart( vertexData , PeGaSys::Render::VertexDeclaration::VertexElement::NORMAL , 0 ) ) ;
            ASSERT( vertBufWrapper.positions != vertBufWrapper.normals ) ; // Positions should not be at same address as normals.

            DEBUG_ONLY( memset( vertBufWrapper.positions , 0xfe , ( vertBufWrapper.capacity - 1 ) * vertBufWrapper.stride ) ) ; // Fill VB with nonsense values to make them obvious.

            ResultCodeE resultCode = ExtractIsoLevel( & vertBufWrapper , isoLevel , valGrid ) ;

            //if( RESULT_OKAY != resultCode ) DEBUG_BREAK() ; // Caller should resize vertex buffer.

            meshVertBuf->SetPopulation( vertBufWrapper.count ) ;

            meshVertBuf->UnlockVertexData() ;

            return resultCode ;
        }




        /** Make a mesh of an intrisic isosurface extracted from the given wrapped grid.

            This mesh should be from a Model.
            Caller should add model to scene.

            Caller should assign a technique, for example using code like the following:

                Effect *    effect     = NEW Effect() ;
                Technique * technique  = effect->GetTechniques().Front() ;
                mesh->SetTechnique( technique ) ;

            \see Mesh_UpdateFromVolume.

        */
        void Mesh_MakeFromVolume( MeshBase * mesh , ApiBase * renderApi , float isoLevel , const GridWrapper * valGrid , VertexDeclaration::VertexFormatE vertFmt )
        {
            PERF_BLOCK( Mesh_MakeFromVolume ) ;

            ASSERT( renderApi ) ;
            ASSERT( NULLPTR == mesh->GetIndexBuffer() ) ;
            ASSERT( /*( VertexDeclaration::POSITION == vertFmt ) ||*/ ( VertexDeclaration::POSITION_NORMAL == vertFmt ) ); // Other formats not yet supported.  If they were, it would likely be up to the caller to populate them.

            ASSERT( valGrid->number[ 0 ] > 1 ) ;    // Grid must have at least 2 points (i.e. 1 cell) in each direction.
            ASSERT( valGrid->number[ 1 ] > 1 ) ;
            ASSERT( valGrid->number[ 2 ] > 1 ) ;

            const size_t numGridCells = ( valGrid->number[ 0 ] - 1 ) * ( valGrid->number[ 1 ] - 1 ) * ( valGrid->number[ 2 ] - 1 ) ;

            mesh->SetPrimitiveType( PRIMITIVE_TRIANGLES ) ;

            VertexBufferBase * meshVertBuf = ( NULLPTR == mesh->GetVertexBuffer() ) ? mesh->NewVertexBuffer( renderApi ) : mesh->GetVertexBuffer() ;

            if( meshVertBuf->GetCapacity() == 0 )
            {   // Space in vertex buffer was not allocated; this is probably a new VB.
                VertexDeclaration  vertexDeclaration( vertFmt ) ;
                meshVertBuf->DeclareVertexFormat( vertexDeclaration ) ;
                meshVertBuf->Allocate( numGridCells ) ;
            }
            else if( meshVertBuf->GetCapacity() < numGridCells )
            {   // Space in vertex buffer was previously allocated but has insufficient capacity.
                ASSERT( meshVertBuf->GetVertexDeclaration().GetVertexFormat() == vertFmt ) ;
                meshVertBuf->ChangeCapacityAndReallocate( numGridCells ) ;
            }

            /* DEBUG_ONLY( ResultCodeE resultCode = ) */ Mesh_UpdateFromVolume( mesh , isoLevel , valGrid ) ;

            //if( RESULT_OKAY != resultCode ) DEBUG_BREAK() ; // Should resize vertex buffer.
        }




        /** Make a sphere mesh using isosurface extraction.

            \see Mesh_MakeFromVolume.

            \note   Normally grid data would be supplied by another data structure.
                    This routine is provided for tests and demonstrations.

        */
        void Mesh_MakeSphere( MeshBase * mesh , ApiBase * renderApi , size_t gridDim , float radius , VertexDeclaration::VertexFormatE vertFmt )
        {
            PERF_BLOCK( Mesh_MakeSphere ) ;

            GridWrapper sdfGrid ;
            GridWrapper_Init( & sdfGrid ) ;
            GridWrapper gradGrid ;
            GridWrapper_Init( & gradGrid ) ;
            GridWrapper_MakeSphere( & sdfGrid , & gradGrid , gridDim , radius ) ;
            Mesh_MakeFromVolume( mesh , renderApi , /* isoLevel */ radius , & sdfGrid , vertFmt ) ;
            GridWrapper_Free( & gradGrid ) ;
            GridWrapper_Free( & sdfGrid ) ;
        }


    } ;
} ;


#if defined( _DEBUG )

void PeGaSys_Render_MarchingCubes_UnitTest()
{
    DebugPrintf( "MarchingCubes::UnitTest ----------------------------------------------\n" ) ;

    DebugPrintf( "MarchingCubes::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif
