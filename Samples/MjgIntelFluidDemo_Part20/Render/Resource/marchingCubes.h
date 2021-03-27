/** \file marchingCubes.h

    \brief Class to generate mesh from grid of values

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_MARCHING_CUBES_H
#define PEGASYS_RENDER_MARCHING_CUBES_H

#include "Render/Resource/vertexBuffer.h"

#include <Core/useTbb.h>

#include <Core/Math/vec3.h>

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
#if USE_TBB
        typedef tbb::atomic< size_t > TbbAtomicSizeT ;
#else
        /** Wrapper class that provides similar functionality as Intel TBB atomic< size_t >, but for single threads.
        */
        class TbbAtomicSizeT
        {
        public:
            TbbAtomicSizeT() {}

            size_t fetch_and_add( size_t increment )
            {
                size_t originalValue = _value ;
                _value += increment ;
                return originalValue ;
            }

            operator size_t () const { return _value ; }

            size_t & operator=( size_t newValue ) { _value = newValue ; return _value ; }

        private:
            size_t _value ;
        } ;
#endif

        class MeshBase ;
        class ApiBase ;

        /** Lightweight wrapper around a generic vertex buffer, used to give marching cubes a place to output triangles.
        */
        struct VertexBufferWrapper
        {
            float *         positions   ;   /// Array of vertex positions extracted from grid.
            float *         normals     ;   /// Array of vertex normals extracted from grid.
            TbbAtomicSizeT  count       ;   /// Number of vertices written into vertices array.
            size_t          capacity    ;   /// Maximum number of vertices that can fit into vertices array.
            size_t          stride      ;   /// Number of bytes between adjacent vertices.
        } ;


        /** Lightweight wrapper around a generic rectilinear grid of scalar floating-point values.
        */
        struct GridWrapper
        {
            float * values          ;   /// Values at grid points.
            size_t  number[3]       ;   /// Number of grid points along each direction.
            size_t  strides[3]      ;   /// Delta, in bytes, per index, between adjacent points in the grid. offset = ix * strides[0] + iy * strides[1] + iz * strides[2]
            Vec3    minPos          ;   /// Location of first grid point, i.e. values[0]
            Vec3    directions[ 3 ] ;   /// Direction vectors corresponding to each index: position = ix * directions[0] + iy * directions[1] + iz * directions[2] + minPos
        } ;


        enum ResultCodeE
        {
            RESULT_OKAY                     ,   /// Isosurface extraction completed without errors.
            RESULT_INSUFFICIENT_CAPACITY        /// Isosurface extraction would not complete since output buffer had insufficient capacity.
        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

        extern void Mesh_MakeFromVolume( MeshBase * mesh , ApiBase * renderApi , float isoLevel , const GridWrapper * valGrid , VertexDeclaration::VertexFormatE vertFmt ) ;
        extern void Mesh_MakeSphere( MeshBase * mesh , ApiBase * renderApi , size_t gridDim , float radius , VertexDeclaration::VertexFormatE vertFmt ) ;

    } ;

} ;

#endif
