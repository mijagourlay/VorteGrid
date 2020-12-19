/*! \file uniformGrid.h

    \brief A container for fast spatial lookups and insertions

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef UNIFORM_GRID_H
#define UNIFORM_GRID_H

#include <math.h>

#include "Core/Math/vec3.h"

#include "Core/wrapperMacros.h"


// Macros --------------------------------------------------------------

/** Whether to use all neighbors when performing downsamping.

    \note   The "diffusing" version of this downsampler might be more appropriate
            when assigning values for the maximal gridpoints.

    Ultimately, the choice of which technique to use depends on which provides better results,
    including performance.  Remember that the results on the low-res grid are effectively only
    starters for the results obtained on the high-res grid, so to that extent even an approximate
    answer provides value.  So it's probably better to favor speed over accuracy here.
*/
#define USE_ALL_NEIGHBORS 1
#if USE_ALL_NEIGHBORS
    /// Apply expression since USE_ALL_NEIGHTBORS is enabled.
    #define IF_USE_NEIGHBORS( expr ) expr
#else
    /// Ignore expression since USE_ALL_NEIGHTBORS is disabled.
    #define IF_USE_NEIGHBORS( expr )
#endif


/// Return the absolute value.
#define ABS( x ) ( ( (x) < 0 ) ? ( - (x) ) : (x) )


// Types --------------------------------------------------------------

/** Base class for uniform grid.

    The shape of this grid is such that the "minimal corner" point
    resides at indices {0,0,0} and the "maximal corner" point
    resides at indices {Nx-1,Ny-1,Nz-1}.

    The number of /points/ in each direction i is N_i.

    A cell is defined by the 8 points that lie at its corners.

    This also implies that the grid must have at least 2 points in
    each direction.

    The number of /cells/ in each direction i is (N_i-1).

    The size of a side i of each cell is therefore
        - s_i = (vMax-vMin)_i / (N_i-1) .

    \verbatim
                  vMin
                   0       1      ...     Nx-1
                  (*)------*-------*-------* 0
                 ./|       |       |       |
                ./ |       |       |       |
               .*  |       |       |       |
               /|  *-------*-------*-------* 1
              / | /|       |       |       |
        Nz-1 *  |/ |       | cell  |       |
             |  *  |       |       |       | .
             | /|  *-------*-------*-------* .
             |/ | /|       |       |       | .
             *  |/ |       |       |       |
             |  *  |       |       |       |
             | /|  *-------*-------*-------* Ny-1
             |/ | /       /       /       /
             *  |/       /       /       /
             |  *-------*-------*-------*
             | /       /       /       /
             |/       /       /       /
             *-------*-------*------(*)vMax
    \endverbatim


*/
class UniformGridGeometry
{
    public:

        /** Construct an empty UniformGridGeometry.

            \see Clear
        */
        UniformGridGeometry()
            : mMinCorner( 0.0f , 0.0f , 0.0f )
            , mGridExtent( 0.0f , 0.0f , 0.0f )
            , mCellExtent( 0.0f , 0.0f , 0.0f )
            , mCellsPerExtent( 0.0f , 0.0f , 0.0f )
        {
            mNumPoints[ 0 ] = mNumPoints[ 1 ] = mNumPoints[ 2 ] = 0 ;
        }


        /** Construct a uniform grid that fits the given geometry.

            \see Clear, DefineShape
        */
        UniformGridGeometry( size_t uNumElements , const Vec3 & vMin , const Vec3 & vMax , bool bPowerOf2 )
        {
            DefineShape( uNumElements , vMin , vMax , bPowerOf2 ) ;
        }

        /// Return minimal corner of this UniformGrid.
        const Vec3 & GetMinCorner( void ) const { return mMinCorner ; }


        /// Return maximal corner of this UniformGrid.
        Vec3        GetMaxCorner( void ) const { return GetMinCorner() + GetExtent() ; }


        /** Return whether the given query point lies inside the boundaries of this grid.
        */
        bool    Encompasses( const Vec3 & queryPoint ) const
        {
            return ( queryPoint >= GetMinCorner() ) && ( queryPoint <= GetMaxCorner() ) ;
        }


        /// Return center position that this UniformGrid contains.
        Vec3        GetCenter() const
        {
            return ( GetMinCorner() + GetMaxCorner() ) * 0.5f ;
        }


        /** Return whether the shape of this object matches that of the given.

            \param that - Other UniformGridGeometry object

            \return true if the shape of this object matches that of that.

        */
        bool ShapeMatches( const UniformGridGeometry & that ) const
        {
            return      ( GetNumPoints( 0 ) == that.GetNumPoints( 0 ) )
                    &&  ( GetNumPoints( 1 ) == that.GetNumPoints( 1 ) )
                    &&  ( GetNumPoints( 2 ) == that.GetNumPoints( 2 ) )
                    &&  ( GetMinCorner()    == that.GetMinCorner()    ) ;
        }


        /** Define the shape a uniform grid such that it fits the given geometry.

            \param uNumElements - number of elements this container will contain.

            \param vMin - minimal coordinate of axis-aligned bounding box.

            \param vMax - maximal coordinate of axis-aligned bounding box.

            \param bPowerOf2 - whether to make each grid dimension a power of 2.
                Doing so simplifies grid subdivision, if this grid will be used
                in a hierarchical grid.

            This makes a uniform grid of cells, where each cell is the same size
            and the side of each cell is nearly the same size.  If the cells are
            3-dimensional then that means each cell is a box, nearly a cube.
            The number of dimensions of the region depends on the actual size of
            the region.  If any size component is smaller than a small threshold
            then this class considers that component to be zero, and reduces the
            dimensionality of the region.  For example, if the region size is
            (2,3,0) then this class considers the region to have 2 dimensions
            (x and y) since the z size is zero.  In this example, the cells
            would be nearly square rectangles (instead of boxes).

        */
        void DefineShape( size_t uNumElements , const Vec3 & vMin , const Vec3 & vMax , bool bPowerOf2 )
        {

            mMinCorner  = vMin ;
            static const float Nudge = 1.0f + FLT_EPSILON ;  // slightly expand size to ensure robust containment even with roundoff
            mGridExtent     = ( vMax - vMin ) * Nudge ;

            Vec3 vSizeEffective( GetExtent() ) ;
            int numDims = 3 ;   // Number of dimensions to region.
            if( 0.0f == vSizeEffective.x )
            {   // X size is zero so reduce dimensionality
                vSizeEffective.x = 1.0f ; // This component will not contribute to the total region volume/area/length.
                mGridExtent.x = 0.0f ;
                -- numDims ;
            }
            if( 0.0f == vSizeEffective.y )
            {   // Y size is zero so reduce dimensionality
                vSizeEffective.y = 1.0f ; // This component will not contribute to the total region volume/area/length.
                mGridExtent.y = 0.0f ;
                -- numDims ;
            }
            if( 0.0f == vSizeEffective.z )
            {   // Z size is zero so reduce dimensionality
                vSizeEffective.z = 1.0f ; // This component will not contribute to the total region volume/area/length.
                mGridExtent.z = 0.0f ;
                -- numDims ;
            }

            // Compute region volume, area or length (depending on dimensionality).
            const float volume              = vSizeEffective.x * vSizeEffective.y * vSizeEffective.z ;
            const float cellVolumeCubeRoot  = powf( volume / float( uNumElements ) , -1.0f / float( numDims ) ) ; // Approximate size of each cell in grid.
            // Compute number of cells in each direction of uniform grid.
            // Choose grid dimensions to fit as well as possible, so that the total number
            // of grid cells is nearly the total number of elements in the contents.
            size_t numCells[3] = { MAX2( 1 , unsigned( GetExtent().x * cellVolumeCubeRoot + 0.5f ) ) ,
                                   MAX2( 1 , unsigned( GetExtent().y * cellVolumeCubeRoot + 0.5f ) ) ,
                                   MAX2( 1 , unsigned( GetExtent().z * cellVolumeCubeRoot + 0.5f ) ) } ;
            if( bPowerOf2 )
            {   // Choose number of gridcells to be powers of 2.
                // This will simplify subdivision in a NestedGrid.
                numCells[ 0 ] = NearestPowerOfTwo( numCells[ 0 ] ) ;
                numCells[ 1 ] = NearestPowerOfTwo( numCells[ 1 ] ) ;
                numCells[ 2 ] = NearestPowerOfTwo( numCells[ 2 ] ) ;
            }
            while( numCells[ 0 ] * numCells[ 1 ] * numCells[ 2 ] >= uNumElements * 8 )
            {   // Grid capacity is excessive.
                // This can occur when the trial numCells is below 0.5 in which case the integer arithmetic loses the subtlety.
                numCells[ 0 ] = MAX2( 1 , numCells[0] / 2 ) ;
                numCells[ 1 ] = MAX2( 1 , numCells[1] / 2 ) ;
                numCells[ 2 ] = MAX2( 1 , numCells[2] / 2 ) ;
            }
            mNumPoints[ 0 ] = numCells[ 0 ] + 1 ; // Increment to obtain number of points.
            mNumPoints[ 1 ] = numCells[ 1 ] + 1 ; // Increment to obtain number of points.
            mNumPoints[ 2 ] = numCells[ 2 ] + 1 ; // Increment to obtain number of points.

            PrecomputeSpacing() ;
        }


        /** Scale given grid.

            \param src - Source uniform grid upon which to base dimensions of this one

            \param scale - amount by which to scale each axis of grid

            \note The number of cells is also scaled.

        */
        void Scale( const UniformGridGeometry & src , const Vec3 & scale )
        {
            mGridExtent     = Vec3( src.mGridExtent.x * scale.x , src.mGridExtent.y * scale.y , src.mGridExtent.z * scale.z ) ;
            mMinCorner      = Vec3( src.mMinCorner.x  * scale.x , src.mMinCorner.y  * scale.y , src.mMinCorner.z  * scale.z ) ;
            mNumPoints[ 0 ] = int( float( src.GetNumCells( 0 ) ) * scale.x ) + 1 ;
            mNumPoints[ 1 ] = int( float( src.GetNumCells( 1 ) ) * scale.y ) + 1 ;
            mNumPoints[ 2 ] = int( float( src.GetNumCells( 2 ) ) * scale.z ) + 1 ;
            PrecomputeSpacing() ;
        }



        /** Create a lower-resolution uniform grid based on another.

            \param src - Source uniform grid upon which to base dimensions of this one

            \param iDecimation - amount by which to reduce the number of grid cells in each dimension.
                Typically this would be 2.

            \note The number of cells is decimated.  The number of points is different.

        */
        void Decimate( const UniformGridGeometry & src , int iDecimation )
        {
            mGridExtent         = src.mGridExtent ;
            mMinCorner          = src.mMinCorner ;
            mNumPoints[ 0 ]     = src.GetNumCells( 0 ) / iDecimation + 1 ;
            mNumPoints[ 1 ]     = src.GetNumCells( 1 ) / iDecimation + 1 ;
            mNumPoints[ 2 ]     = src.GetNumCells( 2 ) / iDecimation + 1 ;
            if( iDecimation > 1 )
            {   // Decimation could reduce dimension and integer arithmetic could make value be 0, which is useless if src contained any data.
                mNumPoints[ 0 ] = MAX2( 2 , GetNumPoints( 0 ) ) ;
                mNumPoints[ 1 ] = MAX2( 2 , GetNumPoints( 1 ) ) ;
                mNumPoints[ 2 ] = MAX2( 2 , GetNumPoints( 2 ) ) ;
            }
            PrecomputeSpacing() ;
        }



        /** Copy shape information from another UniformGrid into this one.
        */
        void CopyShape( const UniformGridGeometry & src )
        {
            Decimate( src , 1 ) ;
        }


        /** Get world-space dimensions of UniformGridGeometry.
        */
        const Vec3 & GetExtent() const { return mGridExtent ; }


        /** Whether this geometry has zero extent.
        */
        bool HasZeroExtent() const { return GetExtent() == Vec3( 0.0f , 0.0f , 0.0f ) ; }


        /** Get reciprocal of cell spacing.
        */
        const Vec3 & GetCellsPerExtent( void ) const { return mCellsPerExtent ; }


        /** Get number of grid points along the given dimension.

            \param index - dimension queried, where 0 means x, 1 means y and 2 means z.

            \note The number of cells in each direction i is GetNumPoints(i) - 1.

        */
        const unsigned &    GetNumPoints( const unsigned & index ) const    { return mNumPoints[ index ] ; }


        /** Get number of grid cells along the given dimension.

            \param index - dimension queried, where 0 means x, 1 means y and 2 means z.

            \see GetNumPoints

        */
        unsigned        GetNumCells( const unsigned & index ) const    { return GetNumPoints( index ) - 1 ; }


        /** Get total number of gridpoints.

            \note This returns the number of gridpoints defined by the geometry, which is separate
                    from the capacity of the container.  When the container has been initialized,
                    these should be the same, but prior to initializing the container,
                    GetGridCapacity can be non-zero even when the container size is zero.

        */
        unsigned GetGridCapacity( void ) const { return GetNumPoints( 0 ) * GetNumPoints( 1 ) * GetNumPoints( 2 ) ; }


        /** Return extent (in world units) of a grid cell.
        */
        const Vec3 &    GetCellSpacing( void ) const    { return mCellExtent ; }


        /** Return volume (in world units) of entire grid.
        */
        const float     GetVolume( void ) const     { return GetExtent().x * GetExtent().y * GetExtent().z ; }


        /** Return volume (in world units) of a grid cell.
        */
        const float     GetCellVolume( void ) const     { return mCellExtent.x * mCellExtent.y * mCellExtent.z ; }


        /** Return extent (in world units) of a grid cell.
        */
        const Vec3      GetCellCenter( int ix , int iy , int iz ) const
        {
            const Vec3 vOffset(   float( ix ) * GetCellSpacing().x
                                , float( iy ) * GetCellSpacing().y
                                , float( iz ) * GetCellSpacing().z ) ;
            const Vec3 vMin( GetMinCorner() + vOffset ) ;
            const Vec3 vCenter( vMin + GetCellSpacing() * 0.5f ) ;
            return vCenter ;
        }


        /** Compute indices into contents array of a point at a given position.

            \param vPosition - position of a point.  It must be within the region of this container.

            \param indices - Indices into contents array of a point at vPosition.

            \see IndicesFromOffset, PositionFromOffset, OffsetOfPosition, PositionFromIndices

            \note Derived class defines the actual contents array.

        */
        void IndicesOfPosition( unsigned indices[4] , const Vec3 & vPosition ) const
        {
            // Notice the pecular test here.  vPosition may lie slightly outside of the extent give by vMax.
            // Review the geometry described in the class header comment.
            Vec3 vPosRel( vPosition - GetMinCorner() ) ;   // position of given point relative to container region
            Vec3 vIdx( vPosRel.x * GetCellsPerExtent().x , vPosRel.y * GetCellsPerExtent().y , vPosRel.z * GetCellsPerExtent().z ) ;
        #if 0   // Original code.
            indices[0] = unsigned( vIdx.x ) ;
            indices[1] = unsigned( vIdx.y ) ;
            indices[2] = unsigned( vIdx.z ) ;
        #elif 1 // Optimization 1: change control word once for all 3 conversions
            const WORD OldCtrlWord = Changex87FloatingPointToTruncate() ;
            indices[0] = StoreFloatAsInt( vIdx.x ) ;
            indices[1] = StoreFloatAsInt( vIdx.y ) ;
            indices[2] = StoreFloatAsInt( vIdx.z ) ;
            SetFloatingPointControlWord( OldCtrlWord ) ;
        #elif 0 // Optimization 2: Use SSE instructions that do not require 16-byte alignment
            _asm {
                movdqu      xmm0    , vIdx
                cvttps2dq   xmm0    , xmm0
                movdqu      indices , xmm0
            }
        #elif 0 // Optimization 3: Use SSE instructions that assume 16-byte alignment
            _asm {
                cvttps2dq   xmm0    , vIdx
                movdqa      indices , xmm0
            }
        #elif 0 // Optimization 4: use approximation.  Faster, but gives incorrect results sometimes.
            indices[0] = FastIntFromFloatApproximate( vIdx.x ) ;
            indices[1] = FastIntFromFloatApproximate( vIdx.y ) ;
            indices[2] = FastIntFromFloatApproximate( vIdx.z ) ;
        #endif
        }


        /** Get indices of nearest grid point.

            \param indices      (out) Indices of nearest grid point.

            \param vPosition    (in) Position being queried.

            This contrasts with IndicesOfPosition, which gets the indices of the nearest grid *cell*,
            which is tantamount to getting the minimal corner gridpoint of that cell.
            IndicesOfPosition is like rounding down whereas this routine is like
            rounding to nearest.
        */
        void IndicesOfNearestGridPoint( unsigned indices[4] , const Vec3 & vPosition ) const
        {
            
            IndicesOfPosition( indices , vPosition ) ;
            
            Vec3            vMinCorner ;
            PositionFromIndices( vMinCorner , indices ) ;
            const Vec3  vDiff   = vPosition - vMinCorner ; // Relative location of position within its containing grid cell.
            const Vec3  tween   = Vec3( vDiff.x * GetCellsPerExtent().x , vDiff.y * GetCellsPerExtent().y , vDiff.z * GetCellsPerExtent().z ) ;
            if( tween.x >= 0.5f ) ++ indices[ 0 ] ;
            if( tween.y >= 0.5f ) ++ indices[ 1 ] ;
            if( tween.z >= 0.5f ) ++ indices[ 2 ] ;
        }


        /** Compute indices into contents array of a point at a given position.
        
            Assumes floating point control word is set to use truncation instead of nearest, for rounding.

            \see IndicesOfPosition, StoreFloatAsInt, Changex87FloatingPointToTruncate.

        */
        void IndicesOfPosition_AssumesFpcwSetToTruncate( unsigned indices[4] , const Vec3 & vPosition ) const
        {
            // Notice the pecular test here.  vPosition may lie slightly outside of the extent give by vMax.
            // Review the geometry described in the class header comment.
            Vec3 vPosRel( vPosition - GetMinCorner() ) ;   // position of given point relative to container region
            Vec3 vIdx( vPosRel.x * GetCellsPerExtent().x , vPosRel.y * GetCellsPerExtent().y , vPosRel.z * GetCellsPerExtent().z ) ;
            // The following 3 float-to-int conversions assume the
            // floating-point-control-word is set to truncate, by the outer caller.
            indices[0] = StoreFloatAsInt( vIdx.x ) ;
            indices[1] = StoreFloatAsInt( vIdx.y ) ;
            indices[2] = StoreFloatAsInt( vIdx.z ) ;
        }


        /** Compute offset into contents array of a point at a given position.

            \param vPosition - position of a point.  It must be within the region of this container.

            \return Offset into contents array of a point at vPosition.

            \see IndicesFromOffset, PositionFromOffset, PositionFromIndices

            \note Derived class defines the actual contents array.

        */
        unsigned    OffsetOfPosition( const Vec3 & vPosition ) const
        {
            unsigned indices[4] ;
            IndicesOfPosition( indices , vPosition ) ;
            const unsigned offset = indices[0] + GetNumPoints( 0 ) * ( indices[1] + GetNumPoints( 1 ) * indices[2] ) ;
            return offset ;
        }


        /** Compute position of minimal corner of grid cell with given indices.

            \param vPosition    (out) Position of minimal corner of grid cell.

            \param indices      (in) Grid cell indices.

            \note Rarely would you want to compute position from indices in this way.
                    Typically, this kind of computation occurs inside a triply-nested loop,
                    in which case the procedure should compute each component
                    separately.  Furthermore, such a routine would cache
                    GetCellSpacing instead of computing it each iteration.

        */
        void    PositionFromIndices( Vec3 & vPosition , const unsigned indices[3] ) const
        {
            vPosition.x = GetMinCorner().x + float( indices[0] ) * GetCellSpacing().x ;
            vPosition.y = GetMinCorner().y + float( indices[1] ) * GetCellSpacing().y ;
            vPosition.z = GetMinCorner().z + float( indices[2] ) * GetCellSpacing().z ;
        }


        /** Compute X,Y,Z grid cell indices from offset into contents array.

            \param indices - Individual X,Y,Z component grid cell indices.

            \param offset - Offset into mContents.
        */
        void    IndicesFromOffset( unsigned indices[3] , const unsigned & offset ) const
        {
            indices[2] = offset / ( GetNumPoints(0) * GetNumPoints(1) ) ;
            indices[1] = ( offset - indices[2] * GetNumPoints(0) * GetNumPoints(1) ) / GetNumPoints(0) ;
            indices[0] = offset - GetNumPoints(0) * ( indices[1] + GetNumPoints(1) * indices[2] ) ;
        }


        /** Get position of grid cell minimum corner.

            \param vPos - position of grid cell minimum corner

            \param offset - offset into contents array

            Each grid cell spans a region (whose size is given by GetCellSpacing)
            starting at a location which this routine returns.  So the grid cell
            with the given offset spans the region from vPos (as this routine
            assigns) to vPos + GetCellSpacing().

            \note Derived class provides actual contents array.

        */
        void    PositionFromOffset( Vec3 & vPos , const unsigned & offset )
        {
            unsigned indices[3] ;
            IndicesFromOffset( indices , offset ) ;
            vPos.x = GetMinCorner().x + float( indices[0] ) * GetCellSpacing().x ;
            vPos.y = GetMinCorner().y + float( indices[1] ) * GetCellSpacing().y ;
            vPos.z = GetMinCorner().z + float( indices[2] ) * GetCellSpacing().z ;
        }


        /** Get offset into contents array given indices.

            \param indices - indices specifying a grid cell

            \return offset into contents array

            \note Typically this routine would not be efficient to use, except for special cases.
                    Often, one writes a triple-nested loop iterating over each
                    component of indices, in which case it is more efficient
                    to compute the z and y terms of the offset separately and 
                    combine them with the x term in the inner-most loop.
                    This routine is useful primarily when there is no coherence
                    between the indices of this iteration and the previous or next.

            \note Derived class provides actual contents array.

        */
        unsigned OffsetFromIndices( const unsigned indices[3] ) const
        {
            return indices[0] + GetNumPoints(0) * ( indices[1] + GetNumPoints(1) * indices[2] ) ;
        }


    protected:

        /** Precompute grid spacing, to optimize OffsetOfPosition and other utility routines.
        */
        void PrecomputeSpacing( void )
        {
            mCellExtent.x       = GetExtent().x / float( GetNumCells( 0 ) ) ;
            mCellExtent.y       = GetExtent().y / float( GetNumCells( 1 ) ) ;
            mCellExtent.z       = GetExtent().z / float( GetNumCells( 2 ) ) ;
            mCellsPerExtent.x   = float( GetNumCells( 0 ) ) / GetExtent().x ;
            mCellsPerExtent.y   = float( GetNumCells( 1 ) ) / GetExtent().y ;
            if( 0.0f == GetExtent().z )
            {   // Avoid divide-by-zero for 2D domains that lie in the XY plane.
                mCellsPerExtent.z   = 1.0f / FLT_MIN ;
            }
            else
            {
                mCellsPerExtent.z   = float( GetNumCells( 2 ) ) / GetExtent().z ;
            }
        }




        /** Return the offset associated with the cell whose indices are second-to-last in each direction.

            Same as nx * ( ny * ( nz - 1 ) - 1 ) - 2.
        */
        unsigned GetOffsetOfPenultimateCell( void )
        {
            return (GetNumPoints(0)-2) + GetNumPoints(0) * ( (GetNumPoints(1)-2) + GetNumPoints(1) * (GetNumPoints(2)-2) ) ;
        }


        /** Clear out any existing shape information.
        */
        void Clear()
        {
            mMinCorner      =
            mGridExtent     =
            mCellExtent     =
            mCellsPerExtent = Vec3( 0.0f , 0.0f , 0.0f ) ;
            mNumPoints[ 0 ] = mNumPoints[ 1 ] = mNumPoints[ 2 ] = 0 ;
        }


        Vec3    mMinCorner      ;   ///< Minimum position (in world units) of grid in X, Y and Z directions.
        Vec3    mGridExtent     ;   ///< Size (in world units) of grid in X, Y and Z directions.
        Vec3    mCellExtent     ;   ///< Size (in world units) of a cell.
        Vec3    mCellsPerExtent ;   ///< Reciprocal of cell size (precomputed once to avoid excess divides).
        size_t  mNumPoints[ 3 ] ;   ///< Number of gridpoints along X, Y and Z directions.

          ///< Whether Interpolate is being called.
} ;




/** Templated container for fast spatial lookups and insertions.
*/
template <class ItemT> class UniformGrid : public UniformGridGeometry
{
    public:

        typedef UniformGridGeometry Parent ;    ///< Nickname for UniformGridGeometry.

        /** Construct an empty UniformGrid.
            \see Initialize
        */
        UniformGrid() : UniformGridGeometry() { }


        /** Construct a uniform grid container that fits the given geometry.
            \see Initialize
        */
        UniformGrid( unsigned uNumElements , const Vec3 & vMin , const Vec3 & vMax , bool bPowerOf2 )
            : UniformGridGeometry( uNumElements , vMin , vMax , bPowerOf2 )
        {
        }


        /** Copy shape from given uniform grid.
        */
        explicit UniformGrid( const UniformGridGeometry & that )
            : UniformGridGeometry( that )
        {
        }


        /** Copy constructor for empty uniform grids.

            This copy constructor does not copy contained contents.  A proper
            copy ctor should deep-copy all of its data in order to operate with
            containers, in particular STL vector) when it reallocates arrays.
            But reallocating these containers would entail massive memory moves,
            which although possible, would be inefficient.  Meanwhile, NestedGrid
            also pushes UniformGrids onto a vector, which requires using a copy
            ctor to initialize the new element.  So that code makes empty elements
            to push onto the vector, and arrange for this copy ctor to handle those
            empties properly.

            Meanwhile we want to catch any unintentional copies of actual data,
            so this method catches any attempt to copy populated UniformGrid objects.
        */
        UniformGrid( const UniformGrid & that )
            : UniformGridGeometry( that )
        {
        }

        /// Return item at given offset, where offset should come from OffsetOfPosition or OffsetFromIndices.
              ItemT &       operator[]( const unsigned & offset )       {  return mContents[ offset ] ; }
        const ItemT &       operator[]( const unsigned & offset ) const {  return mContents[ offset ] ; }

        /// Return item at given indices.
              ItemT &       operator[]( const unsigned indices[] )      { return mContents[ OffsetFromIndices( indices ) ] ; }

        /// Return item at given position.
              ItemT &       operator[]( const Vec3 & vPosition )        { return mContents[ OffsetOfPosition( vPosition ) ] ; }
        const ItemT &       operator[]( const Vec3 & vPosition ) const  { return mContents[ OffsetOfPosition( vPosition ) ] ; }


        /** Scale each value in this grid by the given scalar.

            \param scale - amount by which to scale each value in this grid

        */
        void Scale( float scale )
        {
            const unsigned numCells = GetGridCapacity() ;
            for( unsigned offset = 0 ; offset < numCells ; ++ offset )
            {
                ItemT & rVal = (*this)[ offset ] ;
                rVal *= scale ;
            }
        }


        /** Initialize contents to whatever default ctor provides.
        */
        void Init( const ItemT & initialValue = ItemT() )
        {
            mContents.Resize( GetGridCapacity() , initialValue ) ;
        }


        void DefineShape( size_t uNumElements , const Vec3 & vMin , const Vec3 & vMax , bool bPowerOf2 )
        {
            mContents.Clear() ;
            UniformGridGeometry::DefineShape( uNumElements , vMin , vMax , bPowerOf2 ) ;
        }


        /** Return the number of cells in this grid which have been assigned values.
        */
        size_t Size( void ) const { return mContents.Size() ; }


        /** Return whether this container contains any items.
        */
        bool Empty( void ) const { return mContents.Empty() ; }


        /** Create an empty container based on a given uniform grid container, but with lower resolution.

            \param src - Source UniformGridGeometry upon which to base dimensions of this container

            \param iDecimation - amount by which to reduce resolution (number of grid cells in each dimension).
                Typically this would be 2.

        */
        void Decimate( const UniformGridGeometry & src , int iDecimation )
        {
            UniformGridGeometry::Decimate( src , iDecimation ) ;
        }


        static void UnitTest( void ) ;


        /** Compute statistics of data in a uniform grid.

            \param min - minimum of all values in grid.  Caller must initialize to large values before calling this routine.

            \param max - maximum of all values in grid.  Caller must initialize to smale values before calling this routine.

        */
        void ComputeStatistics( ItemT & min , ItemT & max ) const
        {
            max = min = (*this)[ 0 ] ;
            const unsigned numCells = GetGridCapacity() ;
            for( unsigned offset = 0 ; offset < numCells ; ++ offset )
            {
                const ItemT & rVal = (*this)[ offset ] ;
                min = MIN2( min , rVal ) ;
                max = MAX2( max , rVal ) ;
            }
        }



        /** Interpolate values from grid to get value at given position.

            \param vResult      Interpolated value corresponding to value of grid contents at vPosition.

            \param vPosition    Position to sample.
        */
        void Interpolate( ItemT & vResult , const Vec3 & vPosition ) const
        {
            unsigned        indices[4] ; // Indices of grid cell containing position.
            
            IndicesOfPosition( indices , vPosition ) ;
            
            Vec3            vMinCorner ;
            PositionFromIndices( vMinCorner , indices ) ;
            const Vec3      vDiff         = vPosition - vMinCorner ; // Relative location of position within its containing grid cell.
            const Vec3      tween         = Vec3( vDiff.x * GetCellsPerExtent().x , vDiff.y * GetCellsPerExtent().y , vDiff.z * GetCellsPerExtent().z ) ;
            const Vec3      oneMinusTween = Vec3( 1.0f , 1.0f , 1.0f ) - tween ;
            const unsigned  numXY         = GetNumPoints( 0 ) * GetNumPoints( 1 ) ;
            const unsigned  offsetX0Y0Z0  = OffsetFromIndices( indices ) ;
            const unsigned  offsetX1Y0Z0  = offsetX0Y0Z0 + 1 ;
            const unsigned  offsetX0Y1Z0  = offsetX0Y0Z0 + GetNumPoints(0) ;
            const unsigned  offsetX1Y1Z0  = offsetX0Y0Z0 + GetNumPoints(0) + 1 ;
            const unsigned  offsetX0Y0Z1  = offsetX0Y0Z0 + numXY ;
            const unsigned  offsetX1Y0Z1  = offsetX0Y0Z0 + numXY + 1 ;
            const unsigned  offsetX0Y1Z1  = offsetX0Y0Z0 + numXY + GetNumPoints(0) ;
            const unsigned  offsetX1Y1Z1  = offsetX0Y0Z0 + numXY + GetNumPoints(0) + 1 ;
            vResult =     ( ( oneMinusTween.x * (*this)[ offsetX0Y0Z0 ]
                            +         tween.x * (*this)[ offsetX1Y0Z0 ] ) * oneMinusTween.y
                          + ( oneMinusTween.x * (*this)[ offsetX0Y1Z0 ]
                            +         tween.x * (*this)[ offsetX1Y1Z0 ] ) * tween.y        ) * oneMinusTween.z
                        + ( ( oneMinusTween.x * (*this)[ offsetX0Y0Z1 ]
                            +         tween.x * (*this)[ offsetX1Y0Z1 ] ) * oneMinusTween.y
                          + ( oneMinusTween.x * (*this)[ offsetX0Y1Z1 ]
                            +         tween.x * (*this)[ offsetX1Y1Z1 ] ) * tween.y        ) * tween.z ;
        }



        /** Interpolate values from grid to get value at given position.

            \param vResult  Interpolated value corresponding to value of grid contents at vPosition.

            \param vPosition Position to sample.

            \note   This version assumes the floating-point-control-word is set
                    to "truncate" -- which the caller must do by calling
                    Changex87FloatingPointToTruncate or equivalent, and
                    afterwords, typically the caller should call
                    SetFloatingPointControlWord to return the FPCW back to "rounding" (the
                    default) afterwards.  This optimization is specific to
                    Intel chipsets, and dramatically improves speed.  Since
                    this interpolation is one of the hotest spots for particle
                    advection, which in turn is one of the hotest spots in a
                    particle simulation, this kind of manually tuning is
                    worthwhile.

            \see    Changex87FloatingPointToTruncate, IndicesOfPosition_AssumesFpcwSetToTruncate.

        */
        void Interpolate_AssumesFpcwSetToTruncate( ItemT & vResult , const Vec3 & vPosition ) const
        {
            unsigned        indices[4] ; // Indices of grid cell containing position.

            
            IndicesOfPosition_AssumesFpcwSetToTruncate( indices , vPosition ) ;
            

            Vec3            vMinCorner ;
            PositionFromIndices( vMinCorner , indices ) ;
            const unsigned  offsetX0Y0Z0 = OffsetFromIndices( indices ) ;
            const Vec3      vDiff         = vPosition - vMinCorner ; // Relative location of position within its containing grid cell.
            const Vec3      tween         = Vec3( vDiff.x * GetCellsPerExtent().x , vDiff.y * GetCellsPerExtent().y , vDiff.z * GetCellsPerExtent().z ) ;
            const Vec3      oneMinusTween = Vec3( 1.0f , 1.0f , 1.0f ) - tween ;
            const unsigned  numXY         = GetNumPoints( 0 ) * GetNumPoints( 1 ) ;
            const unsigned  offsetX1Y0Z0  = offsetX0Y0Z0 + 1 ;
            const unsigned  offsetX0Y1Z0  = offsetX0Y0Z0 + GetNumPoints(0) ;
            const unsigned  offsetX1Y1Z0  = offsetX0Y1Z0 + 1 ;
            const unsigned  offsetX0Y0Z1  = offsetX0Y0Z0 + numXY ;
            const unsigned  offsetX1Y0Z1  = offsetX0Y0Z1 + 1 ;
            const unsigned  offsetX0Y1Z1  = offsetX0Y0Z0 + numXY + GetNumPoints(0) ;
            const unsigned  offsetX1Y1Z1  = offsetX0Y1Z1 + 1 ;
            vResult =     ( ( oneMinusTween.x * (*this)[ offsetX0Y0Z0 ]
                            +         tween.x * (*this)[ offsetX1Y0Z0 ] ) * oneMinusTween.y
                          + ( oneMinusTween.x * (*this)[ offsetX0Y1Z0 ]
                            +         tween.x * (*this)[ offsetX1Y1Z0 ] ) * tween.y        ) * oneMinusTween.z
                        + ( ( oneMinusTween.x * (*this)[ offsetX0Y0Z1 ]
                            +         tween.x * (*this)[ offsetX1Y0Z1 ] ) * oneMinusTween.y
                          + ( oneMinusTween.x * (*this)[ offsetX0Y1Z1 ]
                            +         tween.x * (*this)[ offsetX1Y1Z1 ] ) * tween.y        ) * tween.z ;
        }



        /** Accumulate given value into grid at given position.

            \param vPosition - position of a "source" whose contents this routine stores in a grid cell.

            \param item - value of "source" to store into the grid cell that contains vPosition.

            \note "Accumulate" accumulates values; it does not overwrite them.
                    This routine does not populate the grid according to a function whose values
                    are given by "item".  Instead, this routine treats each insertion as though
                    the cell contains a specified "source", in addition to other sources that
                    might have already been inserted.

            \note Since this routine accumulates values, it is likely prudent to initialize
                    the values of each gridpoints to zero before calling this routine.

        */
        void Accumulate( const Vec3 & vPosition , const ItemT & item )
        {
            unsigned        indices[4] ; // Indices of grid cell containing position.
            IndicesOfPosition( indices , vPosition ) ;
            Vec3            vMinCorner ;
            PositionFromIndices( vMinCorner , indices ) ;
            const unsigned  offsetX0Y0Z0 = OffsetFromIndices( indices ) ;
            const Vec3      vDiff         = vPosition - vMinCorner ; // Relative location of position within its containing grid cell.
            const Vec3      tween         = Vec3( vDiff.x * GetCellsPerExtent().x , vDiff.y * GetCellsPerExtent().y , vDiff.z * GetCellsPerExtent().z ) ;
            const Vec3      oneMinusTween = Vec3( 1.0f , 1.0f , 1.0f ) - tween ;
            const unsigned  numXY         = GetNumPoints( 0 ) * GetNumPoints( 1 ) ;
            const unsigned  offsetX1Y0Z0  = offsetX0Y0Z0 + 1 ;
            const unsigned  offsetX0Y1Z0  = offsetX0Y0Z0 + GetNumPoints(0) ;
            const unsigned  offsetX1Y1Z0  = offsetX0Y0Z0 + GetNumPoints(0) + 1 ;
            const unsigned  offsetX0Y0Z1  = offsetX0Y0Z0 + numXY ;
            const unsigned  offsetX1Y0Z1  = offsetX0Y0Z0 + numXY + 1 ;
            const unsigned  offsetX0Y1Z1  = offsetX0Y0Z0 + numXY + GetNumPoints(0) ;
            const unsigned  offsetX1Y1Z1  = offsetX0Y0Z0 + numXY + GetNumPoints(0) + 1 ;
            (*this)[ offsetX0Y0Z0 ] += oneMinusTween.x * oneMinusTween.y * oneMinusTween.z * item ;
            (*this)[ offsetX1Y0Z0 ] +=         tween.x * oneMinusTween.y * oneMinusTween.z * item ;
            (*this)[ offsetX0Y1Z0 ] += oneMinusTween.x *         tween.y * oneMinusTween.z * item ;
            (*this)[ offsetX1Y1Z0 ] +=         tween.x *         tween.y * oneMinusTween.z * item ;
            (*this)[ offsetX0Y0Z1 ] += oneMinusTween.x * oneMinusTween.y *         tween.z * item ;
            (*this)[ offsetX1Y0Z1 ] +=         tween.x * oneMinusTween.y *         tween.z * item ;
            (*this)[ offsetX0Y1Z1 ] += oneMinusTween.x *         tween.y *         tween.z * item ;
            (*this)[ offsetX1Y1Z1 ] +=         tween.x *         tween.y *         tween.z * item ;
        }




        /** Restrict values from a high-resolution grid into a low-resolution grid.

            In the vernacular of multi-grid solvers, the "restrict" operation reduces
            the resolution of a grid, so this operation is tantamount to down-sampling.
            It contrasts with the "interpolate" operation, which up-samples.

            This routine assumes this loRes grid is empty and the given hiRes layer is populated.

            \param hiRes - hiRes grid from which information will be aggregated.

            \note This loRes grid must have 3 or greater points in at least one of its dimensions.

            \see SolvePoissonMultiGrid UpSampleFrom

        */
        void DownSample( const UniformGrid< ItemT > & hiRes )
        {
            UniformGrid< ItemT > &  loRes        = * this ;
            const unsigned          numPointsHiRes[3]   = { hiRes.GetNumPoints( 0 ) , hiRes.GetNumPoints( 1 ) , hiRes.GetNumPoints( 2 ) } ;
            const unsigned  &       numXhiRes           = hiRes.GetNumPoints( 0 ) ;
            const unsigned          numXYhiRes          = numXhiRes * hiRes.GetNumPoints( 1 ) ;
            static const float      fMultiplierTable[]  = { 8.0 , 4.0 , 2.0 , 1.0 } ;

            // number of cells in each grid cluster
            const unsigned pClusterDims[] = {   hiRes.GetNumCells( 0 ) / loRes.GetNumCells( 0 )
                                            ,   hiRes.GetNumCells( 1 ) / loRes.GetNumCells( 1 )
                                            ,   hiRes.GetNumCells( 2 ) / loRes.GetNumCells( 2 ) } ;

            const unsigned  numPointsLoRes[3]   = { loRes.GetNumPoints( 0 ) , loRes.GetNumPoints( 1 ) , loRes.GetNumPoints( 2 ) } ;
            const unsigned  numXYLoRes          = loRes.GetNumPoints( 0 ) * loRes.GetNumPoints( 1 ) ;
            const unsigned  idxShifts[3]        = { pClusterDims[0] / 2 , pClusterDims[1] / 2 , pClusterDims[2] / 2 } ;

            // Since this loop iterates over each destination cell,
            // it should readily parallelize without contention.
            //
            // Note that the if-statements inside this loop could (and perhaps should) be
            // moved outside the loop.  Accomplish this by limiting the loRes indices to
            // be in [1,N-2] and then creating 6 2D loops below, for the boundary planes.
            unsigned idxLoRes[3] ;
            for( idxLoRes[2] = 0 ; idxLoRes[2] < numPointsLoRes[2] ; ++ idxLoRes[2] )
            {
                const unsigned offsetZ = idxLoRes[2] * numXYLoRes ;
                for( idxLoRes[1] = 0 ; idxLoRes[1] < numPointsLoRes[1] ; ++ idxLoRes[1] )
                {
                    const unsigned offsetYZ = idxLoRes[1] * loRes.GetNumPoints( 0 ) + offsetZ ;
                    for( idxLoRes[0] = 0 ; idxLoRes[0] < numPointsLoRes[0] ; ++ idxLoRes[0] )
                    {   // For each cell in the loRes layer...
                        const unsigned  offsetXYZ   = idxLoRes[0] + offsetYZ ;
                        ItemT        &  rValLoRes  = loRes[ offsetXYZ ] ;
                        float           multiplier  = 0.0f ;
                        unsigned clusterMinIndices[ 3 ] ;
                        NestedGrid<Vec3>::GetChildClusterMinCornerIndex( clusterMinIndices , pClusterDims , idxLoRes ) ;
                        unsigned increment[3] ;
                        int      shiftHiRes[3] ;
                        unsigned idxHiRes[3] ;

                        // Code below accumulates values so destination needs to start at zero.
                        memset( & rValLoRes , 0 , sizeof( rValLoRes ) ) ;

                        // For each cell of hiRes layer in this grid cluster...
                        for( increment[2] = 0 ; increment[2] < pClusterDims[2] IF_USE_NEIGHBORS( + idxShifts[2] ) ; ++ increment[2] )
                        {
                            shiftHiRes[2] = increment[2] IF_USE_NEIGHBORS( - idxShifts[2] ) ;
                            idxHiRes[2] = clusterMinIndices[2] + shiftHiRes[2] ;
                            if( idxHiRes[2] < numPointsHiRes[2] )
                            {
                                const unsigned offsetZ  = idxHiRes[2] * numXYhiRes ;
                                for( increment[1] = 0 ; increment[1] < pClusterDims[1] IF_USE_NEIGHBORS( + idxShifts[1] ) ; ++ increment[1] )
                                {
                                    shiftHiRes[1] = increment[1] IF_USE_NEIGHBORS( - idxShifts[1] ) ;
                                    idxHiRes[1] = clusterMinIndices[1] + shiftHiRes[1] ;
                                    if( idxHiRes[1] < numPointsHiRes[1] )
                                    {
                                        const unsigned offsetYZ = idxHiRes[1] * numXhiRes + offsetZ ;
                                        for( increment[0] = 0 ; increment[0] < pClusterDims[0] IF_USE_NEIGHBORS( + idxShifts[0] ) ; ++ increment[0] )
                                        {
                                            shiftHiRes[0] = increment[0] IF_USE_NEIGHBORS( - idxShifts[0] ) ;
                                            idxHiRes[0] = clusterMinIndices[0] + shiftHiRes[0] ;
                                            if( idxHiRes[0]  < numPointsHiRes[0] )
                                            {
                                                const unsigned  offsetXYZ       = idxHiRes[0]  + offsetYZ ;
                                                const unsigned  manhattanDist   = ABS( shiftHiRes[0] ) + ABS( shiftHiRes[1] ) + ABS( shiftHiRes[2] ) ;
                                                const ItemT &   rValHiRes       = hiRes[ offsetXYZ ] ;
                                            #if USE_ALL_NEIGHBORS
                                                multiplier += fMultiplierTable[ manhattanDist ] ;
                                                rValLoRes  += fMultiplierTable[ manhattanDist ] * rValHiRes ;
                                            #else
                                                multiplier += 1.0f ;
                                                rValLoRes  += rValHiRes ;
                                            #endif
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        // Normalize sum to account for number of gridpoints that contributed to this gridpoint.
                    #if USE_ALL_NEIGHBORS
                        rValLoRes /= 64.0f ;
                    #else
                        rValLoRes /= multiplier ;
                    #endif
                    }
                }
            }

        }


        /** Interpolate value from the given low-resolution grid into this high-resolution grid.

            In the vernacular of multi-grid solvers, the "interpolate" operation increases
            the resolution of a grid, so this operation is tantamount to up-sampling.
            It contrasts with the "restrict" operation, which down-samples.

            This routine assumes the given high-resolution grid (this)
            is empty and the given loRes grid is populated.

            \param loRes - low-resolution grid from which information will be read.

            \see SolvePoissonMultiGrid DownSample

            \note As of 2009nov19, this routine does NOT properly "undo" the operations of DownSampleInto.
                    In multi-grid vernacular, this operation does NOT apply the transpose
                    of the Restriction operation.  This is evident by virtue of the fact
                    that after this operation, zeroth moment between the low-res and high-res
                    grids DO NOT MATCH.
                    A proper implementation would depend on the USE_ALL_NEIGHBORS flag,
                    and would preserve zeroth moment.
                    When USE_ALL_NEIGHBORS is DISabled, then this routine could simply
                    copy values from the low-res to high-res grids, without interpolation.
                    When USE_ALL_NEIGHBORS is ENabled, then this routine would need to use
                    a different interpolation algorithm -- not tri-linear, but something like
                    quadratic, where values come from all 27 neighbors of a position.

        */
        void UpSample( const UniformGrid< ItemT > & loRes )
        {
            UniformGrid< ItemT > &  hiRes         = * this ;
            const unsigned          numPointsHiRes[3]   = { hiRes.GetNumPoints( 0 ) , hiRes.GetNumPoints( 1 ) , hiRes.GetNumPoints( 2 ) } ;
            const unsigned  &       numXhiRes           = hiRes.GetNumPoints( 0 ) ;
            const unsigned          numXYhiRes          = numXhiRes * hiRes.GetNumPoints( 1 ) ;
            // Nudge sampling positions so they lie strictly within the grid.
            const Vec3              vPosMinAdjust       = Vec3( fabsf( hiRes.GetMinCorner().x )
                                                              , fabsf( hiRes.GetMinCorner().y )
                                                              , fabsf( hiRes.GetMinCorner().z ) ) * 2.0f * FLT_EPSILON ;
            const Vec3              vPosMin             = hiRes.GetMinCorner() + vPosMinAdjust ;
            const Vec3              vSpacing            = hiRes.GetCellSpacing() * ( 1.0f - 4.0f * FLT_EPSILON ) ;

            // Since this loop iterates over each destination cell,
            // it should readily parallelize without contention.
            unsigned idxHiRes[3] ;
            for( idxHiRes[2] = 0 ; idxHiRes[2] < numPointsHiRes[2] ; ++ idxHiRes[2] )
            {
                const unsigned  offsetZ     = idxHiRes[2] * numXYhiRes ;
                Vec3            vPosition   ;
                vPosition.z = vPosMin.z + float( idxHiRes[2] ) * vSpacing.z ;

                for( idxHiRes[1] = 0 ; idxHiRes[1] < numPointsHiRes[1] ; ++ idxHiRes[1] )
                {
                    const unsigned offsetYZ = idxHiRes[1] * hiRes.GetNumPoints( 0 ) + offsetZ ;
                    vPosition.y = vPosMin.y + float( idxHiRes[1] ) * vSpacing.y ;

                    for( idxHiRes[0] = 0 ; idxHiRes[0] < numPointsHiRes[0] ; ++ idxHiRes[0] )
                    {   // For each cell in the loRes layer...
                        const unsigned  offsetXYZ   = idxHiRes[0] + offsetYZ ;
                        ItemT        &  rValHiRes   = hiRes[ offsetXYZ ] ;
                        vPosition.x = vPosMin.x + float( idxHiRes[0] ) * vSpacing.x ;
                        loRes.Interpolate( rValHiRes , vPosition ) ;
                    }
                }
            }

        }



        void Clear( void )
        {
            mContents.Clear() ;
            Parent::Clear() ;
        }


        void GenerateBrickOfBytes( const char * strFilenameBase , unsigned uFrame ) const ;



        /** Compute zeroth moment of contents of this container.
        */
        ItemT ComputeZerothMoment( void ) const
        {
            //const unsigned  numPoints[3]  = { GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) } ;
            //const unsigned  numXY         = GetNumPoints( 0 ) * GetNumPoints( 1 ) ;

            ItemT     firstMoment ;
            memset( & firstMoment , 0 , sizeof( firstMoment ) ) ;
            #if USE_ALL_NEIGHBORS
                for( unsigned offsetP = 0 ; offsetP < GetGridCapacity() ; ++ offsetP )
                {
                    firstMoment += operator[]( offsetP ) * GetCellVolume() ;
                }
            #else
                // Use a limited domain, because of the way the maximal gridpoints are handled:
                // It boils down to the semantics of the last gridpoint.  This firstMoment calculation
                // assumes that the value for the gridcell comes from the minimal corner gridpoint,
                // which is not correct.  The semantics of the grid are such that the value in the
                // cell varies across the cell, where its values are specified by the gridpoints that
                // surround each cell.  In that case, we would need a different integral technique,
                // such as the trapezoidal rule (in 3D).
                //#define BOGUS_MINUS_1
                #define BOGUS_MINUS_1 -1
                unsigned idx[3] ;
                for( idx[2] = 0 ; idx[2] < numPoints[2] BOGUS_MINUS_1 ; ++ idx[2] )
                {
                    const unsigned offsetZ = idx[2] * numXY ;
                    for( idx[1] = 0 ; idx[1] < numPoints[1] BOGUS_MINUS_1 ; ++ idx[1] )
                    {
                        const unsigned offsetYZ = idx[1] * GetNumPoints( 0 ) + offsetZ ;
                        for( idx[0] = 0 ; idx[0] < numPoints[0] BOGUS_MINUS_1 ; ++ idx[0] )
                        {   // For each cell in the parent layer...
                            firstMoment += operator[]( idx ) * GetCellVolume() ;
                        }
                    }
                }
            #endif
            return firstMoment ;
        }


    private:
        Vector<ItemT>       mContents           ;   ///< 3D array of items.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
