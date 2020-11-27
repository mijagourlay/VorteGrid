/*! \file uniformGrid.h

    \brief A container for fast spatial lookups and insertions

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef UNIFORM_GRID_H
#define UNIFORM_GRID_H

#include <math.h>

#include "Core/Math/vec3.h"

#include "../wrapperMacros.h"


// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Base class for uniform grid.

    The shape of this grid is such that the "minimal corner" point
    resides at indices {0,0,0} and the "maximal corner" point
    resides at indices {Nx-1,Ny-1,Nz-1}.

    The number of /points/ in each direction i is N_i.

    A cell is defined by the 8 points that lie at its corners.

    This also implies that the grid must have at least 2 points in
    each direction.

    The number of /cells/ in each direction i is (N_i-1).

    The size of a side i of each cell is therefore
        s_i = (vMax-vMin)_i / (N_i-1) .


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


*/
class UniformGridGeometry
{
    public:

        /*! \brief Construct an empty UniformGridGeometry.

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


        /*! \brief Construct a uniform grid that fits the given geometry.

            \see Clear, DefineShape
        */
        UniformGridGeometry( unsigned uNumElements , const Vec3 & vMin , const Vec3 & vMax , bool bPowerOf2 )
        {
            DefineShape( uNumElements , vMin , vMax , bPowerOf2 ) ;
        }


        const Vec3 & GetMinCorner( void ) const { return mMinCorner ; }


        /*! \brief Define the shape a uniform grid such that it fits the given geometry.

            \param uNumElements - number of elements this container will contain.

            \param vMin - minimal coordinate of axis-aligned bounding box.

            \param vMax - maximal coordinate of axis-aligned bounding box.

            \param bPowerOf2 - whether to make each grid dimension a power of 2.
                Doing so simplifies grid subdivision, if this grid will be used in a hierarchical grid.

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
            unsigned numCells[3] = { MAX2( 1 , unsigned( GetExtent().x * cellVolumeCubeRoot + 0.5f ) ) ,
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


        /*! \brief Create a lower-resolution uniform grid based on another

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



        /*! \brief Copy shape information from another UniformGrid into this one
        */
        void CopyShape( const UniformGridGeometry & src )
        {
            Decimate( src , 1 ) ;
        }



        /*! \brief Get world-space dimensions of UniformGridGeometry
        */
        const Vec3 & GetExtent( void ) const { return mGridExtent ; }


        /*! \brief Get reciprocal of cell spacing.
        */
        const Vec3 & GetCellsPerExtent( void ) const { return mCellsPerExtent ; }


        /*! \brief Get number of grid points along the given dimension

            \param index - dimension queried, where 0 means x, 1 means y and 2 means z.

            \note The number of cells in each direction i is GetNumPoints(i) - 1.

        */
        const unsigned &    GetNumPoints( const unsigned & index ) const    { return mNumPoints[ index ] ; }


        /*! \brief Get number of grid cells along the given dimension

            \param index - dimension queried, where 0 means x, 1 means y and 2 means z.

            \see GetNumPoints

        */
        unsigned        GetNumCells( const unsigned & index ) const    { return GetNumPoints( index ) - 1 ; }


        /*! \brief Get total number of gridpoints

            \note This returns the number of gridpoints defined by the geometry, which is separate
                    from the capacity of the container.  When the container has been initialized,
                    these should be the same, but prior to initializing the container,
                    GetGridCapacity can be non-zero even when the container size is zero.

        */
        unsigned GetGridCapacity( void ) const { return GetNumPoints( 0 ) * GetNumPoints( 1 ) * GetNumPoints( 2 ) ; }


        /*! \brief Return extent (in world units) of a grid cell.
        */
        const Vec3 &    GetCellSpacing( void ) const                       { return mCellExtent ; }


        /*! \brief Compute indices into contents array of a point at a given position

            \param vPosition - position of a point.  It must be within the region of this container.

            \param indices - Indices into contents array of a point at vPosition.

            \see IndicesFromOffset, PositionFromOffset, OffsetOfPosition.

            \note Derived class defines the actual contents array.

        */
        void IndicesOfPosition( unsigned indices[3] , const Vec3 & vPosition ) const
        {
            // Notice the pecular test here.  vPosition may lie slightly outside of the extent give by vMax.
            // Review the geometry described in the class header comment.
            Vec3 vPosRel( vPosition - GetMinCorner() ) ;   // position of given point relative to container region
            Vec3 vIdx( vPosRel.x * GetCellsPerExtent().x , vPosRel.y * GetCellsPerExtent().y , vPosRel.z * GetCellsPerExtent().z ) ;
            indices[0] = unsigned( vIdx.x ) ;
            indices[1] = unsigned( vIdx.y ) ;
            indices[2] = unsigned( vIdx.z ) ;
        }


        /*! \brief Compute offset into contents array of a point at a given position

            \index vPosition - position of a point.  It must be within the region of this container.

            \return Offset into contents array of a point at vPosition.

            \see IndicesFromOffset, PositionFromOffset.

            \note Derived class defines the actual contents array.

        */
        unsigned    OffsetOfPosition( const Vec3 & vPosition )
        {
            unsigned indices[3] ;
            IndicesOfPosition( indices , vPosition ) ;
            const unsigned offset = indices[0] + GetNumPoints( 0 ) * ( indices[1] + GetNumPoints( 1 ) * indices[2] ) ;
            return offset ;
        }


        /*! \brief Compute position of minimal corner of grid cell with given indices

            \param position - position of minimal corner of grid cell

            \param indices - grid cell indices.

            \note Rarely if ever would you want to compute position from indices in this way.
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


        /*! \brief Compute X,Y,Z grid cell indices from offset into contents array.

            \param indices - Individual X,Y,Z component grid cell indices.

            \param offset - Offset into mContents.
        */
        void    IndicesFromOffset( unsigned indices[3] , const unsigned & offset )
        {
            indices[2] = offset / ( GetNumPoints(0) * GetNumPoints(1) ) ;
            indices[1] = ( offset - indices[2] * GetNumPoints(0) * GetNumPoints(1) ) / GetNumPoints(0) ;
            indices[0] = offset - GetNumPoints(0) * ( indices[1] + GetNumPoints(1) * indices[2] ) ;
        }


        /*! \brief Get position of grid cell minimum corner.

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


    protected:

        /*! \brief Precompute grid spacing, to optimize OffsetOfPosition and other utility routines.
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




        /*! \brief Return the offset associated with the cell whose indices are second-to-last in each direction.

            Same as nx * ( ny * ( nz - 1 ) - 1 ) - 2.
        */
        unsigned GetOffsetOfPenultimateCell( void )
        {
            return (GetNumPoints(0)-2) + GetNumPoints(0) * ( (GetNumPoints(1)-2) + GetNumPoints(1) * (GetNumPoints(2)-2) ) ;
        }


        /*! \brief Get offset into contents array given indices

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


        /*! \brief Clear out any existing shape information
        */
        void Clear( void )
        {
            mMinCorner      =
            mGridExtent     =
            mCellExtent     =
            mCellsPerExtent = Vec3( 0.0f , 0.0f , 0.0f ) ;
            mNumPoints[ 0 ] = mNumPoints[ 1 ] = mNumPoints[ 2 ] = 0 ;
        }


        Vec3                mMinCorner      ;   ///< Minimum position (in world units) of grid in X, Y and Z directions.
        Vec3                mGridExtent     ;   ///< Size (in world units) of grid in X, Y and Z directions.
        Vec3                mCellExtent     ;   ///< Size (in world units) of a cell.
        Vec3                mCellsPerExtent ;   ///< Reciprocal of cell size (precomputed once to avoid excess divides).
        unsigned            mNumPoints[ 3 ] ;   ///< Number of gridpoints along X, Y and Z directions.
} ;




/*! \brief Templated container for fast spatial lookups and insertions
*/
template <class ItemT> class UniformGrid : public UniformGridGeometry
{
    public:

        typedef UniformGridGeometry Parent ;

        /*! \brief Construct an empty UniformGrid.
            \see Initialize
        */
        UniformGrid() : UniformGridGeometry() { }


        /*! \brief Construct a uniform grid container that fits the given geometry.
            \see Initialize
        */
        UniformGrid( unsigned uNumElements , const Vec3 & vMin , const Vec3 & vMax , bool bPowerOf2 )
            : UniformGridGeometry( uNumElements , vMin , vMax , bPowerOf2 )
        {
        }


        /*! \brief Copy shape from given uniform grid
        */
        explicit UniformGrid( const UniformGridGeometry & that )
            : UniformGridGeometry( that )
        {
        }


        /*! \brief Copy constructor for empty uniform grids

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


              ItemT &       operator[]( const unsigned & offset )       { return mContents[ offset ] ; }
        const ItemT &       operator[]( const unsigned & offset ) const { return mContents[ offset ] ; }

        ItemT &             operator[]( const Vec3 & vPosition )        { return mContents[ OffsetOfPosition( vPosition ) ] ; }


        /*! \brief Initialize contents to whatever default ctor provides.
        */
        void Init( void )
        {
            mContents.Resize( GetGridCapacity() ) ;
        }


        void DefineShape( size_t uNumElements , const Vec3 & vMin , const Vec3 & vMax , bool bPowerOf2 )
        {
            mContents.Clear() ;
            UniformGridGeometry::DefineShape( uNumElements , vMin , vMax , bPowerOf2 ) ;
        }


        /*! \brief Return the number of cells in this grid which have been assigned values.
        */
        size_t Size( void ) const { return mContents.Size() ; }


        /*! \brief Create an empty container based on a given uniform grid container, but with lower resolution

            \param src - Source UniformGridGeometry upon which to base dimensions of this container

            \param iDecimation - amount by which to reduce resolution (number of grid cells in each dimension).
                Typically this would be 2.

        */
        void Decimate( const UniformGridGeometry & src , int iDecimation )
        {
            UniformGridGeometry::Decimate( src , iDecimation ) ;
        }


        static void UnitTest( void ) ;


        /*! \brief Compute statistics of data in a uniform grid.

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



        /*! \brief Interpolate values from grid to get value at given position

            \param vPosition - position to sample

            \return Interpolated value corresponding to value of grid contents at vPosition.

        */
        void Interpolate( ItemT & vResult , const Vec3 & vPosition ) const
        {
            unsigned        indices[3] ; // Indices of grid cell containing position.
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
            vResult = oneMinusTween.x * oneMinusTween.y * oneMinusTween.z * (*this)[ offsetX0Y0Z0 ]
                    +         tween.x * oneMinusTween.y * oneMinusTween.z * (*this)[ offsetX1Y0Z0 ]
                    + oneMinusTween.x *         tween.y * oneMinusTween.z * (*this)[ offsetX0Y1Z0 ]
                    +         tween.x *         tween.y * oneMinusTween.z * (*this)[ offsetX1Y1Z0 ]
                    + oneMinusTween.x * oneMinusTween.y *         tween.z * (*this)[ offsetX0Y0Z1 ]
                    +         tween.x * oneMinusTween.y *         tween.z * (*this)[ offsetX1Y0Z1 ]
                    + oneMinusTween.x *         tween.y *         tween.z * (*this)[ offsetX0Y1Z1 ]
                    +         tween.x *         tween.y *         tween.z * (*this)[ offsetX1Y1Z1 ] ;
        }




        /*! \brief Insert given value into grid at given position
        */
        void Insert( const Vec3 & vPosition , const ItemT & item )
        {
            unsigned        indices[3] ; // Indices of grid cell containing position.
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


        void Clear( void )
        {
            mContents.Clear() ;
            Parent::Clear() ;
        }


        void GenerateBrickOfBytes( const char * strFilenameBase , unsigned uFrame ) const ;


    private:
        Vector<ItemT>       mContents           ;   ///< 3D array of items.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
