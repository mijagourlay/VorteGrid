/** \file nestedGrid.h

    \brief Templated nested uniform grid container, a hierarchical, octree-like spatial partition

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.mijagourlay.com/

    \author Written and copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

*/
#ifndef NESTED_GRID_H
#define NESTED_GRID_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "Core/Memory/newWrapper.h"

#include "Core/Performance/perfBlock.h"

#include "Core/SpatialPartition/uniformGrid.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/** Templated nested uniform grid container, a hierarchical, octree-like spatial partition.
*/
template <class ItemT> class NestedGrid
{
    public:

        typedef UniformGrid< ItemT > Layer ;    ///< Abbreviation for UniformGrid<ItemT>

        /** Construct a blank nested uniform grid spatial partition.
        */
        NestedGrid()
            : mDecimations( 0 )
        {
        }


        /** Construct an unpopulated nested uniform grid spatial partition, based on a given UniformGrid.

            \see Initialize

        */
        NestedGrid( const UniformGridGeometry & src )
            : mDecimations( 0 )
        {
            Initialize( src ) ;
        }


        /** Destruct a nested uniform grid spatial partition.
        */
        ~NestedGrid()
        {
            delete [] mDecimations ;
        }


        /** Initialize an unpopulated nested uniform grid spatial partition, based on a given UniformGrid.

            \param src - UniformGrid upon which this NestedGrid is based.

        */
        void Initialize( const UniformGridGeometry & src )
        {
            PERF_BLOCK( NestedGrid__Initialize ) ;

            mLayers.Clear() ;
            const size_t numLayers = PrecomputeNumLayers( src ) ;
            mLayers.Reserve( numLayers ) ;  // Preallocate number of layers to avoid reallocation during PushBack.
            AddLayer( src , 1 ) ;
            unsigned index = 1 ;
            while( mLayers[ index-1 ].GetGridCapacity() > 8 /* a cell has 8 corners */ )
            {   // Layer to decimate has more than 1 cell.
                AddLayer( mLayers[ index - 1 ] , 2 ) ; // Initialize child layer based on decimation of its parent grid.
                ++ index ;
            }

            PrecomputeDecimations() ;
        }


        /** Add a layer to the top of the nested grid.

            \param layerTemplate - UniformGridGeometry defining child layer.

            \param iDecimation - Amount by which to decimate child layer, in each direction.

            This facilitates building the tree from leaves to root.
            This method also pre-allocates memory for the newly added layer,
            and initializes its contents to whatever the default constructor returns.

        */
        void AddLayer( const UniformGridGeometry & layerTemplate , unsigned iDecimation )
        {
            mLayers.PushBack( Layer() ) ;
            mLayers.Back().Decimate( layerTemplate , iDecimation ) ;
            mLayers.Back().Init() ;
        }


        /** Return number of layers in tree.
        */
        size_t GetDepth( ) const { return mLayers.Size() ; }


        /** Get layer, a uniform grid, at specified depth of this tree.

            \param index - depth of layer to obtain, where 0 means leaf layer and GetDepth()-1 means root layer.

            \return UniformGrid<ItemT> of specified layer.
        */
              Layer & operator[]( size_t index )       { return mLayers[ index ] ; }
        const Layer & operator[]( size_t index ) const { return mLayers[ index ] ; }


        /** Return decimations array for the given parent layer.

            A NestedGrid is a collection of layers of UniformGrid objects, where
            each layer is the parent of the child below it.  The dimensions
            (that is, the number of grid cells) of each parent are smaller than
            the dimensions of its child, by a uniform amount, which is the
            "decimation".  Note that each grid has 3 dimensions, and each
            dimension has a separate decimation.

            A trio of decimation values {i,j,k} also constitutes the dimensions
            of a "cluster" of grid cells in the child layer that corrispond
            to a grid cell in the parent layer.

            \see AddLayer, UniformGrid::Decimate, ComputeDecimations.
        */
        const unsigned * GetDecimations( size_t uParentLayer ) const
        {
            ASSERT( uParentLayer > 0 ) ;
            ASSERT( uParentLayer < GetDepth() ) ;
            return mDecimations[ uParentLayer ] ;
        }


        /** Get indices of minimal cell in child layer of cluster represented by specified cell in parent layer.

            Each cell in a parent layer represents a grid cluster of typically
            2*2*2=8 cells in the child layer.  This routine calculates the index of
            the "minimal" cell in the child layer grid cluster, i.e. the cell in the
            child layer which corresponds to minimum corner cell of the grid cluster
            represented by the cell in the parent layer with the specified index.

            The cells in the child layer that belong to the same grid cluster would
            be visited by this code:

            \verbatim

                int i[3] ; // i is the increment past the minimum corner cell in the grid cluster.
                int j[3] ; // j indexes into the child layer.
                for( i[2] = 0 ; i[2] <= decimations[2] ; ++ i[2] )
                {
                    j[2] = i[2] + clusterMinIndices[2] ;
                    for( i[1] = 0 ; i[1] <= decimations[1] ; ++ i[1] )
                    {
                        j[1] = i[1] + clusterMinIndices[1] ;
                        for( i[0] = 0 ; i[0] <= decimations[0] ; ++ i[0] )
                        {
                            j[0] = i[0] + clusterMinIndices[0] ;
                            // Use j to index into child layer.
                        }
                    }
                }

            \endverbatim

            \param clusterMinIndices - (out) index of minimal cell in child layer grid cluster represented by given parent cell

            \param decimations - (in) ratios of dimensions of child layer to its parent, for each axis.
                    This must be the same as the result of calling GetDecimations for the intended parent layer.

            \param indicesOfParentCell - (in) index of cell in parent layer.

            \see GetDecimations

        */
        static void GetChildClusterMinCornerIndex( unsigned clusterMinIndices[3] , const unsigned decimations[3] , const unsigned indicesOfParentCell[3] )
        {
            clusterMinIndices[ 0 ] = indicesOfParentCell[ 0 ] * decimations[ 0 ] ;
            clusterMinIndices[ 1 ] = indicesOfParentCell[ 1 ] * decimations[ 1 ] ;
            clusterMinIndices[ 2 ] = indicesOfParentCell[ 2 ] * decimations[ 2 ] ;
        }


        /** Get the indices into the parent layer, given the indices into a child layer.

            \param parentIndices    (out) Indices into the parent layer.

            \param childIndices     (in) Indices into a child layer.

            \param uParentLayer     Index of the parent layer.  Must exceed zero.
                                    (Child has index uParentLayer-1.)

        */
        void GetParentIndices( unsigned parentIndices[ 3 ] , const unsigned childIndices[ 3 ] , size_t uParentLayer )
        {
            ASSERT( uParentLayer > 0 ) ;
            ASSERT( uParentLayer < GetDepth() ) ;
            parentIndices[ 0 ] = childIndices[ 0 ] / mDecimations[ uParentLayer ][ 0 ] ;
            parentIndices[ 1 ] = childIndices[ 1 ] / mDecimations[ uParentLayer ][ 1 ] ;
            parentIndices[ 2 ] = childIndices[ 2 ] / mDecimations[ uParentLayer ][ 2 ] ;
        }


        /// Return whether this container is empty.
        bool Empty() const
        {
            return mLayers.Empty() ;
        }


        /// Clear the contents of this NestedGrid.
        void Clear( )
        {
            PERF_BLOCK( NestedGrid__Clear ) ;

            for( unsigned iLayer = 0 ; iLayer < GetDepth() ; ++ iLayer )
            {
                mLayers[ iLayer ].Clear() ;
            }
            mLayers.Clear() ;
        }




        /** Restrict value from a child layer into a parent layer of the multi-resolution grid.

            In the vernacular of multi-grid solvers, the "restrict" operation reduces
            the resolution of a grid, so this operation is tantamount to down-sampling.
            It contrasts with the "interpolate" operation, which up-samples.

            This routine assumes the given parent layer is empty and its child layer (i.e. the layer
            with index uParentLayer-1) is populated.

            \param uParentLayer - index of parent layer into which aggregated information will be stored.
                This must be greater than 0 because the base layer, which has no children, has index 0.

                \note The parent layer must have 3 or greater points in at least one of its dimensions.

            \see SolvePoissonMultiGrid UpSampleFrom

        */
        void DownSampleInto( unsigned uParentLayer , UniformGridGeometry::AccuracyVersusSpeedE accuracyVsSpeed = UniformGridGeometry::SLOWER_MORE_ACCURATE )
        {
            PERF_BLOCK( NestedGrid__DownSampleInto ) ;

            ASSERT( uParentLayer > 0 ) ;
            ASSERT( uParentLayer < GetDepth() ) ;
            Layer   &   rParentLayer    = operator[]( uParentLayer     ) ;
            Layer   &   rChildLayer     = operator[]( uParentLayer - 1 ) ;
            rParentLayer.DownSample( rChildLayer , accuracyVsSpeed ) ;
        }




        /** Interpolate value from a parent layer into a child layer of the multi-resolution grid.

            In the vernacular of multi-grid solvers, the "interpolate" operation increases
            the resolution of a grid, so this operation is tantamount to up-sampling.
            It contrasts with the "restrict" operation, which down-samples.

            This routine assumes the given child layer (that is, the layer with index uParentLayer-1)
            is empty and its parent layer (i.e. the layer with index uParentLayer) is populated.

            \param uParentLayer - index of parent layer from which information will be read.
                This must be greater than 0 because the base layer, which has no children, has index 0.

                \note The parent layer must have 3 or greater points in at least one of its dimensions.

            \see SolvePoissonMultiGrid DownSampleInto

        */
        void UpSampleFrom( unsigned uParentLayer , UniformGridGeometry::RegionE region = UniformGridGeometry::ENTIRE_DOMAIN )
        {
            PERF_BLOCK( NestedGrid__UpSampleFrom ) ;

            ASSERT( uParentLayer > 0 ) ;
            ASSERT( uParentLayer < GetDepth() ) ;
            Layer           &       rParentLayer        = operator[]( uParentLayer     ) ;
            Layer           &       rChildLayer         = operator[]( uParentLayer - 1 ) ;
            rChildLayer.UpSample( rParentLayer , region ) ;
        }




        static void UnitTest( ) ;



    private:
        NestedGrid( const NestedGrid & that ) ; ///< Disallow copy construction
        NestedGrid & operator= ( const NestedGrid & that ) ; ///< Disallow copy


        /** Precompute the total number of layers this nested grid will contain.

            \param src - UniformGrid upon which this NestedGrid is based.

        */
        size_t PrecomputeNumLayers( const UniformGridGeometry & src )
        {
            size_t numLayers      = 1 ;    // Tally src layer.
            size_t numPoints[3]   = { src.GetNumPoints( 0 ) , src.GetNumPoints( 1 ) , src.GetNumPoints( 2 ) } ;
            size_t size = numPoints[0] * numPoints[1] * numPoints[2] ;
            while( size > 8 /* a cell has 8 corners */ )
            {   // Layer has more than 1 cell.
                ++ numLayers ;
                // Decimate number of cells (where #cells = #points-1):
#pragma warning( push )
#pragma warning( disable : 4267 ) // conversion from 'size_t' to 'const unsigned int', possible loss of data
                numPoints[0] = Max2( ( numPoints[0] - 1 ) / 2 , size_t( 1 ) ) + 1 ;
                numPoints[1] = Max2( ( numPoints[1] - 1 ) / 2 , size_t( 1 ) ) + 1 ;
                numPoints[2] = Max2( ( numPoints[2] - 1 ) / 2 , size_t( 1 ) ) + 1 ;
#pragma warning( pop )
                size = numPoints[0] * numPoints[1] * numPoints[2] ;
            }
            return numLayers ;
        }


        /** Compute decimations, in each direction, for specified parent layer.

            \param decimations - (out) ratio of dimensions between child layer and its parent.

            \param uParentLayer - index of parent layer.
                                Child has index uParentLayer-1.
                                Layer 0 has no child so providing "0" is invalid.

            This method effectively gives the number of child cells in each
            grid cluster that a parent cell represents.

            Each non-leaf layer in this NestedGrid is a decimation of its child
            layer. Typically that decimation is 2 in each direction, but the
            decimation can also be 1, or, more atypically, any other integer.
            Each child typically has twice as many cells in each direction as
            its parent.

            \note This assumes each parent has an integer decimation of its child.

            \see GetDecimations

        */
        void ComputeDecimations( unsigned decimations[3] , unsigned uParentLayer ) const
        {
            const Layer & parent = (*this)[ uParentLayer     ] ;
            const Layer & child  = (*this)[ uParentLayer - 1 ] ;
            decimations[ 0 ] = child.GetNumCells( 0 ) / parent.GetNumCells( 0 ) ;
            decimations[ 1 ] = child.GetNumCells( 1 ) / parent.GetNumCells( 1 ) ;
            decimations[ 2 ] = child.GetNumCells( 2 ) / parent.GetNumCells( 2 ) ;
        }


        /** Precompute decimations for each layer.

            This provides the number of grid cells per cluster
            of a child of each layer.

            \note The child layer has index one less than the parent layer index.
                    That implies there is no such thing as "parent layer 0".
                    Layer 0  has no children. That further implies there is no
                    meaningful value for decimations at uParentLayer==0.

        */
        void PrecomputeDecimations( void )
        {
            const size_t numLayers = GetDepth() ;

            delete [] mDecimations ;    // Delete old decimations array
            
            // Precompute decimations for each layer.
            mDecimations = NEW unsigned[ numLayers ][3] ;
            for( unsigned uParentLayer = 1 ; uParentLayer < numLayers ; ++ uParentLayer )
            {   // For each parent layer...
                ComputeDecimations( mDecimations[ uParentLayer ] , uParentLayer ) ;
            }
            // Layer 0 is strictly a child (i.e. has no children), so has no decimations.
            // Assign the values with useless nonsense to make this more obvious.
            mDecimations[0][0] = mDecimations[0][1] = mDecimations[0][2] = 0 ;
        }




        VECTOR< Layer > mLayers             ;   ///< Dynamic array of UniformGrid, one for each layer.
        unsigned     (* mDecimations)[3]    ;   ///< Cache of cluster sizes.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
