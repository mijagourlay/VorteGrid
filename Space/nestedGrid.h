/*! \file nestedGrid.h

    \brief Templated nested uniform grid container, a hierarchical, octree-like spatial partition

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef NESTED_GRID_H
#define NESTED_GRID_H

#include <math.h>

#include "Core/Math/vec3.h"

#include "uniformGrid.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Templated nested uniform grid container, a hierarchical, octree-like spatial partition

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

*/
template <class ItemT> class NestedGrid
{
    public:
        typedef UniformGrid< ItemT > Layer ;    ///< Abbreviation for UniformGrid<ItemT>

        /*! \brief Construct a blank nested uniform grid spatial partition
        */
        NestedGrid()
            : mDecimations( 0 )
        {
        }


        /*! \brief Construct an unpopulated nested uniform grid spatial partition, based on a given UniformGrid

            \see Initialize

        */
        NestedGrid( const Layer & src )
            : mDecimations( 0 )
        {
            Initialize( src ) ;
        }


        /*! \brief Construct a blank nested uniform grid spatial partition
        */
        ~NestedGrid()
        {
            delete [] mDecimations ;
        }


        /*! \brief Initialize an unpopulated nested uniform grid spatial partition, based on a given UniformGrid

            \param src - UniformGrid upon which this NestedGrid is based.

        */
        void Initialize( const Layer & src )
        {
            mLayers.Clear() ;
            const unsigned numLayers = PrecomputeNumLayers( src ) ;
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


        /*! \brief Add a layer to the top of the nested grid.

            \param layerTemplate - UniformGridGeometry defining child layer.

            \param iDecimation - amount by which to decimate child layer, in each direction.

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


        /*! \brief Return number of layers in tree
        */
        size_t GetDepth( void ) const { return mLayers.Size() ; }


        /*! \brief Get layer, a uniform grid, at specified depth of this tree

            \param index - depth of layer to obtain, where 0 means leaf layer and GetDepth()-1 means root layer.

            \return UniformGrid<ItemT> of specified layer.
        */
              Layer & operator[]( size_t index )       { return mLayers[ index ] ; }
        const Layer & operator[]( size_t index ) const { return mLayers[ index ] ; }


        const unsigned * GetDecimations( size_t iParentLayer ) const
        {
            return mDecimations[ iParentLayer ] ;
        }


        /*! \brief Get indices of minimal cell in child layer of cluster represented by specified cell in parent layer.

            Each cell in a parent layer represents a grid cluster of typically 8 cells
            in the child layer.  This routine calculates the index of the "minimal"
            cell in the child layer grid cluster, i.e. the cell in the child layer
            which corresponds to minimum corner cell of the grid cluster represented
            by the cell in the parent layer with the specified index.

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
        void GetChildClusterMinCornerIndex( unsigned clusterMinIndices[3] , const unsigned decimations[3] , const unsigned indicesOfParentCell[3] )
        {
            clusterMinIndices[ 0 ] = indicesOfParentCell[ 0 ] * decimations[ 0 ] ;
            clusterMinIndices[ 1 ] = indicesOfParentCell[ 1 ] * decimations[ 1 ] ;
            clusterMinIndices[ 2 ] = indicesOfParentCell[ 2 ] * decimations[ 2 ] ;
        }

        void Clear( void )
        {
            for( unsigned iLayer = 0 ; iLayer < GetDepth() ; ++ iLayer )
            {
                mLayers[ iLayer ].Clear() ;
            }
            mLayers.Clear() ;
        }

        static void UnitTest( void ) ;

    private:
        NestedGrid( const NestedGrid & that ) ; // Disallow copy construction
        NestedGrid & operator= ( const NestedGrid & that ) ; // Disallow copy


        /*! \brief Precompute the total number of layers this nested grid will contain

            \param src - UniformGrid upon which this NestedGrid is based.

        */
        unsigned PrecomputeNumLayers( const Layer & src )
        {
            unsigned numLayers      = 1 ;    // Tally src layer.
            unsigned numPoints[3]   = { src.GetNumPoints( 0 ) , src.GetNumPoints( 1 ) , src.GetNumPoints( 2 ) } ;
            unsigned size = numPoints[0] * numPoints[1] * numPoints[2] ;
            while( size > 8 /* a cell has 8 corners */ )
            {   // Layer has more than 1 cell.
                ++ numLayers ;
                // Decimate number of cells (where #cells = #points-1):
                numPoints[0] = MAX2( ( numPoints[0] - 1 ) / 2 , 1 ) + 1 ;
                numPoints[1] = MAX2( ( numPoints[1] - 1 ) / 2 , 1 ) + 1 ;
                numPoints[2] = MAX2( ( numPoints[2] - 1 ) / 2 , 1 ) + 1 ;
                size = numPoints[0] * numPoints[1] * numPoints[2] ;
            }
            return numLayers ;
        }


        /*! \brief Compute decimations, in each direction, for specified parent layer

            \param decimations - (out) ratio of dimensions between child layer and its parent.

            \param iParentLayer - index of parent layer.
                                Child has index iParentLayer-1.
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
        void ComputeDecimations( unsigned decimations[3] , unsigned iParentLayer ) const
        {
            const Layer & parent = (*this)[ iParentLayer     ] ;
            const Layer & child  = (*this)[ iParentLayer - 1 ] ;
            decimations[ 0 ] = child.GetNumCells( 0 ) / parent.GetNumCells( 0 ) ;
            decimations[ 1 ] = child.GetNumCells( 1 ) / parent.GetNumCells( 1 ) ;
            decimations[ 2 ] = child.GetNumCells( 2 ) / parent.GetNumCells( 2 ) ;
        }


        /*! \brief Precompute decimations for each layer.

            This provides the number of grid cells per cluster
            of a child of each layer.

            \note The child layer has index one less than the parent layer index.
                    That implies there is no such thing as "parent layer 0".
                    Layer 0  has no children. That further implies there is no
                    meaningful value for decimations at iParentLayer==0.

        */
        void PrecomputeDecimations( void )
        {
            const size_t numLayers = GetDepth() ;

            delete [] mDecimations ;    // Delete old decimations array
            
            // Precompute decimations for each layer.
            mDecimations = new unsigned[ numLayers ][3] ;
            for( unsigned iLayer = 1 ; iLayer < numLayers ; ++ iLayer )
            {   // For each parent layer...
                ComputeDecimations( mDecimations[ iLayer ] , iLayer ) ;
            }
            // Layer 0 is strictly a child (i.e. has no children), so has no decimations.
            // Assign the values with useless nonsense to make this more obvious.
            mDecimations[0][0] = mDecimations[0][1] = mDecimations[0][2] = 0 ;
        }


        Vector< Layer > mLayers             ;   ///< Dynamic array of UniformGrids.
        unsigned     (* mDecimations)[3]    ;   ///< Cache of cluster sizes
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
