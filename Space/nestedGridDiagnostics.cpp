/*! \file NestedGrid.cpp

    \brief Templated nested uniform grid container, a hierarchical, octree-like spatial partition

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "nestedGrid.h"



/* static */ void NestedGrid<unsigned>::UnitTest( void )
{
#if defined( _DEBUG )
    static unsigned const num = 1024 ;
    static const Vec3 vRange( 2.0f , 3.0f , 5.0f ) ; // range of random positions
    Vec3 vMin( -0.5f * vRange ) ;   // Minimum coordinate of UniformGrid.
    Vec3 vMax(  0.5f * vRange ) ;   // Maximum coordinate of UniformGrid.
    UniformGrid<unsigned> ug( num , vMin , vMax , true ) ;

    NestedGrid<unsigned> ng0( ug ) ;

    UniformGrid<unsigned> & ngRoot = ng0[ 0 ] ;
    ASSERT( & ngRoot != & ug ) ; // layer in nested grid should be a copy, not the original.
    ASSERT( ngRoot.GetNumPoints( 0 ) == ug.GetNumPoints( 0 ) ) ;    // Root layer should have identical storage as original UniformGrid.
    ASSERT( ngRoot.GetNumPoints( 1 ) == ug.GetNumPoints( 1 ) ) ;
    ASSERT( ngRoot.GetNumPoints( 2 ) == ug.GetNumPoints( 2 ) ) ;
    ASSERT( ngRoot.GetExtent() == ug.GetExtent() ) ; // Root layer should have same geometry as original UniformGrid.

    ASSERT( ng0.GetDepth() == 5 ) ;
    for( unsigned iLayer = ng0.GetDepth() - 1 ; iLayer > 0 ; -- iLayer )
    {
        unsigned decimations[ 3 ] ;
        ng0.ComputeDecimations( decimations , iLayer ) ;
        ASSERT( ( decimations[0] >= 1 ) && ( decimations[0] <= 2 ) ) ;
        ASSERT( ( decimations[1] >= 1 ) && ( decimations[1] <= 2 ) ) ;
        ASSERT( ( decimations[2] >= 1 ) && ( decimations[2] <= 2 ) ) ;
        unsigned indexIntoChild[ 3 ] ;
        unsigned indexOfParentCell[ 3 ] = { 0 , 0 , 0 } ;
        UniformGrid<unsigned> & layerParent = ng0[ iLayer     ] ;
        UniformGrid<unsigned> & layerChild  = ng0[ iLayer - 1 ] ;
        ASSERT( layerChild.GetNumCells( 0 ) == layerParent.GetNumCells( 0 ) * decimations[ 0 ] ) ;
        ASSERT( layerChild.GetNumCells( 1 ) == layerParent.GetNumCells( 1 ) * decimations[ 1 ] ) ;
        ASSERT( layerChild.GetNumCells( 2 ) == layerParent.GetNumCells( 2 ) * decimations[ 2 ] ) ;
        const Vec3 &        vGridMinCornerParent    = layerParent.GetMinCorner() ;
        const Vec3          vSpacingParent          = layerParent.GetCellSpacing() ;
        const Vec3 &        vGridMinCornerChild     = layerChild.GetMinCorner() ;
        const Vec3          vSpacingChild           = layerChild.GetCellSpacing() ;
        ASSERT( vSpacingParent.x == vSpacingChild.x * decimations[0] ) ;
        ASSERT( vSpacingParent.y == vSpacingChild.y * decimations[1] ) ;
        ASSERT( vSpacingParent.z == vSpacingChild.z * decimations[2] ) ;
        for( indexOfParentCell[2] = 0 ; indexOfParentCell[2] < layerParent.GetNumPoints( 2 ) ; ++ indexOfParentCell[2] )
        {
            Vec3 vCellMinCornerParent , vCellMaxCornerParent ;
            vCellMinCornerParent.z = vGridMinCornerParent.z + float( indexOfParentCell[2]     ) * vSpacingParent.z ;
            vCellMaxCornerParent.z = vGridMinCornerParent.z + float( indexOfParentCell[2] + 1 ) * vSpacingParent.z ;
            for( indexOfParentCell[1] = 0 ; indexOfParentCell[1] < layerParent.GetNumPoints( 1 ) ; ++ indexOfParentCell[1] )
            {
                vCellMinCornerParent.y = vGridMinCornerParent.y + float( indexOfParentCell[1]     ) * vSpacingParent.y ;
                vCellMaxCornerParent.y = vGridMinCornerParent.y + float( indexOfParentCell[1] + 1 ) * vSpacingParent.y ;
                for( indexOfParentCell[0] = 0 ; indexOfParentCell[0] < layerParent.GetNumPoints( 0 ) ; ++ indexOfParentCell[0] )
                {
                    vCellMinCornerParent.x = vGridMinCornerParent.x + float( indexOfParentCell[0]     ) * vSpacingParent.x ;
                    vCellMaxCornerParent.x = vGridMinCornerParent.x + float( indexOfParentCell[0] + 1 ) * vSpacingParent.x ;

                    ng0.GetChildClusterMinCornerIndex( indexIntoChild , decimations , indexOfParentCell ) ;
                    ASSERT( ( indexIntoChild[0] >= 0 ) && ( indexIntoChild[0] >= indexOfParentCell[0] ) && ( indexIntoChild[0] <= layerChild.GetNumPoints( 0 ) ) ) ;
                    ASSERT( ( indexIntoChild[1] >= 0 ) && ( indexIntoChild[1] >= indexOfParentCell[1] ) && ( indexIntoChild[1] <= layerChild.GetNumPoints( 1 ) ) ) ;
                    ASSERT( ( indexIntoChild[2] >= 0 ) && ( indexIntoChild[2] >= indexOfParentCell[2] ) && ( indexIntoChild[2] <= layerChild.GetNumPoints( 2 ) ) ) ;

                    Vec3 vCellMinCornerChild , vCellMaxCornerChild ;
                    vCellMinCornerChild.x = vGridMinCornerChild.x + float( indexIntoChild[0]                  ) * vSpacingChild.x ;
                    vCellMaxCornerChild.x = vGridMinCornerChild.x + float( indexIntoChild[0] + decimations[0] ) * vSpacingChild.x ;
                    vCellMinCornerChild.y = vGridMinCornerChild.y + float( indexIntoChild[1]                  ) * vSpacingChild.y ;
                    vCellMaxCornerChild.y = vGridMinCornerChild.y + float( indexIntoChild[1] + decimations[1] ) * vSpacingChild.y ;
                    vCellMinCornerChild.z = vGridMinCornerChild.z + float( indexIntoChild[2]                  ) * vSpacingChild.z ;
                    vCellMaxCornerChild.z = vGridMinCornerChild.z + float( indexIntoChild[2] + decimations[2] ) * vSpacingChild.z ;

                    ASSERT( vCellMinCornerParent.Resembles( vCellMinCornerChild ) ) ;
                    ASSERT( vCellMaxCornerParent.Resembles( vCellMaxCornerChild ) ) ;

                    //fprintf( stderr , "layer[%u] decimations={%u,%u,%u} iParent={%2u,%2u,%2u} iChild={%2u,%2u,%2u}\n" , iLayer ,
                    //    decimations[0] , decimations[1] , decimations[2] , indexOfParentCell[0] , indexOfParentCell[1] , indexOfParentCell[2] ,
                    //    indexIntoChild[0] , indexIntoChild[1] , indexIntoChild[2] ) ;
                }
            }
        }
        //fprintf( stderr , "\n" ); // Put vertical white space between printouts for each layer.
    }
#endif
}