/*! \file vorticityDistribution.cpp

    \brief Vorticity distributions for initializing fluid flow fields

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "Core/Math/vec3.h"
#include "Space/uniformGrid.h"

#include "vorticityDistribution.h"




/*! \brief A very small number, between FLT_EPSILON and FLT_MIN.
*/
static const float sTiny = expf( 0.5f * ( logf( FLT_EPSILON ) + logf( FLT_MIN ) ) ) ;




/*! \brief Create a vorticity field using vortex particles

    \param vortons - (out) array of vortex particles

    \param fMagnitude - maximum value of vorticity in the ring

    \param vortexRing - characteristics of a vortex ring

    \param numVortonsMax - maximum number of vortons this routine will generate.
        This routine may (and likely will) generate fewer vortons than this.
        This effectivly specifies the density of vortons, i.e. how finely resolved the vortons will be.
        Suggested value provided should be at least 512, which corresponds to an 8x8x8 grid.

*/
void AssignVorticity( Vector<Vorton> & vortons , float fMagnitude , unsigned numVortonsMax , const IVorticityDistribution & vorticityDistribution )
{
    const Vec3          vDimensions     = vorticityDistribution.GetDomainSize() ;       // length of each side of grid box
    const Vec3          vCenter         ( 0.0f , 0.0f , 0.0f ) ;                        // Center of vorticity distribution
    const Vec3          vMin            ( vCenter - 0.5f * vDimensions ) ;              // Minimum corner of box containing vortons
    const Vec3          vMax            ( vMin + vDimensions ) ;                        // Maximum corner of box containing vortons
    UniformGridGeometry skeleton        ( numVortonsMax , vMin , vMax , true ) ;
    unsigned            numCells[3]     = {     MAX2( 1 , skeleton.GetNumCells(0)) 
                                            ,   MAX2( 1 , skeleton.GetNumCells(1))
                                            ,   MAX2( 1 , skeleton.GetNumCells(2)) } ;  // number of grid cells in each direction of virtual uniform grid

    // Total number of cells should be as close to numVortonsMax as possible without going over.
    // Worst case allowable difference would be numVortonsMax=7 and numCells in each direction is 1 which yields a ratio of 1/7.
    // But in typical situations, the user would like expect total number of virtual cells to be closer to numVortonsMax than that.
    // E.g. if numVortonsMax=8^3=512 somehow yielded numCells[0]=numCells[1]=numCells[2]=7 then the ratio would be 343/512~=0.67.
    while( numCells[0] * numCells[1] * numCells[2] > numVortonsMax )
    {   // Number of cells is excessive.
        // This can happen when the trial number of cells in any direction is less than 1 -- then the other two will likely be too large.
        numCells[0] = MAX2( 1 , numCells[0] / 2 ) ;
        numCells[1] = MAX2( 1 , numCells[1] / 2 ) ;
        numCells[2] = MAX2( 1 , numCells[2] / 2 ) ;
    }

    const float         oneOverN[3]     = { 1.0f / float( numCells[0] ) , 1.0f / float( numCells[1] ) , 1.0f / float( numCells[2] ) } ;
    const Vec3          gridCellSize    ( vDimensions.x * oneOverN[0] , vDimensions.y * oneOverN[1] , vDimensions.z * oneOverN[2] ) ;
    float               vortonRadius    = powf( gridCellSize.x * gridCellSize.y * gridCellSize.z , 1.0f / 3.0f ) * 0.5f ;
    if( 0.0f == vDimensions.z )
    {   // z size is zero, so domain is 2D.
        vortonRadius = powf( gridCellSize.x * gridCellSize.y , 0.5f ) * 0.5f ;
    }
    const Vec3          vNoise          ( 0.0f * gridCellSize ) ;

    Vec3        position            = Vec3( 0.0f , 0.0f , 0.0f ) ;                              // vorton position
    unsigned    index[3]            ;   // index of each position visited
    // Iterate through each point in a uniform grid.
    // If probe position is inside vortex core, add a vorton there.
    // This loop could be rewritten such that it only visits points inside the core,
    // but this loop structure can readily be reused for a wide variety of configurations.
    for( index[2] = 0 ; index[2] < numCells[2] ; ++ index[2] )
    {   // For each z-coordinate...
        position.z = ( float( index[2] ) + 0.25f ) * gridCellSize.z + vMin.z ;
        for( index[1] = 0 ; index[1] < numCells[1] ; ++ index[1] )
        {   // For each y-coordinate...
            position.y = ( float( index[1] ) + 0.25f ) * gridCellSize.y + vMin.y ;
            for( index[0] = 0 ; index[0] < numCells[0] ; ++ index[0] )
            {   // For each x-coordinate...
                position.x = ( float( index[0] ) + 0.25f ) * gridCellSize.x + vMin.x ;
                position += RandomSpread( vNoise ) ;
                Vec3 vorticity ;
                vorticityDistribution.AssignVorticity( vorticity , position , vCenter ) ;
                Vorton vorton( position , vorticity * fMagnitude , vortonRadius ) ;
                if( vorticity.Mag2() > sTiny )
                {   // Vorticity is significantly non-zero.
                    vortons.PushBack( vorton ) ;
                }
            }
        }
    }
}
