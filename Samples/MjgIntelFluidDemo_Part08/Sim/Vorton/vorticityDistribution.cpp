/*! \file vorticityDistribution.cpp

    \brief Vorticity distributions for initializing fluid flow fields

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "Core/Math/vec3.h"
#include "Space/uniformGrid.h"

#include "vorticityDistribution.h"




/*! \brief A very small number, between FLT_EPSILON and FLT_MIN.
*/
const float sNegligibleEnstrophyThreshold = expf( 0.5f * ( logf( FLT_EPSILON ) + logf( FLT_MIN ) ) ) ;
const float sNegligibleVorticityThreshold = sqrtf( sNegligibleEnstrophyThreshold ) ;




/*! \brief Addend 2 place-holder vortons at the corners of a bounding box

    \param vortons - (out) array of vortex particles

    \param vMin - position of minimal corner

    \param vMax - position of maximal corner

    Create 2 "placeholder" vortons that define the corners of the grid.
    These define the size of the grid.  The simulation fits the grid tightly
    and we want the vortons' initial positions to reside in the center
    of the grid cells -- except these 2 corner vortons.

*/
void AddCornerVortons( Vector<Vorton> & vortons , const Vec3 & vMin , const Vec3 & vMax , float vortonRadius )
{
}




/*! \brief Create a vorticity field using vortex particles

    \param vortons - (out) array of vortex particles

    \param fMagnitude - maximum value of vorticity in the distribution

    \param numVortonsMax - maximum number of vortons this routine will generate.
        This routine may (and likely will) generate fewer vortons than this.
        This effectivly specifies the density of vortons, i.e. how finely resolved the vortons will be.
        Suggested value provided should be at least 512, which corresponds to an 8x8x8 grid.

    \param vorticityDistribution - characteristics of vorticity distribution

    \param translation - spatial offset applied to each vorton.  Useful if you
                    want to move the vorticity to a given location,
                    because typically a vorticity distribution will
                    be centered about its local origin.  Think of
                    "translation" as a local-to-world translation.

*/
void AssignVortons( Vector<Vorton> & vortons , float fMagnitude , unsigned numVortonsMax , const IVorticityDistribution & vorticityDistribution , Vec3 translation )
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

    const float         sizeFactor      = 1.0f ;
    const float         volRatio        = 1.0f / POW3( sizeFactor ) ;
    const float         oneOverN[3]     = { 1.0f / float( numCells[0] ) , 1.0f / float( numCells[1] ) , 1.0f / float( numCells[2] ) } ;
    const Vec3          gridCellSize    ( vDimensions.x * oneOverN[0] , vDimensions.y * oneOverN[1] , vDimensions.z * oneOverN[2] ) ;
    float               vortonRadius    = powf( gridCellSize.x * gridCellSize.y * gridCellSize.z , 1.0f / 3.0f ) * 0.5f * sizeFactor ;
    if( 0.0f == vDimensions.z )
    {   // z size is zero, so domain is 2D.
        vortonRadius = powf( gridCellSize.x * gridCellSize.y , 0.5f ) * 0.5f ;
    }

#if 0
    // Create 2 "placeholder" vortons that define the corners of the grid.
    // These define the size of the grid.  The simulation fits the grid tightly
    // and we want the vortons' initial positions to reside in the center
    // of the grid cells -- except these 2 corner vortons.
    AddCornerVortons( vortons , vMin , vMin + vDimensions , vortonRadius ) ;
#endif

    Vec3        positionLocal            = Vec3( 0.0f , 0.0f , 0.0f ) ;  // vorton positionLocal
    unsigned    index[3]            ;   // index of each positionLocal visited
    // Iterate through each point in a uniform grid.
    for( index[2] = 0 ; index[2] < numCells[2] ; ++ index[2] )
    {   // For each z-coordinate...
        positionLocal.z = ( float( index[2] ) + 0.25f ) * gridCellSize.z + vMin.z ;
        for( index[1] = 0 ; index[1] < numCells[1] ; ++ index[1] )
        {   // For each y-coordinate...
            positionLocal.y = ( float( index[1] ) + 0.25f ) * gridCellSize.y + vMin.y ;
            for( index[0] = 0 ; index[0] < numCells[0] ; ++ index[0] )
            {   // For each x-coordinate...
                positionLocal.x = ( float( index[0] ) + 0.25f ) * gridCellSize.x + vMin.x ;
                Vec3 vorticity ;
                float density ;
                vorticityDistribution.AssignVorton( vorticity , density , positionLocal , vCenter ) ;
                const Vec3 positionGlobal = positionLocal + translation ;
                Vorton vorton( positionGlobal , vorticity * fMagnitude * volRatio , vortonRadius ) ;
                if( vorticity.Mag2() > sNegligibleEnstrophyThreshold )
                {   // Vorticity is significantly non-zero.
                    vorton.mDensity = density ;
                    vortons.PushBack( vorton ) ;
                }
            }
        }
    }
}
