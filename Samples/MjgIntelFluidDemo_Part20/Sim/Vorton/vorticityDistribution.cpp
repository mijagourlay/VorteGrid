/** \file vorticityDistribution.cpp

    \brief Vorticity distributions for initializing fluid flow fields

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-18/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "vorticityDistribution.h"

#include "Core/Math/vec3.h"
#include "Core/SpatialPartition/uniformGrid.h"
#include "Core/Performance/perfBlock.h"



/// Value of entrophy below which it is treated as zero.
const float sNegligibleEnstrophyThreshold = expf( 0.5f * ( logf( FLT_EPSILON ) + logf( FLT_MIN ) ) ) ;

/// Value of vorticity magnitude below which it is treated as zero.
const float sNegligibleVorticityThreshold = sqrtf( sNegligibleEnstrophyThreshold ) ;




/** Addend 2 place-holder vortons at the corners of a bounding box.

    \param vortons (out) Array of vortex particles.

    \param vMin     Position of minimal corner.

    \param vMax     Position of maximal corner.

    \param vortonRadius Radius of each vorton to add.

    Create 2 "placeholder" vortons that define the corners of the grid.
    These define the size of the grid.  The simulation fits the grid tightly
    and we want the vortons' initial positions to reside in the center
    of the grid cells -- except these 2 corner vortons.

*/
void AddCornerVortons( VECTOR<Vorton> & vortons , const Vec3 & vMin , const Vec3 & vMax , float vortonRadius )
{
    PERF_BLOCK( AddCornerVortons ) ;

#if defined( _DEBUG )
    Vorton vorton ;
    vorton.SetVorticity( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
    vorton.SetRadius( vortonRadius ) ;
    vorton.mPosition                = vMin * ( 1.0f - FLT_EPSILON ) ;
    vortons.PushBack( vorton ) ;
    vorton.mPosition                = vMax * ( 1.0f - FLT_EPSILON ) ;
    vortons.PushBack( vorton ) ;
#else
    (void) vortons , vMin , vMax , vortonRadius ; // Avoid "unreferenced local parameter" warning.
#endif
}




/** Create a vorticity field using vortex particles.

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
void AssignVortons( VECTOR<Vorton> & vortons , float fMagnitude , unsigned numVortonsMax , const IVorticityDistribution & vorticityDistribution , Vec3 translation )
{
    PERF_BLOCK( AssignVortons ) ;

    const Vec3          vDimensions     = vorticityDistribution.GetDomainSize() ;       // length of each side of grid box
    const Vec3          vCenter         ( 0.0f , 0.0f , 0.0f ) ;                        // Center of vorticity distribution
    const Vec3          vMin            ( vCenter - 0.5f * vDimensions ) ;              // Minimum corner of box containing vortons
    const Vec3          vMax            ( vMin + vDimensions ) ;                        // Maximum corner of box containing vortons
    UniformGridGeometry skeleton        ( numVortonsMax , vMin , vMax , true ) ;
    size_t              numCells[3]     = {     Max2( size_t( 1 ) , skeleton.GetNumCells(0)) 
                                            ,   Max2( size_t( 1 ) , skeleton.GetNumCells(1))
                                            ,   Max2( size_t( 1 ) , skeleton.GetNumCells(2)) } ;  // number of grid cells in each direction of virtual uniform grid

    // Total number of cells should be as close to numVortonsMax as possible without going over.
    // Worst case allowable difference would be numVortonsMax=7 and numCells in each direction is 1 which yields a ratio of 1/7.
    // But in typical situations, the user would like expect total number of virtual cells to be closer to numVortonsMax than that.
    // E.g. if numVortonsMax=8^3=512 somehow yielded numCells[0]=numCells[1]=numCells[2]=7 then the ratio would be 343/512~=0.67.
    while( numCells[0] * numCells[1] * numCells[2] > numVortonsMax )
    {   // Number of cells is excessive.
        // This can happen when the trial number of cells in any direction is less than 1 -- then the other two will likely be too large.
        numCells[0] = Max2( size_t( 1 ) , numCells[0] / 2 ) ;
        numCells[1] = Max2( size_t( 1 ) , numCells[1] / 2 ) ;
        numCells[2] = Max2( size_t( 1 ) , numCells[2] / 2 ) ;
    }

    // sizeFactor controls whether vortons should overlap or fit inside grid cell.
    //  0.500 : Vorton exactly inscribes cubical grid cell.
    //          Each sphere fits exactly inside a grid cell.
    //  0.561 : Vortons have volume of closest-packing equal spheres. V=pi/sqrt(18) --> 4*pi*r^3/3=pi/sqrt(18) --> 4*r^3/3=1/(3*sqrt(2)) --> 4*r^3=1/sqrt(2) --> r^3=1/(4*sqrt(2))=1/sqrt(32)=32^-(1/2) --> r=32^((-1/2)*(1/3))=32^(-1/6) = 0.56123102415468649071676652483959
    //          All spheres fit exactly inside entire grid, if arranged as tight as possible.
    //  0.620 : Vorton has same volume as grid cell. V=1 --> 4*pi*r^3/3=1 --> r^3=3/(4*pi) --> r=pow(3/(4*pi),0.333)=0.6203504908994000166680068120774
    //          Implies spheres are meant to overlap.
    //  0.866 : Vorton exactly circumscribes cubical grid cell.  r=sqrt(3/4)=0.86602540378443864676372317075294
    //          All corners of cell just barely touch centered sphere.
    //  1.000 : Vorton spans width of 2 grid cells.
    //          Adjacent vortons just barely touch each others' centers.
    const float         sizeFactor      = pow( 32.0f , -1.0f / 6.0f ) ; // 0.56123102415468649071676652483959
    const float         volRatio        = 1.0f / Pow3( sizeFactor ) ;
    const float         oneOverN[3]     = { 1.0f / float( numCells[0] ) , 1.0f / float( numCells[1] ) , 1.0f / float( numCells[2] ) } ;
    const Vec3          gridCellSize    ( vDimensions.x * oneOverN[0] , vDimensions.y * oneOverN[1] , vDimensions.z * oneOverN[2] ) ;
    const float         gridCellVolume  = gridCellSize.x * gridCellSize.y * gridCellSize.z ;
    float               vortonRadius    = powf( gridCellVolume , 1.0f / 3.0f ) * sizeFactor ;
    if( 0.0f == vDimensions.z )
    {   // z size is zero, so domain is 2D.
        const float gridCellArea = gridCellSize.x * gridCellSize.y ;
        vortonRadius = powf( gridCellArea , 0.5f ) * sizeFactor ;
    }

    ASSERT( vortonRadius != 0.0f ) ;

#if 0
    // Create 2 "placeholder" vortons that define the corners of the grid.
    // These define the size of the grid.  The simulation fits the grid tightly
    // and we want the vortons' initial positions to reside in the center
    // of the grid cells -- except these 2 corner vortons.
    AddCornerVortons( vortons , vMin , vMin + vDimensions , vortonRadius ) ;
#endif

    Vec3        positionLocal            = Vec3( 0.0f , 0.0f , 0.0f ) ;  // vorton positionLocal
    unsigned    index[3]            ;   // index of each positionLocal visited
    static const float offset = 0.5f ;  // Visit center of each cell.
    // Iterate through each point in a uniform grid.
    for( index[2] = 0 ; index[2] < numCells[2] ; ++ index[2] )
    {   // For each z-coordinate...
        positionLocal.z = ( float( index[2] ) + offset ) * gridCellSize.z + vMin.z ;
        for( index[1] = 0 ; index[1] < numCells[1] ; ++ index[1] )
        {   // For each y-coordinate...
            positionLocal.y = ( float( index[1] ) + offset ) * gridCellSize.y + vMin.y ;
            for( index[0] = 0 ; index[0] < numCells[0] ; ++ index[0] )
            {   // For each x-coordinate...
                positionLocal.x = ( float( index[0] ) + offset ) * gridCellSize.x + vMin.x ;
                Vec3 vorticity ;
                float density ;
                vorticityDistribution.AssignVorton( vorticity , density , positionLocal , vCenter ) ;
                ASSERT( density > 0.0f ) ;
                const Vec3 positionGlobal = positionLocal + translation ;
                Vorton vorton( positionGlobal , vorticity * fMagnitude * volRatio , vortonRadius ) ;
                if( vorticity.Mag2() > sNegligibleEnstrophyThreshold )
                {   // Vorticity is significantly non-zero.
                    vorton.mDensity     = density ;
                    ASSERT( density > 0.0f ) ;
                #if ENABLE_FIRE
                    vorton.mFuelFraction  = 0.0f ;
                    vorton.mFlameFraction = 0.0f ;
                    vorton.mSmokeFraction = 0.0f ;
                #endif
                    //if( density < 0.0f )
                    //{ // HACK: set size to negative so renderer colors this vorton as "light" instead of "heavy"
                    //    vorton.mSize *= -1.0f ;
                    //}
                    vortons.PushBack( vorton ) ;
                }
            }
        }
    }
}
