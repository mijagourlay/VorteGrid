/** \file pclOpPopulateVelocityGrid.cpp

    \brief Operation to populate a grid with velocity values from particles.

    \see http://www.mijagourlay.com/

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdlib.h>

#include "Core/Performance/perfBlock.h"

#include "Core/SpatialPartition/uniformGridMath.h"

#include "Particles/particle.h"

#include "Particles/Operation/pclOpFindBoundingBox.h"
#include "Particles/Operation/pclOpPopulateVelocityGrid.h"




/** Populate a UniformGrid with velocity values from particles.

    \param velocityGrid (out) Grid into which to transfer velocity values.
        Shape must already be defined.

    \param particles    Dynamic array of particles whose velocity information to
                        accumulate into velocityGrid.

*/
void Particles::PopulateVelocityGrid( UniformGrid< Vec3 > & velocityGrid , const VECTOR< Particle > & particles )
{
    ASSERT( velocityGrid.Empty() ) ;

    velocityGrid.Init( Vec3( 0.0f , 0.0f , 0.0f ) ) ;   // Reserve memory for velocity grid and initialize all values to zero.

    DEBUG_ONLY( Vec3 velocitySumVortons( 0.0f , 0.0f , 0.0f ) ) ;

    // Amount of contribution to each grid point.
    UniformGrid< float > ugParticleContribution( static_cast< UniformGridGeometry >( velocityGrid ) ) ;
    ugParticleContribution.Init( 0.0f ) ;

    // Populate velocity grid.
    const size_t numParticles = particles.Size() ;
    for( size_t uParticle = 0 ; uParticle < numParticles ; ++ uParticle )
    {   // For each particle in the array...
        const Particle  &   rParticle   = particles[ uParticle ] ;
        const Vec3      &   rPosition   = rParticle.mPosition   ;
        ASSERT( ! IsNan( rPosition ) && ! IsInf( rPosition ) ) ;

        DEBUG_ONLY( const unsigned  uOffset = velocityGrid.OffsetOfPosition( rPosition ) ) ;
        ASSERT( uOffset < velocityGrid.GetGridCapacity() ) ;

        velocityGrid.Accumulate( rPosition , rParticle.mVelocity ) ;

        DEBUG_ONLY( velocitySumVortons += rParticle.mVelocity ) ;

        // Tally number of particle in each cell.
        // Since velocity should average (not add), need to divide by contribution later.
        ugParticleContribution.Accumulate( rPosition , 1.0f ) ;
    }

#if defined( _DEBUG )
    {
        Vec3 velocitySumGrid( 0.0f , 0.0f , 0.0f ) ;
        for( unsigned offset = 0 ; offset < velocityGrid.GetGridCapacity() ; ++ offset )
        {   // For each cell in velocity grid...
            velocitySumGrid += velocityGrid[ offset ] ;
        }
        ASSERT( velocitySumVortons.Resembles( velocitySumGrid , 0.01f ) ) ;
    }
#endif

    // Turn velocity sums into averages.
    for( unsigned offset = 0 ; offset < velocityGrid.GetGridCapacity() ; ++ offset )
    {   // For each cell in grid...
        if( ugParticleContribution[ offset ] > 0.0f )
        {
            const float oneOverContribution = 1.0f / ugParticleContribution[ offset ] ;
            velocityGrid[ offset ] *= oneOverContribution ;
        }
    }

    ASSERT( ! velocityGrid.HasZeroExtent() ) ;
}




void PclOpPopulateVelocityGrid::Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ )
{
    PERF_BLOCK( PclOpPopulateVelocityGrid__Operate ) ;

    if(     particles.Empty()       // No particles -- possibly none yet created or all dead.
        ||  ( 0 == mVelocityGrid )  // No velocity grid -- possbly deactivated at runtime.
        )
    {   // No appropriate data so do nothing.
        return ;
    }

    if( ! mBoundingBox )
    {   // Bounding box not provided so compute one.
        Vec3 minCorner( FLT_MAX , FLT_MAX , FLT_MAX ) ;
        Vec3 maxCorner( - minCorner ) ;
        PclOpFindBoundingBox::FindBoundingBox( particles , minCorner , maxCorner ) ;
        const float & margin = particles[ 0 ].GetRadius() ;
        const Vec3  nudge( margin * Vec3( 1.0f , 1.0f , 1.0f ) ) ;
        minCorner -= nudge ;
        maxCorner += nudge ;

        mVelocityGrid->Clear() ;
        mVelocityGrid->DefineShape( particles.Size() , minCorner , maxCorner , true ) ;
    }
    else if( mUpdateBoundingBox )
    {   // New bounding box should contain old one and given particles.
        // Solving boundary conditions likely pushes particles outside the
        // original bounding box, so resist the temptation to reuse any
        // bounding box found prior to satisfying boundary conditions.
        Vec3 minCorner( mBoundingBox[ 0 ] ) ;
        Vec3 maxCorner( mBoundingBox[ 1 ] ) ;
        PclOpFindBoundingBox::FindBoundingBox( particles , minCorner , maxCorner ) ;
        const float & margin = particles[ 0 ].GetRadius() ;
        const Vec3  nudge( margin * Vec3( 1.0f , 1.0f , 1.0f ) ) ;
        minCorner -= nudge ;
        maxCorner += nudge ;

        mVelocityGrid->Clear() ;
        mVelocityGrid->DefineShape( particles.Size() , minCorner , maxCorner , true ) ;
    }

    Particles::PopulateVelocityGrid( * mVelocityGrid , particles ) ;
}