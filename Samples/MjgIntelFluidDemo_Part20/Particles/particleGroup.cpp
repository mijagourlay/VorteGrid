/** \file particleGroup.cpp

    \brief Group of particles and operations to perform on them.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "particleGroup.h"

#include <Core/Performance/perfBlock.h>

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

ParticleGroup::~ParticleGroup()
{
    Clear() ;
}




ParticleGroup::ParticleGroup( const ParticleGroup & that )
{
    this->operator=( that ) ;
}




ParticleGroup & ParticleGroup::operator=( const ParticleGroup & that )
{
    if( this != & that )
    {   // Not self-copy.
        Clear() ;   // Delete all previous items in this object.
        mParticles = that.mParticles ;
        for( ConstIterator pclOpIter = that.mParticleOps.Begin() ; pclOpIter != that.mParticleOps.End() ; ++ pclOpIter )
        {   // For each particle operation in the original group...
            IParticleOperation * pclOpOrig = * pclOpIter ;
            // Duplicate the original.
            IParticleOperation * pclOpDupe = pclOpOrig->Clone() ;
            // Remember the duplicate.
            mParticleOps.PushBack( pclOpDupe ) ;
        }
    }
    return * this ;
}




void ParticleGroup::Clear()
{
    mParticles.Clear() ;
    while( ! mParticleOps.Empty() )
    {
        IParticleOperation * pOp = mParticleOps.Back() ;
        delete pOp ;
        mParticleOps.PopBack() ;
    }
}




/** Perform all particle operations in this group.
    \see IParticleOperation
*/
void ParticleGroup::Update( float timeStep , unsigned uFrame )
{
    PERF_BLOCK( ParticleGroup__Update ) ;

    const size_t numOps = mParticleOps.Size() ;

    for( size_t iOp = 0 ; iOp < numOps ; ++ iOp )
    {   // Run operations in order.
        IParticleOperation * pOp = mParticleOps[ iOp ] ;
        pOp->Operate( mParticles , timeStep , uFrame ) ;
    }
}



size_t  ParticleGroup::IndexOfOperation( IParticleOperation * const pclOpAddress ) const
{
    const size_t numOps = mParticleOps.Size() ;

    for( size_t iOp = 0 ; iOp < numOps ; ++ iOp )
    {   // For each particle operation...
        IParticleOperation * const pOp = mParticleOps[ iOp ] ;
        if( pclOpAddress == pOp )
        {   // Found sought operation.
            return iOp ;
        }
    }
    return INVALID_INDEX ;
}




#include <Particles/Operation/pclOpFindBoundingBox.h>
#include <Particles/Operation/pclOpWind.h>
#include <Particles/Operation/pclOpAdvect.h>
#include <Particles/Operation/pclOpEvolve.h>
#include <Particles/Operation/pclOpAssignScalarFromGrid.h>
#include <Particles/Operation/pclOpPopulateVelocityGrid.h>
#include <Particles/Operation/pclOpEmit.h>
#include <Particles/Operation/pclOpKillAge.h>
#include <Particles/particleGroup.h>
#include <Particles/particleSystemManager.h>

ParticleGroup * CreateSampleParticleGroup( const Vec3 & basePosition , const Vec3 & acceleration )
{
    ParticleGroup * pclGrpFlames = new ParticleGroup ;

    // Start with particle destruction and creation operations.

    {   // First kill particles to make room for new ones.
        PclOpKillAge * pclOpKillAge = new PclOpKillAge ;
        pclOpKillAge->mAgeMax    = 90 ;
        pclGrpFlames->PushBack( pclOpKillAge ) ;
    }
    {   // Second, emit new particles before moving them.
        Particle emitterTemplate ;
        emitterTemplate.mPosition               = basePosition ;
        emitterTemplate.mVelocity               = Vec3( 0.0f , 0.0f , 2.0f ) ;
        emitterTemplate.mSize                   = 0.0625f ;   // Also see where Particles::Emit reassigns mSize.  See calculation of pclSize.
        emitterTemplate.mDensity                = 1.0f ; // Actual value does not matter for this case.
#   if ENABLE_FIRE
        emitterTemplate.mFuelFraction           = 0.0f ;
        emitterTemplate.mFlameFraction          = 0.0f ;
        emitterTemplate.mSmokeFraction          = 1.0f ;
#   endif

        Particle emitterSpread ;
        emitterSpread.mPosition                 = Vec3( 0.5f , 0.5f , 0.5f ) ;
        emitterSpread.mVelocity                 = Vec3( 0.5f , 0.5f , 0.5f ) ;

        PclOpEmit     * pclOpEmit               = new PclOpEmit ;

        pclOpEmit->mTemplate                    = emitterTemplate ;
        pclOpEmit->mTemplate.mAngularVelocity   = FLT_EPSILON * Vec3( 1.0f , 1.0f , 1.0f ) ;
        pclOpEmit->mSpread                      = emitterSpread ;
        pclOpEmit->mSpread.mDensity             = 0.0f ;
        pclOpEmit->mEmitRate                    = 100.0f ;

        pclGrpFlames->PushBack( pclOpEmit ) ;
    }

    // Particle motion operations follow.

    {
        PclOpAccelerate * pclOpAccelerate = new PclOpAccelerate ;

        pclOpAccelerate->mAcceleration = acceleration ;
        pclGrpFlames->PushBack( pclOpAccelerate ) ;
    }
    {   // Wind replaces a portion of the particle velocity.
        PclOpWind   * pclOpWind = new PclOpWind ;

        pclOpWind->mWindWeight  = 0.02f ;
        pclOpWind->mSrcWeight   = 0.98f ;
        pclOpWind->mWind        = Vec3( 0.1f , 0.0f , 0.0f ) ;
        pclGrpFlames->PushBack( pclOpWind ) ;
    }
    {   // Evolve runs after all operations that set velocity.
        PclOpEvolve * pclOpEvolve = new PclOpEvolve ;
        pclGrpFlames->PushBack( pclOpEvolve ) ;
    }

    {   // Find bounding box after particles have moved.
        PclOpFindBoundingBox * pclOpFindBoundingBox = new PclOpFindBoundingBox ;
        pclGrpFlames->PushBack( pclOpFindBoundingBox ) ;
    }

    return pclGrpFlames ;
}
