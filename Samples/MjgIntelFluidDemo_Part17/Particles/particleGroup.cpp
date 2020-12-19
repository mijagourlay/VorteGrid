/** \file particleGroup.cpp

    \brief Group of particles and operations to perform on them.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "particleGroup.h"

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
