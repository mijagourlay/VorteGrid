/** \file particleSystem.cpp

    \brief Particle system

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "particleSystem.h"

#include <Core/Performance/perfBlock.h>

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

ParticleSystem::~ParticleSystem()
{
    Clear() ;
}




ParticleSystem::ParticleSystem( const ParticleSystem & that )
{
    this->operator=( that ) ;
}




ParticleSystem & ParticleSystem::operator=( const ParticleSystem & that )
{
    if( this != & that )
    {   // Not self-copy.
        Clear() ;   // Delete all previous items in this object.
        for( ConstIterator pclGrpIter = that.mParticleGroups.Begin() ; pclGrpIter != that.mParticleGroups.End() ; ++ pclGrpIter )
        {   // For each particle group in the original system...
            ParticleGroup * pclGrpOrig = * pclGrpIter ;
            // Duplicate the original.
            ParticleGroup * pclGrpDupe = new ParticleGroup( * pclGrpOrig ) ;
            // Remember the duplicate.
            mParticleGroups.PushBack( pclGrpDupe ) ;
        }
        // Copy and execute indirect assignments.
        CopyAssignments( that ) ;
    }
    return * this ;
}




void ParticleSystem::Clear()
{
    while( ! mParticleGroups.Empty() )
    {   // For each group in this system...
        ParticleGroup * particleGroup = mParticleGroups.Back() ;
        particleGroup->Clear() ;
        mParticleGroups.PopBack() ;
    }
}




/** Process all particle groups in this system.
    \see ParticleGroup
*/
void ParticleSystem::Update( float timeStep , unsigned uFrame )
{
    PERF_BLOCK( ParticleSystem__Update ) ;

    const size_t numParticleGroups = mParticleGroups.Size() ;
    for( size_t iPclGrp = 0 ; iPclGrp < numParticleGroups ; ++ iPclGrp )
    {   // Run through groups in order.
        ParticleGroup * particleGroup = mParticleGroups[ iPclGrp ] ;
        particleGroup->Update( timeStep , uFrame ) ;
    }
}




/** Find sought group and return its index.  Useful for associating other data with each group.
*/
size_t ParticleSystem::FindIndexOfGroup( ParticleGroup * soughtGroup )
{
    const size_t numGroups = mParticleGroups.Size() ;
    for( size_t index = 0 ; index < numGroups ; ++ index )
    {   // For each group in this system...
        ParticleGroup * group = mParticleGroups[ index ] ;
        if( group == soughtGroup )
        {   // Found sought group.
            return index ;
        }
    }
    return mParticleGroups.Size() ; // Return an invalid index to inform caller soughtGroup not found.
}




/** Remember to assign a plain-old-data value specified indirectly.
*/
void ParticleSystem::AddIndirectPodAssignment( const IndirectAddress & dst , const IndirectAddress & src , size_t sizeInBytes )
{
    IndirectPodAssignment ipa( dst , src , sizeInBytes ) ;
    ipa.Assign() ;
    mAssignments.PushBack( ipa ) ;
}




/** Remember to assign an address specified indirectly.
*/
void ParticleSystem::AddIndirectAddressAssignment( const IndirectAddress & dst , const IndirectAddress & src )
{
    IndirectPodAssignment iaa( dst , src ) ;
    iaa.Assign() ;
    mAssignments.PushBack( iaa ) ;

#if _DEBUG
    {
        IndirectPodAssignment & ipa = mAssignments.Back() ;
        ReferenceIndex refIdxDummy ;
        FindReferent( refIdxDummy , ipa.GetDstBinding().GetBaseAddress() ) ;
        FindReferent( refIdxDummy , ipa.GetSrcBinding().GetBaseAddress() ) ;
    }
#endif
}




bool ParticleSystem::FindReferent( ReferenceIndex & refIdx , void * referentAddress ) const
{
    const size_t numParticleGroups = mParticleGroups.Size() ;
    for( size_t iPclGrp = 0 ; iPclGrp < numParticleGroups ; ++ iPclGrp )
    {   // For each particle group...
        ParticleGroup * particleGroup = mParticleGroups[ iPclGrp ] ;
        if( particleGroup == referentAddress )
        {   // Group address matches referent.
            refIdx.mGroupIndex = iPclGrp ;
            // Mark refIdx operation as non-applicable.
            refIdx.mOperationIndex = INDEX_PARENT ;
            return true ; // Found referent (a partcle group).
        }
        else
        {   // Group address does NOT match referent.
            // Get index of particle operation matching referentAddress.
            refIdx.mOperationIndex = particleGroup->IndexOfOperation( static_cast< IParticleOperation * >( referentAddress ) ) ;
            if( ParticleGroup::INVALID_INDEX != refIdx.mOperationIndex )
            {   // Operation index is valid.
                refIdx.mGroupIndex = iPclGrp ;
                return true ; // Found referent (a particle operation).
            }
        }
    }

    // Failed to find any matches in any group or operation.
    refIdx.mGroupIndex     = INDEX_PARENT ;
    refIdx.mOperationIndex = ParticleGroup::INVALID_INDEX ;

    FAIL() ;
    return false ;
}




void * ParticleSystem::Dereference( const ReferenceIndex & refIdx ) const
{
    ASSERT( refIdx.mGroupIndex != INDEX_PARENT ) ;
    ASSERT( refIdx.mGroupIndex < mParticleGroups.Size() ) ;
    ParticleGroup * pclGrp = mParticleGroups[ refIdx.mGroupIndex ] ;
    if( ParticleGroup::INVALID_INDEX == refIdx.mOperationIndex )
    {   // Referent is a particle group.
        return pclGrp ;
    }
    else
    {   // Referent is a particle operation.
        IParticleOperation * pclOp = pclGrp->GetOperation( refIdx.mOperationIndex ) ;
        return pclOp ;
    }
}




/** Rectify and copy assignments from that to this.
*/
void ParticleSystem::CopyAssignments( const ParticleSystem & that )
{
    for( AssignmentContainer::ConstIterator iter = that.mAssignments.Begin() ; iter != that.mAssignments.End() ; ++ iter )
    {   // For each indirect assignment in that system...
        const IndirectPodAssignment & thatIpa = * iter ;
        // Find referents in that system.
        ReferenceIndex refIdxDst ;
        ReferenceIndex refIdxSrc ;
        that.FindReferent( refIdxDst , thatIpa.GetDstBinding().GetBaseAddress() ) ;
        that.FindReferent( refIdxSrc , thatIpa.GetSrcBinding().GetBaseAddress() ) ;
        // Find base addresses in this system which corresponds to those in that.
        void * dstBase = Dereference( refIdxDst ) ;
        void * srcBase = Dereference( refIdxSrc ) ;
        IndirectAddress dstAddr( dstBase , thatIpa.GetDstBinding().GetOffsetInBytes() ) ;
        IndirectAddress srcAddr( srcBase , thatIpa.GetSrcBinding().GetOffsetInBytes() ) ;
        switch( thatIpa.GetAssignmentType() )
        {   // Depending on assignment type...
            case IndirectPodAssignment::VALUE:
            {
                AddIndirectPodAssignment( dstAddr , srcAddr , thatIpa.GetSizeInBytes() ) ;
            }
            break ;
            case IndirectPodAssignment::ADDRESS:
            {
                AddIndirectAddressAssignment( dstAddr , srcAddr ) ;
            }
            break ;
        }
    }
}
