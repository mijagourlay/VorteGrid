/** \file particleSystem.h

    \brief Particle system

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include "Particles/particleGroup.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/** Indirect address of some data.
*/
class IndirectAddress
{
    public:
        IndirectAddress( void * baseAddress , size_t offsetInBytes )
            : mBaseAddress( baseAddress )
            , mOffsetInBytes( offsetInBytes )
        {
        }

        IndirectAddress( void * baseAddress , void * memberAddress )
            : mBaseAddress( baseAddress )
            , mOffsetInBytes( static_cast< char * >( memberAddress ) - static_cast< char * >( baseAddress ) )
        {
            ASSERT( memberAddress > baseAddress ) ;
        }

        IndirectAddress( const void * baseAddress , const void * memberAddress )
            : mBaseAddress( const_cast< void * >( baseAddress ) )
            , mOffsetInBytes( static_cast< const char * >( memberAddress ) - static_cast< const char * >( baseAddress ) )
        {
            ASSERT( memberAddress > baseAddress ) ;
        }

        void * GetBaseAddress() const   { return mBaseAddress   ; }
        size_t GetOffsetInBytes() const { return mOffsetInBytes ; }

    private:
        void *  mBaseAddress    ;
        size_t  mOffsetInBytes  ;
} ;




/** Assignment of indirect plain-old-data.

    Represents the assignment of a plain-old-data value or address of some
    member variables, to another member variable.

    Useful for representing a one-way relationship between two objects,
    especially when those objects need to be cloned, and the clones
    need to have that relationship with each other (not with the original
    objects).
*/
class IndirectPodAssignment
{
    public:
        enum AssignmentType
        {
            VALUE   ,
            ADDRESS
        } ;

        IndirectPodAssignment( const IndirectAddress & dst , const IndirectAddress & src , size_t sizeInBytes )
            : mDstBinding( dst )
            , mSrcBinding( src )
            , mSizeInBytes( sizeInBytes )
            , mAssignmentType( VALUE )
        {
        }

        IndirectPodAssignment( const IndirectAddress & dst , const IndirectAddress & src )
            : mDstBinding( dst )
            , mSrcBinding( src )
            , mSizeInBytes( sizeof( void * ) )
            , mAssignmentType( ADDRESS )
        {
        }

        const IndirectAddress & GetDstBinding() const { return mDstBinding ; }
        const IndirectAddress & GetSrcBinding() const { return mSrcBinding ; }
        size_t GetSizeInBytes() const { return mSizeInBytes ; }
        AssignmentType GetAssignmentType() const { return mAssignmentType ; }

        void Assign()
        {
            switch( mAssignmentType )
            {
                case VALUE  : AssignValue()   ; break ;
                case ADDRESS: AssignAddress() ; break ;
            }
        }

    private:
        void AssignValue()
        {
            ASSERT( VALUE == mAssignmentType ) ;
            void * dstAddress = static_cast< char * >( mDstBinding.GetBaseAddress() ) + mDstBinding.GetOffsetInBytes() ;
            void * srcAddress = static_cast< char * >( mSrcBinding.GetBaseAddress() ) + mSrcBinding.GetOffsetInBytes() ;
            memcpy( dstAddress , srcAddress , mSizeInBytes ) ;
        }

        void AssignAddress()
        {
            ASSERT( ADDRESS == mAssignmentType ) ;
            ASSERT( sizeof( void * ) == mSizeInBytes ) ;
            void * dstAddress = static_cast< char * >( mDstBinding.GetBaseAddress() ) + mDstBinding.GetOffsetInBytes() ;
            void * srcAddress = static_cast< char * >( mSrcBinding.GetBaseAddress() ) + mSrcBinding.GetOffsetInBytes() ;
            memcpy( dstAddress , & srcAddress , mSizeInBytes ) ;
        }

        IndirectAddress mDstBinding     ;   ///< Address of l-value to assign
        IndirectAddress mSrcBinding     ;   ///< Address of right-hand side of assignment
        size_t          mSizeInBytes    ;   ///< Size of value to assign
        AssignmentType  mAssignmentType ;   ///< Whether to use value or address of right-hand side
} ;




/** Container for particle system, which is a single coherent "particle effect".

    A ParticleSystem contains multiple ParticleGroups where each group is a "layer" which
    contains multiple ParticleOperations and a Particle array.

    The ParticleSystemManager manages multiple ParticleSystems.
*/
class ParticleSystem
{
    public:
        typedef VECTOR< ParticleGroup * >      PclGrpContainer ;
        typedef PclGrpContainer::Iterator      Iterator ;
        typedef PclGrpContainer::ConstIterator ConstIterator ;

        ParticleSystem() {}
        ~ParticleSystem() ;
        ParticleSystem( const ParticleSystem & that ) ;
        ParticleSystem & operator=( const ParticleSystem & that ) ;

        /// Add given ParticleGroup to the end of the current list of them.
        void PushBack( ParticleGroup * pclGrp )
        {
            mParticleGroups.PushBack( pclGrp ) ;
        }

        Iterator      Begin()       { return mParticleGroups.Begin() ; }
        ConstIterator Begin() const { return mParticleGroups.Begin() ; }

        Iterator      End()       { return mParticleGroups.End()   ; }
        ConstIterator End() const { return mParticleGroups.End()   ; }

              ParticleGroup * GetGroup( size_t groupIndex )         { return mParticleGroups[ groupIndex ] ; }
        const ParticleGroup * GetGroup( size_t groupIndex ) const   { return mParticleGroups[ groupIndex ] ; }

        size_t GetNumGroups() const { return mParticleGroups.Size() ; }
        size_t FindIndexOfGroup( ParticleGroup * soughtGroup ) ;

        void Clear() ;
        void Update( float timeStep , unsigned uFrame ) ;

        void AddIndirectPodAssignment( const IndirectAddress & dst , const IndirectAddress & src , size_t sizeInBytes ) ;
        void AddIndirectAddressAssignment( const IndirectAddress & dst , const IndirectAddress & src ) ;

    private:
        struct ReferenceIndex
        {
            size_t  mGroupIndex     ;
            size_t  mOperationIndex ;
        } ;

        static const size_t INDEX_PARENT = static_cast< size_t >( -1 ) /* Really ~0 but size depends on architecture. */ ;

        bool    FindReferent( ReferenceIndex & refIdx , void * referentAddress ) const ;
        void *  Dereference( const ReferenceIndex & refIdx ) const ;

        void    CopyAssignments( const ParticleSystem & that ) ;

        typedef VECTOR< IndirectPodAssignment > AssignmentContainer ;

        PclGrpContainer     mParticleGroups ;   ///< Dynamic array of ParticleGroup objects that this ParticleSystem owns.
        AssignmentContainer mAssignments    ;   ///< Data value assignments to perform.
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------

#endif
