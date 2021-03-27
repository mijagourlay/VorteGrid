/** \file particleGroup.h

    \brief Group of particles and operations to perform on them.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_GROUP_H
#define PARTICLE_GROUP_H

#include "Operation/particleOperation.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

/** Group of particles and operations to perform on them.
*/
class ParticleGroup
{
    public:
        typedef VECTOR< IParticleOperation * >  PclOpContainer ;
        typedef PclOpContainer::Iterator        Iterator ;
        typedef PclOpContainer::ConstIterator   ConstIterator ;

        ParticleGroup() {}
        ~ParticleGroup() ;
        ParticleGroup( const ParticleGroup & that ) ;
        ParticleGroup & operator=( const ParticleGroup & that ) ;

        /// Add given ParticleOperation to the end of the current list of them.
        void PushBack( IParticleOperation * pclOp )
        {
            mParticleOps.PushBack( pclOp ) ;
        }

        size_t GetNumParticles() const { return mParticles.Size() ; }

        bool HasParticles() const { return ! mParticles.Empty() ; }

        bool HasOperations() const { return mParticleOps.Empty() ; }

        Iterator Begin() { return mParticleOps.Begin() ; }
        Iterator End()   { return mParticleOps.End()   ; }

        void Clear() ;
        void Update( float timeStep , unsigned uFrame ) ;

              VECTOR< Particle > & GetParticles()       { return mParticles ; }
        const VECTOR< Particle > & GetParticles() const { return mParticles ; }

        IParticleOperation * GetOperation( size_t pclOpIdx ) const { return mParticleOps[ pclOpIdx ] ; }

        size_t  IndexOfOperation( IParticleOperation * pclOpAddress ) const ;

        static const size_t INVALID_INDEX = static_cast< size_t >( -1 ) ;

    private:
        VECTOR< Particle >              mParticles      ;   ///< Dynamic array of particles which this group owns and on which all ParticleOperations in this group act.
        VECTOR< IParticleOperation * >  mParticleOps    ;   ///< Dynamic array of particle operations which operate on the particles that this group owns.
} ;

// Public variables ------------------------------------------------------------

// Public functions ------------------------------------------------------------

extern ParticleGroup * CreateSampleParticleGroup( const Vec3 & basePosition , const Vec3 & acceleration ) ;

#endif
