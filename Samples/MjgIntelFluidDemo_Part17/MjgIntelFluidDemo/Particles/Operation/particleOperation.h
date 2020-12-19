/** \file particleOperation.h

    \brief Particle operation abstract base class.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_H
#define PARTICLE_OPERATION_H

#include "Core/wrapperMacros.h"
#include "../particle.h"

// Macros --------------------------------------------------------------

#define VIRTUAL_CONSTRUCTORS( ClassT )                                      \
    /** Virtual constructor: Create another instance of this class. */      \
    virtual IParticleOperation * Create() { return new ClassT() ; }         \
    /** Virtual copy constructor: Clone this object. */                     \
    virtual IParticleOperation * Clone() { return new ClassT( * this ) ; }


// Types --------------------------------------------------------------

/** Particle operation abstract base class.
*/
class IParticleOperation
{
    public:
        /// Construct a particle operation.
        IParticleOperation() {}

        /// Destruct this particle operation.
        virtual ~IParticleOperation() {}

        /// Create another instance of this class.
        virtual IParticleOperation * Create() = 0 ;

        /// Clone this object.
        virtual IParticleOperation * Clone() = 0 ;

        /// Perform an operation on the given set of particles over a given duration.
        virtual void Operate( VECTOR< Particle > & particles , float timeStep , unsigned uFrame ) = 0 ;
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
