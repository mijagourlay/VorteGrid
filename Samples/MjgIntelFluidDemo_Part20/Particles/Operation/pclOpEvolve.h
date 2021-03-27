/** \file pclOpEvolve.h

    \brief Operation to evolve particle states -- position from velocity, orientation from angular velocity.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_EVOLVE_H
#define PARTICLE_OPERATION_EVOLVE_H

#include "particleOperation.h"

/** Operation to evolve particle states -- position from velocity, orientation from angular velocity.

    \see EvolveParticlesSlice
*/
class PclOpEvolve : public IParticleOperation
{
    public:
        PclOpEvolve()
        {}

        VIRTUAL_CONSTRUCTORS( PclOpEvolve ) ;
        
        void Operate(  VECTOR< Particle > & particles , float timeStep , unsigned /* uFrame */ ) ;
} ;

#endif