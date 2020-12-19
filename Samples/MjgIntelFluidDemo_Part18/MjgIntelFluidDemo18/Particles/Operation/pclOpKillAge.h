/** \file pclOpKillAge.h

    \brief Operation to kill particles based on their age.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_KILL_AGE_H
#define PARTICLE_OPERATION_KILL_AGE_H

#include "particleOperation.h"

/** Operation to kill particles based on their age.
*/
class PclOpKillAge : public IParticleOperation
{
    public:
        PclOpKillAge()
            : mAgeMax( 0 )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpKillAge ) ;

        void Operate( VECTOR< Particle > & particles , float /* timeStep */ , unsigned uFrame ) ;

        int mAgeMax                 ;   ///< Maximum age of particles
} ;

#endif