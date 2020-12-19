/** \file pclOpAdvect.h

    \brief Operation to assign particle velocity according to a velocity field.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_ASSIGN_VELOCITY_FROM_FIELD_H
#define PARTICLE_OPERATION_ASSIGN_VELOCITY_FROM_FIELD_H

#include "particleOperation.h"

/** Operation to assign particle velocity according to a velocity field.

    \see AssignParticleVelocityFromField_Slice
*/
class PclOpAssignVelocityFromField : public IParticleOperation
{
    public:
        PclOpAssignVelocityFromField()
            : mVelocityGrid( 0 )
            , mGridWeight( 1.0f )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpAssignVelocityFromField ) ;
        
        void Operate(  VECTOR< Particle > & particles , float timeStep , unsigned /* uFrame */ ) ;

        const UniformGrid< Vec3  > *    mVelocityGrid   ;   ///< Grid of velocity values.
        float                           mGridWeight     ;   ///< Amount of velocity from grid to assign to particles, each frame.
} ;

#endif