/** \file pclOpWind.h

    \brief Operation to push particles along a direction up to a maximum speed.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_WIND_H
#define PARTICLE_OPERATION_WIND_H

#include "particleOperation.h"

/** Particle operation to push particles along a direction up to a maximum speed.
*/
class PclOpWind : public IParticleOperation
{
    public:
        PclOpWind()
            : mWind( 1.0f , 0.0f , 0.0f )
            , mSrcWeight( 0.875f )
            , mWindWeight( 0.125f )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpWind ) ;

        void Operate(  VECTOR< Particle > & particles , float timeStep , unsigned uFrame ) ;

        Vec3    mWind       ;   ///< Wind velocity
        float   mSrcWeight  ;   ///< Fraction of original velocity to keep
        float   mWindWeight ;   ///< Fraction of wind velocity to assign
} ;

#endif