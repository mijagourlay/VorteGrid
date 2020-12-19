/** \file pclOpPopulateVelocityGrid.h

    \brief Operation to populate a grid with velocity values from particles.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_POPULATE_VELOCITY_GRID_H
#define PARTICLE_OPERATION_POPULATE_VELOCITY_GRID_H

#include "particleOperation.h"

/** Operation to populate a grid with velocity values from particles.
*/
class PclOpPopulateVelocityGrid : public IParticleOperation
{
    public:
        PclOpPopulateVelocityGrid()
            : mVelocityGrid( 0 )
            , mBoundingBox( 0 )
            , mUpdateBoundingBox( true )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpPopulateVelocityGrid ) ;

        void Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ ) ;

        UniformGrid< Vec3 > *   mVelocityGrid       ;   ///< Grid of velocity values
        Vec3 *                  mBoundingBox        ;   ///< Bounding box.
        bool                    mUpdateBoundingBox  ;   ///< Whether to update bounding box with particles.
} ;


namespace Particles
{
    extern void PopulateVelocityGrid( UniformGrid< Vec3 > & velocityGrid , const VECTOR< Particle > & particles ) ;
}

#endif