/** \file pclOpAssignScalarFromGrid.h

    \brief Operation to assign scalar member of particles from a grid.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_ASSIGN_SCALAR_FROM_GRID_H
#define PARTICLE_OPERATION_ASSIGN_SCALAR_FROM_GRID_H

#include "particleOperation.h"

/** Operation to assign scalar member of particles from a grid.
*/
class PclOpAssignScalarFromGrid : public IParticleOperation
{
    public:
        PclOpAssignScalarFromGrid()
            : mMemberOffsetInBytes( 0 )
            , mScalarGrid( 0 )
            , mDivideByParticleCount( false )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpAssignScalarFromGrid ) ;

        void Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ ) ;

        size_t                          mMemberOffsetInBytes    ;   ///< Offset, in bytes, from start of particle, to scalar member to assign.
        const UniformGrid< float > *    mScalarGrid             ;   ///< Grid of density values
        bool                            mDivideByParticleCount  ;   ///< Whether to divide by number of particles per grid cell.

    private:
        void CountParticlesPerCell( const VECTOR< Particle > & particles , UniformGrid< unsigned > & ugParticleCount ) ;
        void DivideByNumParticlesPerCell( VECTOR< Particle > & particles , const UniformGrid< unsigned > & ugParticleCount ) ;
} ;

#endif