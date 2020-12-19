/*! \file PclOpAssignScalarFromGrid.h

    \brief Particle operation to assign scalar member of particles from a grid.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_ASSIGN_SCALAR_FROM_GRID_H
#define PARTICLE_OPERATION_ASSIGN_SCALAR_FROM_GRID_H

#include "particleOperation.h"

/** Particle operation to assign density information to particles from a grid.
*/
class PclOpAssignScalarFromGrid : public IParticleOperation
{
    public:
        PclOpAssignScalarFromGrid()
            : mMemberOffsetInBytes( 0 )
            , mScalarGrid( 0 )
            , mDivideByParticleCount( false )
        {}

        void Operate(  Vector< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ )
        {
            QUERY_PERFORMANCE_ENTER ;

            if( mScalarGrid->HasZeroExtent() )
            {   // Scalar grid is empty.  Probably first iteration.
                return ; // Nothing to do.
            }

            Particles::AssignScalarFromGrid( particles , mMemberOffsetInBytes , * mScalarGrid ) ;

            if( mDivideByParticleCount )
            {
                UniformGrid< unsigned > ugParticleCount( * mScalarGrid ) ;
                CountParticlesPerCell( particles , ugParticleCount ) ;
                DivideByNumParticlesPerCell( particles , ugParticleCount ) ;
            }

            QUERY_PERFORMANCE_EXIT( Particles_AssignScalarFromGrid ) ;
        }

        size_t                          mMemberOffsetInBytes    ;   ///< Offset, in bytes, from start of particle, to scalar member to assign.
        const UniformGrid< float > *    mScalarGrid             ;   ///< Grid of density values
        bool                            mDivideByParticleCount  ;   ///< Whether to divide by number of particles per grid cell.

    private:
        void CountParticlesPerCell( const Vector< Particle > & particles , UniformGrid< unsigned > & ugParticleCount )
        {
            ugParticleCount.Init( 0 ) ;
            const size_t numParticles = particles.Size() ;
            for( unsigned offset = 0 ; offset < numParticles ; ++ offset )
            {   // For each particle...
                const Particle & rParticle = particles[ offset ] ;
                // Tally the number of particles in its cell.
                ++ ugParticleCount[ rParticle.mPosition ] ;
            }
        }

        void DivideByNumParticlesPerCell( Vector< Particle > & particles , const UniformGrid< unsigned > & ugParticleCount )
        {
            const size_t numParticles = particles.Size() ;
            for( unsigned offset = 0 ; offset < numParticles ; ++ offset )
            {   // For each particle...
                Particle &  rParticle           = particles[ offset ] ;
                float &     value               = *(float*) ( ( (char*) & rParticle ) + mMemberOffsetInBytes ) ;
                const float numParticlesPerCell = ugParticleCount[ rParticle.mPosition ] ;

                // Divide value by number of particles in the same cell as this one.
                value /= float( numParticlesPerCell ) ;
            }
        }
} ;

#endif
