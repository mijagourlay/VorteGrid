/*! \file PclOpFluidBodyInteraction.h

    \brief Particle operation to interact vorton-based fluid with rigid bodies

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_OPERATION_FLUID_BODY_INTERACTION_H
#define PARTICLE_OPERATION_FLUID_BODY_INTERACTION_H

#include "Sim/Particle/particleOperation.h"

/*! \brief Particle operation to push particles up to a maximum speed
*/
class PclOpFluidBodyInteraction : public IParticleOperation
{
    public:
        PclOpFluidBodyInteraction( void )
            : mParticles( 0 )
            , mVelocityGrid( 0 )
            , mAmbientFluidDensity( 0.0f )
            , mRigidBodies( 0 )
            , mRespectAngVel( true )
        {}

        void Operate( float timeStep , unsigned uFrame )
        {
            QUERY_PERFORMANCE_ENTER ;
            FluidBodySim::SolveBoundaryConditions( * mParticles , * mVelocityGrid , mAmbientFluidDensity , * mRigidBodies , mRespectAngVel ) ;
            QUERY_PERFORMANCE_EXIT( PclOp_FluidBodyInteraction ) ;
        }

        Vector< Particle > *        mParticles              ;   ///< Dynamic array of particles whose bounding box to find
        const UniformGrid< Vec3 > * mVelocityGrid           ;   ///< Grid of velocity values
        float                       mAmbientFluidDensity    ;   ///< Fluid density in the absence of fluid particles
        Vector< RigidBody * > *     mRigidBodies            ;   ///< Dynamic array of addresses of rigid bodies
        bool                        mRespectAngVel          ;   ///< Whether to use particle angular velocity in interaction
} ;

#endif
