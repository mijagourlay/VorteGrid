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
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/

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
        PclOpFluidBodyInteraction()
            : mVelocityGrid( 0 )
            , mDensityDeviationGrid( 0 )
            , mAmbientFluidDensity( 0.0f )
            , mGravityAcceleration( Vec3( 0.0f , 0.0f , 0.0f ) )
            , mRigidBodies( 0 )
            , mRespectAngVel( true )
        {}

        void Operate(  Vector< Particle > & particles , float timeStep , unsigned uFrame )
        {
            (void) timeStep ; // Avoid "unreferenced formal parameter" warning.
            (void) uFrame   ; // Avoid "unreferenced formal parameter" warning.
            QUERY_PERFORMANCE_ENTER ;
            FluidBodySim::SolveBoundaryConditions( particles , * mVelocityGrid , mAmbientFluidDensity , * mRigidBodies , mRespectAngVel ) ;
            if( mDensityDeviationGrid != 0 )
            {
                FluidBodySim::BuoyBodies( * mDensityDeviationGrid , mAmbientFluidDensity , mGravityAcceleration , * mRigidBodies ) ;
            }
            QUERY_PERFORMANCE_EXIT( PclOp_FluidBodyInteraction ) ;
        }

        const UniformGrid< Vec3 > *     mVelocityGrid           ;   ///< Grid of velocity values
        const UniformGrid< float > *    mDensityDeviationGrid   ;   ///< Grid of density deviation-about-ambient values
        float                           mAmbientFluidDensity    ;   ///< Fluid density in the absence of fluid particles
        Vec3                            mGravityAcceleration    ;   ///< Acceleration due to gravity
        Vector< RigidBody * > *         mRigidBodies            ;   ///< Dynamic array of addresses of rigid bodies
        bool                            mRespectAngVel          ;   ///< Whether to use particle angular velocity in interaction
} ;

#endif
