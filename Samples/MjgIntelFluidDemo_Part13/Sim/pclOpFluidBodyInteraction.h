/*! \file PclOpFluidBodyInteraction.h

    \brief Particle operation to interact vorton-based fluid with rigid bodies.

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
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_FLUID_BODY_INTERACTION_H
#define PARTICLE_OPERATION_FLUID_BODY_INTERACTION_H

#include "Sim/Particle/particleOperation.h"

/** Particle operation to interact vorton-based fluid with rigid bodies.
*/
class PclOpFluidBodyInteraction : public IParticleOperation
{
    public:
        PclOpFluidBodyInteraction()
            : mVelocityGrid( 0 )
            , mDensityDeviationGrid( 0 )
            , mAmbientFluidDensity( 0.0f )
            , mFluidSpecificHeatCapacity( 1.0f )
            , mGravityAcceleration( Vec3( 0.0f , 0.0f , 0.0f ) )
            , mPhysicalObjects( 0 )
            , mRespectAngVel( true )
        {}

        void Operate(  Vector< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ )
        {
            QUERY_PERFORMANCE_ENTER ;
            FluidBodySim::SolveBoundaryConditions( particles , * mVelocityGrid , mAmbientFluidDensity , mFluidSpecificHeatCapacity , * mPhysicalObjects , mRespectAngVel ) ;
            if( mDensityDeviationGrid != 0 )
            {   // Caller wants to use this fluid to buoy bodies submerged in it.
                FluidBodySim::BuoyBodies( * mDensityDeviationGrid , mAmbientFluidDensity , mGravityAcceleration , * mPhysicalObjects ) ;
            }
            QUERY_PERFORMANCE_EXIT( PclOp_FluidBodyInteraction ) ;
        }

        const UniformGrid< Vec3 > *             mVelocityGrid               ;   ///< Grid of velocity values
        const UniformGrid< float > *            mDensityDeviationGrid       ;   ///< Grid of density deviation-about-ambient values
        float                                   mAmbientFluidDensity        ;   ///< Fluid density in the absence of fluid particles
        float                                   mFluidSpecificHeatCapacity  ;   ///< Specific heat capacity of the fluid.
        Vec3                                    mGravityAcceleration        ;   ///< Acceleration due to gravity
        Vector< Impulsion::PhysicalObject * > * mPhysicalObjects            ;   ///< Dynamic array of addresses of physical objects.
        bool                                    mRespectAngVel              ;   ///< Whether to use particle angular velocity in interaction
} ;

#endif
