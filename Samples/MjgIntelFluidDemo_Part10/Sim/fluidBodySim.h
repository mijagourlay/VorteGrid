/*! \file FluidBodySim.h

    \brief Simulation with mutually interacting fluid and rigid bodies

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
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef FLUID_BODY_SIM_H
#define FLUID_BODY_SIM_H

#include <math.h>

#include "RigidBody/rbSphere.h"
#include "Vorton/vortonSim.h"


// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Simulation with mutually interacting fluid and rigid bodies
*/
class FluidBodySim
{
    public:
        static void SolveBoundaryConditions( Vector< Particle > & particles , const UniformGrid< Vec3 > & velGrid , float ambientFluidDensity , const Vector< RigidBody * > & rigidBodies , bool bRespectAngVel ) ;
        static void RemoveEmbeddedParticles( Vector< Particle > & particles , const Vector< RigidBody * > & rigidBodies ) ;
        static void BuoyBodies( const UniformGrid< float > & densityGrid , float ambientFluidDensity , const Vec3 & gravityAcceleration , const Vector< RigidBody * > & rigidBodies ) ;

        static void UnitTest( void ) ;

    private:
        FluidBodySim( const FluidBodySim & ) ;             // Disallow copy construction
        FluidBodySim & operator=( const FluidBodySim & ) ; // Disallow assignment

        static void CollideTracersSlice( Vector< Particle > & particles , float ambientFluidDensity , const RbSphere & rSphere , Vec3 & rImpulseOnBody , size_t iPclStart , size_t iPclEnd ) ;

    #if USE_TBB
        friend class FluidBodySim_CollideTracers_TBB ;
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
