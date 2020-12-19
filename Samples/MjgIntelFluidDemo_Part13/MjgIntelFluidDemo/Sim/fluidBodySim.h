/*! \file FluidBodySim.h

    \brief Simulation with mutually interacting fluid and rigid bodies

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
#ifndef FLUID_BODY_SIM_H
#define FLUID_BODY_SIM_H

#include <math.h>

#include "Impulsion/physicalObject.h"
#include "Vorton/vortonSim.h"


// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/** Simulation with mutually interacting fluid and rigid bodies.
*/
class FluidBodySim
{
    public:
        static void SolveBoundaryConditions( Vector< Particle > & particles , const UniformGrid< Vec3 > & velGrid , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Vector< Impulsion::PhysicalObject * > & physicalObjects , bool bRespectAngVel ) ;
        static void RemoveEmbeddedParticles( Vector< Particle > & particles , const Vector< Impulsion::PhysicalObject * > & physicalObjects ) ;
        static void BuoyBodies( const UniformGrid< float > & densityGrid , float ambientFluidDensity , const Vec3 & gravityAcceleration , const Vector< Impulsion::PhysicalObject * > & physicalObjects ) ;

    private:
        FluidBodySim( const FluidBodySim & ) ;             // Disallow copy construction
        FluidBodySim & operator=( const FluidBodySim & ) ; // Disallow assignment

        static void CollideVortonsSlice ( Vector< Particle > & particles , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , float & heatToBody , size_t iPclStart , size_t iPclEnd ) ;
        static void CollideVortonsReduce( Vector< Particle > & particles , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , float & heatToBody , size_t iPclStart , size_t iPclEnd , size_t grainSize ) ;
        static void CollideTracersSlice ( Vector< Particle > & particles , float ambientFluidDensity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , size_t iPclStart , size_t iPclEnd ) ;
        static void CollideTracersReduce( Vector< Particle > & particles , float ambientFluidDensity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , size_t iPclStart , size_t iPclEnd , size_t grainSize ) ;

    #if USE_TBB
        friend class FluidBodySim_CollideVortons_TBB ;
        friend class FluidBodySim_CollideTracers_TBB ;
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
