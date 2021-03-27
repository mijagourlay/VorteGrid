/** \file FluidBodySim.h

    \brief Simulation with mutually interacting fluid and rigid bodies

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-17/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef FLUID_BODY_SIM_H
#define FLUID_BODY_SIM_H

#include "VortonFluid/vortonSim.h"

#include "Core/Performance/perfBlock.h"

#include <math.h>


// Macros --------------------------------------------------------------

#define COMPUTE_PARTICLE_PROXIMITY_TO_WALLS 1

// Types --------------------------------------------------------------

namespace Impulsion
{
    class PhysicalObject ;
} ;

/** Simulation with mutually interacting fluid and rigid bodies.

    \note   This class lost all of its members so at this point it's just a namespace.
    \todo   Convert this class to a namespace.
*/
class FluidBodySim
{
    public:
        static void SolveBoundaryConditions( VECTOR< Particle > & particles , float ambientFluidDensity , float fluidSpecificHeatCapacity , const VECTOR< Impulsion::PhysicalObject * > & physicalObjects , bool bRespectAngVel ) ;
        static void RemoveEmbeddedParticles( VECTOR< Particle > & particles , const VECTOR< Impulsion::PhysicalObject * > & physicalObjects ) ;
        static void BuoyBodies( const UniformGrid< float > & densityGrid , float ambientFluidDensity , const Vec3 & gravityAcceleration , const VECTOR< Impulsion::PhysicalObject * > & physicalObjects ) ;

    #if COMPUTE_PARTICLE_PROXIMITY_TO_WALLS
        static void ComputeParticleProximityToWalls( VECTOR< float > & proximities , const VECTOR< Particle > & particles , const VECTOR< Impulsion::PhysicalObject * > & physicalObjects , const float maxProximity ) ;
    #endif

    #if POISON_DENSITY_BASED_ON_GRIDPOINTS_INSIDE_WALLS
        static void PoisonDensity( UniformGrid< float > & densityGrid , VECTOR< Impulsion::PhysicalObject * > & physicalObjects , const float testRadius ) ;
    #endif

    #if POISON_DENSITY_GRADIENT_BASED_ON_GRIDPOINTS_INSIDE_WALLS
        static void PoisonDensityGradient( UniformGrid< Vec3 > & densityGradientGrid , VECTOR< Impulsion::PhysicalObject * > & physicalObjects , const float testRadius ) ;
    #endif

    #if COMPUTE_PRESSURE_GRADIENT
        static void PoisonPressure( UniformGrid< float > & pressureGrid , VECTOR< Impulsion::PhysicalObject * > & physicalObjects , const float testRadius ) ;
    #endif

    #if PROFILE
        static unsigned GetNumVortonBodyHits() ;
        static unsigned GetNumTracerBodyHits() ;
    #endif

    private:
        FluidBodySim( const FluidBodySim & ) ;             // Disallow copy construction
        FluidBodySim & operator=( const FluidBodySim & ) ; // Disallow assignment

        static void CollideVortonsSlice ( VECTOR< Particle > & particles , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , float & rHeatToBody , float & rSumPclTemperature , size_t & rNumPclsCollided , size_t iPclStart , size_t iPclEnd ) ;
        static void CollideVortonsReduce( VECTOR< Particle > & particles , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , float & rHeatToBody , float & rSumPclTemperature , size_t & rNumPclsCollided , size_t iPclStart , size_t iPclEnd , size_t grainSize ) ;
        static void CollideTracersSlice ( VECTOR< Particle > & particles ,                                                               const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody ,                                                                                size_t iPclStart , size_t iPclEnd ) ;
        static void CollideTracersReduce( VECTOR< Particle > & particles ,                                                               const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody ,                                                                                size_t iPclStart , size_t iPclEnd , size_t grainSize ) ;

    #if USE_TBB
        friend class FluidBodySim_CollideVortons_TBB ;
        friend class FluidBodySim_CollideTracers_TBB ;
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
