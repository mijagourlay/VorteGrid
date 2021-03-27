/** \file PclOpFluidBodyInteraction.cpp

    \brief Particle operation to interact vorton-based fluid with rigid bodies.

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "FluidBodySim/pclOpFluidBodyInteraction.h"

#include "FluidBodySim/fluidBodySim.h"

#include "Core/Performance/perfBlock.h"




void PclOpFluidBodyInteraction::Operate(  VECTOR< Particle > & particles , float /*timeStep*/ , unsigned /*uFrame*/ )
{
    PERF_BLOCK( PclOpFluidBodyInteraction__Operate ) ;

    ASSERT( mPhysicalObjects != 0 ) ;

#if REDUCE_CONVERGENCE
    SolveBoundaryConditionsAndReduceDivergence( particles , timeStep , uFrame ) ;
#else
    FluidBodySim::SolveBoundaryConditions( particles , mAmbientFluidDensity , mFluidSpecificHeatCapacity , * mPhysicalObjects , mRespectAngVel ) ;
#endif

    if( mDensityGrid != 0 )
    {   // Caller wants to use this fluid to buoy bodies submerged in it.
        ASSERT( ! mDensityGrid->HasZeroExtent() ) ;
        ASSERT( mFluidSpecificHeatCapacity != 0.0f ) ;
        FluidBodySim::BuoyBodies( * mDensityGrid , mAmbientFluidDensity , mGravityAcceleration , * mPhysicalObjects ) ;
    }
}
