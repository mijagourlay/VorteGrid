/** \file PclOpFluidBodyInteraction.cpp

    \brief Particle operation to interact vorton-based fluid with rigid bodies.

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "pclOpFluidBodyInteraction.h"

#include "fluidBodySim.h"

#include "Core/Performance/perf.h"




void PclOpFluidBodyInteraction::Operate(  VECTOR< Particle > & particles , float /*timeStep*/ , unsigned /*uFrame*/ )
{
    ASSERT( mPhysicalObjects != 0 ) ;
    QUERY_PERFORMANCE_ENTER ;

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
    QUERY_PERFORMANCE_EXIT( PclOp_FluidBodyInteraction ) ;
}
