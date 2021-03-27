/** \file PclOpFluidBodyInteraction.h

    \brief Particle operation to interact vorton-based fluid with rigid bodies.

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PARTICLE_OPERATION_FLUID_BODY_INTERACTION_H
#define PARTICLE_OPERATION_FLUID_BODY_INTERACTION_H

#include "Particles/Operation/particleOperation.h"




// Forward declaration
namespace Impulsion
{
    class PhysicalObject ;
} ;




/** Particle operation to interact vorton-based fluid with rigid bodies.
*/
class PclOpFluidBodyInteraction : public IParticleOperation
{
    public:
        PclOpFluidBodyInteraction()
            : mDensityGrid( 0 )
        #if REDUCE_CONVERGENCE
            , mVortonIndicesGrid( 0 )
        #endif
            , mAmbientFluidDensity( 0.0f )
            , mFluidSpecificHeatCapacity( 1.0f )
            , mGravityAcceleration( Vec3( 0.0f , 0.0f , 0.0f ) )
            , mPhysicalObjects( 0 )
            , mRespectAngVel( true )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpFluidBodyInteraction ) ;

    #if REDUCE_CONVERGENCE
        ////////////////////////////////////////////////////////////////////////////////
        // This REDUCE_CONVERGENCE code was a temporary work-around for issues in the
        // vorton simulation introduced in Part 14.  It uses an ad-hoc divergence
        // reduction scheme.  Do not spend time trying to unravel its mysteries.  Expect
        // it to disappear soon.
        ////////////////////////////////////////////////////////////////////////////////
        /** Reduce divergence in number density of particles, while satisfying boundary conditions.
        */
        void SolveBoundaryConditionsAndReduceDivergence(  VECTOR< Particle > & particles , float timeStep , unsigned uFrame )
        {
            extern float gVortonSim_DisplacementMax    ;
            extern int   gVortonSim_NumRelaxationIters ;
            float displacementMax     = - FLT_MAX ;
            float displacementMaxPrev =   FLT_MAX ;
            int iter ;
            for( iter = 0 ; iter < 3 /* 32 */ ; ++ iter )
            {
                FluidBodySim::SolveBoundaryConditions( particles , mAmbientFluidDensity , mFluidSpecificHeatCapacity , * mPhysicalObjects , mRespectAngVel ) ;
                if( mVortonIndicesGrid )
                {
                    displacementMax = VortonSim::ReduceDivergence( reinterpret_cast< VECTOR< Vorton > * >( & particles ) , * mVortonIndicesGrid ) ;
                }
                if( ( displacementMax < FLT_EPSILON ) && ( displacementMaxPrev < FLT_EPSILON ) )
                {   // Worst displacement was small enough to ignore.
                    break ; // Stop relaxing particles.
                }
                displacementMaxPrev = displacementMax ; // Remember worst displacement for reporting.
                gVortonSim_DisplacementMax = displacementMaxPrev ;    // Report previous worst displacement (since last one was zero and not interesting).
            }
            gVortonSim_NumRelaxationIters = iter ;
        }
    #endif


        void Operate(  VECTOR< Particle > & particles , float timeStep , unsigned uFrame ) ;

        const UniformGrid< float > *            mDensityGrid                ;   ///< Grid of fluid density values.

    #if REDUCE_CONVERGENCE
        const UniformGrid< VECTOR< unsigned > > * mVortonIndicesGrid        ;
    #endif

        float                                   mAmbientFluidDensity        ;   ///< Fluid density in the absence of fluid particles
        float                                   mFluidSpecificHeatCapacity  ;   ///< Specific heat capacity of the fluid.
        Vec3                                    mGravityAcceleration        ;   ///< Acceleration due to gravity
        VECTOR< Impulsion::PhysicalObject * > * mPhysicalObjects            ;   ///< Dynamic array of addresses of physical objects.
        bool                                    mRespectAngVel              ;   ///< Whether to use particle angular velocity in interaction
} ;

#endif