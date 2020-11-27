/*! \file FluidBodySim.h

    \brief Simulation with mutually interacting fluid and rigid bodies

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
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
        /*! \brief Construct a simulation with fluid and rigid bodies
        */
        FluidBodySim( float viscosity , float density )
            : mVortonSim( viscosity , density )
        {}

        void                    Initialize( unsigned numTracersPerCellCubeRoot ) ;
        void                    Update( float timeStep , unsigned uFrame ) ;
        VortonSim &             GetVortonSim( void )    { return mVortonSim ; }
        Vector< RbSphere > &    GetSpheres( void )      { return mSpheres ; }
        void                    Clear( void )
        {
            mVortonSim.Clear() ;
            mSpheres.Clear() ;
        }

        static void UnitTest( void ) ;

    private:
        FluidBodySim( const FluidBodySim & that ) ;             // Disallow copy construction
        FluidBodySim & operator=( const FluidBodySim & that ) ; // Disallow assignment

        void RemoveEmbeddedParticles( void ) ;
        void SolveBoundaryConditions( void ) ;

        VortonSim           mVortonSim ;
        Vector< RbSphere >  mSpheres   ;
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------


#endif
