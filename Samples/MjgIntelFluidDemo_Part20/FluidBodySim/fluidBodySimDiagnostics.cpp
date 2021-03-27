/*! \file FluidBodySimDiagnostics.cpp

    \brief Simulation with mutually interacting fluid and rigid bodies, diagnostic routines

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "Core/Performance/perf.h"
#include "Sim/Vorton/vorticityDistribution.h"

#include "fluidBodySim.h"


/* static */ void FluidBodySim::UnitTest( void )
{
#if USE_TBB
    tbb::task_scheduler_init tbb_init ;
#endif

    static const float viscosity = 0.01f ;
    static const float density   = 1.0f ;
    FluidBodySim fluidBodySim( viscosity , density ) ;

    static const float      fRadius         = 1.0f ;
    static const float      fThickness      = 1.0f ;
    static const float      fMagnitude      = 1.0f ;
    static const unsigned   numCellsPerDim  = 16 ;
    static const unsigned   numVortonsMax   = numCellsPerDim * numCellsPerDim * numCellsPerDim ;

    Vector< Vorton > & vortons = fluidBodySim.GetVortonSim().GetVortons() ;

#if 0 // vortex ring -- vorticity in [0,1]
    AssignVorticity( vortons , fMagnitude , numVortonsMax , VortexRing( fRadius , fThickness , Vec3( 0.0f , 0.0f , 1.0f ) ) ) ;
#elif 1 // "jet" vortex ring -- velocity in [0,1]
    AssignVorticity( vortons , fMagnitude , numVortonsMax , JetRing( fRadius , fThickness , Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;
#elif 0 // Noise
    AssignVorticity( vortons , 0.125f * FLT_EPSILON , 2048 , VortexNoise( Vec3( 4.0f * fThickness , 0.5f * fThickness , 0.5f * fThickness ) ) ) ;
#elif 0 // 2D sheet
    AssignVorticity( vortons , fMagnitude , numVortonsMax , VortexSheet( fThickness , /* variation */ 0.0f , /* width */ 2.0f * fThickness ) ) ;
#elif 0 // Vortex sheet with spanwise variation
    AssignVorticity( vortons , fMagnitude , numVortonsMax , VortexSheet( fThickness , /* variation */ 0.2f , /* width */ 7.0f * fThickness ) ) ;
#elif 0 // Vortex tube
    AssignVorticity( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 2.0f * fThickness , 2 , 0 ) ) ;
#elif 0 // 2 orthogonal vortex tubes
    AssignVorticity( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 4.0f * fThickness , 2 , -1 ) ) ;
    AssignVorticity( vortons , fMagnitude , numVortonsMax , VortexTube( fThickness , /* variation */ 0.0f , /* width */ 4.0f * fThickness , 2 ,  1 ) ) ;
#endif

    // Add a sphere
#if 1   // Translating sphere
    fluidBodySim.GetSpheres().PushBack( RbSphere( Vec3( -2.0f , 0.0f , 0.0f ) , Vec3( 1.0f , 0.0f , 0.0f ) , 0.2f , 0.2f ) ) ;
#else   // Spinning sphere
    fluidBodySim.GetSpheres().PushBack( RbSphere( Vec3( 0.0f , 0.0f , 0.0f ) , Vec3( 0.0f , 0.0f , 0.0f ) , 1.0f , 0.2f ) ) ;
    fluidBodySim.GetSpheres()[0].ApplyImpulsiveTorque( Vec3( 0.0f , 0.2f , 0.0f ) ) ; // Make sphere spin
#endif

    fluidBodySim.Initialize( 3 ) ;

#if USE_TBB
    tbb::tick_count time0 = tbb::tick_count::now() ;
#endif

    double timeNow = 0.0f ;

    for( unsigned uFrame = 0 ; uFrame < 0 ; ++ uFrame )
    {
        DEBUG_ONLY( fprintf( stderr , "sim frame %u\n" , uFrame ) ) ;
        static const float  timeStep    = 1.0f / 30.0f ;
        fluidBodySim.Update( timeStep , uFrame ) ;
        timeNow += timeStep ;
    }

#if USE_TBB
    tbb::tick_count timeFinal = tbb::tick_count::now() ;
    fprintf( stderr , "tbb duration=%g second\n" , (timeFinal - time0).seconds() ) ;
#endif

}