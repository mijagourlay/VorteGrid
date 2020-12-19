/** \file particleSystemConfiguration.h

    \brief Routines to configure particle systems.

    These structs and routines stand proxy for defining particle systems using some definitions from a configuration file.

*/
#include "Core/Containers/vector.h"

class PclOpEmit ;
class PclOpSeedSurfaceTracers ;
class PclOpFindBoundingBox ;
class PclOpVortonSim ;
class PclOpFluidBodyInteraction ;
class PclOpPopulateVelocityGrid ;
class PclOpAssignScalarFromGrid ;
class PclOpAssignVelocityFromField ;
class PclOpEvolve ;
class PclOpWind ;
class PclOpKillAge ;
class ParticleGroup ;
class ParticleSystem ;

namespace Impulsion
{
    class PhysicalObject ;
} ;

struct FluidVortonPclGrpInfo
{
    ParticleGroup               *   mParticleGroup                  ;   ///< Particle group for vortons.
    PclOpEmit                   *   mPclOpEmit                      ;   ///< Emit vortons.
    PclOpVortonSim              *   mPclOpVortonSim                 ;   ///< Update velocity grid due to vorton-based fluid simulation.
    PclOpFluidBodyInteraction   *   mPclOpFluidBodInte              ;   ///< Interact vortons with rigid bodies.
    PclOpPopulateVelocityGrid   *   mPclOpPopulateVelocityGrid      ;   ///< Populate velocity grid from particles.
    PclOpEvolve                 *   mPclOpEvolve                    ;   ///< Update particle position & orientation from velocities.
    PclOpWind                   *   mPclOpWind                      ;   ///< Apply wind to vortons.
    PclOpKillAge                *   mPclOpKillAge                   ;   ///< Kill old vortons.
} ;

struct FluidTracerPclGrpInfo
{
    ParticleGroup               *   mParticleGroup                  ;   ///< Particle group for tracers.
    PclOpEmit                   *   mPclOpEmit                      ;   ///< Emit passive tracer particles.
// For testing only:
PclOpSeedSurfaceTracers     *   mPclOpSeedSurfaceTracers        ;   ///< Seed tracer particles at fluid surface.
    PclOpFindBoundingBox        *   mPclOpFindBoundingBox           ;   ///< Find bounding box containing all tracers.
    PclOpFluidBodyInteraction   *   mPclOpFluidBodInte              ;   ///< Interact passive tracers with rigid bodies.
    PclOpAssignScalarFromGrid   *   mPclOpAssignDensityFromGrid     ;   ///< Assign density to tracers from grid populated from vortons.
    PclOpAssignScalarFromGrid   *   mPclOpAssignFuelFromGrid        ;   ///< Assign fuel to tracers from grid populated from vortons.
    PclOpAssignScalarFromGrid   *   mPclOpAssignFlameFromGrid       ;   ///< Assign flame to tracers from grid populated from vortons.
    PclOpAssignScalarFromGrid   *   mPclOpAssignSmokeFromGrid       ;   ///< Assign smoke to tracers from grid populated from vortons.
    PclOpAssignVelocityFromField*   mPclOpAssignVelocityFromField   ;   ///< Advect tracers according to velocity field.
    PclOpWind                   *   mPclOpWind                      ;   ///< Apply wind to tracers.
    PclOpEvolve                 *   mPclOpEvolve                    ;   ///< Update particle position & orientation.
    PclOpKillAge                *   mPclOpKillAge                   ;   ///< Kill old passive tracer particles.
} ;

extern ParticleSystem * CreateFluidParticleSystems( FluidVortonPclGrpInfo & vortonPclGrpInfo , FluidTracerPclGrpInfo & tracerPclGrpInfo , const float viscosity , const float ambientFluidDensity , const float fluidSpecificHeatCapacity , VECTOR< Impulsion::PhysicalObject * > & physicalObjects ) ;
extern ParticleSystem * CreateNonFluidParticleSystem( size_t i ) ;
