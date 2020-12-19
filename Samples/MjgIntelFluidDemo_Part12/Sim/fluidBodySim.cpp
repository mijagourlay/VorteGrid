/*! \file FluidBodySim.cpp

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <float.h>

#include <xmmintrin.h>  // SSE intrinsics

#include "Core/Performance/perf.h"
#include "Sim/Vorton/vorticityDistribution.h"

#include "fluidBodySim.h"


#if PROFILE
unsigned gNumVortonBodyHits = 0 ;
unsigned gNumTracerBodyHits = 0 ;
#endif



#if USE_TBB
    /** Function object to collide passive tracer particles with rigid bodies.
    */
    class FluidBodySim_CollideTracers_TBB
    {
            Vector< Particle > &    mTracers                ;   ///< Dynamic array of tracers
            const RbSphere  &       mRbSphere               ;   ///< Reference to RbSphere object colliding with tracers
            float                   mAmbientFluidDensity    ;   ///< Fluid density in the absence of particles

            WORD        mMasterThreadFloatingPointControlWord   ;   ///< FPCW from spawning thread.
            unsigned    mMasterThreadMmxControlStatusRegister   ;   ///< MXCSR from spawning thread.

            size_t      mBegin      ;   ///< Loop start for use with parallel_reduce.
            size_t      mEnd        ;   ///< Loop end for use with parallel_reduce.
            size_t      mGrainSize  ;   ///< Target number of elements to process per thread.

        public:
            /// Constructor to use with parallel_invoke.
            FluidBodySim_CollideTracers_TBB( Vector< Particle > & rTracers , float ambientFluidDensity , const RbSphere & rRbSphere , size_t begin , size_t end , size_t grainSize )
                : mTracers( rTracers )
                , mAmbientFluidDensity( ambientFluidDensity )
                , mRbSphere( rRbSphere )
                , mBegin( begin )
                , mEnd( end )
                , mGrainSize( grainSize )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }

            /// Constructor to use with parallel_reduce.
            FluidBodySim_CollideTracers_TBB( Vector< Particle > & rTracers , float ambientFluidDensity , const RbSphere & rRbSphere )
                : mTracers( rTracers )
                , mAmbientFluidDensity( ambientFluidDensity )
                , mRbSphere( rRbSphere )
                , mBegin( 0 )
                , mEnd( 0 )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }

            /// Splitting copy constructor used by TBB parallel_reduce.
            FluidBodySim_CollideTracers_TBB( FluidBodySim_CollideTracers_TBB & that , tbb::split )
                : mTracers( that.mTracers )
                , mRbSphere( that.mRbSphere )
                , mAmbientFluidDensity( that.mAmbientFluidDensity )
                , mImpulseOnBody( that.mImpulseOnBody )
                , mMasterThreadFloatingPointControlWord( that.mMasterThreadFloatingPointControlWord )
                , mMasterThreadMmxControlStatusRegister( that.mMasterThreadMmxControlStatusRegister )
            {
            }

            /// Invocation operator for use with parallel_invoke.
            void operator() () const
            {   // Compute collisions for a subset of tracers.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mImpulseOnBody = FluidBodySim::CollideTracersReduce( mTracers , mAmbientFluidDensity , mRbSphere , mBegin , mEnd , mGrainSize ) ;
            }

            /// Invocation operator called by parallel_reduce.
            void operator() ( const tbb::blocked_range<size_t> & r )
            {   // Compute collisions for a subset of tracers.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                FluidBodySim::CollideTracersSlice( mTracers , mAmbientFluidDensity , mRbSphere , mImpulseOnBody , r.begin() , r.end() ) ;
            }

            /// Join the results of two threads spaned by parallel_reduce.
            void join( const FluidBodySim_CollideTracers_TBB & other )
            {   // Reduce the results of 2 threads
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mImpulseOnBody += other.mImpulseOnBody ;
            }
            mutable Vec3    mImpulseOnBody ;    ///< Impulse applied by tracers to rigid body
    } ;
#endif




/** Select boundary condition handling scheme.

    The vorticity reassigned to this vortex should be such that
    the fluid velocity, relative to the body surface, is zero.

    We can approximate this in a number of ways.  Here are some options:

    (a) Ignore all other contributions other than that of this vortex.
        Reassign this vorton's vorticity so that the fluid velocity,
        due only to this one vorton, is zero at the body surface.
        This simple scheme yields surprisingly pleasing results,
        perhaps better than (b) below, probably because, for all its simplicitiy,
        it does not double-count thie influence of the vorton being moved.
        And also, if the vorton is in contact with the body, its influence is
        likely dominate the ambient field anyway.

    (b) Interpolate the velocity at this point using the existing velocity field
        (which spuriously includes the influence of this vorton at its old position),
        then reassign this vorton's vorticity to counteract that.
        This can lead to instability if not damped.
        This technique is probably the worst of the three supplied here.

    (c) Interpolate velocity at this point using the existing velocity field,
        remove the contribution to that due to this vorton's old position and vorticity,
        then reassing this vorton's vorticity to counteract that.
        This arguably the most accurate of those mentioned here.

    (d) Like (c) but also (hypothetically) updating the ambient flow each time
        a vorton gets moved.  This would entail a tremendous amount of computation,
        so I mention it here only for the sake of providing a coherent sense of
        what is theoretically possible.

    In lieu of treating this as accurately as hypothetically possible, we recognize
    the following fact: Each of these schemes applies a correction to any vortons
    interacting with the body.  Choices (b) and (c) are likely to over-correct,
    not under-correct, since they omit the persistent change in the ambient flow
    (for this time step).  That implies we can (and should) apply only a portion
    of the correction, to each vorton, at each time step.  See DELAY_SHEDDING in
    the code below.

*/
#define BOUNDARY_NO_SLIP_NO_THRU                        1

/// BOUNDARY_RESPECTS_AMBIENT_FLOW only takes effect if BOUNDARY_NO_SLIP_NO_THRU is true.
#define BOUNDARY_RESPECTS_AMBIENT_FLOW                  1

/// BOUNDARY_AMBIENT_FLOW_OMITS_VORTON_OLD_POSITION only takes effect if BOUNDARY_RESPECTS_AMBIENT_FLOW is true.
#define BOUNDARY_AMBIENT_FLOW_OMITS_VORTON_OLD_POSITION 1




/** Whether flow affects rigid bodies immersed in the fluid.

    Normally we will leave this enabled but for testing we can disable it.
*/
#define FLOW_AFFECTS_BODY 1




/** Remove particles within rigid bodies.

    This routine should only be called initially, to remove
    excess particles initially inside rigid bodies.

    This is not meant to be called during simulation updates.
    Instead, run collision detection and response.

    \note This routine does NOT remove particles merely intersecting
            with the given rigid bodies; the particle center must
            reside inside a rigid body in order for this routine
            to kill it.

    \see SolveBoundaryConditions

*/
void FluidBodySim::RemoveEmbeddedParticles( Vector< Particle > & particles , const Vector< RigidBody * > & rigidBodies )
{
    if( particles.Empty() )
    {   // No particles to remove.
        return ; // Quit before accessing particles[0] below.
    }

    const size_t numBodies      = rigidBodies.Size() ;
    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each sphere in the simulation...
        RbSphere &  rSphere         = ( RbSphere & ) * rigidBodies[ uBody ] ;

        Particle * pParticles = & particles[0] ;
        for( size_t iParticle = 0 ; iParticle < particles.Size() ; )
        {   // For each passive tracer particle in the simulation...
            Particle & rParticle = pParticles[ iParticle ] ;
            const Vec3  vSphereToTracer = rParticle.mPosition - rSphere.GetPosition() ;   // vector from sphere center to tracer
            const float fSphereToTracer = vSphereToTracer.Magnitude() ;
            if( fSphereToTracer < ( rSphere.mRadius /* Note the lack of rParticle.mSize in this expression. */ ) )
            {   // Particle is inside body.
                // Delete particle.
                Particles::Kill( particles , iParticle ) ;
                // Deleted particle so now this same index contains a different tracer.
            }
            else
            {   // Did not delete particle so skip past it.
                ++ iParticle ;
            }
        }
    }
}




/** Collide tracer particles with rigid bodies.

    \param rSphere - reference to a spherical rigid body

    \param iPclStart - starting index of tracer particle to process.
        iPclStart must be less than the total number of tracer particles.

    \param iPclEnd - one past ending index of tracer particle to process.
        iPclStart must be less than or equal to the total number of tracer particles.

    \see SolveBoundaryConditions
*/
/* static */ void FluidBodySim::CollideTracersSlice( Vector< Particle > & particles , float ambientFluidDensity , const RbSphere & rSphere , Vec3 & rImpulseOnBody , const size_t iPclStart , const size_t iPclEnd )
{
    QUERY_PERFORMANCE_ENTER ;

    Vec3 impulseOnBody( 0.0f , 0.0f , 0.0f ) ; // Impulse particles apply to rigid body

    Particle * pTracers = & particles[ 0 ] ;
    // Collide tracers with rigid body.
    for( size_t uTracer = iPclStart ; uTracer < iPclEnd ; ++ uTracer )
    {   // For each tracer in the simulation...
        // _mm_prefetch( (char*)( & pTracers[ uTracer + 5 ] ) , _MM_HINT_T0 ) ;
        Particle &  rTracer         = pTracers[ uTracer ] ;
        const Vec3  vSphereToTracer = rTracer.mPosition - rSphere.GetPosition() ;   // vector from body center to tracer
        const float fSphereToTracer = vSphereToTracer.Magnitude() ;
        const float fCombinedRadii  = rTracer.mSize + rSphere.mRadius ;
        if( fSphereToTracer < fCombinedRadii )
        {   // Tracer is colliding with body.
            #if PROFILE
                ++ gNumTracerBodyHits ;
            #endif

            // Project tracer to outside of body.
            // This places the particle on the body surface.
            const float distRescale         = ( rSphere.mRadius + rTracer.mSize ) * ( 1.0f + FLT_EPSILON ) / fSphereToTracer ;
            const Vec3  vDisplacementNew    = vSphereToTracer * distRescale ;
            rTracer.mPosition = rSphere.GetPosition() + vDisplacementNew ;

            // Transfer linear momentum between particle and body.
            const Vec3  vVelDueToRotation   = rSphere.GetAngVelocity() ^ vDisplacementNew ; // linear velocity, at particle new position, due to body rotation
            const Vec3  vVelNew             = rSphere.GetVelocity() + vVelDueToRotation ;   // Total linear velocity of vorton at its new position, due to sticking to body
            #if FLOW_AFFECTS_BODY
            const Vec3  vMinusPclVelChange  = rTracer.mVelocity - rSphere.GetVelocity() + vVelDueToRotation ;   // (negative of) linear velocity change applied to vorton, minus vVelDueToRotation for Magnus effect
            impulseOnBody += vMinusPclVelChange * rTracer.GetMass( ambientFluidDensity ) ;
            #endif
            rTracer.mVelocity = vVelNew ;   // If same tracer is involved in another contact before advection, this will conserve momentum.
        }
    }
    rImpulseOnBody = impulseOnBody ;

    QUERY_PERFORMANCE_EXIT( FluidBodySim_CollideTracersSlice ) ;
}




#if USE_TBB
/** Collide body with fluid particles using a deterministic, multi-threaded, recursive algorithm.

    \return Impulse to apply to body.
*/
/* static */ Vec3 FluidBodySim::CollideTracersReduce( Vector< Particle > & particles , float ambientFluidDensity , const RbSphere & rSphere , const size_t iPclStart , const size_t iPclEnd , const size_t grainSize )
{
    Vec3 impulseOnBody ;    // Accumulator for impulse due to particles on the body.
    const size_t indexSpan = iPclEnd - iPclStart ;
    if( indexSpan <= grainSize )
    {   // Sub-problem fits into a single serial chunk.
        CollideTracersSlice( particles , ambientFluidDensity , rSphere , impulseOnBody , iPclStart , iPclEnd ) ;
    }
    else
    {   // Problem remains large enough to split into pieces.
        size_t iPclMiddle = iPclStart + indexSpan / 2 ;
        // Create one functor for each sub-problem.
        FluidBodySim_CollideTracers_TBB ct1( particles , ambientFluidDensity , rSphere , iPclStart  , iPclMiddle , grainSize ) ;
        FluidBodySim_CollideTracers_TBB ct2( particles , ambientFluidDensity , rSphere , iPclMiddle , iPclEnd    , grainSize ) ;
        // Invoke both sub-problems, each on a separate thread.
        tbb::parallel_invoke( ct1 , ct2 ) ;
        // Combine results from each thread: the total impulse due to the particles processed by each thread.
        impulseOnBody = ct1.mImpulseOnBody + ct2.mImpulseOnBody ;
    }
    return impulseOnBody ;
}
#endif




/** Calculate and apply buoyancy on rigid bodies immersed in a fluid.

    \param densityDeviationGrid Uniform grid of density deviation values.
            Each value represents the density deviation about the ambient.

    \param ambientFluidDensity  Fluid density in the absense of fluid
            particles.  This treatment allows particles to represent deviations
            in ambient fluid density, thereby allowing for a more arbitrary and
            economical usage of particles.

    \param gravityAcceleration  Acceleration due to gravity.

    \param rigidBodies  Dynamic array of rigid bodies.

*/
/* static */ void FluidBodySim::BuoyBodies( const UniformGrid< float > & densityDeviationGrid , float ambientFluidDensity , const Vec3 & gravityAcceleration , const Vector< RigidBody * > & rigidBodies )
{
QUERY_PERFORMANCE_ENTER ;

    const size_t    numBodies       = rigidBodies.Size() ;
    //const size_t    numParticles    =   particles.Size() ;
    const Vec3      gravityDir      = gravityAcceleration.GetDirFast() ;

    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each body in the simulation...
        RbSphere &  rSphere         = (RbSphere &) * rigidBodies[ uBody ] ;
        // Compute profile of fluid density around body
        float densityDeviationAtQueryPoint          ; // fluid density at query points.
        float densityDeviationSum           =   0   ; // Average fluid density in region of body.
        float divisor                       = 1.0f  ;
        // Sample fluid density at multiple places within the body region.
        Vec3 vQueryPos = rSphere.GetPosition() ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationSum , vQueryPos ) ;
        }
        vQueryPos = rSphere.GetPosition() + 0.5f * rSphere.mRadius * gravityDir ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationAtQueryPoint , vQueryPos ) ;
            densityDeviationSum += densityDeviationAtQueryPoint ;
            divisor += 1.0f ;
        }
        vQueryPos = rSphere.GetPosition() - 0.5f * rSphere.mRadius * gravityDir ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationAtQueryPoint , vQueryPos ) ;
            densityDeviationSum += densityDeviationAtQueryPoint ;
            divisor += 1.0f ;
        }
        vQueryPos = rSphere.GetPosition() + rSphere.mRadius * gravityDir ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationAtQueryPoint , vQueryPos ) ;
            densityDeviationSum += densityDeviationAtQueryPoint ;
            divisor += 1.0f ;
        }
        vQueryPos = rSphere.GetPosition() - rSphere.mRadius * gravityDir ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationAtQueryPoint , vQueryPos ) ;
            densityDeviationSum += densityDeviationAtQueryPoint ;
            divisor += 1.0f ;
        }
        // Average fluid density samples.
        const float densityAverage  = densityDeviationSum / divisor + ambientFluidDensity ;
        // Approximate body buoyancy force.
        const float massDisplaced   = densityAverage * rSphere.GetVolume() ;
        const float bodyMass        = 1.0f / rSphere.GetInverseMass() ;
        // Sum buoyancy and gravity forces.
        const Vec3  netForce        = gravityAcceleration * ( bodyMass - massDisplaced ) ;
        rSphere.ApplyBodyForce( netForce ) ;
    }
QUERY_PERFORMANCE_EXIT( FluidBodySim_BuoyBodies ) ;
}




/** Collide particles with rigid bodies.

    \param particles - dynamic array of particles

    \param velGrid - uniform grid of velocity values. NO LONGER USED.
            Instead, the velocity at each "contact point" comes directly from
            the particle velocity.

    \param ambientFluidDensity - fluid density in the absense of
            fluid particles.  This treatment allows particles to represent
            deviations in ambient fluid density, thereby allowing for a more
            arbitrary and economical usage of particles.

    \param fluidSpecificHeatCapacity - Heat capacity per unit mass of the fluid.

    \param rigidBodies - dynamic array of rigid bodies.

    \param bRespectAngularVelocity - This indicates whether a particle-body
            collision should consider the linear or angular velocity of the
            particle, treating it either as a tracer or as a source of
            vorticity. The former applies linear impulses to rigid bodies,
            whereas the latter apply torques.  Furthermore, vortons must change
            their vorticity to satisfy the no-slip and no-through boundary
            conditions.  See article 4 for details.  The dichotomy is somewhat
            artificial; in principle the simulation could consist entirely of
            vortons, without any passive tracers. But to keep computational cost
            down, the simulation typically has far fewer vortons than passive
            tracers. To avoid applying extraneous impulses to rigid bodies, the
            simulation treats vortons and tracers separately, relegating linear
            momentum to tracers and angular momentum to vortons, when applying
            impulses to rigid bodies.

    This uses a simplified form of "penalty" scheme which transfers linear and
    angular momentum between fluid particles and bodies immersed in the fluid.

    This implementation has some physical inaccuracies, described in comments in
    the code.

    A proper treatment of fluid-body interaction typically entails computing
    potential flow terms to add to the velocity induced by vorticity (which
    satisfies no-through boudnary conditions), and computing vorticity flux
    generated by viscous interactions between the fluid and body (which
    satisfies no-slip boundary conditions).  Or alternatively, one can introduce
    a "penalty" term into the fluid equations which pushes particles outside of
    bodies.

    In contrast, this ad-hoc scheme immediately projects particles embedded in
    bodies, to outside the bodies, and reassigns vorton vorticity (and hence
    angular momentum). This change in angular momentum in the flow is countered
    by an equal change in the body.

    Such accounting would only ever cause bodies to spin, not to translate, so
    in addition, particles in contact with the body also imparts a linear
    momentum in proportion to the difference in relative velocity between the
    particle and the body.  This second part, the transfer of linear momentum,
    is not physically accurate, but the scheme is simpler than the alternatives,
    and leads to visually plausible results.

    Upon tracking total kinetic energy of the fluid and bodies, we should see it
    decrease (not increase) which would be consistent with transfering kinetic
    energy to heat. If the equations governing the fluid motion took into
    account heat (e.g. through a change in density which would then appear as a
    baroclinic term in the vorticity equation), then we could balance the energy
    budget.

    This whole approach, including elsewhere, ignores fluid viscosity, which
    theoretically should influence the shear stress.  Introducing that should be
    straightforward but on the other hand, traditional treatments of bodies in
    "inviscid" fluids still introduce vorticity, and in this simulation,
    "viscosity" is tantamout to the rate vorticity diffuses "away from
    boundaries".  That is, low diffusion looks nice in the free fluid, and high
    diffusion looks nice near boundaries of solid bodies.  We want no-slip
    inviscid fluids.

*/
/* static */ void FluidBodySim::SolveBoundaryConditions( Vector< Particle > & particles , const UniformGrid< Vec3 > & velGrid , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Vector< RigidBody * > & rigidBodies , bool bRespectAngularVelocity )
{
#if ENABLE_FLUID_BODY_SIMULATION
QUERY_PERFORMANCE_ENTER ;


    const size_t    numBodies       = rigidBodies.Size() ;
    const size_t    numParticles    =   particles.Size() ;

    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each body in the simulation...
        RbSphere &  rSphere         = (RbSphere &) * rigidBodies[ uBody ] ;

        if( bRespectAngularVelocity )
        {   // Treat particles as vortons.
            QUERY_PERFORMANCE_ENTER ;

            // Collide vortons with rigid body.
            for( unsigned uVorton = 0 ; uVorton < numParticles ; ++ uVorton )
            {   // For each vorton in the simulation...
                Vorton &    rVorton             = (Vorton &) particles[ uVorton ] ;
                const float vortRadius          = rVorton.GetRadius() ;
                const float vortRadius2         = vortRadius * vortRadius ;
                const Vec3  vSphereToVorton     = rVorton.mPosition - rSphere.GetPosition() ;   // vector from body center to vorton
                const float fSphereToVorton     = vSphereToVorton.Magnitude() ;
                const Vec3  vSphereToVortonDir  = vSphereToVorton / fSphereToVorton ;
                // This boundary thickness compensates for low discretization resolution,
                // by spreading the influence of the body surface to just outside the body,
                // deeper into the fluid.  This also has an effect somewhat like
                // instantaneous viscous diffusion, in the immediate vicinity of
                // the boundary.  It should be kept as small as possible,
                // but must be at least 1.  A value of 1 means only vortons
                // colliding with the body receive influence.  A value of 2 seems
                // most appropriate since that is the size of a grid cell, so
                // 2 essentially means vortons within a grid cell receive influence.
                // So a value in [1,2] seems appropriate. But values over 1.2 trap
                // vortons inside the body, because the "bend" can draw vortons back
                // toward the body.
                // Note, the larger fBndThkFactor is, the more vortons get influenced,
                // which drives the simulation to instability and also costs more CPU
                // time due to the increased number of vortons involved.
                const float fBndThkFactor       = 1.2f ; // Thickness of boundary, in vorton radii.
                const float fBoundaryThickness  = fBndThkFactor * vortRadius ; // Thickness of boundary, i.e. region within which body sheds vorticity into fluid.

                if( fSphereToVorton < ( rSphere.mRadius + fBoundaryThickness ) )
                {   // Vorton is interacting with body.

                    #if PROFILE
                        ++ gNumVortonBodyHits ;
                    #endif

                    // Compute "contact" point, near where vorton touched body.
                    const Vec3 vContactPtRelBody        = vSphereToVortonDir * rSphere.mRadius ;
                    const Vec3 vContactPtWorld          = vContactPtRelBody + rSphere.GetPosition() ;

                    // Compute velocity of body at contact point.
                    // NOTE: Handling rigid body rotation in this way neglects the "vorticity" of the rigid body.  See notes on paper from 2009dec05.
                    const Vec3 vVelDueToRotAtConPt      = rSphere.GetAngVelocity() ^ vContactPtRelBody  ; // linear velocity, of body at contact point, due to its own rotation
                    const Vec3 vVelBodyAtConPt          = rSphere.GetVelocity() + vVelDueToRotAtConPt   ; // Total linear velocity of body at contact point

                    const Vec3 vVorticityOld            = rVorton.GetVorticity() ;  // Cache to compute change in angular momentum.

                    // Each scheme below projects this vorton to the body surface,
                    // but the exact location depends on the scheme.

    #if ! BOUNDARY_NO_SLIP_NO_THRU   // Assign vorticity to spin like the object.
                    // Place vorton tangent to body surface along surface normal.
                    const float distRescale         = ( rSphere.mRadius + vortRadius ) * ( 1.0f + FLT_EPSILON ) ;
                    const Vec3  vDisplacementNew    = vSphereToVortonDir * distRescale ;
                    rVorton.mPosition               = rSphere.mPosition + vDisplacementNew ;
                    const Vec3  vAngVelDiff         = rVorton.mAngularVelocity - rSphere.mAngVelocity ; // (negative of) change in angular velocity applied to vorton
                    rVorton.mAngularVelocity        = rSphere.mAngVelocity ;                            // Assign vorticity of vorton at its new position.

    #else // BOUNDARY_NO_SLIP_NO_THRU:

                    (void) velGrid ; // Avoid "unreferenced formal parameter" warning.

        #if ! BOUNDARY_RESPECTS_AMBIENT_FLOW
                    // This assigns a vorticity such that the fluid velocity,
                    // relative to the body velocity at the contact point, is zero.
                    // NOTE: This neglects the ambient flow due to other vortons.
                    const Vec3   velFlowRelBodyAtColPt = - vVelBodyAtConPt ;

        #else // BOUNDARY_RESPECTS_AMBIENT_FLOW
                    // Make relative fluid velocity at body nearest this vorton,
                    // due to "ambient" flow, to be zero.
                    Vec3 velAmbientAtContactPt ; // Velocity due to entire vorton field at contact point.
                #if 0
                    // Interpolate ambient velocity at that point on the sphere.
                    velGrid.Interpolate( velAmbientAtContactPt , vContactPtWorld ) ;
                #else
                    // Sneaky, cheap, approximately correct and includes velocity due to external influences -- reuse vorton velocity as ambient velocity.
                    velAmbientAtContactPt = rVorton.mVelocity ;
                #endif

            #if ! BOUNDARY_AMBIENT_FLOW_OMITS_VORTON_OLD_POSITION
                    // Compute relative velocity between body (at contact point) and ambient flow.
                    // NOTE: This neglects the fact that the ambient flow in mVelGrid also includes the
                    //       influence of this same vorton, at its previous position.  If this interaction
                    //       did not displace this vorton much, that could be a significant omission.
                    const Vec3 velFlowRelBodyAtColPt( velAmbientAtContactPt - vVelBodyAtConPt ) ;

            #else // BOUNDARY_AMBIENT_FLOW_OMITS_VORTON_OLD_POSITION
                    // Compute velocity induced by this vorton, from its old location, at contact point.
                    Vec3    velDueToVort( 0.0f , 0.0f , 0.0f ) ;
                    rVorton.AccumulateVelocity( velDueToVort , vContactPtWorld ) ;

                    // Compute relative velocity between body at contact point and ambient flow,
                    // subtracting the influence due to the vorton from the interpolated velocity.
                    const Vec3 velFlowRelBodyAtConPt( velAmbientAtContactPt - velDueToVort - vVelBodyAtConPt ) ;
            #endif
        #endif

                    // Place vorton tangent to body surface.
                    // Choose boundary solution point along a "bend" (b),
                    //              b_hat = w_hat ^ v_hat
                    //              |b|   = vortonRadius
                    // which is not necessarily along surface normal, r_hat,
                    // and where vorticity lies perpendicular to this plane
                    // formed by a direction vector and the ambient velocity:
                    //
                    // Approach 1: Solve at contact point.
                    // Reposition the vorton, typically partially inside the body.
                    //  Direction vector is the bend vector:
                    //              w_hat = v_hat ^ b_hat
                    //
                    // Approach 2: Solve at auxiliary point.
                    // Fully ejected the vorton from the body and solve boundary
                    // condition at an auxiliary point.
                    //  Direction vector is the surface normal:
                    //              w_hat = r_hat ^ v_hat
                    //
                    // Vorticity w is given by AssignByVelocity.
                    //
                    // Approach 1:
                    //          ,,.--..,           --:   ambient flow velocity
                    //       .'`        `'.      v  /| relative to body velocity
                    //     ,'              `\      /       at collition point
                    //    /     body         \    / ,..-..,
                    //   |                    |  /-`       `',
                    //  |               r      |/             \
                    //  |          o---------->C,   b          \
                    //  |                     || `'-,           |
                    //   |      C marks       |'     `'o        |
                    //    \      contact     /|   vorton with   |
                    //     `.    point.     /  \    counter-   /
                    //       '.,         ,-`    \  clockwise  /
                    //          `''--''``        `.,  flow _.`
                    //                              `''-''`
                    //
                    // Approach 2:
                    //                              --:     ambient flow velocity
                    //          ,,.--..,          v  /| relative to body velocity
                    //       .'`        `'.         /           at solution point
                    //     ,'              `\      / ,..-..,
                    //    /     body         \    /-`       `',
                    //   |                    |  /             \
                    //  |               r      |S,   b          \   S marks
                    //  |          o---------->C  `'-,           |  solution
                    //  |                      '      `'o        |   point.
                    //   |      C marks       '|   vorton with   |
                    //    \      contact     /  \    counter-   /
                    //     `.    point.     /    \  clockwise  /
                    //       '.,         ,-`      `.,  flow _.`
                    //          `''--''``            `''-''`
                    //
                    // This figure depicts the flow field after ejecting
                    // the vorton from the body interior.  Vorticity
                    // is assigned to the vorton such that the flow field
                    // satisfies no-through and no-slip boundary conditions
                    // at the contact point.
                    const Vec3  vSurfNormal         = vSphereToVorton.GetDirFast() ;
                    const Vec3  vVelDir             = velFlowRelBodyAtConPt.GetDirFast() ;
                    const Vec3  vVortDir            = vSurfNormal ^ vVelDir ;
                          Vec3  vBendDir            = vVortDir ^ vVelDir ;
                    vBendDir.NormalizeFast() ;
                    const float fBodySurfToVortCtr  = fSphereToVorton - rSphere.mRadius ;

                #if 1   // Approach 1:
                    // If vorton was inside body, push it outside body, otherwise just pivot vorton about contact point.
                    const float fBendDist           = fBodySurfToVortCtr < vortRadius ? vortRadius : fBodySurfToVortCtr ;
                    const Vec3  vBend               = fBendDist * vBendDir ;
                    rVorton.mPosition               = vContactPtWorld - vBend ;
                    const Vec3 &vSolnPos            = vContactPtWorld ;
                #else   // Approach 2:
                    // If vorton was inside body, push it outside body.
                    const float fDisplacement       = fBodySurfToVortCtr < vortRadius ? vortRadius : fBodySurfToVortCtr ;
                    const Vec3  vDisplacement       = fDisplacement * vSurfNormal ;
                    rVorton.mPosition               = vContactPtWorld - vDisplacement ;
                    // Choose boundary solution proxy location.
                    // Why not solve at the actual contact point?  Because doing usually entails
                    // moving the vorton to a location where it still intersects the body.
                    // If the ambient flow is ever exactly parallel to the surface normal,
                    // then the vorton would lie with its center on the body.  This makes
                    // ejecting the vorton in a single step impossible, and sometimes
                    // leads to numerical instabilities.
                    // By making a proxy location on the vorton, we allow the vorton to move outside
                    // the body and can concoct a solution at that location that satisfies
                    // boundary conditions at the contact point.  This means that the
                    // solution is not in general satisfied exactly at the contact point,
                    // but within a vorton-radius of it.  Note that this is no worse
                    // than the fact that even if the boundary condition was solved at the
                    // contact point, it would not be solved at points adjacent to it on the
                    // body surface.  Either way, the solution is approximate.
                    // This way, the vorton can be ejected immediately.
                    const Vec3  vSolnPos            = rVorton.mPosition + vBendDir * vortRadius ;
                #endif

                    static bool bApplyTorque = true ;
                    if( bApplyTorque )
                    {
                        // Assign the vorticity of that vorton at its new position.
                        // This assigns a vorticity such that the fluid velocity (relative
                        // to the body velocity) at the contact point, is zero.
                        // This treatment assumes that this vorton is the largest contributor
                        // to flow velocity at the contact point, and that the "ambient" flow
                        // is not influenced by this vorton.
                        rVorton.AssignByVelocity( vSolnPos , - velFlowRelBodyAtConPt ) ;

                        // Compute additional component of vorticity due to rotation of rigid body.
                        // The angular velocity of a vortex is half its vorticity.
                        // This compensates for the lack of a complete integral around the surface,
                        // and for the inaccuracy of the approximation that the largest contribution
                        // to flow velocity at the "contact point" is due to the nearest vorton.
                        // Rigid body rotation, however, behaves as though the entire rigid body
                        // is a path with the same vorticity, and as such, its contribution to fluid velocity
                        // is itself vortical.
                        // The contribution to linear velocity due to rigid body rotation must remain,
                        // however it neglects this "coherent vorticity" contribution.
                        // So we must keep both.
                        // I tuned sMultiplier by comparing results with those of the "free slip"
                        // treatment, which takes into account only body rotation, not translation,
                        // and diffuses vorticity from the body into the fluid.
                        // For this calibration, I used a test case that has a rotating (not
                        // translating) ball inside a fluid.
                        // The scaling makes some sense: The units of vorticity are the same as angular velocity,
                        // so sMultiplier must be dimensionless.
                        // Meanwhile, the larger the sphere is, the more contribution rotation has to linear velocity,
                        // hence the more this term has to compensate, so sMultiplier should be proportional to sphere radius.
                        // Likewise, the more vortices there are per volume, the smaller each is,
                        // and the less contribution each has to velocity, so sMultiplier should have inverse
                        // proportion to vorton radius.
                        // The factor of 2 could be explained by the fact that 1 of these terms
                        // simply counter-acts the rigid body rotation contribution to linear velocity,
                        // so we need "yet more" of this term for it to approach what rigid body rotation would act like as a big coherent vortex.
                        // Lastly, the proper treatment of this entails an integral over the surface of the body, so the larger its area,
                        // the more contribution we expect it to have on each vortex.
                        // If that sounds hand-wavy, that's because it is.  I have not investigated this very much.
                        // It would be worth studying the vortex flux of a purely translating and a purely rotating sphere.
                        // As an optimization, we could drop the factor of 2 and replace GetRadius with mSize since radius=size/2.
                        const float sMultiplier = 2.0f * rSphere.mRadius / vortRadius ;
                        // The formula below uses the relation angVel=vorticity/2.
                        rVorton.mAngularVelocity += ( sMultiplier * rSphere.GetAngVelocity() - vVorticityOld * 0.5f ) /* * mVortonSim.GetViscosity() */ ;

                        // Make vorticity change less abrupt.
                        // Some of the boundary condition techniques are unstable with
                        // gain>threshold, where threshold varies by technique.
                        // E.g. choice "b" requires fGain<0.5 (or so).
                        // Even when the technique is stable, lowering gain can help reduce
                        // spurious high enstrophy spikes that arise due to discretization errors.
                        // In a viscous simulation, diffusion would smooth out such spikes,
                        // but we want this sim to work with zero viscosity.
                        //
                        // It also seems likely that thicker boundaries would
                        // require smaller values of gain, since thicker boundaries
                        // imply more vortons get altered each frame, and none of the
                        // techniques take that into account until the next frame.
                        // The relationship is likely to turn out to be fGain ~ 1/(thickness^2)
                        // since the number of vortons affected is proportional to thickness^2.
                        //
                        // This time-averaging has a vaguely similar effect as a very
                        // localized diffusion, in that it keeps vorticity smoother.
                        //
                        // If fGain is too small then vortices might not shed fast enough.

                        const float fGain         = 0.1f ;
                        const float fOneMinusGain = 1.0f - fGain ;
                        rVorton.SetVorticity( fGain * rVorton.GetVorticity() + fOneMinusGain * vVorticityOld ) ;
                    }
                    // Factor of 0.5 here is due to angVel = 0.5*vorticity (that is, vorticity = 2 angVel).
                    const Vec3  vAngVelDiff     = 0.5f * ( vVorticityOld - rVorton.GetVorticity() ) ;   // (negative of) change in angular velocity applied to vorton
    #endif

                    static bool bApplyHeat = true ;
                    if( bApplyHeat )
                    {   // Conduct heat between body and fluid.
                        const float vortonTemperatureOld        = rVorton.GetTemperature( ambientFluidDensity ) ;
                        const float temperatureDifference       = rSphere.GetTemperature() - vortonTemperatureOld ;
                        const float heatConduction              = temperatureDifference * rSphere.GetThermalConductivity() * vortRadius ;
                        const float timeStep                    = 0.03f ;
                        const float heatExchange                = heatConduction * timeStep ;
                        const float vortonTemperatureNew        = vortonTemperatureOld + heatExchange * fluidSpecificHeatCapacity ;
                        rVorton.SetTemperature( ambientFluidDensity , vortonTemperatureNew ) ;
                        rSphere.SetTemperature( rSphere.GetTemperature() - heatExchange * rSphere.GetOneOverHeatCapacity() ) ;
                    }

                    #if FLOW_AFFECTS_BODY
                    // Transfer angular momentum from vorton to body.
                    // Unlike with the linear momentum exchange above, this
                    // exactly (to machine precision anyway) preserves
                    // angular momentum at each time step.
                    //
                    // For computing angular momentum, treat each vorton
                    // as a spinning solid spherical rigid body.  Its angular
                    // momentum is therefore I omega where I (moment of inertia)
                    // is 2 M R^2/5 where M is the mass and R the radius of the vorton.
                    //
                    // Compute mass as density times volume.
                    // This is playing a little fast-and-loose but it is simple, it
                    // appeals to intuition and the results look plausible.
                    //
                    // Note that here, if 2 vortons with the same vorticity (related
                    // to angular velocity) and density overlap then the net angular velocity
                    // is double that of only one of those vortons in the same region.
                    // The doubling of angular momentum comes from the doubling of circulation,
                    // not the doubling of density.
                    //
                    // The formula here could be optimized to remove some extraneous multiplies.
                    // For example, this computes volume, which has some constant factors,
                    // then multiplies that by another constant factor.
                    // Also, volume has r^3, which is r*r^2, and later this formula multiplies by r*2,
                    // so we could save another multiply and (possibly) memory fetch for r.
                    // Furthermore computing volume from size has some extra multiplies we could consolidate.
                    // All in all I doubt it's worth the effort, but if profiles show this expression
                    // is a hot-spot then consider reducing it.
                    const float vortMass                = rVorton.GetMass( ambientFluidDensity ) ;
                    const float fMomentOfInertiaVorton  = 0.4f * vortMass * vortRadius2 ;
                    rSphere.ApplyImpulsiveTorque( vAngVelDiff * fMomentOfInertiaVorton ) ;  // Apply angular impulse (impulsive torque) to body
                    #endif

                    #if 0   // This is redundant with the treatment of passive tracers.
                    // Transfer linear momentum between vorton and body.
                    // Note that this does not strictly conserve linear momentum, in the sense
                    // that this "transaction" of linear momentum has no bearing on the fluid
                    // advection.  That is because the subsequent update step summarily discards the
                    // vorton velocity assigned here.  But, the body then eventually
                    // "catches up" with the flow moving past it, at which point the body stops
                    // absorbing a lot of new momentum from the fluid.  Stationary objects never
                    // move, so absorb momentum indefinitely, but again, the fluid never loses
                    // that linear momentum (directly anyway), so no harm there.
                    {
                        #if FLOW_AFFECTS_BODY
                        const Vec3  vMinusPclVelChange  = rVorton.mVelocity - vVelBodyAtConPt ; // (negative of) total linear velocity change applied to vorton
                        rSphere.ApplyImpulse( vMinusPclVelChange * rVorton.GetVolume() ) ;      // Apply linear impulse to body
                        #endif
                        rVorton.mVelocity = vVelBodyAtConPt ;  // If same vorton is involved in another contact before advection, this will conserve linear momentum within this phase.
                    }
                    #endif
                }
            }

            QUERY_PERFORMANCE_EXIT( FluidBodySim_SolveBoundaryConditions_Vortons ) ;
        }
        else
        {   // Treat particles as non-rotating
            QUERY_PERFORMANCE_ENTER ;

            Vec3 vImpulseOnBody ; // Linear impulse applied by tracers to rigid body

        #if USE_TBB    // Disable multi-threading this process because bottleneck is memory bandwidth, not CPU speed.
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize = MAX2( 1 , numParticles / gNumberOfProcessors ) ;
            // Compute tracer-body collisions using multiple threads.
            vImpulseOnBody = CollideTracersReduce( particles , ambientFluidDensity , rSphere , 0 , numParticles , grainSize ) ;
        #else
            CollideTracersSlice( particles , ambientFluidDensity , rSphere , vImpulseOnBody , 0 , numParticles ) ;
        #endif

            rSphere.ApplyImpulse( vImpulseOnBody ) ; // Apply linear impulse from tracers to rigid body

            QUERY_PERFORMANCE_EXIT( FluidBodySim_SolveBoundaryConditions_Tracers ) ;
        }
    }


QUERY_PERFORMANCE_EXIT( FluidBodySim_SolveBoundaryConditions ) ;

#endif
}
