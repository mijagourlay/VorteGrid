/*! \file FluidBodySim.cpp

    \brief Simulation with mutually interacting fluid and rigid bodies

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
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
    /*! \brief Function object to collide passive tracer particles with rigid bodies
    */
    class FluidBodySim_CollideTracers_TBB
    {
            Vector< Particle > &    mTracers    ;    ///< Dynamic array of tracers
            const RbSphere  &       mRbSphere   ;    ///< Reference to RbSphere object colliding with tracers
        public:
            FluidBodySim_CollideTracers_TBB( Vector< Particle > & rTracers , RbSphere & rRbSphere )
                : mTracers( rTracers )
                , mRbSphere( rRbSphere )
            {}

            // Special "map" copy constructor used by TBB
            FluidBodySim_CollideTracers_TBB( FluidBodySim_CollideTracers_TBB & that , tbb::split )
                : mTracers( that.mTracers )
                , mRbSphere( that.mRbSphere )
                , mImpulseOnBody( that.mImpulseOnBody )
            {}

            void operator() ( const tbb::blocked_range<size_t> & r )
            {   // Compute collisions for a subset of tracers.
                FluidBodySim::CollideTracersSlice( mTracers , mRbSphere , mImpulseOnBody , r.begin() , r.end() ) ;
            }

            void join( const FluidBodySim_CollideTracers_TBB & other )
            {   // Reduce the results of 2 threads
                mImpulseOnBody += other.mImpulseOnBody ;
            }
            Vec3                mImpulseOnBody ;    ///< Impulse applied by tracers to rigid body
    } ;
#endif




/*! \brief Select boundary condition handling scheme

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
// BOUNDARY_RESPECTS_AMBIENT_FLOW only takes effect if BOUNDARY_NO_SLIP_NO_THRU is true
#define BOUNDARY_RESPECTS_AMBIENT_FLOW                  1
// BOUNDARY_AMBIENT_FLOW_OMITS_VORTON_OLD_POSITION only takes effect if BOUNDARY_RESPECTS_AMBIENT_FLOW is true.
#define BOUNDARY_AMBIENT_FLOW_OMITS_VORTON_OLD_POSITION 1




/*! \brief Whether flow affects rigid bodies immersed in the fluid.

    Normally we will leave this enabled but for testing we can disable it.
*/
#define FLOW_AFFECTS_BODY 1




/*! \brief Remove particles within rigid bodies

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
    const size_t numParticles   = particles.Size() ;
    const size_t numBodies      = rigidBodies.Size() ;
    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each sphere in the simulation...
        RbSphere &  rSphere         = ( RbSphere & ) * rigidBodies[ uBody ] ;

        Particle * pParticles = & particles[0] ;
        for( size_t iParticle = 0 ; iParticle < numParticles ; )
        {   // For each passive tracer particle in the simulation...
            Particle & rParticle = pParticles[ iParticle ] ;
            const Vec3  vSphereToTracer = rParticle.mPosition - rSphere.mPosition ;   // vector from sphere center to tracer
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




/*! \brief Collide tracer particles with rigid bodies

    \param rSphere - reference to a spherical rigid body

    \param iPclStart - starting index of tracer particle to process.
        iPclStart must be less than the total number of tracer particles.

    \param iPclEnd - one past ending index of tracer particle to process.
        iPclStart must be less than or equal to the total number of tracer particles.

    \see SolveBoundaryConditions
*/
/* static */ void FluidBodySim::CollideTracersSlice( Vector< Particle > & particles , const RbSphere & rSphere , Vec3 & rImpulseOnBody , const size_t iPclStart , const size_t iPclEnd )
{
    QUERY_PERFORMANCE_ENTER ;

    Vec3 impulseOnBody( 0.0f , 0.0f , 0.0f ) ; // Impulse particles apply to rigid body

    Particle * pTracers = & particles[ 0 ] ;
    // Collide tracers with rigid body.
    for( size_t uTracer = iPclStart ; uTracer < iPclEnd ; ++ uTracer )
    {   // For each tracer in the simulation...
        // _mm_prefetch( (char*)( & pTracers[ uTracer + 5 ] ) , _MM_HINT_T0 ) ;
        Particle &  rTracer         = pTracers[ uTracer ] ;
        const Vec3  vSphereToTracer = rTracer.mPosition - rSphere.mPosition ;   // vector from body center to tracer
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
            rTracer.mPosition = rSphere.mPosition + vDisplacementNew ;

            // Transfer linear momentum between vorton and body.
            const Vec3  vVelDueToRotation   = rSphere.mAngVelocity ^ vDisplacementNew ; // linear velocity, at vorton new position, due to body rotation
            const Vec3  vVelNew             = rSphere.mVelocity + vVelDueToRotation ;   // Total linear velocity of vorton at its new position, due to sticking to body
            #if FLOW_AFFECTS_BODY
            const Vec3  vMinusPclVelChange  = rTracer.mVelocity - vVelNew ;             // (negative of) total linear velocity change applied to vorton
            impulseOnBody += vMinusPclVelChange * rTracer.mMass ;
            #endif
            rTracer.mVelocity = vVelNew ;   // If same tracer is involved in another contact before advection, this will conserve momentum.
        }
    }
    rImpulseOnBody = impulseOnBody ;

    QUERY_PERFORMANCE_EXIT( FluidBodySim_CollideTracersSlice ) ;
}




/*! \brief Collide particles with rigid bodies

    \param particles - dynamic array of particles

    \param velGrid - uniform grid of velocity values.
            NO LONGER USED.  Instead, the velocity
            at each "contact point" comes directly from
            the particle velocity.

    \param rigidBodies - dynamic array of rigid bodies.

    \param bRespectAngularVelocity - This indicates whether a
            particle-body collision should consider the linear
            or angular velocity of the particle, treating it
            either as a tracer or as a source of vorticity.
            The former applies linear impulses to rigid bodies,
            whereas the latter apply torques.  Furthermore,
            vortons must change their vorticity to satisfy the
            no-slip and no-through boundary conditions.  See
            article 4 for details.  The dichotomy is somewhat
            artificial; in principle the simulation could consist
            entirely of vortons, without any passive tracers.
            But to keep computational cost down, the simulation
            typically has far fewer vortons than passive tracers.
            To avoid applying extraneous impulses to rigid bodies,
            the simulation treats vortons and tracers separately,
            relegating linear momentum to tracers and angular
            momentum to vortons, when applying impulses to rigid bodies.

    This uses a simplified form of "penalty" scheme
    which transfers linear and angular momentum between
    fluid particles and bodies immersed in the fluid.

    This implementation has some physical inaccuracies,
    described in comments in the code.

    A proper treatment of fluid-body interaction typically
    entails computing potential flow terms to add to the
    velocity induced by vorticity (which satisfies no-through
    boudnary conditions), and computing vorticity flux generated
    by viscous interactions between the fluid and body (which
    satisfies no-slip boundary conditions).  Or alternatively,
    one can introduce a "penalty" term into the fluid equations
    which pushes particles outside of bodies.

    In contrast, this ad-hoc scheme immediately projects particles embedded
    in bodies, to outside the bodies, and reassigns vorton vorticity
    (and hence angular momentum). This change in angular momentum in
    the flow is countered by an equal change in the body.

    Such accounting would only ever cause bodies to spin, not to
    translate, so in addition, particles in contact with the body
    also imparts a linear momentum in proportion to the difference in
    relative velocity between the particle and the body.  This second
    part, the transfer of linear momentum, is not physically accurate,
    but the scheme is simpler than the alternatives, and leads
    to visually plausible results.


    Upon tracking total kinetic energy of the fluid and bodies,
    we should see it decrease (not increase) which would be
    consistent with transfering kinetic energy to heat.
    If the equations governing the fluid motion took into account
    heat (e.g. through a change in density which would then
    appear as a baroclinic term in the vorticity equation),
    then we could balance the energy budget.

    This whole approach, including elsewhere, ignores fluid viscosity,
    which theoretically should influence the shear stress.  Introducing
    that should be straightforward but on the other hand, traditional
    treatments of bodies in "inviscid" fluids still introduce vorticity,
    and in this simulation, "viscosity" is tantamout to the rate
    vorticity diffuses "away from boundaries".  That is, low diffusion
    looks nice in the free fluid, and high diffusion looks nice near
    boundaries of solid bodies.  We want no-slip inviscid fluids.

*/
/* static */ void FluidBodySim::SolveBoundaryConditions( Vector< Particle > & particles , const UniformGrid< Vec3 > & velGrid , const Vector< RigidBody * > & rigidBodies , bool bRespectAngularVelocity )
{
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
                const Vec3  vSphereToVorton     = rVorton.mPosition - rSphere.mPosition ;   // vector from body center to vorton
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
                const float fBoundaryThickness  = fBndThkFactor * rVorton.GetRadius() ; // Thickness of boundary, i.e. region within which body sheds vorticity into fluid.

                if( fSphereToVorton < ( rSphere.mRadius + fBoundaryThickness ) )
                {   // Vorton is interacting with body.

                    #if PROFILE
                        ++ gNumVortonBodyHits ;
                    #endif

                    // Compute "contact" point, near where vorton touched body.
                    const Vec3 vContactPtRelBody        = vSphereToVortonDir * rSphere.mRadius ;
                    const Vec3 vContactPtWorld          = vContactPtRelBody + rSphere.mPosition ;

                    // Compute velocity of body at contact point.
                    // NOTE: Handling rigid body rotation in this way neglects the "vorticity" of the rigid body.  See notes on paper from 2009dec05.
                    const Vec3 vVelDueToRotAtConPt      = rSphere.mAngVelocity ^ vContactPtRelBody  ; // linear velocity, of body at contact point, due to its own rotation
                    const Vec3 vVelBodyAtConPt          = rSphere.mVelocity + vVelDueToRotAtConPt   ; // Total linear velocity of body at contact point

                    const Vec3 vVorticityOld            = rVorton.GetVorticity() ;  // Cache to compute change in angular momentum.

                    // Each scheme below projects this vorton to the body surface,
                    // but the exact location depends on the scheme.

    #if ! BOUNDARY_NO_SLIP_NO_THRU   // Assign vorticity to spin like the object.
                    // Place vorton tangent to body surface along surface normal.
                    const float distRescale         = ( rSphere.mRadius + rVorton.GetRadius() ) * ( 1.0f + FLT_EPSILON ) ;
                    const Vec3  vDisplacementNew    = vSphereToVortonDir * distRescale ;
                    rVorton.mPosition               = rSphere.mPosition + vDisplacementNew ;
                    const Vec3  vAngVelDiff         = rVorton.mAngularVelocity - rSphere.mAngVelocity ; // (negative of) change in angular velocity applied to vorton
                    rVorton.mAngularVelocity        = rSphere.mAngVelocity ;                            // Assign vorticity of vorton at its new position.

    #else // BOUNDARY_NO_SLIP_NO_THRU:
        #if ! BOUNDARY_RESPECTS_AMBIENT_FLOW
                    // This assigns a vorticity such that the fluid velocity,
                    // relative to the body velocity at the contact point, is zero.
                    // NOTE: This neglects the ambient flow due to other vortons.
                    const Vec3   velFlowRelBodyAtColPt = - vVelBodyAtConPt ;

        #else // BOUNDARY_RESPECTS_AMBIENT_FLOW
                    // Make relative fluid velocity at body nearest this vorton,
                    // due to "ambient" flow, to be zero.
                    Vec3 velAmbientAtContactPt ; // Velocity due to entire vorton field at collision point.
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
                    const Vec3 velFlowRelBodyAtColPt( velAmbientAtContactPt - velDueToVort - vVelBodyAtConPt ) ;
            #endif
        #endif
                    // Place vorton tangent to body surface along a "bend" (b),
                    //              b_hat = w_hat ^ v_hat
                    //              |b|   = vortonRadius
                    // which is not necessarily along surface normal, r_hat,
                    // and where vorticity lies perpendicular to this plane
                    // formed by the surface normal and the velocity:
                    //              w_hat = r_hat ^ v_hat
                    // Vorticity w is given by AssignByVelocity.
                    //
                    //          ,,.--..,           --:   ambient flow velocity
                    //       .'`        `'.      v  /| relative to body velocity
                    //     ,'              `\      /       at collition point
                    //    /     body         \    / ,..-..,
                    //   |                    |  /-`       `',
                    //  |               r      |/             \
                    //  |          o---------->*,   b          \
                    //  |                     || `'-,           |
                    //   |      * marks       |'     `'o        |
                    //    \      contact     /|   vorton with   |
                    //     `.    point.     /  \    counter-   /
                    //       '.,         ,-`    \  clockwise  /
                    //          `''--''``        `.,  flow _.`
                    //                              `''-''`
                    // This figure depicts the flow field after ejecting
                    // the vorton from the body interior.  Vorticity
                    // is assigned to the vorton such that the flow field
                    // satisfies no-through and no-slip boundary conditions
                    // at the contact point.
                    const Vec3  vSurfNormal         = vSphereToVorton.GetDir() ;
                    const Vec3  vVelDir             = velFlowRelBodyAtColPt.GetDir() ;
                    const Vec3  vVortDir            = vSurfNormal ^ vVelDir ;
                          Vec3  vBendDir            = vVortDir ^ vVelDir ;
                    vBendDir.Normalize() ;
                    const float fBodySurfToVortCtr  = fSphereToVorton - rSphere.mRadius ;
                    // If vorton was inside body, push it outside body, otherwise just pivot vorton about contact point.
                    const float vortRadius          = rVorton.GetRadius() ;
                    const float fBendDist           = fBodySurfToVortCtr < vortRadius ? vortRadius : fBodySurfToVortCtr ;
                    const Vec3  vBend               = fBendDist * vBendDir ;
                    rVorton.mPosition               = vContactPtWorld - vBend ;

                    {
                        // Assign the vorticity of that vorton at its new position.
                        // This assigns a vorticity such that the fluid velocity (relative
                        // to the body velocity) at the contact point, is zero.
                        // This treatment assumes that this vorton is the largest contributor
                        // to flow velocity at the contact point, and that the "ambient" flow
                        // is not influenced by this vorton.
                        rVorton.AssignByVelocity( vContactPtWorld , - velFlowRelBodyAtColPt ) ;

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
                        // Last, the factor of 2 could be explained by the fact that 1 of these terms
                        // simply counter-acts the rigid body rotation contribution to linear velocity,
                        // so we need "yet more" of this term for it to approach what rigid body rotation would act like as a big coherent vortex.
                        // Lastly, the proper treatment of this entails an integral over the surface of the body, so the larger its area,
                        // the more contribution we expect it to have on each vortex.
                        // If that sounds hand-wavy, that's because it is.  I have not investigated this very much.
                        // It would be worth studying the vortex flux of a purely translating and a purely rotating sphere.
                        // As an optimization, we could drop the factor of 2 and replace GetRadius with mSize since radius=size/2.
                        const float sMultiplier = 2.0f * rSphere.mRadius / rVorton.GetRadius() ;
                        // The formula below uses the relation angVel=vorticity/2.
                        rVorton.mAngularVelocity += ( sMultiplier * rSphere.mAngVelocity - vVorticityOld * 0.5f ) /* * mVortonSim.GetViscosity() */ ;

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
                        // since the number of vortons affected is proportional to thicnkness^2.
                        //
                        // This time-averaging has a vaguely similar effect as a very
                        // localized diffusion, in that it keeps vorticity smoother.
                        //
                        // If fGain is too small then vortices might not shed fast enough.

                        const float fGain         = 0.1f ;
                        const float fOneMinusGain = 1.0f - fGain ;
                        rVorton.SetVorticity( fGain * rVorton.GetVorticity() + fOneMinusGain * vVorticityOld ) ;
                    }

                    const Vec3  vAngVelDiff     = 0.5f * ( vVorticityOld - rVorton.GetVorticity() ) ;   // (negative of) change in angular velocity applied to vorton
    #endif

                    // Transfer angular momentum from vorton to body.
                    // Unlike with the linear momentum exchange above, this
                    // exactly preserves angular momentum at each time step.
                    #if FLOW_AFFECTS_BODY
                    const float fMomentOfInertialVorton = 0.3f * rVorton.mMass ;
                    rSphere.ApplyImpulsiveTorque( vAngVelDiff * fMomentOfInertialVorton ) ;  // Apply angular impulse (impulsive torque) to body
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
                        rSphere.ApplyImpulse( vMinusPclVelChange * rVorton.mMass ) ;            // Apply linear impulse to body
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

        #if 1 && USE_TBB    // Disable multi-threading this process because bottleneck is memory bandwidth, not CPU speed.
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize =  MAX2( 1 , numParticles / gNumberOfProcessors ) ;
            // Compute tracer-body collisions using multiple threads.
            FluidBodySim_CollideTracers_TBB ct( particles , rSphere ) ;
            parallel_reduce( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , ct ) ;
            vImpulseOnBody = ct.mImpulseOnBody ;
        #else
            CollideTracersSlice( particles , rSphere , vImpulseOnBody , 0 , numParticles ) ;
        #endif

            rSphere.ApplyImpulse( vImpulseOnBody ) ; // Apply linear impulse from tracers to rigid body

            QUERY_PERFORMANCE_EXIT( FluidBodySim_SolveBoundaryConditions_Tracers ) ;
        }
    }
}
