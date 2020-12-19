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
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <float.h>

#include <xmmintrin.h>  // SSE intrinsics

#include "Core/Performance/perf.h"
#include "Sim/Vorton/vorticityDistribution.h"

#include "Collision/sphereShape.h"
#include "Collision/convexPolytope.h"

#include "fluidBodySim.h"


#if PROFILE
unsigned gNumVortonBodyHits = 0 ;
unsigned gNumTracerBodyHits = 0 ;
#endif



#if USE_TBB
    /** Function object to collide vortons (vortex particles) with rigid bodies.
    */
    class FluidBodySim_CollideVortons_TBB
    {
            Vector< Particle > &                mVortons                    ;   ///< Dynamic array of vortons.
            const Impulsion::PhysicalObject &   mPhysicalObject             ;   ///< Reference to PhysicalObject colliding with vortons.
            float                               mAmbientFluidDensity        ;   ///< Fluid density in the absence of particles.
            float                               mFluidSpecificHeatCapacity  ;   ///< Amount fluid temperature changes put unit heat, per unit mass.

            WORD        mMasterThreadFloatingPointControlWord   ;   ///< FPCW from spawning thread.
            unsigned    mMasterThreadMmxControlStatusRegister   ;   ///< MXCSR from spawning thread.

            size_t      mBegin      ;   ///< Loop start for use with parallel_reduce.
            size_t      mEnd        ;   ///< Loop end for use with parallel_reduce.
            size_t      mGrainSize  ;   ///< Target number of elements to process per thread.

        public:
            /// Constructor to use with parallel_invoke, which is deterministic.
            FluidBodySim_CollideVortons_TBB( Vector< Particle > & rVortons , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Impulsion::PhysicalObject & physObj , size_t begin , size_t end , size_t grainSize )
                : mVortons( rVortons )
                , mAmbientFluidDensity( ambientFluidDensity )
                , mFluidSpecificHeatCapacity( fluidSpecificHeatCapacity )
                , mPhysicalObject( physObj )
                , mBegin( begin )
                , mEnd( end )
                , mGrainSize( grainSize )
                , mLinearImpulseOnBody( 0.0f , 0.0f , 0.0f )
                , mAngularImpulseOnBody( 0.0f , 0.0f , 0.0f )
                , mHeatToBody( 0.0f )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }

            /// Constructor to use with parallel_reduce, which is non-deterministic.
            FluidBodySim_CollideVortons_TBB( Vector< Particle > & rVortons , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Impulsion::PhysicalObject & physObj )
                : mVortons( rVortons )
                , mAmbientFluidDensity( ambientFluidDensity )
                , mFluidSpecificHeatCapacity( fluidSpecificHeatCapacity )
                , mPhysicalObject( physObj )
                , mBegin( 0 )       // Not used with parallel_reduce
                , mEnd( 0 )         // Not used with parallel_reduce
                , mGrainSize( 0 )   // Not used with parallel_reduce
                , mLinearImpulseOnBody( 0.0f , 0.0f , 0.0f )
                , mAngularImpulseOnBody( 0.0f , 0.0f , 0.0f )
                , mHeatToBody( 0.0f )

            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }

            /// Splitting copy constructor used by TBB parallel_reduce, which is non-deterministic.
            FluidBodySim_CollideVortons_TBB( FluidBodySim_CollideVortons_TBB & that , tbb::split )
                : mVortons( that.mVortons )
                , mPhysicalObject( that.mPhysicalObject )
                , mAmbientFluidDensity( that.mAmbientFluidDensity )
                , mFluidSpecificHeatCapacity( that.mFluidSpecificHeatCapacity )
                , mMasterThreadFloatingPointControlWord( that.mMasterThreadFloatingPointControlWord )
                , mMasterThreadMmxControlStatusRegister( that.mMasterThreadMmxControlStatusRegister )
                , mBegin( 0 )       // Not used with parallel_reduce
                , mEnd( 0 )         // Not used with parallel_reduce
                , mGrainSize( 0 )   // Not used with parallel_reduce
                , mLinearImpulseOnBody( that.mLinearImpulseOnBody )
                , mAngularImpulseOnBody( that.mAngularImpulseOnBody )
                , mHeatToBody( that.mHeatToBody )
            {
            }

            /// Invocation operator for use with parallel_invoke, which is deterministic.
            void operator() () const
            {   // Compute collisions for a subset of tracers.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                FluidBodySim::CollideVortonsReduce( mVortons , mAmbientFluidDensity , mFluidSpecificHeatCapacity , mPhysicalObject ,  mLinearImpulseOnBody , mAngularImpulseOnBody , mHeatToBody , mBegin , mEnd , mGrainSize ) ;
            }

            /// Invocation operator called by parallel_reduce, which is non-deterministic.
            void operator() ( const tbb::blocked_range<size_t> & r )
            {   // Compute collisions for a subset of tracers.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                FluidBodySim::CollideVortonsSlice( mVortons , mAmbientFluidDensity , mFluidSpecificHeatCapacity , mPhysicalObject , mLinearImpulseOnBody , mAngularImpulseOnBody , mHeatToBody , r.begin() , r.end() ) ;
            }

            /// Join the results of two threads spaned by parallel_reduce, which is non-deterministic.
            void join( const FluidBodySim_CollideVortons_TBB & other )
            {   // Reduce the results of 2 threads
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mLinearImpulseOnBody    += other.mLinearImpulseOnBody   ;
                mAngularImpulseOnBody   += other.mAngularImpulseOnBody  ;
                mHeatToBody             += other.mHeatToBody            ;
            }

            // Thread-local storage:
            mutable Vec3    mLinearImpulseOnBody    ;   ///< Linear impulse applied by vortons to rigid body.
            mutable Vec3    mAngularImpulseOnBody   ;   ///< Angular impulse applied by vortons to rigid body.
            mutable float   mHeatToBody             ;   ///< Heat applied by vortons to rigid body.
    } ;




    /** Function object to collide passive tracer particles with rigid bodies.
    */
    class FluidBodySim_CollideTracers_TBB
    {
            Vector< Particle > &                mTracers                ;   ///< Dynamic array of tracers
            const Impulsion::PhysicalObject &   mPhysicalObject         ;   ///< Reference to PhysicalObject colliding with tracers
            float                               mAmbientFluidDensity    ;   ///< Fluid density in the absence of particles

            WORD        mMasterThreadFloatingPointControlWord   ;   ///< FPCW from spawning thread.
            unsigned    mMasterThreadMmxControlStatusRegister   ;   ///< MXCSR from spawning thread.

            size_t      mBegin      ;   ///< Loop start for use with parallel_reduce.
            size_t      mEnd        ;   ///< Loop end for use with parallel_reduce.
            size_t      mGrainSize  ;   ///< Target number of elements to process per thread.

        public:
            /// Constructor to use with parallel_invoke.
            FluidBodySim_CollideTracers_TBB( Vector< Particle > & rTracers , float ambientFluidDensity , const Impulsion::PhysicalObject & physObj , size_t begin , size_t end , size_t grainSize )
                : mTracers( rTracers )
                , mAmbientFluidDensity( ambientFluidDensity )
                , mPhysicalObject( physObj )
                , mBegin( begin )
                , mEnd( end )
                , mGrainSize( grainSize )
                , mLinearImpulseOnBody( 0.0f , 0.0f , 0.0f )
                , mAngularImpulseOnBody( 0.0f , 0.0f , 0.0f )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }

            /// Constructor to use with parallel_reduce, which is non-deterministic.
            FluidBodySim_CollideTracers_TBB( Vector< Particle > & rTracers , float ambientFluidDensity , const Impulsion::PhysicalObject & physObj )
                : mTracers( rTracers )
                , mAmbientFluidDensity( ambientFluidDensity )
                , mPhysicalObject( physObj )
                , mBegin( 0 )       // Not used with parallel_reduce
                , mEnd( 0 )         // Not used with parallel_reduce
                , mGrainSize( 0 )   // Not used with parallel_reduce
                , mLinearImpulseOnBody( 0.0f , 0.0f , 0.0f )
                , mAngularImpulseOnBody( 0.0f , 0.0f , 0.0f )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }

            /// Splitting copy constructor used by TBB parallel_reduce, which is non-deterministic.
            FluidBodySim_CollideTracers_TBB( FluidBodySim_CollideTracers_TBB & that , tbb::split )
                : mTracers( that.mTracers )
                , mPhysicalObject( that.mPhysicalObject )
                , mAmbientFluidDensity( that.mAmbientFluidDensity )
                , mBegin( 0 )       // Not used with parallel_reduce
                , mEnd( 0 )         // Not used with parallel_reduce
                , mGrainSize( 0 )   // Not used with parallel_reduce
                , mMasterThreadFloatingPointControlWord( that.mMasterThreadFloatingPointControlWord )
                , mMasterThreadMmxControlStatusRegister( that.mMasterThreadMmxControlStatusRegister )
                , mLinearImpulseOnBody( that.mLinearImpulseOnBody )
                , mAngularImpulseOnBody( that.mAngularImpulseOnBody )
            {
            }

            /// Invocation operator for use with parallel_invoke.
            void operator() () const
            {   // Compute collisions for a subset of tracers.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                FluidBodySim::CollideTracersReduce( mTracers , mAmbientFluidDensity , mPhysicalObject , mLinearImpulseOnBody , mAngularImpulseOnBody , mBegin , mEnd , mGrainSize ) ;
            }

            /// Invocation operator called by parallel_reduce, which is non-deterministic.
            void operator() ( const tbb::blocked_range<size_t> & r )
            {   // Compute collisions for a subset of tracers.
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                FluidBodySim::CollideTracersSlice( mTracers , mAmbientFluidDensity , mPhysicalObject , mLinearImpulseOnBody , mAngularImpulseOnBody , r.begin() , r.end() ) ;
            }

            /// Join the results of two threads spaned by parallel_reduce, which is non-deterministic.
            void join( const FluidBodySim_CollideTracers_TBB & other )
            {   // Reduce the results of 2 threads
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mLinearImpulseOnBody  += other.mLinearImpulseOnBody  ;
                mAngularImpulseOnBody += other.mAngularImpulseOnBody ;
            }

            // Thread-local storage:
            mutable Vec3    mLinearImpulseOnBody    ;   ///< Linear impulse applied by tracers to rigid body.
            mutable Vec3    mAngularImpulseOnBody   ;   ///< Angular impulse applied by tracers to rigid body.
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
void FluidBodySim::RemoveEmbeddedParticles( Vector< Particle > & particles , const Vector< Impulsion::PhysicalObject * > & physicalObjects )
{
    if( particles.Empty() )
    {   // No particles to remove.
        return ; // Quit before accessing particles[0] below.
    }

    const size_t numPhysObjs = physicalObjects.Size() ;
    for( unsigned idxPhysObj = 0 ; idxPhysObj < numPhysObjs ; ++ idxPhysObj )
    {   // For each sphere in the simulation...
        Impulsion::PhysicalObject &  physObj  = * physicalObjects[ idxPhysObj ] ;

        Particle * pParticles = & particles[ 0 ] ;
        for( size_t iParticle = 0 ; iParticle < particles.Size() ; )
        {   // For each passive tracer particle in the simulation...
            Particle & rParticle = pParticles[ iParticle ] ;
            const Vec3  vSphereToTracer = rParticle.mPosition - physObj.GetBody()->GetPosition() ;   // vector from sphere center to tracer
            const float fSphereToTracer = vSphereToTracer.Magnitude() ;
            if( fSphereToTracer < physObj.GetCollisionShape()->GetBoundingSphereRadius() /* Note the lack of rParticle.mSize in this expression. */ )
            {   // Particle is inside bounding sphere of rigid body.
                if( physObj.GetCollisionShape()->GetShapeType() == Collision::SphereShape::sShapeType )
                {   // Rigid body is a sphere, and particle is inside it.
                    // Delete particle.
                    Particles::Kill( particles , iParticle ) ;
                }
                else if( physObj.GetCollisionShape()->GetShapeType() == Collision::ConvexPolytope::sShapeType )
                {   // Rigid body is a polytope.
                    // Test for collision.
                    const Collision::ConvexPolytope *   convexPolytope      = static_cast< const Collision::ConvexPolytope *  >( physObj.GetCollisionShape() ) ;
                    const Vec3 &                        physObjPosition     = physObj.GetBody()->GetPosition() ;
                    const Mat33 &                       physObjOrientation  = physObj.GetBody()->GetOrientation() ;
                    size_t                              idxPlane ;
                    const float                         contactDistance     = convexPolytope->ContactDistance( rParticle.mPosition , physObjPosition , physObjOrientation , idxPlane ) ;

                    if( contactDistance < rParticle.GetRadius() )
                    {   // Tracer is in contact rigid body.
                        // Delete particle.
                        Particles::Kill( particles , iParticle ) ;
                    }
                    else
                    {   // Tracer is NOT in contact with rigid body.
                        ++ iParticle ; // Did not delete particle so skip past it.
                    }
                }
            }
            else
            {   // Did not delete particle so skip past it.
                ++ iParticle ;
            }
        }
    }
}




void FluidBodySim::CollideVortonsSlice( Vector< Particle > & particles , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , float & heatToBody , size_t iPclStart , size_t iPclEnd )
{
    rLinearImpulseOnBody    = Vec3( 0.0f , 0.0f , 0.0f ) ;
    rAngularImpulseOnBody   = Vec3( 0.0f , 0.0f , 0.0f ) ;
    heatToBody              = 0.0f ;

    // Collide vortons with rigid body.
    for( size_t uVorton = iPclStart ; uVorton < iPclEnd ; ++ uVorton )
    {   // For each vorton in the simulation...
        Vorton &    rVorton             = static_cast< Vorton & >( particles[ uVorton ] ) ;
        const float vortRadius          = rVorton.GetRadius() ;
        const float vortRadius2         = vortRadius * vortRadius ;
        const Vec3  vSphereToVorton     = rVorton.mPosition - physObj.GetBody()->GetPosition() ;   // vector from body center to vorton
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
        const float     fBndThkFactor       = 1.2f ; // Thickness of boundary, in vorton radii.
        const float     fBoundaryThickness  = fBndThkFactor * vortRadius ; // Thickness of boundary, i.e. region within which body sheds vorticity into fluid.
        const float &   physObjRadius       = physObj.GetCollisionShape()->GetBoundingSphereRadius() ;

        // Compute "contact" point, near where vorton touched body.
        Vec3 vContactPtRelBody  ;
        Vec3 vContactPtWorld    ;

        if( fSphereToVorton < ( physObjRadius + fBoundaryThickness ) )
        {   // Vorton lies within bounding sphere of rigid body.
            if( physObj.GetCollisionShape()->GetShapeType() == Collision::SphereShape::sShapeType )
            {   // Rigid body is a sphere, and vorton is inside it.
                // Compute "contact" point, near where vorton touched body.
                vContactPtRelBody   = vSphereToVortonDir * physObjRadius ;
                vContactPtWorld     = vContactPtRelBody + physObj.GetBody()->GetPosition() ;
            }
            else
            {   // Rigid body is a polytope.
                // Test for collision.
                const Collision::ConvexPolytope *   convexPolytope      = static_cast< const Collision::ConvexPolytope *  >( physObj.GetCollisionShape() ) ;
                const Vec3 &                        physObjPosition     = physObj.GetBody()->GetPosition() ;
                const Mat33 &                       physObjOrientation  = physObj.GetBody()->GetOrientation() ;
                unsigned                            idxPlane ;
                const float                         contactDistance     = convexPolytope->ContactDistance( rVorton.mPosition , physObjPosition , physObjOrientation , idxPlane ) ;

                if( contactDistance < rVorton.GetRadius() )
                {   // Tracer is in contact rigid body.
                    // Compute contact point.
                    Vec3       contactNormal ;
                    vContactPtWorld     = convexPolytope->ContactPoint( rVorton.mPosition , physObjOrientation , idxPlane , contactDistance , contactNormal ) ;
                    vContactPtRelBody   = vContactPtWorld - physObj.GetBody()->GetPosition() ;
                }
                else
                {   // Vorton is NOT in contact with rigid body.
                    continue ;  // Skip to next vorton.
                }
            }

            #if PROFILE
                ++ gNumVortonBodyHits ;
            #endif

            // Compute velocity of body at contact point.
            // NOTE: Handling rigid body rotation in this way neglects the "vorticity" of the rigid body.  See notes on paper from 2009dec05.
            const Vec3 vVelDueToRotAtConPt      = physObj.GetBody()->GetAngularVelocity() ^ vContactPtRelBody  ; // linear velocity, of body at contact point, due to its own rotation
            const Vec3 vVelBodyAtConPt          = physObj.GetBody()->GetVelocity() + vVelDueToRotAtConPt   ; // Total linear velocity of body at contact point

            const Vec3 vVorticityOld            = rVorton.GetVorticity() ;  // Cache to compute change in angular momentum.

            // Each scheme below projects this vorton to the body surface,
            // but the exact location depends on the scheme.

    #if ! BOUNDARY_NO_SLIP_NO_THRU   // Assign vorticity to spin like the object.
            // Place vorton tangent to body surface along surface normal.
            const float distRescale         = ( rSphere.GetRadius() + vortRadius ) * ( 1.0f + FLT_EPSILON ) ;
            const Vec3  vDisplacementNew    = vSphereToVortonDir * distRescale ;
            rVorton.mPosition               = physObj.GetBody()->GetPosition() + vDisplacementNew ;
            const Vec3  vAngVelDiff         = rVorton.mAngularVelocity - rSphere.mAngVelocity ; // (negative of) change in angular velocity applied to vorton
            rVorton.mAngularVelocity        = physObj.GetBody()->GetAngularVelocity() ;         // Assign vorticity of vorton at its new position.

    #else // BOUNDARY_NO_SLIP_NO_THRU:

            //(void) velGrid ; // Avoid "unreferenced formal parameter" warning.

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
            Vec3    velDueToVortOld( 0.0f , 0.0f , 0.0f ) ;
            rVorton.AccumulateVelocity( velDueToVortOld , vContactPtWorld ) ;

            // Compute relative velocity between body at contact point and ambient flow,
            // subtracting the influence due to the vorton from the interpolated velocity.
            const Vec3 velFlowRelBodyAtConPt( velAmbientAtContactPt - velDueToVortOld - vVelBodyAtConPt ) ;
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
            const float fBodySurfToVortCtr  = fSphereToVorton - physObjRadius ;

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
                const float sMultiplier = 2.0f * physObjRadius / vortRadius ;
                // The formula below uses the relation angVel=vorticity/2.
                rVorton.mAngularVelocity += ( sMultiplier * physObj.GetBody()->GetAngularVelocity() - vVorticityOld * 0.5f ) /* * mVortonSim.GetViscosity() */ ;

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
                const float vortonTemperatureOld    = rVorton.GetTemperature( ambientFluidDensity ) ;
                const float temperatureDifference   = physObj.GetThermalProperties().GetTemperature() - vortonTemperatureOld ;
                const float heatConduction          = temperatureDifference * physObj.GetThermalProperties().GetThermalConductivity() * vortRadius ;
                const float interactionDuration     = 0.03f ; // duration over which a particle exchanges heat with body during collision.
                const float heatExchange            = heatConduction * interactionDuration ;
                const float vortonTemperatureNew    = vortonTemperatureOld + heatExchange * fluidSpecificHeatCapacity ;
                rVorton.SetTemperature( ambientFluidDensity , vortonTemperatureNew ) ;
                heatToBody -= heatExchange * 0.01f ;
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
            rAngularImpulseOnBody += vAngVelDiff * fMomentOfInertiaVorton ;
            #endif

            // Transfer linear momentum between vorton and body.
            // Note: This is redundant with the treatment of passive tracers,
            // so caller should omit if appropriate.
            {   // Compute change in velocity at contact point due to change in vorticity,
                // thence compute impulse on body that would conserve momentum.
                Vec3    velDueToVortNew( 0.0f , 0.0f , 0.0f ) ;
                rVorton.AccumulateVelocity( velDueToVortNew , vContactPtWorld ) ;
                const Vec3 velChange = velDueToVortNew - velDueToVortOld ;
                rLinearImpulseOnBody -= velChange * rVorton.GetMass( ambientFluidDensity ) ;
                rVorton.mVelocity = velDueToVortNew ;  // If same vorton is involved in another contact before advection, this will conserve linear momentum within this phase.
            }
        }
    }
}




#if USE_TBB
/** Collide body with vortex fluid particles using a deterministic, multi-threaded, recursive algorithm.

    \return Impulse to apply to body.
*/
/* static */ void FluidBodySim::CollideVortonsReduce( Vector< Particle > & particles , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , float & rHeatOnBody , size_t iPclStart , size_t iPclEnd , size_t grainSize )
{
    const size_t indexSpan = iPclEnd - iPclStart ;
    if( indexSpan <= grainSize )
    {   // Sub-problem fits into a single serial chunk.
        CollideVortonsSlice( particles , ambientFluidDensity , fluidSpecificHeatCapacity , physObj , rLinearImpulseOnBody , rAngularImpulseOnBody , rHeatOnBody , iPclStart , iPclEnd ) ;
    }
    else
    {   // Problem remains large enough to split into pieces.
        size_t  iPclMiddle = iPclStart + indexSpan / 2 ;
        // Create one functor for each sub-problem.
        FluidBodySim_CollideVortons_TBB cv1( particles , ambientFluidDensity , fluidSpecificHeatCapacity , physObj , iPclStart  , iPclMiddle , grainSize ) ;
        FluidBodySim_CollideVortons_TBB cv2( particles , ambientFluidDensity , fluidSpecificHeatCapacity , physObj , iPclMiddle , iPclEnd    , grainSize ) ;
        // Invoke both sub-problems, each on a separate thread.
        tbb::parallel_invoke( cv1 , cv2 ) ;
        // Combine results from each thread.
        rLinearImpulseOnBody    = cv1.mLinearImpulseOnBody  + cv2.mLinearImpulseOnBody  ;
        rAngularImpulseOnBody   = cv1.mAngularImpulseOnBody + cv2.mAngularImpulseOnBody ;
        rHeatOnBody             = cv1.mHeatToBody           + cv2.mHeatToBody           ;
    }
}
#endif




/** Collide tracer particles with rigid bodies.

    \param rSphere - reference to a spherical rigid body

    \param iPclStart - starting index of tracer particle to process.
        iPclStart must be less than the total number of tracer particles.

    \param iPclEnd - one past ending index of tracer particle to process.
        iPclStart must be less than or equal to the total number of tracer particles.

    \see SolveBoundaryConditions
*/
/* static */ void FluidBodySim::CollideTracersSlice( Vector< Particle > & particles , float ambientFluidDensity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , const size_t iPclStart , const size_t iPclEnd )
{
    Vec3                            linearImpulseOnBody ( 0.0f , 0.0f , 0.0f ) ; // Linear  impulse particles apply to rigid body.
    Vec3                            angularImpulseOnBody( 0.0f , 0.0f , 0.0f ) ; // Angular impulse particles apply to rigid body.
    const Impulsion::RigidBody *    rigidBody       = physObj.GetBody() ;
    const Vec3 &                    physObjPosition = rigidBody->GetPosition() ;
    const Vec3 &                    physObjVelocity = rigidBody->GetVelocity() ;
    const float &                   physObjRadius   = physObj.GetCollisionShape()->GetBoundingSphereRadius() ;

    static const float elasticity           = 0.01f ; // An inelastic collision implies no-through boundary conditions.
    static const float impactCoefficient    = 1.0f + elasticity ;

    Particle * pTracers = & particles[ 0 ] ;
    // Collide tracers with rigid body.
    for( size_t uTracer = iPclStart ; uTracer < iPclEnd ; ++ uTracer )
    {   // For each tracer in the simulation...
        // _mm_prefetch( (char*)( & pTracers[ uTracer + 5 ] ) , _MM_HINT_T0 ) ;
        Particle &  rTracer         = pTracers[ uTracer ] ;
        const Vec3  vSphereToTracer = rTracer.mPosition - physObjPosition ;   // vector from body center to tracer
        const float fSphereToTracer = vSphereToTracer.Magnitude() ;
        const float fCombinedRadii  = rTracer.GetRadius() + physObjRadius ;
        if( fSphereToTracer < fCombinedRadii )
        {   // Tracer lies within bounding sphere of rigid body.
            Vec3    contactPoint    ;
            Vec3    contactNormal   ;
            Vec3    vDisplacementNew ;    // New position of particle, relative to body center.
            if( physObj.GetCollisionShape()->GetShapeType() == Collision::SphereShape::sShapeType )
            {   // Rigid body is a sphere, and tracer is inside it.
            #if 0
                // Compute contact point taking into account both position and velocity.
                // This is more correct but slower.
                const Vec3  relVel                  = rTracer.mVelocity - physObjVelocity ;
                const Vec3  relVelDir               = relVel.GetDirFast() ;
                const Vec3  relPosAlongRelVel       = fSphereToTracer * relVelDir ;
                const Vec3  displacementTransverse  = vSphereToTracer - relPosAlongRelVel ;
                const float dispTransMag            = displacementTransverse.MagnitudeFast() ;
                const float dispLongMag             = fsqrtf( POW2( physObjRadius ) - POW2( dispTransMag ) ) ;
                const Vec3  displacementLongitudinal = dispLongMag * relVelDir ;
                const Vec3  displacementNew         = displacementTransverse + displacementLongitudinal ;
                vDisplacementNew        = displacementNew ;
            #else
                const float distRescale = ( physObjRadius + rTracer.GetRadius() ) * ( 1.0f + FLT_EPSILON ) / fSphereToTracer ;
                // Compute contact point using only positions, neglecting velocity.
                // This is less correct but faster.
                // This can lead to particle teleporting through objects.
                vDisplacementNew        = vSphereToTracer * distRescale ;
            #endif
                contactPoint            = vDisplacementNew ;
                contactNormal           = vSphereToTracer.GetDirFast() ;
            }
            else if( physObj.GetCollisionShape()->GetShapeType() == Collision::ConvexPolytope::sShapeType )
            {   // Rigid body is a polytope.
                // Test for collision.
                const Collision::ConvexPolytope *   convexPolytope      = static_cast< const Collision::ConvexPolytope *  >( physObj.GetCollisionShape() ) ;
                const Mat33 &                       physObjOrientation  = rigidBody->GetOrientation() ;
                unsigned                            idxPlane ;
                const float                         contactDistance     = convexPolytope->CollisionDistance( rTracer.mPosition , rTracer.mVelocity , physObjPosition , physObjOrientation , physObjVelocity , idxPlane ) ;

                if( contactDistance < rTracer.GetRadius() )
                {   // Tracer contacts rigid body.
                    // Compute contact point.
                    contactPoint     = convexPolytope->ContactPoint( rTracer.mPosition , physObjOrientation , idxPlane , contactDistance , contactNormal ) ;
                    vDisplacementNew = contactPoint + rTracer.GetRadius() * contactNormal - physObjPosition ;
                }
                else
                {   // Tracer is NOT in contact with rigid body.
                    continue ;  // Skip to next tracer.
                }
            }
            else
            {   // Unknown rigid body shape.
            }

            #if PROFILE
                ++ gNumTracerBodyHits ;
            #endif

            const Vec3  vVelDueToRotation   = rigidBody->GetAngularVelocity() ^ vDisplacementNew ; // linear velocity, at particle new position, due to body rotation.
            const Vec3  velContactPoint     = physObjVelocity + vVelDueToRotation ; // Total body linear velocity, at contact point, due to translation and rotation.

            // Transfer momentum between particle and body.
    #if 1
            // Extract normal component (i.e. going toward surface) of particle velocity.
            // Invert that.  Adding that back to the particle should zero out the "through" component.
            // Adding back some additional anti-normal would account for some elasticity.
            // TODO: Tangential component should also be reduced, based on viscosity.
            // See HandleCollision for reference, but be aware that routine expects the transformed Plane.
            const Vec3  velRelative         = rTracer.mVelocity - velContactPoint ;
            const float speedNormal         = velRelative * contactNormal ;
            Vec3  vVelNew( velContactPoint ) ;
            // This assert fails when the collision check found particle behind a plane, such that particle was not moving deeper behind that plane.
            // CollisionDistance logic attempts to prevent such detections, but neglects body rotation.
            // Taking that into account will fix the problem.
            // For spheres, this will still assert for the formulation of contactNormal that neglects motion.  See comments above.
            // Particles toward the rear should still be projected toward the front, because they entered from the front.
            //
            //
            const Vec3  impulse             = - speedNormal * contactNormal ; // Minus because speedNormal is negative.
            vVelNew = rTracer.mVelocity + impulse * impactCoefficient ;

        #if 0 && defined( _DEBUG )
            {   // These asserts fail for spheres due to the contactNormal formulation issue described above.
                const Vec3 velRelNew        = vVelNew - velContactPoint ;
                const float speedNormalNew  = velRelNew * contactNormal ;
            }
        #endif
    #else
            // Assume that after colliding with the body, particle velocity will equal that of the rigid body.
            // In fact, it will equal that of the field.  And the field is, by definition, equal to that of the body
            // within and on the body.
            // This is tantamount to a purely inelastic collision.
            const Vec3  vVelNew             = velContactPoint ;   // Total linear velocity of particle at its new position, due to sticking to body
    #endif

            #if FLOW_AFFECTS_BODY
            const Vec3  vMinusPclVelChange                      = rTracer.mVelocity - vVelNew ;
            const Vec3  linearImpulseOnBodyFromCurrentParticle  = vMinusPclVelChange * rTracer.GetMass( ambientFluidDensity ) ;
            linearImpulseOnBody  += linearImpulseOnBodyFromCurrentParticle ;
            const Vec3  positionRelativeToBody                  = contactPoint - physObjPosition ;
            angularImpulseOnBody += positionRelativeToBody ^ linearImpulseOnBodyFromCurrentParticle ;
            #endif

            // Project tracer to outside of body.
            // This places the particle on the body surface.
            rTracer.mPosition = physObjPosition + vDisplacementNew ;

            rTracer.mVelocity = vVelNew ;   // If same tracer is involved in another contact before advection, this will conserve momentum.
        }
    }
    rLinearImpulseOnBody  = linearImpulseOnBody  ;
    rAngularImpulseOnBody = angularImpulseOnBody ;
}




#if USE_TBB
/** Collide body with fluid particles using a deterministic, multi-threaded, recursive algorithm.

    \return Impulse to apply to body.
*/
/* static */ void FluidBodySim::CollideTracersReduce( Vector< Particle > & particles , float ambientFluidDensity , const Impulsion::PhysicalObject & physObj , Vec3 & rLinearImpulseOnBody , Vec3 & rAngularImpulseOnBody , const size_t iPclStart , const size_t iPclEnd , const size_t grainSize )
{
    const size_t indexSpan = iPclEnd - iPclStart ;
    if( indexSpan <= grainSize )
    {   // Sub-problem fits into a single serial chunk.
        CollideTracersSlice( particles , ambientFluidDensity , physObj , rLinearImpulseOnBody , rAngularImpulseOnBody , iPclStart , iPclEnd ) ;
    }
    else
    {   // Problem remains large enough to split into pieces.
        size_t iPclMiddle = iPclStart + indexSpan / 2 ;
        // Create one functor for each sub-problem.
        FluidBodySim_CollideTracers_TBB ct1( particles , ambientFluidDensity , physObj , iPclStart  , iPclMiddle , grainSize ) ;
        FluidBodySim_CollideTracers_TBB ct2( particles , ambientFluidDensity , physObj , iPclMiddle , iPclEnd    , grainSize ) ;
        // Invoke both sub-problems, each on a separate thread.
        tbb::parallel_invoke( ct1 , ct2 ) ;
        // Combine results from each thread: the total impulse due to the particles processed by each thread.
        rLinearImpulseOnBody  = ct1.mLinearImpulseOnBody  + ct2.mLinearImpulseOnBody  ;
        rAngularImpulseOnBody = ct1.mAngularImpulseOnBody + ct2.mAngularImpulseOnBody ;
    }
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
/* static */ void FluidBodySim::BuoyBodies( const UniformGrid< float > & densityDeviationGrid , float ambientFluidDensity , const Vec3 & gravityAcceleration , const Vector< Impulsion::PhysicalObject * > & physicalObjects )
{
QUERY_PERFORMANCE_ENTER ;

    const size_t    numPhysObjs     = physicalObjects.Size() ;
    //const size_t    numParticles    =   particles.Size() ;
    const Vec3      gravityDir      = gravityAcceleration.GetDirFast() ;

    for( unsigned idxPhysObj = 0 ; idxPhysObj < numPhysObjs ; ++ idxPhysObj )
    {   // For each body in the simulation...
        Impulsion::PhysicalObject &  physObj = * physicalObjects[ idxPhysObj ] ;
        // Compute profile of fluid density around body
        float densityDeviationAtQueryPoint          ; // fluid density at query points.
        float densityDeviationSum           =   0   ; // Average fluid density in region of body.
        float divisor                       = 1.0f  ;
        const Vec3 &  physObjPos            = physObj.GetBody()->GetPosition() ;
        const float & physObjRadius         = physObj.GetCollisionShape()->GetBoundingSphereRadius() ;
        const Vec3    sampleOffset          = physObjRadius * gravityDir ;
        // Sample fluid density at multiple places within the body region.
        Vec3 vQueryPos = physObjPos ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationSum , vQueryPos ) ;
        }
        vQueryPos = physObjPos + 0.5f * sampleOffset ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationAtQueryPoint , vQueryPos ) ;
            densityDeviationSum += densityDeviationAtQueryPoint ;
            divisor += 1.0f ;
        }
        vQueryPos = physObjPos - 0.5f * sampleOffset ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationAtQueryPoint , vQueryPos ) ;
            densityDeviationSum += densityDeviationAtQueryPoint ;
            divisor += 1.0f ;
        }
        vQueryPos = physObjPos + sampleOffset ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationAtQueryPoint , vQueryPos ) ;
            densityDeviationSum += densityDeviationAtQueryPoint ;
            divisor += 1.0f ;
        }
        vQueryPos = physObjPos - sampleOffset ;
        if( densityDeviationGrid.Encompasses( vQueryPos ) )
        {
            densityDeviationGrid.Interpolate( densityDeviationAtQueryPoint , vQueryPos ) ;
            densityDeviationSum += densityDeviationAtQueryPoint ;
            divisor += 1.0f ;
        }
        // Average fluid density samples.
        const float densityAverage  = densityDeviationSum / divisor + ambientFluidDensity ;
        // Approximate body buoyancy force.
        const float massDisplaced   = densityAverage * physObj.GetVolume() ;
        const float bodyMass        = physObj.GetBody()->GetMass() ;
        // Sum buoyancy and gravity forces.
        const Vec3  netForce        = gravityAcceleration * ( bodyMass - massDisplaced ) ;
        physObj.GetBody()->ApplyBodyForce( netForce ) ;
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
/* static */ void FluidBodySim::SolveBoundaryConditions( Vector< Particle > & particles , const UniformGrid< Vec3 > & velGrid , float ambientFluidDensity , float fluidSpecificHeatCapacity , const Vector< Impulsion::PhysicalObject * > & physicalObjects , bool bRespectAngularVelocity )
{
#if ENABLE_FLUID_BODY_SIMULATION
QUERY_PERFORMANCE_ENTER ;


    const size_t    numPhysObjs     = physicalObjects.Size() ;
    const size_t    numParticles    = particles.Size() ;

    for( unsigned idxPhysObj = 0 ; idxPhysObj < numPhysObjs ; ++ idxPhysObj )
    {   // For each body in the simulation...
        Impulsion::PhysicalObject & physObj = * physicalObjects[ idxPhysObj ] ;
        Vec3 vLinearImpulseOnBody  ; // Linear  impulse applied by particles to rigid body.
        Vec3 vAngularImpulseOnBody ; // Angular impulse applied by particles to rigid body.

        if( bRespectAngularVelocity )
        {   // Treat particles as vortons.
            QUERY_PERFORMANCE_ENTER ;

            float heatToBody ;

        #if USE_TBB
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize = MAX2( 1 , numParticles / gNumberOfProcessors ) ;
            // Compute tracer-body collisions using multiple threads.
#if 1
            CollideVortonsReduce( particles , ambientFluidDensity , fluidSpecificHeatCapacity , physObj , vLinearImpulseOnBody , vAngularImpulseOnBody , heatToBody , 0 , numParticles , grainSize ) ;
#else // For comparison.  This is not deterministic (but works otherwise).
FluidBodySim_CollideVortons_TBB cv( particles , ambientFluidDensity , fluidSpecificHeatCapacity , physObj ) ;
parallel_reduce( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , cv ) ;
vLinearImpulseOnBody    = cv.mLinearImpulseOnBody   ;
vAngularImpulseOnBody   = cv.mAngularImpulseOnBody  ;
heatToBody              = cv.mHeatToBody            ;
#endif
        #else
            CollideVortonsSlice( particles , ambientFluidDensity , fluidSpecificHeatCapacity , physObj , vLinearImpulseOnBody , vAngularImpulseOnBody , heatToBody , 0 , numParticles ) ;
        #endif

            // Skip applying linear impulse because it is redundant with doing so for tracers, and tracers yield more spatial resolution.
            //physObj.GetBody()->ApplyImpulse( vLinearImpulseOnBody ) ; // Apply linear impulse from vortons to rigid body

            physObj.GetBody()->ApplyImpulsiveTorque( vAngularImpulseOnBody ) ; // Apply angular impulse from vortons to rigid body
            physObj.GetThermalProperties().SetTemperature( physObj.GetThermalProperties().GetTemperature() + heatToBody * physObj.GetThermalProperties().GetOneOverHeatCapacity() ) ;

            QUERY_PERFORMANCE_EXIT( FluidBodySim_SolveBoundaryConditions_Vortons ) ;
        }
        else
        {   // Treat particles as non-rotating
            QUERY_PERFORMANCE_ENTER ;

        #if USE_TBB
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize = MAX2( 1 , numParticles / gNumberOfProcessors ) ;
            // Compute tracer-body collisions using multiple threads.
            CollideTracersReduce( particles , ambientFluidDensity , physObj , vLinearImpulseOnBody , vAngularImpulseOnBody , 0 , numParticles , grainSize ) ;
        #else
            CollideTracersSlice( particles , ambientFluidDensity , physObj , vLinearImpulseOnBody , vAngularImpulseOnBody , 0 , numParticles ) ;
        #endif

            physObj.GetBody()->ApplyImpulse( vLinearImpulseOnBody ) ; // Apply linear impulse from tracers to rigid body.
            physObj.GetBody()->ApplyImpulsiveTorque( vAngularImpulseOnBody ) ; // Apply angular impulse from tracers to rigid body.

            QUERY_PERFORMANCE_EXIT( FluidBodySim_SolveBoundaryConditions_Tracers ) ;
        }
    }


QUERY_PERFORMANCE_EXIT( FluidBodySim_SolveBoundaryConditions ) ;

#endif
}
