/** \file rigidBody.h

    \brief Rigid body for physics simulation.

    \author Copyright 2011-2012 MJG; All rights reserved.
*/
#ifndef IMPULSION_RIGID_BODY_H
#define IMPULSION_RIGID_BODY_H

#include "Core/Containers/vector.h"

#include "Core/Math/vec3.h"
#include "Core/Math/mat33.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace Impulsion
{
    /** Rigid body for physics simulation.
    */
    class RigidBody
    {
        public:
            RigidBody() ;
            RigidBody( const Vec3 & position , const Vec3 & velocity , const float mass ) ;
            ~RigidBody() ;

            bool operator==( const RigidBody & that ) const ;

            void SetPosition( const Vec3 & pos ) { mPosition = pos ; }
            const Vec3 & GetPosition() const { return mPosition ; }

            void SetOrientation( const Mat33 & orientation ) { mOrientation = orientation ; }
            const Mat33 & GetOrientation() const { return mOrientation ; }

            void SetMomentum( const Vec3 & momentum ) { mMomentum = momentum ; }
            const Vec3 & GetMomentum() const { return mMomentum ; }

            void SetAngularMomentum( const Vec3 & angularMomentum )
            {
                ASSERT( ! IsNan( angularMomentum ) ) ;
                mAngularMomentum = angularMomentum ;
            }
            const Vec3 & GetAngularMomentum() const{ return mAngularMomentum ; }

            void SetMassAndInertiaTensor( float mass , const Mat33 & inertiaTensor ) ;

            void SetInverseInertiaTensor( const Mat33 & inverseInertiaTensor ) { mInvInertiaTensor = inverseInertiaTensor ; }

            float GetReciprocalMass() const { return mReciprocalMass ; }
            float GetMass() const { return 1.0f / mReciprocalMass ; }

            /** Apply a force to this rigid body along a line through its center of mass.

                \see ApplyForceAt, ApplyTorque.
            */
            void ApplyBodyForce( const Vec3 & force )
            {
                mNetForces += force ;
            }

            /** Apply a torque to this rigid body.

                \see ApplyTorqueAt, ApplyForce.
            */
            void ApplyTorque( const Vec3 & torque )
            {
                mNetTorque += torque ;
            }



            Vec3 TorqueOfForceAt( const Vec3 & force , const Vec3 & position ) const
            {
                const Vec3 positionRelativeToBody = position - GetPosition() ;
                const Vec3 torque                 = positionRelativeToBody ^ force ;
                return torque ;
            }


            /** Apply a force at a given position to this rigid body.

                \see ApplyImpulseAt, ApplyForce, ApplyTorque.
            */
            void ApplyForceAt( const Vec3 & force , const Vec3 & position )
            {
                ApplyBodyForce( force ) ;
                ApplyTorque( TorqueOfForceAt( force , position ) ) ;
            }


            /** Apply an impulse to this rigid body.

                \see ApplyForce, ApplyImpulsiveTorque.
            */
            void ApplyImpulse( const Vec3 & impulse )
            {
                mMomentum       += impulse ;                          // Apply impulse.
                mLinearVelocity  = mReciprocalMass * GetMomentum() ;  // Update linear velocity accordingly.
            }
            

            /** Apply an impulsive torque to this rigid body.

                \see ApplyTorque, ApplyImpulse.
            */
            void ApplyImpulsiveTorque( const Vec3 & impulsiveTorque )
            {
                mAngularMomentum += impulsiveTorque ;                          // Apply impulsive torque.
                ASSERT( ! IsNan( mAngularMomentum ) ) ;
                mAngularVelocity  = mInvInertiaTensor * GetAngularMomentum() ; // Update angular velocity accordingly.
            }


            /** Apply an impulse at a given position to this rigid body.

                \see ApplyForceAt, ApplyImpulsiveTorque.
            */
            void ApplyImpulseAt( const Vec3 & impulse , const Vec3 & position )
            {
                mMomentum      += impulse ;                     // Apply impulse.
                mLinearVelocity = mReciprocalMass * mMomentum ; // Update linear velocity accordingly.
                ApplyImpulsiveTorque( TorqueOfForceAt( impulse , position ) ) ;
            }



            /// Set linear velocity and momentum given a linear velocity.
            void SetVelocity( const Vec3 & velocity )
            {
                mLinearVelocity = velocity ;
                mMomentum       = velocity * GetMass() ;
            }

            const Vec3 & GetVelocity() const        { return mLinearVelocity ; }

            /// Set angular velocity and momentum given an angular velocity.
            void SetAngularVelocity( const Vec3 & angularVelocity )
            {
                mAngularVelocity    = angularVelocity ;
                mAngularMomentum    = GetInertiaTensor().Transform( angularVelocity ) ;
                ASSERT( ! IsNan( mAngularMomentum ) ) ;
            }

            const Vec3 & GetAngularVelocity() const { return mAngularVelocity ; }


            void    Update( float timeStep ) ;

        protected:
            Vec3    mPosition           ;   ///< Position in world-space of center-of-mass.
            Mat33   mOrientation        ;   ///< Orientation in world-space.  Assumes body at zero rotation has diagonal inertia tensor.
            Vec3    mMomentum           ;   ///< Linear momentum in world units.
            Vec3    mAngularMomentum    ;   ///< Angular momentum in world units.
            float   mReciprocalMass     ;   ///< One over mass.  Converts momentum to linear velocity.
            Mat33   mInvInertiaTensor   ;   ///< Inverse of inertia tensor.  Converts angular momentum to angular velocity.
            Vec3    mNetForces          ;   ///< Net forces to apply during Update.  Accumulator.
            Vec3    mNetTorque          ;   ///< Net torque to apply during Update.  Accumulator.
            Vec3    mLinearVelocity     ;   ///< Linear velocity, auxiliary variable subordinate to momentum.
            Vec3    mAngularVelocity    ;   ///< Angular velocity, auxiliary variable subordinate to angular momentum.

        private:
            Mat33 GetInertiaTensor() const
            {
                return mInvInertiaTensor.Inverse() ;
            }
    } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

extern void RigidBody_UpdateSystem( const VECTOR< RigidBody * > & rigidBodies , float timeStep , unsigned uFrame ) ;
extern Vec3 RigidBody_ComputeAngularMomentum( const VECTOR< RigidBody * > & rigidBodies ) ;

#if defined( UNIT_TEST )
extern void RigidBody_UnitTest() ;
#endif

} ;

#endif
