/*! \file rigidBody.h

    \brief Rigid body base class

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "Core/Math/mat33.h"
#include "wrapperMacros.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Rigid body base class
*/
class RigidBody
{
    public:
        /*! \brief Construct a rigid body
        */
        RigidBody()
            : mPosition( 0.0f , 0.0f , 0.0f )
            , mVelocity( 0.0f , 0.0f , 0.0f )
            , mOrientation( 0.0f , 0.0f , 0.0f )
            , mAngVelocity( 0.0f , 0.0f , 0.0f )
            , mInverseMass( 0.0f )
            , mInertiaInv( Mat33_xIdentity )
            , mForce( 0.0f , 0.0f , 0.0f )
            , mTorque( 0.0f , 0.0f , 0.0f )
            , mMomentum( 0.0f , 0.0f , 0.0f )
            , mAngMomentum( 0.0f , 0.0f , 0.0f )
        {
        }

        RigidBody( const Vec3 & vPos , const Vec3 & vVelocity , const float & fMass )
            : mPosition( vPos )
            , mVelocity( vVelocity )
            , mOrientation( 0.0f , 0.0f , 0.0f )
            , mAngVelocity( 0.0f , 0.0f , 0.0f )
            , mInverseMass( 1.0f / fMass )
            , mInertiaInv( Mat33_xIdentity * mInverseMass ) // Not really valid but better than uninitialized, and derived class should assign.
            , mForce( 0.0f , 0.0f , 0.0f )
            , mTorque( 0.0f , 0.0f , 0.0f )
            , mMomentum( vVelocity * fMass )
            , mAngMomentum( 0.0f , 0.0f , 0.0f )
        {
        }


        /*! \brief Apply a force to a rigid body at a given location
        */
        void ApplyForce( const Vec3 & vForce , const Vec3 & vPosition )
        {
            mForce += vForce ;                  // Accumulate forces
            const Vec3 vPosRelBody = vPosition - mPosition ;
            mTorque += vPosRelBody ^ vForce ;   // Accumulate torques
        }


        /*! \brief Apply an impulse to a rigid body through its center-of-mass (i.e. without applying a torque
        */
        void ApplyImpulse( const Vec3 & vImpulse )
        {
            mMomentum  += vImpulse ;                    // Apply impulse
            mVelocity   = mInverseMass * mMomentum ;    // Update linear velocity accordingly
        }


        /*! \brief Apply an impulse to a rigid body at a given location
        */
        void ApplyImpulse( const Vec3 & vImpulse , const Vec3 & vPosition )
        {
            mMomentum += vImpulse ;                         // Apply impulse
            mVelocity   = mInverseMass * mMomentum ;        // Update linear velocity accordingly
            const Vec3 vPosRelBody = vPosition - mPosition ;
            ApplyImpulsiveTorque( vPosRelBody ^ vImpulse ) ;
        }


        /*! \brief Apply an impulsive torque to a rigid body
        */
        void ApplyImpulsiveTorque( const Vec3 & vImpulsiveTorque )
        {
            mAngMomentum += vImpulsiveTorque ;          // Apply impulsive torque
            mAngVelocity = mInertiaInv * mAngMomentum ; // Update angular velocity accordingly
        }


        /*! \brief Update a rigid body from the previous to the next moment in time.

            \param timeStep - duration between previous and current time steps.

        */
        void Update( const float & timeStep )
        {
            mMomentum       += mForce * timeStep ;
            mVelocity        = mInverseMass * mMomentum ;
            mPosition       += mVelocity * timeStep ;
            mAngMomentum    += mTorque * timeStep ;
            // Correctly updating angular velocity and orientation involves these formulae:
            // Create an orientation matrix (which is unitary), called xOrient.
            // Update angular velocity using this formula:
            // mAngVelocity = xOrient * mInertiaInv * xOrient.Transpose() * mAngMomentum ;
            // Create a skew-symmetric matrix Omega from mAngVel using Rodriques' formula.
            // Update orientation using this formula:
            // xOrient += Omega * xOrient * timeStep ;
            // Re-orthonormalize xOrient:
            // xOrient.x.Normalize() ;
            // xOrient.z = xOrient.x ^ xOrient.y ;
            // xOrient.z.Normalize() ;
            // xOrient.y = xOrient.z ^ xOrient.x ;
            // Compute axis-angle form from xOrient and store in mOrientation.
            // Instead, we here assume mInertiaInv is symmetric and uniform (i.e. spherical),
            // thus inertia tensor is the same in body and world frames.
            mAngVelocity = mInertiaInv * mAngMomentum ;
            // This code also treat orientation as though it updates
            // like linear quantities, which is incorrect, but this
            // will get us through the day, since for this fluid sim we
            // only care about angular momentum of rigid bodies, not orientation.
            mOrientation += mAngVelocity * timeStep ; // This is a weird hack but it serves our purpose for this situation.

            // Zero out force and torque accumulators, for next update.
            mForce = mTorque = Vec3( 0.0f , 0.0f , 0.0f ) ;
        }

        static void UpdateSystem( Vector< RigidBody > & rigidBodies , float timeStep , unsigned uFrame ) ;
        
        Vec3    mPosition	    ;   ///< Position (in world units) of center of vortex particle
        Vec3    mVelocity       ;   ///< Linear velocity of sphere
        Vec3    mOrientation    ;   ///< Orientation of sphere in axis-angle form
        Vec3    mAngVelocity    ;   ///< Angular velocity of sphere

    protected:
        float   mInverseMass    ;   ///< Reciprocal of the mass of this body
        Mat33   mInertiaInv     ;   ///< Inverse of inertial tensor

    private:
        Vec3    mForce          ;   ///< Total force applied to this body for a single frame.
        Vec3    mTorque         ;   ///< Total torque applied to this body for a single frame.
        Vec3    mMomentum       ;   ///< Linear momentum of sphere
        Vec3    mAngMomentum    ;   ///< Angular momentum of sphere
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
