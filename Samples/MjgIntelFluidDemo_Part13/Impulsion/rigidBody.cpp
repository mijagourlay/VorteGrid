/** \file rigidBody.cpp

    \brief Rigid body for physics simulation.

    \author Copyright 2011-2012 MJG; All rights reserved.
*/

#include <memory.h>

//#include "Core/Utility/macros.h"

#include "Impulsion/rigidBody.h"

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace Impulsion {

/** Construct a rigid body for a physics simulation.
*/
RigidBody::RigidBody()
    : mPosition( 0.0f , 0.0f , 0.0f )
    , mOrientation( Mat33_xIdentity )
    , mMomentum( 0.0f , 0.0f , 0.0f )
    , mAngularMomentum( 0.0f , 0.0f , 0.0f )
    , mReciprocalMass( 1.0f )
    , mInvInertiaTensor( Mat33_xIdentity * mReciprocalMass )
    , mNetForces( 0.0f , 0.0f , 0.0f )
    , mNetTorque( 0.0f , 0.0f , 0.0f )
    , mLinearVelocity( 0.0f , 0.0f , 0.0f )
    , mAngularVelocity( 0.0f , 0.0f , 0.0f )
{
}




/** Convenience constructor to create a RigidBody given its position, velocity and mass.

    \note   For computing the inertia tensor, this assumes the rigid body has a spherical
            symmetric mass distribution.
*/
RigidBody::RigidBody( const Vec3 & position , const Vec3 & velocity , const float mass )
    : mPosition( position )
    , mOrientation( Mat33_xIdentity )
    , mMomentum( 0.0f , 0.0f , 0.0f )
    , mAngularMomentum( 0.0f , 0.0f , 0.0f )
    , mReciprocalMass( 1.0f / mass )
    , mInvInertiaTensor( Mat33_xIdentity * mReciprocalMass )
    , mNetForces( 0.0f , 0.0f , 0.0f )
    , mNetTorque( 0.0f , 0.0f , 0.0f )
    , mLinearVelocity( velocity )
    , mAngularVelocity( 0.0f , 0.0f , 0.0f )
{
    SetVelocity( velocity ) ;
}




/** Destruct physically dynamic object.
*/
RigidBody::~RigidBody()
{
}



/// Test whether this RigidBody equals another.
bool RigidBody::operator==( const RigidBody & that ) const
{
    return memcmp( this , & that , sizeof( that ) ) == 0 ;
}




void RigidBody::SetMassAndInertiaTensor( float mass , const Mat33 & inertiaTensor )
{
    mReciprocalMass = 1.0f / mass ;
    mInvInertiaTensor = inertiaTensor.Inverse() ;
}




void RigidBody::Update( float timeStep )
{
    // Update linear quantities.
    const Vec3 momentumBefore = GetMomentum() ;
    mMomentum += mNetForces * timeStep ;
    const Vec3 velocityAvg = 0.5f * ( momentumBefore + GetMomentum() ) * mReciprocalMass ;
    mPosition += timeStep * velocityAvg ;

    // Update angular quantities.
    const Vec3  angMomBefore    = GetAngularMomentum() ;
    mAngularMomentum += mNetTorque * timeStep ;
    const Vec3  angVelAverage   = mInvInertiaTensor * ( 0.5f * ( GetAngularMomentum() + angMomBefore ) ) ;
    const Mat33 timeDerivative  = Mat33::CrossProductMatrix( angVelAverage ) ;
    const Mat33 angVelOperator  = timeDerivative * GetOrientation() ;
    mOrientation = mOrientation + timeStep * angVelOperator ;
    mOrientation.Orthonormalize() ;

    // Update auxiliary values.
    mLinearVelocity  = GetMomentum() * mReciprocalMass ;
    mAngularVelocity = mInvInertiaTensor * GetAngularMomentum() ;

    // Zero out forces and torques.
    mNetForces = mNetTorque = Vec3( 0.0f , 0.0f , 0.0f ) ;
}




#if defined( UNIT_TEST )

#include "Core/File/debugPrint.h"
#include "Core/Memory/newWrapper.h"

void RigidBody_UnitTest()
{
    DebugPrintf( "Impulsion::RigidBody::UnitTest ----------------------------------------------\n" ) ;

    {
        RigidBody body ;
        const RigidBody bodyOrig( body ) ;
        body.Update( 1.0f ) ;
    }
    {
        RigidBody body ;
        // Translate along +X axis.
        body.SetMomentum( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
        // Default mass is 1 so momentum and velocity are "the same".
        // Each update should therefore translate object along +X axis by same
        // as timeStep.
        static const int    numUpdates  = 1000 ;
        static const float  timeStep    = 0.01f ;
        for( int i = 0 ; i < numUpdates ; ++ i )
        {
            const float distance = timeStep * float( i ) ;
            body.Update( timeStep ) ;
        }
    }
    {
        RigidBody body ;
        // Accelerate along +X axis.
        const Vec3 force( 1.0f , 0.0f , 0.0f ) ;
        // Default mass is 1 so force and acceleration are "the same".
        static const int    numUpdates  = 1000 ;
        static const float  timeStep    = 0.01f ;
        for( int i = 0 ; i < numUpdates ; ++ i )
        {
            const float distance = 0.5f * force.x * POW2( timeStep * float( i ) ) ;
            body.ApplyBodyForce( force ) ;
            body.Update( timeStep ) ;
        }
    }
    {
        RigidBody body ;
        // Spin about +X axis.
        body.SetAngularMomentum( Vec3( 1.0f , 0.0f , 0.0f ) ) ;
        // Default moment of inertia is 1 so angular momentum and velocity are "the same".
        // Each update should therefore rotate object about +X axis by same
        // as timeStep radians.
        static const int    numUpdates  = 1000 ;
        static const float  timeStep    = 0.01f ;
        for( int i = 0 ; i < numUpdates ; ++ i )
        {
            const float angle = timeStep * float( i ) ;

            Mat33 pegasysMatrixRotateX ;
            pegasysMatrixRotateX.SetRotationX( angle ) ;
            //Mat33_DebugPrint( body.GetOrientation() ) ;
            //Mat33_DebugPrint( pegasysMatrixRotateX ) ;

            body.Update( timeStep ) ;
        }
    }
    {
        RigidBody body ;
        // Torque about +X axis.
        const Vec3 torque( 0.2f , 0.0f , 0.0f ) ;
        // Default moment of inertia is 1 so torque and angular acceleration are "the same".
        static const int    numUpdates  = 1000 ;
        static const float  timeStep    = 0.01f ;
        for( int i = 0 ; i < numUpdates ; ++ i )
        {
            const float angle = 0.5f * torque.x * POW2( timeStep * float( i ) ) ;

            Mat33 pegasysMatrixRotateX ;
            pegasysMatrixRotateX.SetRotationX( angle ) ;
            //Mat33_DebugPrint( body.GetOrientation() ) ;
            //Mat33_DebugPrint( pegasysMatrixRotateX ) ;

            body.ApplyTorque( torque ) ;
            body.Update( timeStep ) ;
        }
    }

    DebugPrintf( "Impulsion::RigidBody::UnitTest: THE END ----------------------------------------------\n" ) ;
}

#endif





/** Update rigid bodies.
*/
void RigidBody_UpdateSystem( const Vector< RigidBody * > & rigidBodies , float timeStep , unsigned /* uFrame */ )
{
    const size_t numBodies = rigidBodies.Size() ;

    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each body in the simulation...
        RigidBody & rBody = * rigidBodies[ uBody ] ;
        // Update body physical state
        rBody.Update( timeStep ) ;
    }
}




Vec3 RigidBody_ComputeAngularMomentum( const Vector< RigidBody * > & rigidBodies )
{
    Vec3         angMom( 0.0f , 0.0f , 0.0f ) ;
    const size_t numBodies = rigidBodies.Size() ;

    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each body in the simulation...
        RigidBody & rBody = * rigidBodies[ uBody ] ;
        // Update body physical state
        angMom += rBody.GetAngularMomentum() ;
    }
    return angMom ;
}

} ;
