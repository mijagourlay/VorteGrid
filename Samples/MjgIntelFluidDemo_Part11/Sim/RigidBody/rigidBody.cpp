/*! \file rigidBody.cpp

    \brief Rigid body base class

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "Core/Math/vec3.h"

#include "rigidBody.h"

/*! \brief Update rigid bodies
*/
/* static */ void RigidBody::UpdateSystem( const Vector< RigidBody * > & rigidBodies , float timeStep , unsigned uFrame )
{
    (void) uFrame ; // Avoid "unreferenced formal parameter" warning.
    const size_t numBodies = rigidBodies.Size() ;

    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each body in the simulation...
        RigidBody & rBody = * rigidBodies[ uBody ] ;
        // Update body physical state
        rBody.Update( timeStep ) ;
    }
}




/* static */ Vec3 RigidBody::ComputeAngularMomentum( const Vector< RigidBody * > & rigidBodies )
{
    Vec3         angMom( 0.0f , 0.0f , 0.0f ) ;
    const size_t numBodies = rigidBodies.Size() ;

    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each body in the simulation...
        RigidBody & rBody = * rigidBodies[ uBody ] ;
        // Update body physical state
        angMom += rBody.mAngMomentum ;
    }
    return angMom ;
}
