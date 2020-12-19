/*! \file rigidBody.cpp

    \brief Rigid body base class

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
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
