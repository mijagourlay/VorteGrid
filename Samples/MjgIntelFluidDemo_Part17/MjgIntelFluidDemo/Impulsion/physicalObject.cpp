/** \file physicalObject.cpp

    \brief Representation of a physical object that can collide and move.

    \author Copyright 2012 MJG; All rights reserved.
*/

#include "physicalObject.h"

namespace Impulsion
{

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------

/* static */ const float PhysicalObject::sAmbientTemperature = 300.0f ;


// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

void PhysicalObject::SetCollisionShape( Collision::ShapeBase * collisionShape )
{
    ASSERT( 0 == mCollisionShape ) ; // Not allowed to change collision shape after it was set.

    mCollisionShape = collisionShape ;
}




/** Update rigid bodies associated with the given physical objects.
*/
void PhysicalObject_UpdateSystem( const VECTOR< PhysicalObject * > & physicalObjects , float timeStep , unsigned /* uFrame */ )
{
    const size_t numPhysObjs = physicalObjects.Size() ;

    for( unsigned idxPhysObj = 0 ; idxPhysObj < numPhysObjs ; ++ idxPhysObj )
    {   // For each body in the simulation...
        RigidBody & rBody = * physicalObjects[ idxPhysObj ]->GetBody() ;
        // Update body physical state
        rBody.Update( timeStep ) ;
    }
}




/** Compute total angular momentum of all rigid bodies associated with the given physical objects.
*/
Vec3 PhysicalObject_ComputeAngularMomentum( const VECTOR< PhysicalObject * > & physicalObjects )
{
    Vec3         angMom( 0.0f , 0.0f , 0.0f ) ;
    const size_t numPhysObjs = physicalObjects.Size() ;

    for( unsigned idxPhysObj = 0 ; idxPhysObj < numPhysObjs ; ++ idxPhysObj )
    {   // For each body in the simulation...
        const RigidBody & rBody = * physicalObjects[ idxPhysObj ]->GetBody() ;
        // Update body physical state
        angMom += rBody.GetAngularMomentum() ;
    }
    return angMom ;
}




#if defined( UNIT_TEST )

void UnitTests()
{
    PhysicalObject physicalObject( 0 , 0 ) ;
}

#endif

} ;
