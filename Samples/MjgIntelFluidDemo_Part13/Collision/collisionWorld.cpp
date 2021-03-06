/** \file collisionWorld.cpp

    \brief Container for collidable shapes.

    \author Copyright 2012 MJG; All rights reserved.
*/

#include "collisionWorld.h"

namespace Collision
{

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

void CollisionWorld::CollideShapes( float /* timeStep */ )
{
    const ShapeIterator end = mShapes.End() ;
    for( ShapeIterator iter = mShapes.Begin() ; iter != end ; ++ iter )
    {
        ShapeBase * shape = * iter ;
        (void) shape ;
    }
}

} ;
