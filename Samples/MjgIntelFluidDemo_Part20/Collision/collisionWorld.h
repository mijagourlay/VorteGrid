/** \file collisionWorld.h

    \brief Container for collidable shapes.

    \author Copyright 2012 MJG; All rights reserved.
*/
#ifndef SHAPE_COLLISION_WORLD_H
#define SHAPE_COLLISION_WORLD_H

#include "Core/Containers/vector.h"
#include "Collision/collisionShape.h"

namespace Collision
{

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

/** Container for collidable shapes.
*/
class CollisionWorld
{
    public:
        CollisionWorld()
        {
        }

        void    AddShape( ShapeBase * shape )
        {
            mShapes.PushBack( shape ) ;
        }

        void    CollideShapes( float timeStep ) ;

    private:
        typedef VECTOR< ShapeBase * >  ShapeContainer  ;
        typedef ShapeContainer::Iterator        ShapeIterator   ;
        ShapeContainer                          mShapes         ; ///< Collidable shapes.
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

} ;

#endif
