/** \file sphereShape.h

    \brief Spherical collidable shape.

    \author Copyright 2012 MJG; All rights reserved.
*/
#ifndef SHAPE_SPHERE_SHAPE_H
#define SHAPE_SPHERE_SHAPE_H

#include "sphere.h"
#include "collisionShape.h"

namespace Collision
{

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

/** A Sphere Shape that knows its own type.
*/
class SphereShape : public ShapeBase
{
    public:
        static const ShapeType sShapeType = 'stsp' ;

        /** Construct sphere.
        */
        SphereShape()
            : ShapeBase( sShapeType )
        {
        }


        /** Construct sphere given a position and a radius.
        */
        explicit SphereShape( const float radius )
            : ShapeBase( sShapeType , radius )
        {
        }
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

} ;

#endif
