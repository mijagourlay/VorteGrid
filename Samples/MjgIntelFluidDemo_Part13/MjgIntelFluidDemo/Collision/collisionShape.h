/** \file shape.h

    \brief Abstraction of a shape, used for testing collisions.

    \author Copyright 2012 MJG; All rights reserved.
*/
#ifndef SHAPE_H
#define SHAPE_H

#include "Core/Math/vec3.h"
#include "Core/Math/vec4.h"

#include "sphere.h"

namespace Collision
{

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

typedef unsigned ShapeType ;    ///< Type identifier for shapes.

class ShapeBase
{
    public:
        static const ShapeType sUnknownShapeType = 'stuk' ;

        explicit ShapeBase( ShapeType shapeType )
            : mShapeType( shapeType )
            , mBoundingSphereRadius( -1.0f )
        {
        }

        ShapeBase( ShapeType shapeType , float boundingSphereRadius )
            : mShapeType( shapeType )
            , mBoundingSphereRadius( boundingSphereRadius )
        {
        }

        const ShapeType & GetShapeType() const { return mShapeType ; }

        void SetBoundingSphereRadius( float boundingSphereRadius ) { mBoundingSphereRadius = boundingSphereRadius ; }
        const float & GetBoundingSphereRadius() const { return mBoundingSphereRadius ; }

    private:
        ShapeType   mShapeType              ;   ///< Type identifier for this shape.  Crude form of RTTI.
        float       mBoundingSphereRadius   ;   ///< Radius of sphere that contains this shape. Used for broad phase collision detection.
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

#if defined( UNIT_TEST )
extern void UnitTests() ;
#endif

} ;

#endif
