/** \file sphere.h

    \brief Round, symmetrical geometric object.

    \author Copyright 2012 MJG; All rights reserved.
*/
#ifndef SHAPE_SPHERE_H
#define SHAPE_SPHERE_H

#include "Core/Math/vec3.h"
#include "Core/Math/vec4.h"

namespace Collision
{

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

/** Round, symmetrical geometric object.
*/
class Sphere
{
    public:
        /** Construct sphere.
        */
        Sphere()
            : mPositionRadius( Vec3( 0.0f , 0.0f , 0.0f ) , -1.0f )
        {
        }


        /** Construct sphere given a position and a radius.
        */
        Sphere( const Vec3 & position , const float radius )
            : mPositionRadius( position , radius )
        {
            ASSERT( radius >= 0.0f ) ;
        }


        const Vec3 & GetPosition() const
        {
            return reinterpret_cast< const Vec3 & >( mPositionRadius ) ;
        }

        void SetPosition( const Vec3 & position )
        {
            mPositionRadius.x = position.x ;
            mPositionRadius.y = position.y ;
            mPositionRadius.z = position.z ;
        }

        const float & GetRadius() const
        {
            return mPositionRadius.w ;
        }

        void SetRadius( const float radius )
        {
            ASSERT( radius >= 0.0f ) ;
            mPositionRadius.w = radius ;
        }


        float Distance( const Vec3 & point ) const ;
        float Distance( const Sphere & otherSphere ) const ;

    private:
        Vec4    mPositionRadius ;   ///< Position (x,y,w) and radius (w) of this sphere.
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

} ;

#endif
