/** \file plane.h

    \brief Representation of a plane.

    \author Copyright 2012 MJG; All rights reserved.
*/
#ifndef MATH_PLANE_H
#define MATH_PLANE_H

#include "vec4.h"

namespace Math
{

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

class Plane : private Vec4
{
    public:
        /** Construct plane representation from floats.
            \param normal           Plane normal vector, the direction the plane faces.
                    Must be a unit vector.

            \param distFromOrigin   Distance of plane, along normal, from origin.
                    Must be non-negative.
        */
        Plane( const Vec3 & normal , const float distFromOrigin )
        {
            ASSERT( normal.IsNormalized() ) ;
            ASSERT( distFromOrigin >= 0.0f ) ;
            x = normal.x ;
            y = normal.y ;
            z = normal.z ;
            w = distFromOrigin ;
        }


        Plane( const Vec4 & v4 )
        {
            x = v4.x ;
            y = v4.y ;
            z = v4.z ;
            w = v4.w ;
        }


        /** Return direction plane faces.
        */
        const Vec3 & GetNormal() const { return reinterpret_cast< const Vec3 & >( * this ) ; }

        /** Return distance, along normal, of plane from origin.
        */
        const float & GetD() const { return w ; }


        /** Return signed distance of the given point to this plane.
        */
        float Distance( const Vec3 & vPoint ) const
        {
            const float distFromPlane = vPoint.operator*( reinterpret_cast< const Vec3 & >( * this ) ) - w ;
            return distFromPlane ;
        }
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

} ;

#endif
