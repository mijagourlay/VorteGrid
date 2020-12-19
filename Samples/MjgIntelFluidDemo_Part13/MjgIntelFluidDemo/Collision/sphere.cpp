/** \file sphere.cpp

    \brief Representation of a sphere.

    \author Copyright 2012 MJG; All rights reserved.
*/

#include "sphere.h"

namespace Collision
{

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

/** Return distance of given point to this sphere.

    Negative values mean the point lies within the sphere.
*/
//float Sphere::Distance( const Vec3 & point ) const
//{
//    const Vec3  separation         = GetPosition() - point ;
//    const float distanceFromCenter = separation.MagnitudeFast() ;
//    const float distance           = distanceFromCenter - GetRadius() ;
//    return distance ;
//}
//
//
///** Return distance of given other sphere to this sphere.
//
//    Negative values mean the spheres overlap.
//*/
//float Sphere::Distance( const Sphere & otherSphere ) const
//{
//    const Vec3  separation         = GetPosition() - otherSphere.GetPosition() ;
//    const float distanceFromCenter = separation.MagnitudeFast() ;
//    const float distance           = distanceFromCenter - GetRadius() - otherSphere.GetRadius() ;
//    return distance ;
//}

} ;
