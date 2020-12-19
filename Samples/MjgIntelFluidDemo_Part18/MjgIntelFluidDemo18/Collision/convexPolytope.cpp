/** \file convexPolytope.cpp

    \brief Representation of a convex hull.

    \author Copyright 2012 MJG; All rights reserved.
*/

// Microsoft Visual C++ vector library uses exception handling but we do not want to use it.
#ifdef _MSC_VER
#   pragma warning(disable: 4530) // C++ exception handler used, but unwind semantics are not enabled. Specify /EHsc
#endif

#include <algorithm>

#include "convexPolytope.h"

namespace Collision
{

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

/** Set polytope faces given an array of planes.
*/
void ConvexPolytope::SetFaces( const Math::Plane faces[] , const unsigned numFaces )
{
    ASSERT( faces != 0 ) ;
    ASSERT( numFaces > 0 ) ;
    mPlanes.Clear() ;
    mPlanes.Reserve( numFaces ) ;
    DEBUG_ONLY( bool dSign = faces[ 0 ].GetD() >= 0.0f ) ;
    for( unsigned idxFace = 0 ; idxFace < numFaces ; ++ idxFace )
    {   // For each face in faces...
        ASSERT( dSign == ( faces[ idxFace ].GetD() >= 0.0f ) ) ; // All planes must have same sense (interior or exterior).
        mPlanes.PushBack( faces[ idxFace ] ) ;
    }
}




/** Construct a convex polytope from an array of planar faces.
*/
ConvexPolytope::ConvexPolytope( const Math::Plane faces[] , const unsigned numFaces , bool isHole )
    : ShapeBase( sShapeType , isHole )
{
    SetFaces( faces , numFaces ) ;
}




/** Construct a reoriented copy of this ConvexPolytope, given a world-space rotation.
*/
ConvexPolytope::ConvexPolytope( const ConvexPolytope & that , const Mat33 & rotation )
    : ShapeBase( sShapeType , that.IsHole() )
{
    const size_t numFaces = that.mPlanes.Size() ;
    mPlanes.Reserve( numFaces ) ;
    for( unsigned idxFace = 0 ; idxFace < numFaces ; ++ idxFace )
    {   // For each face in the original...
        Vec3 rotatedNormal = rotation.Transform( that.mPlanes[ idxFace ].GetNormal() ) ;
        Math::Plane rotatedPlane( rotatedNormal , that.mPlanes[ idxFace ].GetD() ) ;
        mPlanes.PushBack( rotatedPlane ) ;
    }
}




/** Return distance of the given point from a feature of this convex polytope.

    \param queryPoint   Point in world space being asked about.

    \param idxPlaneLeastPenetration Index of plane in this polytope
        in which the given queryPoint has the least penetration.

    \return Distance of the given point from this convex polytope.
        Value is negative when queryPoint lies within polytope.

        Positive values mean queryPoint lies outside polytope,
        but do not necessarily indicate the actual distance
        to the polytope; actual distance could be larger.
        The reason is that, outside a polytope, there are regions
        where the closest point is to an edge or vertex instead
        of to a face.  This routine assumes that the caller
        only cares about actual penetration, not distance,
        so this routine makes no attempt to handle edge
        cases (no pun intended).

    \note   The notion of "inside" for a hole means that the opposite of what it
            means for non-holes.
*/
float ConvexPolytope::ContactDistance( const Vec3 & queryPoint , unsigned & idxPlaneLeastPenetration ) const
{
    float largestDistance = - FLT_MAX ;
    const size_t numPlanes = mPlanes.Size() ;
    for( unsigned iPlane = 0 ; iPlane < numPlanes ; ++ iPlane )
    {   // For each planar face of this convex hull...
        const float distToPlane = mPlanes[ iPlane ].Distance( queryPoint ) ;
        if( distToPlane > largestDistance )
        {   // Point distance to iPlane is largest of all planes visited so far.
            largestDistance = distToPlane ;
            // Remember this plane.
            idxPlaneLeastPenetration = iPlane ;
        }
    }
    return largestDistance * GetParity() ;
}




/** Return distance of the given point from a feature of this convex polytope.

    \param queryPoint   See other ContactDistance.

    \param position     World-space position of this polytope.

    \param orientation  World-space orientation of this polytope.

    \param idxPlaneLeastPenetration See other ContactDistance.

    \return See other ContactDistance.
*/
float ConvexPolytope::ContactDistance( const Vec3 & queryPoint , const Vec3 & position , const Mat33 & orientation , unsigned & idxPlaneLeastPenetration ) const
{
    ASSERT( orientation.IsOrthonormal() ) ;

    // The polytope should be translated and rotated according to position and orientation,
    // which is tantamount to transforming the query point by the inverse of those.
    // Transforming the point takes less time than transforming each plane. (if done here.)
// NOTE: Could/should pre-transform Planes, outside per-particle loop, instead of transforming each particle.
// As long as num particles exceeds num planes, that would run faster.
// If you pre-transform Planes, remember not to transform plane in ContactPoint.

    const Vec3 translatedPoint = queryPoint - position ;
    const Vec3 reorientedPoint = orientation.TransformByTranspose( translatedPoint ) ;

    return ContactDistance( reorientedPoint , idxPlaneLeastPenetration ) ;
}




/** Return distance of the given moving point from a feature of this convex polytope, taking into account point's velocity.
*/
float ConvexPolytope::CollisionDistance( const Vec3 & queryPoint , const Vec3 & queryPointRelativeVelocity , unsigned & idxPlaneLeastPenetration ) const
{
    float largestDistance = - FLT_MAX ;
    const size_t numPlanes = mPlanes.Size() ;
    for( unsigned iPlane = 0 ; iPlane < numPlanes ; ++ iPlane )
    {   // For each planar face of this convex hull...
        const Math::Plane & testPlane = mPlanes[ iPlane ] ;
        const float distToPlane = testPlane.Distance( queryPoint ) ;
        if( distToPlane > largestDistance )
        {   // Point distance to iPlane is largest of all planes visited so far.
            const float speedThroughPlane = queryPointRelativeVelocity * testPlane.GetNormal() * GetParity() ;
            if(     ( speedThroughPlane <= 0.0f )   // Query point is going deeper through this face.
                ||  ( distToPlane >= 0.0f )         // Query point is outside polytope.
                ||  ( IsHole() )                    // Polytope is a hole.
                )
            {   // Query point is moving deeper through this plane.
                largestDistance = distToPlane ;
                // Remember this plane.
                idxPlaneLeastPenetration = iPlane ;
            }
        }
    }
    return largestDistance * GetParity() ;
}




float ConvexPolytope::CollisionDistance( const Vec3 & queryPoint , const Vec3 & queryPointVelocity , const Vec3 & position , const Mat33 & orientation , const Vec3 & polytopeVelocity , unsigned & idxPlaneLeastPenetration ) const
{
    ASSERT( orientation.IsOrthonormal() ) ;

    // The polytope should be translated and rotated according to position and orientation,
    // which is tantamount to transforming the query point by the inverse of those.
    // Transforming the point takes less time than transforming each plane. (if done here.)
// NOTE: Could/should pre-transform Planes, outside per-particle loop, instead of transforming each particle.
// As long as num particles exceeds num planes, that would run faster.
// If you pre-transform Planes, remember not to transform plane in ContactPoint.

    const Vec3 translatedPoint    = queryPoint - position ;
    const Vec3 reorientedPoint    = orientation.TransformByTranspose( translatedPoint ) ;
    const Vec3 relativeVelocity   = queryPointVelocity - polytopeVelocity ;
    const Vec3 reorientedVelocity = orientation.TransformByTranspose( relativeVelocity ) ;

    return CollisionDistance( reorientedPoint , reorientedVelocity , idxPlaneLeastPenetration ) ;
}


        
        
/** Return distance of the given sphere from a feature of this convex polytope.

    \param queryPoint   Center of sphere being asked about.

    \param sphereRadius Radius of sphere being asked about.

    \param idxPlaneLeastPenetration Index of plane in this polytope
        in which the given sphere has the least penetration.

    \return Distance of the given sphere from this convex polytope.
        Value is negative when sphere overlaps with polytope.

        Positive values mean sphere lies entirely outside polytope,
        but do not necessarily indicate the actual distance
        to the polytope; actual distance could be larger.
        The reason is that, outside a polytope, there are regions
        where the closest point is to an edge or vertex instead
        of to a face.  This routine assumes that the caller
        only cares about actual penetration, not distance,
        so this routine makes no attempt to handle edge
        cases (no pun intended).

    \note This routine will return early if it finds
        any faces for which the sphere lies entirely outside that face.

*/
float ConvexPolytope::ContactDistanceSphere( const Vec3 & queryPoint , const float sphereRadius , unsigned & idxPlaneLeastPenetration ) const
{
    float largestDistance = - FLT_MAX ;
    const size_t numPlanes = mPlanes.Size() ;
    for( unsigned iPlane = 0 ; iPlane < numPlanes ; ++ iPlane )
    {   // For each planar face of this convex hull...
        const float distToPlane = mPlanes[ iPlane ].Distance( queryPoint ) - sphereRadius ;
        if( ! ISignBit( distToPlane ) )
        {   // Point is outside hull.
            ASSERT( distToPlane >= 0.0f ) ;
            // iPlane might not be the closest face.
            // Sphere could be closer, but it cannot be any farther.
            // Remember this plane.
            idxPlaneLeastPenetration = iPlane ;
            return distToPlane ;
        }
        if( distToPlane > largestDistance )
        {   // Sphere penetrates iPlane least, of all planes visited so far.
            largestDistance = distToPlane ;
            // Remember this plane.
            idxPlaneLeastPenetration = iPlane ;
        }
    }
    return largestDistance * GetParity() ;
}




/** Return distance of the given sphere from a feature of this convex polytope.

    \param queryPoint   Center of sphere being asked about.

    \param sphereRadius Radius of sphere being asked about.

    \param position     World-space position of this polytope.

    \param orientation  World-space orientation of this polytope.

    \param idxPlaneLeastPenetration Index of plane in this polytope
        in which the given sphere has the least penetration.

    \return Distance of the given sphere from this convex polytope.
        Value is negative when sphere overlaps with polytope.

        Positive values mean sphere lies entirely outside polytope,
        but do not necessarily indicate the actual distance
        to the polytope; actual distance could be larger.
        The reason is that, outside a polytope, there are regions
        where the closest point is to an edge or vertex instead
        of to a face.  This routine assumes that the caller
        only cares about actual penetration, not distance,
        so this routine makes no attempt to handle edge
        cases (no pun intended).

    \note This routine will return early when it finds
        any faces for which the sphere lies entirely outside that face.

*/
float ConvexPolytope::ContactDistanceSphere( const Vec3 & queryPoint , const float sphereRadius , const Vec3 & position , const Mat33 & orientation , unsigned & idxPlaneLeastPenetration ) const
{
    ASSERT( orientation.IsOrthonormal() ) ;

    // The polytope should be translated and rotated according to position and orientation,
    // which is tantamount to transforming the query point by the inverse of those.
    // Transforming the point takes less time than transforming each plane.
    const Vec3 translatedPoint = queryPoint - position ;
    const Vec3 reorientedPoint = orientation.TransformByTranspose( translatedPoint ) ;
// NOTE: Could/should pre-transform Planes, outside per-particle loop, instead of transforming each particle.
// As long as num particles exceeds num planes, that would run faster.
// If you pre-transform Planes, remember not to transform plane in ContactPoint.

    return ContactDistanceSphere( reorientedPoint , sphereRadius , idxPlaneLeastPenetration ) ;
}




/** Return contact point given a query point, plane and contact distance.

    This routine is meant to yield additional information after running one of
    the distance routines to obtain an appropriate face and distance.
*/
Vec3 ConvexPolytope::ContactPoint( const Vec3 & queryPoint , const unsigned idxPlaneLeastPenetration , const float distance ) const
{
    const Vec3 &    contactNormal   = mPlanes[ idxPlaneLeastPenetration ].GetNormal() * GetParity() ;
    Vec3            contactPoint    = queryPoint - contactNormal * distance ;
    return contactPoint ;
}




/** Return contact point given a query point, plane and contact distance.

    This routine is meant to yield additional information after running one of
    the distance routines to obtain an appropriate face and distance.
*/
Vec3 ConvexPolytope::ContactPoint( const Vec3 & queryPoint , const Mat33 & orientation , const unsigned idxPlaneLeastPenetration , const float distance , Vec3 & contactNormal ) const
{
// NOTE: Could/should pre-transform Planes, outside per-particle loop, instead of transforming each particle.
// As long as num particles exceeds num planes, that would run faster.
// If you pre-transform Planes, remember not to transform plane in ContactPoint.

    const Math::Plane & originalPlane   = mPlanes[ idxPlaneLeastPenetration ] ;
    // Note that for holes, contactNormal is reversed.
    contactNormal                       = orientation.Transform( originalPlane.GetNormal() ) * GetParity() ;
    // For holes, contactNormal is reversed, but so is distance, which should
    // have already had parity applied.  Since parity is applied to both normal
    // and distance, it cancels out so it needs to be applied again here.
    Vec3                contactPoint    = queryPoint - contactNormal * distance ;
    return contactPoint ;
}




/** Convenience utility routine to make a box polytope with given dimensions
*/
void ConvexPolytope_MakeBox( ConvexPolytope & convexPolytope , const Vec3 & dimensions )
{
    const float x = dimensions.x * 0.5f ;
    const float y = dimensions.y * 0.5f ;
    const float z = dimensions.z * 0.5f ;
    const Math::Plane faces[] =
    {
            Math::Plane( Vec3(  1.0f ,  0.0f ,  0.0f ) , x )
        ,   Math::Plane( Vec3( -1.0f ,  0.0f ,  0.0f ) , x )
        ,   Math::Plane( Vec3(  0.0f ,  1.0f ,  0.0f ) , y )
        ,   Math::Plane( Vec3(  0.0f , -1.0f ,  0.0f ) , y )
        ,   Math::Plane( Vec3(  0.0f ,  0.0f ,  1.0f ) , z )
        ,   Math::Plane( Vec3(  0.0f ,  0.0f , -1.0f ) , z )
    } ;

    const unsigned numFaces = sizeof( faces ) / sizeof( faces[ 0 ] ) ;

    convexPolytope.SetFaces( faces , numFaces ) ;

    if( convexPolytope.IsHole() )
    {   // Make inscribed sphere.
        static const float thin = 1.0f - FLT_EPSILON ;
        const float smallestDim = MIN3( x , y , z ) ;
        convexPolytope.SetBoundingSphereRadius( thin * smallestDim ) ;
    }
    else
    {   // Make circumscribing sphere.
        static const float fatten = 1.0f + FLT_EPSILON ;    // Make bounding sphere slightly larger.
        convexPolytope.SetBoundingSphereRadius( 0.5f * fatten * dimensions.Magnitude() ) ;
    }
}


} ;
