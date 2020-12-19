/** \file convexPolytope.h

    \brief Representation of a convex polytope.

    \author Copyright 2012 MJG; All rights reserved.
*/
#ifndef SHAPE_CONVEX_POLYTOPE_H
#define SHAPE_CONVEX_POLYTOPE_H

#include "Core/Math/vec3.h"
#include "Core/Math/mat33.h"
#include "Core/Math/plane.h"

//#include "Core/Containers/vector.h"

#include "collisionShape.h"

namespace Collision
{

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

class ConvexPolytope : public ShapeBase
{
    public:
        static const ShapeType sShapeType = 'stcp' ;

        /** Construct a convex polytope.
        */
        ConvexPolytope() : ShapeBase( sShapeType ) {}

        ConvexPolytope( const Math::Plane faces[] , const unsigned numFaces ) ;
        ConvexPolytope( const ConvexPolytope & that , const Mat33 & rotation ) ;

        void SetFaces( const Math::Plane faces[] , const unsigned numFaces ) ;

        float ContactDistance( const Vec3 & queryPoint , unsigned & idxPlaneLeastPenetration ) const ;
        float ContactDistance( const Vec3 & queryPoint , const Vec3 & position , const Mat33 & orientation , unsigned & idxPlaneLeastPenetration ) const ;
        float CollisionDistance( const Vec3 & queryPoint , const Vec3 & queryPointRelativeVelocity , unsigned & idxPlaneLeastPenetration ) const ;
        float CollisionDistance( const Vec3 & queryPoint , const Vec3 & queryPointVelocity , const Vec3 & position , const Mat33 & orientation , const Vec3 & polytopeVelocity , unsigned & idxPlaneLeastPenetration ) const ;
        float ContactDistanceSphere( const Vec3 & queryPoint , const float sphereRadius , unsigned & idxPlaneLeastPenetration ) const ;
        float ContactDistanceSphere( const Vec3 & queryPoint , float sphereRadius , const Vec3 & position , const Mat33 & orientation , unsigned & idxPlaneLeastPenetration ) const ;
        Vec3  ContactPoint( const Vec3 & queryPoint , const unsigned idxPlaneLeastPenetration , const float distance ) const ;
        Vec3  ContactPoint( const Vec3 & queryPoint , const Mat33 & orientation , const unsigned idxPlaneLeastPenetration , const float distance , Vec3 & contactNormal ) const ;

        const Math::Plane & GetPlane( size_t idxPlane ) const
        {
            return mPlanes[ idxPlane ] ;
        }

    private:
        std::Vector< Math::Plane > mPlanes ;   ///< List of planar faces that constitute this polytope.
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

extern void ConvexPolytope_MakeBox( ConvexPolytope & convexPolytope , const Vec3 & dimensions ) ;

} ;

#endif
