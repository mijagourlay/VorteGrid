/** \file shapeUnitTests.cpp

    \brief Unit tests for Shape classes.

    \author Copyright 2012 MJG; All rights reserved.
*/

#include "Core/File/debugPrint.h"

#include "sphere.h"
#include "convexPolytope.h"


namespace Collision
{

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

#if defined( UNIT_TEST )

static const Math::Plane sTestCubeHullFaces[] =
{
        Math::Plane( Vec3(  1.0f ,  0.0f ,  0.0f ) , 1.0f )
    ,   Math::Plane( Vec3( -1.0f ,  0.0f ,  0.0f ) , 1.0f )
    ,   Math::Plane( Vec3(  0.0f ,  1.0f ,  0.0f ) , 1.0f )
    ,   Math::Plane( Vec3(  0.0f , -1.0f ,  0.0f ) , 1.0f )
    ,   Math::Plane( Vec3(  0.0f ,  0.0f ,  1.0f ) , 1.0f )
    ,   Math::Plane( Vec3(  0.0f ,  0.0f , -1.0f ) , 1.0f )
} ;

static const unsigned sTestCubeNumFaces = sizeof( sTestCubeHullFaces ) / sizeof( sTestCubeHullFaces[ 0 ] ) ;

static void Sphere_UnitTest()
{
    {
        Sphere sphere ;
        ASSERT( sphere.GetPosition() == Vec3( 0.0f , 0.0f , 0.0f ) ) ;
        ASSERT( sphere.GetRadius() == -1.0f ) ;
    }
    {
        Sphere sphere( Vec3( 1.0f , 2.0f , 3.0f ) , 4.0f ) ;
        ASSERT( sphere.GetPosition() == Vec3( 1.0f , 2.0f , 3.0f ) ) ;
        ASSERT( sphere.GetRadius() == 4.0f ) ;
        Vec3 origin( 0.0f , 0.0f , 0.0f ) ;
        const float distFromOrigin = fsqrtf( 1.0f + 4.0f + 9.0f ) - 4.0f ;
        ASSERT( Math::Resembles( sphere.Distance( origin ) , distFromOrigin ) ) ;
    }
}




static void ConvexHull_UnitTest()
{
    {
        ConvexPolytope convexHull ;
    }
    {
        using namespace Math ;
        ConvexPolytope convexHull( sTestCubeHullFaces , sTestCubeNumFaces ) ;
        unsigned idxPlane = ~0U ;

        Vec3  pos( 0.0f , 0.0f , 0.0f ) ;
        Mat33 ori( Mat33_xIdentity ) ;
        Vec3  contactNormal ;

        const Vec3 origin( 0.0f , 0.0f , 0.0f ) ;
        ASSERT( Math::Resembles( convexHull.ContactDistance( origin , pos , ori , idxPlane ) , -1.0f ) ) ;

        {
            const Vec3 xH( 0.5f , 0.0f , 0.0f ) ;
            const float contactDistanceXH = convexHull.ContactDistance( xH , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceXH , -0.5f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointXH = convexHull.ContactPoint( xH , ori , idxPlane , contactDistanceXH , contactNormal ) ;
            ASSERT( contactPointXH.Resembles( Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            const Vec3 x1( 1.0f , 0.0f , 0.0f ) ;
            const float contactDistanceX1 = convexHull.ContactDistance( x1 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceX1 , 0.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointX1 = convexHull.ContactPoint( x1 , ori , idxPlane , contactDistanceX1 , contactNormal ) ;
            ASSERT( contactPointX1.Resembles( Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            const Vec3 x2( 2.0f , 0.0f , 0.0f ) ;
            const float contactDistanceX2 = convexHull.ContactDistance( x2 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceX2 , 1.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointX2 = convexHull.ContactPoint( x2 , ori , idxPlane , contactDistanceX2 , contactNormal ) ;
            ASSERT( contactPointX2.Resembles( Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;
        }
        {
            const Vec3 xMH( -0.5f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( xMH , pos , ori , idxPlane ) , -0.5f ) ) ;
            ASSERT( 1 == idxPlane ) ;

            const Vec3 xM1( -1.0f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( xM1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 1 == idxPlane ) ;

            const Vec3 xM2( -2.0f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( xM2 , pos , ori , idxPlane ) , 1.0f ) ) ;
            ASSERT( 1 == idxPlane ) ;
        }
        {
            const Vec3 yH( 0.0f , 0.5f , 0.0f ) ;
            const float contactDistanceYH = convexHull.ContactDistance( yH , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceYH , -0.5f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointYH = convexHull.ContactPoint( yH , ori , idxPlane , contactDistanceYH , contactNormal ) ;
            ASSERT( contactPointYH.Resembles( Vec3( 0.0f , 1.0f , 0.0f ) ) ) ;

            const Vec3 y1( 0.0f , 1.0f , 0.0f ) ;
            const float contactDistanceY1 = convexHull.ContactDistance( y1 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceY1 , 0.0f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointY1 = convexHull.ContactPoint( y1 , ori , idxPlane , contactDistanceY1 , contactNormal ) ;
            ASSERT( contactPointY1.Resembles( Vec3( 0.0f , 1.0f , 0.0f ) ) ) ;

            const Vec3 y2( 0.0f , 2.0f , 0.0f ) ;
            const float contactDistanceY2 = convexHull.ContactDistance( y2 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceY2 , 1.0f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointY2 = convexHull.ContactPoint( y2 , ori , idxPlane , contactDistanceY2 , contactNormal ) ;
            ASSERT( contactPointY2.Resembles( Vec3( 0.0f , 1.0f , 0.0f ) ) ) ;
        }
        {
            const Vec3 yMH( 0.0f , -0.5f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( yMH , pos , ori , idxPlane ) , -0.5f ) ) ;
            ASSERT( 3 == idxPlane ) ;

            const Vec3 yM1( 0.0f , -1.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( yM1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 3 == idxPlane ) ;

            const Vec3 yM2( 0.0f , -2.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( yM2 , pos , ori , idxPlane ) , 1.0f ) ) ;
            ASSERT( 3 == idxPlane ) ;
        }
        {
            const Vec3 zH( 0.0f , 0.0f , 0.5f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( zH , pos , ori , idxPlane ) , -0.5f ) ) ;
            ASSERT( 4 == idxPlane ) ;

            const Vec3 z1( 0.0f , 0.0f , 1.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( z1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 4 == idxPlane ) ;

            const Vec3 z2( 0.0f , 0.0f , 2.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( z2 , pos , ori , idxPlane ) , 1.0f ) ) ;
            ASSERT( 4 == idxPlane ) ;
        }
        {
            const Vec3 zMH( 0.0f , 0.0f , -0.5f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( zMH , pos , ori , idxPlane ) , -0.5f ) ) ;
            ASSERT( 5 == idxPlane ) ;

            const Vec3 zM1( 0.0f , 0.0f , -1.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( zM1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 5 == idxPlane ) ;

            const Vec3 zM2( 0.0f , 0.0f , -2.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( zM2 , pos , ori , idxPlane ) , 1.0f ) ) ;
            ASSERT( 5 == idxPlane ) ;
        }

        // Translate polytope.
        pos = Vec3( 0.5f , 0.0f , 0.0f ) ;
        {
            const Vec3 xH( 0.5f , 0.0f , 0.0f ) ;
            const float contactDistanceXH = convexHull.ContactDistance( xH , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceXH , -1.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointXH = convexHull.ContactPoint( xH , ori , idxPlane , contactDistanceXH , contactNormal ) ;
            ASSERT( contactPointXH.Resembles( Vec3( 1.5f , 0.0f , 0.0f ) ) ) ;

            const Vec3 x1( 1.0f , 0.0f , 0.0f ) ;
            const float contactDistanceX1 = convexHull.ContactDistance( x1 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceX1 , -0.5f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointX1 = convexHull.ContactPoint( x1 , ori , idxPlane , contactDistanceX1 , contactNormal ) ;
            ASSERT( contactPointX1.Resembles( Vec3( 1.5f , 0.0f , 0.0f ) ) ) ;

            const Vec3 x2( 2.0f , 0.0f , 0.0f ) ;
            const float contactDistanceX2 = convexHull.ContactDistance( x2 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceX2 , 0.5f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointX2 = convexHull.ContactPoint( x2 , ori , idxPlane , contactDistanceX2 , contactNormal ) ;
            ASSERT( contactPointX2.Resembles( Vec3( 1.5f , 0.0f , 0.0f ) ) ) ;
        }

        // Rotate polytope about X.
        pos = Vec3( 0.0f , 0.0f , 0.0f ) ;
        ori.SetRotationX( 0.25f * PI ) ;
        {
            const Vec3 xH( 0.5f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( xH , pos , ori , idxPlane ) , -0.5f ) ) ;
            ASSERT( 0 == idxPlane ) ;

            const Vec3 x1( 1.0f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( x1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;

            const Vec3 x2( 2.0f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( x2 , pos , ori , idxPlane ) , 1.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;
        }
        {
            const float sqrt2 = fsqrtf( 2.0f ) ;
            const Vec3 yH( 0.0f , 0.5f * sqrt2 , 0.0f ) ;
            const float contactDistanceYH = convexHull.ContactDistance( yH , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceYH , -0.5f , 1.0e-3f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointYH = convexHull.ContactPoint( yH , ori , idxPlane , contactDistanceYH , contactNormal ) ;
            const float conPtYH = 0.75f * sqrt2 ;
            const float conPtZH = 0.25f * sqrt2 ;
            ASSERT( contactPointYH.Resembles( Vec3( 0.0f , conPtYH , conPtZH ) , 1.0e-3f ) ) ;

            const Vec3 y1( 0.0f , sqrt2 , 0.0f ) ;
            const float contactDistanceY1 = convexHull.ContactDistance( y1 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceY1 , 0.0f , 1.0e-3f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointY1 = convexHull.ContactPoint( y1 , ori , idxPlane , contactDistanceY1 , contactNormal ) ;
            ASSERT( contactPointY1.Resembles( Vec3( 0.0f , sqrt2 , 0.0f ) , 1.0e-3f ) ) ;

            const Vec3 y2( 0.0f , 2.0f , 0.0f ) ;
            const float contactDistanceY2 = convexHull.ContactDistance( y2 , pos , ori , idxPlane ) ;
            const float cdShouldBe = sqrt2 - 1.0f ;
            ASSERT( Math::Resembles( contactDistanceY2 , cdShouldBe , 1.0e-3f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointY2 = convexHull.ContactPoint( y2 , ori , idxPlane , contactDistanceY2 , contactNormal ) ;
            const float conPtY2 =  0.5f * ( 2.0f + sqrt2 ) ;
            const float conPtZ2 = -0.5f * ( 2.0f - sqrt2 ) ;
            ASSERT( contactPointY2.Resembles( Vec3( 0.0f , conPtY2 , conPtZ2 ) , 1.0e-3f ) ) ;
        }
    }
}




static void PolytopeContainer_UnitTest()
{
    {
        ConvexPolytope convexHull ;
    }
    {
        using namespace Math ;
        ConvexPolytope convexHull( sTestCubeHullFaces , sTestCubeNumFaces , true /* isHole */ ) ;
        unsigned idxPlane = ~0U ;

        Vec3  pos( 0.0f , 0.0f , 0.0f ) ;
        Mat33 ori( Mat33_xIdentity ) ;
        Vec3  contactNormal ;

        const Vec3 origin( 0.0f , 0.0f , 0.0f ) ;
        const float contactDistanceX0 = convexHull.ContactDistance( origin , pos , ori , idxPlane ) ;
        ASSERT( Math::Resembles( contactDistanceX0 ,  1.0f ) ) ;

        {
            const Vec3 xH( 0.5f , 0.0f , 0.0f ) ;
            const float contactDistanceXH = convexHull.ContactDistance( xH , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceXH ,  0.5f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointXH = convexHull.ContactPoint( xH , ori , idxPlane , contactDistanceXH , contactNormal ) ;
            ASSERT( contactPointXH.Resembles( Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            const Vec3 x1( 1.0f , 0.0f , 0.0f ) ;
            const float contactDistanceX1 = convexHull.ContactDistance( x1 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceX1 , 0.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointX1 = convexHull.ContactPoint( x1 , ori , idxPlane , contactDistanceX1 , contactNormal ) ;
            ASSERT( contactPointX1.Resembles( Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;

            const Vec3 x2( 2.0f , 0.0f , 0.0f ) ;
            const float contactDistanceX2 = convexHull.ContactDistance( x2 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceX2 , -1.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointX2 = convexHull.ContactPoint( x2 , ori , idxPlane , contactDistanceX2 , contactNormal ) ;
            ASSERT( contactPointX2.Resembles( Vec3( 1.0f , 0.0f , 0.0f ) ) ) ;
        }
        {
            const Vec3 xMH( -0.5f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( xMH , pos , ori , idxPlane ) ,  0.5f ) ) ;
            ASSERT( 1 == idxPlane ) ;

            const Vec3 xM1( -1.0f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( xM1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 1 == idxPlane ) ;

            const Vec3 xM2( -2.0f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( xM2 , pos , ori , idxPlane ) , -1.0f ) ) ;
            ASSERT( 1 == idxPlane ) ;
        }
        {
            const Vec3 yH( 0.0f , 0.5f , 0.0f ) ;
            const float contactDistanceYH = convexHull.ContactDistance( yH , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceYH ,  0.5f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointYH = convexHull.ContactPoint( yH , ori , idxPlane , contactDistanceYH , contactNormal ) ;
            ASSERT( contactPointYH.Resembles( Vec3( 0.0f , 1.0f , 0.0f ) ) ) ;

            const Vec3 y1( 0.0f , 1.0f , 0.0f ) ;
            const float contactDistanceY1 = convexHull.ContactDistance( y1 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceY1 , 0.0f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointY1 = convexHull.ContactPoint( y1 , ori , idxPlane , contactDistanceY1 , contactNormal ) ;
            ASSERT( contactPointY1.Resembles( Vec3( 0.0f , 1.0f , 0.0f ) ) ) ;

            const Vec3 y2( 0.0f , 2.0f , 0.0f ) ;
            const float contactDistanceY2 = convexHull.ContactDistance( y2 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceY2 , -1.0f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointY2 = convexHull.ContactPoint( y2 , ori , idxPlane , contactDistanceY2 , contactNormal ) ;
            ASSERT( contactPointY2.Resembles( Vec3( 0.0f , 1.0f , 0.0f ) ) ) ;
        }
        {
            const Vec3 yMH( 0.0f , -0.5f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( yMH , pos , ori , idxPlane ) ,  0.5f ) ) ;
            ASSERT( 3 == idxPlane ) ;

            const Vec3 yM1( 0.0f , -1.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( yM1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 3 == idxPlane ) ;

            const Vec3 yM2( 0.0f , -2.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( yM2 , pos , ori , idxPlane ) , -1.0f ) ) ;
            ASSERT( 3 == idxPlane ) ;
        }
        {
            const Vec3 zH( 0.0f , 0.0f , 0.5f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( zH , pos , ori , idxPlane ) ,  0.5f ) ) ;
            ASSERT( 4 == idxPlane ) ;

            const Vec3 z1( 0.0f , 0.0f , 1.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( z1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 4 == idxPlane ) ;

            const Vec3 z2( 0.0f , 0.0f , 2.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( z2 , pos , ori , idxPlane ) , -1.0f ) ) ;
            ASSERT( 4 == idxPlane ) ;
        }
        {
            const Vec3 zMH( 0.0f , 0.0f , -0.5f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( zMH , pos , ori , idxPlane ) ,  0.5f ) ) ;
            ASSERT( 5 == idxPlane ) ;

            const Vec3 zM1( 0.0f , 0.0f , -1.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( zM1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 5 == idxPlane ) ;

            const Vec3 zM2( 0.0f , 0.0f , -2.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( zM2 , pos , ori , idxPlane ) , -1.0f ) ) ;
            ASSERT( 5 == idxPlane ) ;
        }

        // Translate polytope.
        pos = Vec3( 0.5f , 0.0f , 0.0f ) ;
        {
            const Vec3 xH( 0.5f , 0.0f , 0.0f ) ;
            const float contactDistanceXH = convexHull.ContactDistance( xH , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceXH , 1.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointXH = convexHull.ContactPoint( xH , ori , idxPlane , contactDistanceXH , contactNormal ) ;
            ASSERT( contactPointXH.Resembles( Vec3( 1.5f , 0.0f , 0.0f ) ) ) ;

            const Vec3 x1( 1.0f , 0.0f , 0.0f ) ;
            const float contactDistanceX1 = convexHull.ContactDistance( x1 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceX1 ,  0.5f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointX1 = convexHull.ContactPoint( x1 , ori , idxPlane , contactDistanceX1 , contactNormal ) ;
            ASSERT( contactPointX1.Resembles( Vec3( 1.5f , 0.0f , 0.0f ) ) ) ;

            const Vec3 x2( 2.0f , 0.0f , 0.0f ) ;
            const float contactDistanceX2 = convexHull.ContactDistance( x2 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceX2 , -0.5f ) ) ;
            ASSERT( 0 == idxPlane ) ;
            const Vec3 contactPointX2 = convexHull.ContactPoint( x2 , ori , idxPlane , contactDistanceX2 , contactNormal ) ;
            ASSERT( contactPointX2.Resembles( Vec3( 1.5f , 0.0f , 0.0f ) ) ) ;
        }

        // Rotate polytope about X.
        pos = Vec3( 0.0f , 0.0f , 0.0f ) ;
        ori.SetRotationX( 0.25f * PI ) ;
        {
            const Vec3 xH( 0.5f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( xH , pos , ori , idxPlane ) ,  0.5f ) ) ;
            ASSERT( 0 == idxPlane ) ;

            const Vec3 x1( 1.0f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( x1 , pos , ori , idxPlane ) , 0.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;

            const Vec3 x2( 2.0f , 0.0f , 0.0f ) ;
            ASSERT( Math::Resembles( convexHull.ContactDistance( x2 , pos , ori , idxPlane ) , -1.0f ) ) ;
            ASSERT( 0 == idxPlane ) ;
        }
        {
            const float sqrt2 = fsqrtf( 2.0f ) ;
            const Vec3 yH( 0.0f , 0.5f * sqrt2 , 0.0f ) ;
            const float contactDistanceYH = convexHull.ContactDistance( yH , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceYH ,  0.5f , 1.0e-3f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointYH = convexHull.ContactPoint( yH , ori , idxPlane , contactDistanceYH , contactNormal ) ;
            const float conPtYH = 0.75f * sqrt2 ;
            const float conPtZH = 0.25f * sqrt2 ;
            ASSERT( contactPointYH.Resembles( Vec3( 0.0f , conPtYH , conPtZH ) , 1.0e-3f ) ) ;

            const Vec3 y1( 0.0f , sqrt2 , 0.0f ) ;
            const float contactDistanceY1 = convexHull.ContactDistance( y1 , pos , ori , idxPlane ) ;
            ASSERT( Math::Resembles( contactDistanceY1 , 0.0f , 1.0e-3f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointY1 = convexHull.ContactPoint( y1 , ori , idxPlane , contactDistanceY1 , contactNormal ) ;
            ASSERT( contactPointY1.Resembles( Vec3( 0.0f , sqrt2 , 0.0f ) , 1.0e-3f ) ) ;

            const Vec3 y2( 0.0f , 2.0f , 0.0f ) ;
            const float contactDistanceY2 = convexHull.ContactDistance( y2 , pos , ori , idxPlane ) ;
            const float cdShouldBe = sqrt2 - 1.0f ;
            ASSERT( Math::Resembles( contactDistanceY2 , -cdShouldBe , 1.0e-3f ) ) ;
            ASSERT( 2 == idxPlane ) ;
            const Vec3 contactPointY2 = convexHull.ContactPoint( y2 , ori , idxPlane , contactDistanceY2 , contactNormal ) ;
            const float conPtY2 =  0.5f * ( 2.0f + sqrt2 ) ;
            const float conPtZ2 = -0.5f * ( 2.0f - sqrt2 ) ;
            ASSERT( contactPointY2.Resembles( Vec3( 0.0f , conPtY2 , conPtZ2 ) , 1.0e-3f ) ) ;
        }
    }
}




void UnitTests()
{
    DebugPrintf( "Shape::UnitTest ----------------------------------------------\n" ) ;

    Sphere_UnitTest() ;
    ConvexHull_UnitTest() ;
    PolytopeContainer_UnitTest() ;

    DebugPrintf( "Shape::UnitTest: THE END ----------------------------------------------\n" ) ;
}

#endif

} ;
