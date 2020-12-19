/*! \file mat33.h

    \brief 3x3 matrix class

    \author Copyright 2005-2009 MJG; All rights reserved.
*/
#ifndef MAT33_H
#define MAT33_H

#include <math.h>

#include "float.h"
#include "vec3.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

/*! \brief Utility class for 3x3 matrices
*/
struct Mat33
{
    public:
        Mat33() {}
        Mat33(   const Vec3 & x0 , const Vec3 y0 , const Vec3 z0 )
            : x( x0 )
            , y( y0 )
            , z( z0 )
        {
        }

        // unary operators
        Mat33 operator - () const                        { return -1.0f * (*this) ; }
        //Mat33 operator ~ () const { return ... } ; // matrix inverse

        // binary operators
        Mat33 operator + ( const Mat33 & that ) const
        {
            return Mat33( x + that.x , y + that.y , z + that.z ) ;
        }


        Mat33 operator * ( float f ) const
        {
            return Mat33( x * f , y * f , z * f ) ;
        }


        Mat33 operator - ( const Mat33 & rhs ) const     { return (*this) + ( -1.0f * rhs ) ; }

        Mat33 operator / ( float f ) const               { return (*this) * ( 1.0f / f ) ; }

        friend Mat33 operator * ( float f , const Mat33 & x33 ) { return x33 * f ; }


        /*! \brief Transform a vector by this matrix
            \param v3 - Vector to transform
        */
        Vec3 operator* ( const Vec3 & v3 ) const
        {
            return  Vec3(   Vec3( x.x , y.x , z.x ) * v3
                        ,   Vec3( x.y , y.y , z.y ) * v3
                        ,   Vec3( x.z , y.z , z.z ) * v3 ) ;
        }


        /*! \brief Transform a vector by the transpose of a matrix
            \param v3 - Vector to transform
            \param x33 - Transformation matrix
        */
        static friend Vec3 operator* ( const Vec3 & v3 , const Mat33 & x33 )
        {
            return Vec3( x33.x * v3 , x33.y * v3 , x33.z * v3 ) ;
        }


        bool operator == ( const Mat33 & that ) const
        {
            return ( x == that.x ) && ( y == that.y ) && ( z == that.z ) ;
        }

        bool operator != ( const Mat33 & that ) const      { return ! ( (*this) == that ) ; }

        bool Resembles( const Mat33 & that , float tolerance = 1.0e-4f ) const
        {
            const float * thisM = (const float *)   this ;
            const float * thatM = (const float *) & that ;
            for( size_t i = 0 ; i < 9 ; ++ i )
            {
                const float elementDiff = thisM[ i ] - thatM[ i ] ;
                const float absDiff     = fabsf( elementDiff ) ;
                if( absDiff > tolerance )
                {
                    return false ;
                }
            }
            return true ;
        }

        Mat33 operator*( const Mat33 & that ) const    // Matrix-matrix multiplication
        {
            return Mat33(   Vec3( x * that.GetCol0() , x * that.GetCol1() , x * that.GetCol2() ) ,
                            Vec3( y * that.GetCol0() , y * that.GetCol1() , y * that.GetCol2() ) ,
                            Vec3( z * that.GetCol0() , z * that.GetCol1() , z * that.GetCol2() ) ) ;
        }

        /*! \brief Assign rotation matrix through angle about X axis
            \param angle - angle (in radians) to rotate
        */
        void SetRotationX( float angle )
        {
            const float c = cos( angle ) ;
            const float s = sin( angle ) ;
            x = Vec3(  1.0f ,   0.0f ,   0.0f ) ;
            y = Vec3(  0.0f ,    c   ,    s   ) ;
            z = Vec3(  0.0f ,   -s   ,    c   ) ;
        }


        void Orthonormalize()
        {
            x.Normalize() ;
            z = x ^ y ;
            z.Normalize() ;
            y = z ^ x ;
        }


        void OrthonormalizeFast()
        {
            x.NormalizeFast() ;
            z = x ^ y ;
            z.NormalizeFast() ;
            y = z ^ x ;
        }


        /*! \brief Return skew-symmetric matrix that represents axisAngle crossed with the thing on its right.

            \param axisAngle    Vector representing a rotation about an axis,
                                where the axis is the unit vector along axisAngle
                                and the angle is the magnitude of axisAngle.

            \note See Rodrigues' rotation formula.

            \note   When omega is the time-derivative of some orientation theta,
                    then the cross-product of that time-derivative with that
                    orientation acts like a time-derivative of the orientation.
                    That is, if theta is an orientation and omega is its time
                    derivative, then it is also the case that
                        d theta / dt = omega x theta .
                    In this situation, both "theta" and "omega" represent
                    vectors.  In the situation where "theta" represents a
                    matrix, the operation of "omega x" would also need to be a
                    matrix multiplication.  That is what this routine does.
                    So, if THETA is an orientation matrix and omega is its time
                    derivative (as a vector), then the time-derivative of THETA,
                    as a matrix, is CrossProductMatrix( omega ) * THETA.
        */
        static Mat33 CrossProductMatrix( const Vec3 & axisAngle )
        {
            return Mat33(   Vec3( 0.0f          , -axisAngle.z , -axisAngle.y )
                        ,   Vec3( -axisAngle.z  , 0.0f         ,  axisAngle.x )
                        ,   Vec3(  axisAngle.y  , -axisAngle.x , 0.0f         ) ) ;
        }


        /*! \brief Return column vector
            \return Column 0
        */
              Vec3 GetCol0( void )       { return Vec3( x.x , y.x , z.x ) ; }
        const Vec3 GetCol0( void ) const { return Vec3( x.x , y.x , z.x ) ; }
              Vec3 GetCol1( void )       { return Vec3( x.y , y.y , z.y ) ; }
        const Vec3 GetCol1( void ) const { return Vec3( x.y , y.y , z.y ) ; }
              Vec3 GetCol2( void )       { return Vec3( x.z , y.z , z.z ) ; }
        const Vec3 GetCol2( void ) const { return Vec3( x.z , y.z , z.z ) ; }

        //union
        //{
        //    float m[3][3] ;
        //    struct V
        //    {
        //        Vector3 x, y, z ;
        //    } v ;
        //} ;

        Vec3 x ;
        Vec3 y ;
        Vec3 z ;
} ;

// Public variables ------------------------------------------------------------

static const Mat33 Mat33_xIdentity( Vec3( 1.f , 0.f , 0.f ) , Vec3( 0.f , 1.f , 0.f ) , Vec3( 0.f , 0.f , 1.f ) ) ;

// Public functions ------------------------------------------------------------


#endif
