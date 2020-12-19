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

/** Utility class for 3x3 matrices.
*/
struct Mat33
{
    public:
        /// Construct a 3-by-3 matrix.
        Mat33() {}

        /// Construct a 3-by-3 matrix.
        Mat33(   const Vec3 & x0 , const Vec3 y0 , const Vec3 z0 )
            : x( x0 )
            , y( y0 )
            , z( z0 )
        {
        }

        // Unary operators:

        /// Return negative of this matrix.
        Mat33 operator - () const                        { return -1.0f * (*this) ; }

        //Mat33 operator ~ () const { return ... } ; // matrix inverse

        // Binary operators:

        /// Return matrix whose elements are the element-wise sum of two matrices.
        Mat33 operator + ( const Mat33 & that ) const
        {
            return Mat33( x + that.x , y + that.y , z + that.z ) ;
        }

        /// Return a matrix uniformly scaled by a scalar.
        Mat33 operator * ( float f ) const
        {
            return Mat33( x * f , y * f , z * f ) ;
        }

        /// Return a matrix uniformly scaled by a scalar.
        friend Mat33 operator * ( float f , const Mat33 & x33 ) { return x33 * f ; }

        /// Return a matrix whose elements are the element-wise difference between 2 matrices.
        Mat33 operator - ( const Mat33 & rhs ) const     { return (*this) + ( -1.0f * rhs ) ; }

        /// Return a matrix uniformly scaled by the reciprocal of a scalar.
        Mat33 operator / ( float f ) const               { return (*this) * ( 1.0f / f ) ; }

        /** Transform a vector by this matrix.
            \param v3   Vector to transform.
            \return Transformed vector.
        */
        Vec3 operator* ( const Vec3 & v3 ) const
        {
            return  Vec3(   Vec3( x.x , y.x , z.x ) * v3
                        ,   Vec3( x.y , y.y , z.y ) * v3
                        ,   Vec3( x.z , y.z , z.z ) * v3 ) ;
        }


        /** Transform a vector by the transpose of a matrix.
            \param v3   Vector to transform.
            \param x33  Transformation matrix.
            \return Transformed vector.
        */
        static friend Vec3 operator* ( const Vec3 & v3 , const Mat33 & x33 )
        {
            return Vec3( x33.x * v3 , x33.y * v3 , x33.z * v3 ) ;
        }


        /// Return whether two matrices have all the same element values.
        bool operator == ( const Mat33 & that ) const
        {
            return ( x == that.x ) && ( y == that.y ) && ( z == that.z ) ;
        }

        /// Return whether two matrices have any different values.
        bool operator != ( const Mat33 & that ) const      { return ! ( (*this) == that ) ; }

        /// Return whether two matrices have all similar element values.
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

        /// Return a matrix that is the result of matrix-matrix multiplication.
        Mat33 operator*( const Mat33 & that ) const
        {
            return Mat33(   Vec3( x * that.GetCol0() , x * that.GetCol1() , x * that.GetCol2() ) ,
                            Vec3( y * that.GetCol0() , y * that.GetCol1() , y * that.GetCol2() ) ,
                            Vec3( z * that.GetCol0() , z * that.GetCol1() , z * that.GetCol2() ) ) ;
        }

        /** Assign rotation matrix through angle about X axis.
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


        /// Make this matrix have all of its row vectors be orthonormal to each other.
        void Orthonormalize()
        {
            x.Normalize() ;
            z = x ^ y ;
            z.Normalize() ;
            y = z ^ x ;
        }


        /// Make this matrix have all of its row vectors be orthonormal to each other.
        void OrthonormalizeFast()
        {
            x.NormalizeFast() ;
            z = x ^ y ;
            z.NormalizeFast() ;
            y = z ^ x ;
        }


        /** Return skew-symmetric matrix that represents axisAngle crossed with the thing on its right.

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


        /// Return column vector 0.
              Vec3 GetCol0( void )       { return Vec3( x.x , y.x , z.x ) ; }
        /// Return column vector 0.
        const Vec3 GetCol0( void ) const { return Vec3( x.x , y.x , z.x ) ; }
        /// Return column vector 1.
              Vec3 GetCol1( void )       { return Vec3( x.y , y.y , z.y ) ; }
        /// Return column vector 1.
        const Vec3 GetCol1( void ) const { return Vec3( x.y , y.y , z.y ) ; }
        /// Return column vector 2.
              Vec3 GetCol2( void )       { return Vec3( x.z , y.z , z.z ) ; }
        /// Return column vector 2.
        const Vec3 GetCol2( void ) const { return Vec3( x.z , y.z , z.z ) ; }

        Vec3 x ;    ///< 0th row-vector.
        Vec3 y ;    ///< 1st row-vector.
        Vec3 z ;    ///< 2nd row-vector.
} ;

// Public variables ------------------------------------------------------------

/// Identity for a 3-by-3 matrix.
static const Mat33 Mat33_xIdentity( Vec3( 1.f , 0.f , 0.f ) , Vec3( 0.f , 1.f , 0.f ) , Vec3( 0.f , 0.f , 1.f ) ) ;

// Public functions ------------------------------------------------------------


#endif
