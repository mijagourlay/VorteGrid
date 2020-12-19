/** \file mat33.h

    \brief 3x3 matrix class

    \author Copyright 2005-2012 MJG; All rights reserved.
*/
#ifndef MAT33_H
#define MAT33_H

#include <math.h>

#include "float.h"
#include "vec3.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Math
    {
        /** Plain-old-data struct that maps to Mat33, but that can be in a union.
        */
        struct Matrix33
        {
            Vector3 x ;   ///< x-column
            Vector3 y ;   ///< y-column
            Vector3 z ;   ///< z-column
        } ;

    } ;
} ;





/** Utility class for 3x3 matrices.
*/
struct Mat33
{
    public:

        /// Construct an uninitialized 3-by-3 matrix.
        Mat33() {}

        /// Construct a 3-by-3 matrix given its row vectors.
        Mat33( const Vec3 & x0 , const Vec3 y0 , const Vec3 z0 )
            : x( x0 )
            , y( y0 )
            , z( z0 )
        {
        }

        /** Construct Mat33 from Matrix33.

            \param matrix33  Matrix33 plain-old-data structure.

            Note that this is NOT "explicit" so this is a cast constructor.
            This allows Matrix33 to be treated as a Mat33 when we want that,
            but the last of a constructor in Matrix33 allows it to be present in
            unions.
        */
        Mat33( const PeGaSys::Math::Matrix33 & matrix33 )
            : x( matrix33.x )
            , y( matrix33.y )
            , z( matrix33.z )
        {
        }

        // Unary operators:

        /// Return negative of this matrix.
        Mat33 operator - () const                        { return -1.0f * (*this) ; }

        /// Return inverse of this matrix.
        Mat33 operator ~ () const { return Inverse() ; }

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
        friend Mat33 operator * ( float f , const Mat33 & x33 ) { return x33.operator*( f ) ; }

        /// Return a matrix whose elements are the element-wise difference between 2 matrices.
        Mat33 operator - ( const Mat33 & rhs ) const     { return (*this) + ( -1.0f * rhs ) ; }

        /// Return a matrix uniformly scaled by the reciprocal of a scalar.
        Mat33 operator / ( float f ) const               { return (*this) * ( 1.0f / f ) ; }


        /** Transform a vector by this matrix.
            \param v3   Vector to transform.
            \return Transformed vector.
        */
        Vec3 Transform( const Vec3 & v3 ) const
        {
            return  Vec3(   Vec3( x.x , y.x , z.x ) * v3
                        ,   Vec3( x.y , y.y , z.y ) * v3
                        ,   Vec3( x.z , y.z , z.z ) * v3 ) ;
        }


        /** Transform a vector by this matrix.
            \param v3   Vector to transform.
            \return Transformed vector.
        */
        Vec3 operator* ( const Vec3 & v3 ) const
        {
            return Transform( v3 ) ;
        }


        /** Transform a vector by the transpose of a matrix.
            \param v3   Vector to transform.
            \param x33  Transformation matrix.
            \return Transformed vector.
        */
        Vec3 TransformByTranspose( const Vec3 & v3 ) const
        {
            return Vec3( x * v3 , y * v3 , z * v3 ) ;
        }


        /** Transform a vector by the transpose of a matrix.
            \param v3   Vector to transform.
            \param x33  Transformation matrix.
            \return Transformed vector.
        */
        static friend Vec3 operator* ( const Vec3 & v3 , const Mat33 & x33 )
        {
            return x33.TransformByTranspose( v3 ) ;
        }


        /// Return whether two matrices have all the same element values.
        bool operator == ( const Mat33 & that ) const
        {
            return ( x == that.x ) && ( y == that.y ) && ( z == that.z ) ;
        }

        bool operator != ( const Mat33 & that ) const      { return ! ( (*this) == that ) ; }


        Mat33 Transpose() const
        {
            return Mat33(  Vec3( x.x , y.x , z.x )
                        ,  Vec3( x.y , y.y , z.y )
                        ,  Vec3( x.z , y.z , z.z ) ) ;
        }


        float Determinant() const
        {
            const Vec3 v0 = y ^ z ;
            const float determinant = x * v0 ;
            return determinant ;
        }


        Mat33 Inverse() const
        {
            const Vec3 v0 = y ^ z ;
            const Vec3 v1 = z ^ x ;
            const Vec3 v2 = x ^ y ;
            const float determinant = x * v0 ;
            const float oneOverDeterminant = 1.0f / determinant ;
            Mat33 inverse(  Vec3( v0.x , v1.x , v2.x ) * oneOverDeterminant
                         ,  Vec3( v0.y , v1.y , v2.y ) * oneOverDeterminant
                         ,  Vec3( v0.z , v1.z , v2.z ) * oneOverDeterminant ) ;
            return inverse ;
        }


        /// Return whether two matrices have any different values.
        bool Resembles( const Mat33 & that , float tolerance = 1.0e-4f ) const
        {
            return      x.Resembles( that.x , tolerance )
                    &&  y.Resembles( that.y , tolerance )
                    &&  z.Resembles( that.z , tolerance ) ;
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


        /** Assign rotation matrix through angle about Y axis.
            \param angle - angle (in radians) to rotate
        */
        void SetRotationY( float angle )
        {
            const float c = cos( angle ) ;
            const float s = sin( angle ) ;
            x = Vec3(     c ,   0.0f ,    -s ) ;
            y = Vec3(  0.0f ,   1.0f ,  0.0f ) ;
            z = Vec3(     s ,   0.0f   ,   c ) ;
        }


        /** Assign rotation matrix through angle about Z axis.
            \param angle - angle (in radians) to rotate
        */
        void SetRotationZ( float angle )
        {
            const float c = cos( angle ) ;
            const float s = sin( angle ) ;
            x = Vec3(    c ,      s , 0.0f ) ;
            y = Vec3(   -s ,      c , 0.0f ) ;
            z = Vec3( 0.0f ,   0.0f , 1.0f ) ;
        }


        /** Assign rotation matrix from angles about X, Y and Z axes.
            \param angles   Angles (in radians) to rotate about X, Y and Z axes
        */
        void SetRotationXYZ( const Vec3 & angles )
        {
            const float cx   = cos( angles.x ) ;
            const float sx   = sin( angles.x ) ;
            const float cy   = cos( angles.y ) ;
            const float sy   = sin( angles.y ) ;
            const float cz   = cos( angles.z ) ;
            const float sz   = sin( angles.z ) ;
            const float cxsy =  cx * sy ;
            const float sxsy =  sx * sy ;

            x.x =   cy * cz ;
            y.x = - cy * sz ;
            z.x =   sy ;

            x.y =   sxsy * cz + cx * sz ;
            y.y = - sxsy * sz + cx * cz ;
            z.y = - sx * cy ;

            x.z = - cxsy * cz + sx * sz ;
            y.z =   cxsy * sz + sx * cz ;
            z.z =   cx * cy ;
        }


        void GetAngles( Vec3 & angles ) const
        {
            if( z.x < 1.0f )
            {
                if( z.x > -1.0f )
                {
                    angles.y = asin( z.x ) ;
                    angles.x = atan2( - z.y , z.z ) ;
                    angles.z = atan2( - y.x , x.x ) ;
                }
                else
                {   // z.x = -1
                    // Not a unique solution: angles.Z - angles.X = atan2( x.y , y.y )
                    angles.y = - PI * 0.5f ;
                    angles.x = - atan2( x.y , y.y ) ;
                    angles.z = 0.0f ;
                }
            }
            else
            {   // z.x = +1
                // Not a unique solution: angles.Z + angles.X = atan2( x.y , y.y )
                angles.y =  PI * 0.5f ;
                angles.x = atan2( x.y , y.y ) ;
                angles.z = 0.0f ;
            }
        }


        /** Return whether this matrix consists of rows which are mutually orthogonal.

            \note   This definition of "orthogonal" does not match the canonical one,
                    which means "orthonormal" or "unitary".
        */
        bool HasOrthogonalVectors( float tolerance = 1.0e-4 ) const
        {
            const float xy = x * y ;
            const float xz = x * z ;
            const float yz = y * z ;
            return      Math::Resembles( xy , 0.0f , tolerance )
                    &&  Math::Resembles( xz , 0.0f , tolerance )
                    &&  Math::Resembles( yz , 0.0f , tolerance ) ;
        }


        bool IsOrthonormal( float tolerance = 1.0e-4 ) const
        {
            const Vec3 xy  = x ^ y ;
            const Vec3 xz  = x ^ z ;
            const Vec3 yz  = y ^ z ;
            const float mxy = xy.Magnitude() ;
            const float mxz = xz.Magnitude() ;
            const float myz = yz.Magnitude() ;
            return  HasOrthogonalVectors()
                    &&  Math::Resembles( mxy , 1.0f , tolerance )
                    &&  Math::Resembles( mxz , 1.0f , tolerance )
                    &&  Math::Resembles( myz , 1.0f , tolerance ) ;
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
        const Vec3 GetCol0( void ) const { return Vec3( x.x , y.x , z.x ) ; }
        /// Return column vector 1.
              Vec3 GetCol1( void )       { return Vec3( x.y , y.y , z.y ) ; }
        const Vec3 GetCol1( void ) const { return Vec3( x.y , y.y , z.y ) ; }
        /// Return column vector 2.
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

        Vec3 x ;    ///< 0th row-vector.
        Vec3 y ;    ///< 1st row-vector.
        Vec3 z ;    ///< 2nd row-vector.
} ;

// Public variables ------------------------------------------------------------

/// Identity for a 3-by-3 matrix.
static const Mat33 Mat33_xIdentity( Vec3( 1.f , 0.f , 0.f ) , Vec3( 0.f , 1.f , 0.f ) , Vec3( 0.f , 0.f , 1.f ) ) ;

// Public functions ------------------------------------------------------------


#endif
