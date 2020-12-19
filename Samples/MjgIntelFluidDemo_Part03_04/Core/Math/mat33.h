/*! \file mat33.h

    \brief 3x3 matrix class

    \author Copyright 2005-2009 UCF/FIEA/MJG; All rights reserved.
*/
#ifndef MAT33_H
#define MAT33_H

#include <math.h>

#include "float.h"
#include "vec3.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

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


        /*! \brief Transform a vector by the transpose of this matrix
            \param v3 - Vector to transform
        */
        Vec3 operator* ( const Vec3 & v3 ) const
        {
            return Vec3( Vec3( x.x , y.x , z.x ) * v3 , Vec3( x.y , y.y , z.y ) * v3 , Vec3( z.z , y.z , z.z ) * v3 ) ;
        }


        /*! \brief Transform a vector by a matrix
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

        Mat33 operator*( const Mat33 & that ) const    // Matrix-matrix multiplication
        {
            return Mat33(   Vec3( x * that.GetCol0() , x * that.GetCol1() , x * that.GetCol2() ) ,
                            Vec3( y * that.GetCol0() , y * that.GetCol1() , y * that.GetCol2() ) ,
                            Vec3( z * that.GetCol0() , z * that.GetCol1() , z * that.GetCol2() ) ) ;
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


        Vec3 x ;
        Vec3 y ;
        Vec3 z ;
} ;

// Public variables --------------------------------------------------------------

static const Mat33 Mat33_xIdentity( Vec3( 1.f , 0.f , 0.f ) , Vec3( 0.f , 1.f , 0.f ) , Vec3( 0.f , 0.f , 1.f ) ) ;

// Public functions --------------------------------------------------------------

#endif
