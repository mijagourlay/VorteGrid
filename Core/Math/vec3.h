/*! \file vec3.h

    \brief 3-vector class

    \author Copyright 2005-2009 UCF/FIEA/MJG; All rights reserved.
*/
#ifndef VEC3_H
#define VEC3_H

#include <float.h>
#include <math.h>

#include "Core/Math/math.h"

// Macros --------------------------------------------------------------

//! \brief square value
#define POW2( x ) ( ( x ) * ( x ) )

// Types --------------------------------------------------------------

/*! \brief Utility class for 3-vectors
*/
struct Vec3
{
    public:
        Vec3() {}

        //! \brief construct vector from floats
        //! \param fx - x component
        //! \param fy - y component
        //! \param fz - z component
        Vec3( float fx, float fy, float fz ) { x = fx ; y = fy ; z = fz ; }
        Vec3( const float * fa ) { x = fa[0] ; y = fa[1] ; z = fa[2] ; }

        // assignment operators
        //! \brief add vector to self
        Vec3 & operator += ( const Vec3 & rhs )          { x += rhs.x ; y += rhs.y ; z += rhs.z ; return * this ; }
        //! \brief subtract vector from self
        Vec3 & operator -= ( const Vec3 & rhs )          { x -= rhs.x ; y -= rhs.y ; z -= rhs.z ; return * this ; }
        //! \brief multiply scalar to self
        Vec3 & operator *= ( float f )                   { x *= f ; y *= f ; z *= f ; return * this ; }
        //! \brief divide self by scalar
        Vec3 & operator /= ( float f )                   { (*this) *= ( 1.0f / f )  ; return * this ; }

        // unary operators
        //! \brief unary plus -- does nothing
        Vec3 operator + () const                         { return * this ; }
        //! \brief unary minus -- return negative of this vector
        Vec3 operator - () const                         { return Vec3( -x , -y , -z ) ; }

        // binary operators
        //! \brief add two vectors
        Vec3 operator + ( const Vec3 & rhs ) const       { return Vec3( x + rhs.x , y + rhs.y , z + rhs.z ) ; }

        //! \brief subtract two vectors
        Vec3 operator - ( const Vec3 & rhs ) const       { return Vec3( x - rhs.x , y - rhs.y , z - rhs.z ) ; }
        //Vec3 operator - ( const Vec3 & rhs ) const       ; // For CPU performance profiling. See util.cpp.

        //! \brief Divide each component of this vector by each component of that vector.
        Vec3 operator / ( const Vec3 & rhs ) const       { return Vec3( x / rhs.x , y / rhs.y , z / rhs.z ) ; }

        //! \brief multiply vector by scalar
        Vec3 operator * ( float f ) const                { return Vec3( x * f , y * f , z * f ) ; }
        //! \brief divide vector by scalar
        Vec3 operator / ( float f ) const                { return (*this) * ( 1.0f / f ) ; }

        //! \brief multiply vector by scalar
        friend Vec3 operator * ( float f , const Vec3 & v)      { return Vec3( v.x * f , v.y * f , v.z * f ) ; }

        //! \brief compare two vectors for equal values
        bool operator == ( const Vec3 & rhs ) const      { return ( ( x == rhs.x ) && ( y == rhs.y ) && ( z == rhs.z ) ) ; }
        //! \brief compare two vectors for equal values and negate
        bool operator != ( const Vec3 & rhs ) const      { return ( ( x != rhs.x ) || ( y != rhs.y ) || ( z != rhs.z ) ) ; }

        //! \brief determine whether all components of another vector strictly exceed those of this
        bool operator < ( const Vec3 & rhs ) const       { return ( ( x < rhs.x ) && ( y < rhs.y ) && ( z < rhs.z ) ) ; }
        //! \brief determine whether all components of this vector strictly exceed those of another
        bool operator > ( const Vec3 & rhs ) const       { return ( ( x > rhs.x ) && ( y > rhs.y ) && ( z > rhs.z ) ) ; }

        //! \brief determine whether all components of another vector exceed or equal those of this
        bool operator <= ( const Vec3 & rhs ) const
        {
            return ( ( x <= rhs.x ) && ( y <= rhs.y ) && ( z <= rhs.z ) ) ;
        }
        //! \brief determine whether all components of this vector exceed or equal those of another
        bool operator >= ( const Vec3 & rhs ) const
        {
            return ( ( x >= rhs.x ) && ( y >= rhs.y ) && ( z >= rhs.z ) ) ;
        }

        //! \brief compute inner (i.e. dot) product of two vectors
        float operator*( const Vec3 & rhs ) const       { return x * rhs.x + y * rhs.y + z * rhs.z ; }

        //! \brief compute outer (i.e. cross) product of two vectors
        Vec3  operator^( const Vec3 & rhs ) const       { return Vec3( y * rhs.z - z * rhs.y , z * rhs.x - x * rhs.z , x * rhs.y - y * rhs.x ) ; }

        //! \brief return magnitude squared of a vector
        float Mag2() const                              { return x * x + y * y + z * z ; }

        //! \brief return magnitude
        float Magnitude() const                         { return sqrtf( Mag2() ) ; }

        //! \brief return reciprocal magnitude
        float ReciprocalMagnitude() const               { return finvsqrtf( Mag2() ) ; }

        //! \brief return a unit vector in the same direction as this vector.
        Vec3  GetDir() const                            { return (*this) * finvsqrtf( Mag2() + FLT_MIN ) ; }

        //! \brief normalize this vector, i.e. make its length unity
        void  Normalize()                               { * this = GetDir() ; }

        //! \brief Assign a vector to zero
        void  Zero()                                    { x = y = z = 0.0f ; }

        bool IsNormalized( void ) const
        {
            const float mag2 = Mag2() ;
            if( ( mag2 < 1.00001 ) && ( mag2 > 0.99999 ) ) return true ;
            return false ;
        }

        bool Resembles( const Vec3 & other ) const
        {
            const Vec3          vDiff       = other - ( * this ) ;
            const float         fDiffMag2   = vDiff.Mag2() ;
            static const float  sDiffTol    = 1.0e-6f ;
            if( fDiffMag2 >= sDiffTol )
            {   // Absolute difference exceeds tolerance.
                const float fMag2 = this->Mag2() ;
                // Use relative difference.
                return ( fDiffMag2 / fMag2 < sDiffTol ) ;
            }
            return ( fDiffMag2 < sDiffTol ) ;
        }


        /*! \brief Return whether 3 points are colinear

            \param v0 - point in space
            \param v1 - point in space
            \param v2 - point in space

            \return Whether 3 given points are colinear (within tolerance)
        */
        static bool IsColinear( const Vec3 & v0 , const Vec3 & v1 , const Vec3 & v2 )
        {
            const Vec3  v01     = v1 - v0 ;
            const Vec3  v02     = v2 - v0 ;
            const Vec3  cross   = v01 ^ v02 ;
            const float crosMag = cross.Mag2() ;    // area of parallelogram
            return crosMag < 1.0e-6 ;
        }


        /*! \brief Return whether 4 points are coplanar

            \param v0 - point in space
            \param v1 - point in space
            \param v2 - point in space
            \param v3 - point in space

            \return Whether 4 given points are coplanar (within tolerance)
        */
        static bool IsCoplanar( const Vec3 & v0 , const Vec3 & v1 , const Vec3 & v2 , const Vec3 & v3 )
        {
            if( IsColinear( v0 , v1 , v2 ) )
            {   // If any 3 of the given points are colinear then these 4 points are colinear
                return true ;
            }
            const Vec3  v01     = v1 - v0 ;
            const Vec3  v02     = v2 - v0 ;
            const Vec3  v03     = v3 - v0 ;
            const Vec3  cross   = v01 ^ v02 ;
            const float triple  = FAbs( cross * v03 ) ; // volume of parallelepiped
            return triple < 1.0e-6 ;
        }

        float x ;   ///< x-component
        float y ;   ///< y-component
        float z ;   ///< z-component
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

inline Vec3 RandomSpread( const Vec3 & vSpread )
{
    return  Vec3( RandomSpread( vSpread.x )
                , RandomSpread( vSpread.y )
                , RandomSpread( vSpread.z ) ) ;
}

#endif