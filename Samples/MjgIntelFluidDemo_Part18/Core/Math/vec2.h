/** \file vec2.h

    \brief 2-vector class.

    \author Copyright 2005-2012 MJG; All rights reserved.
*/
#ifndef VEC2_H
#define VEC2_H

#include <float.h>
#include <math.h>

#include "Core/Math/math.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Math
    {
        /** Plain-old-data struct for Vec2, but that can be in a union.
        */
        struct Vector2
        {
            float x ;   ///< x-component
            float y ;   ///< y-component
        } ;
    } ;
} ;



/** Utility class for 2-vectors.
*/
struct Vec2 : public PeGaSys::Math::Vector2
{
    public:
        Vec2() {}

        /** Construct 2-vector from floats.
            \param fx - x component
            \param fy - y component
        */
        Vec2( float fx , float fy ) { x = fx ; y = fy ; }

        /** Construct Vec2 from Vector2.

            \param vector2  Vector2 plain-old-data structure.

            Note that this is NOT "explicit" so this is a cast constructor.
            This allows Vector2 to be treated as a Vec2 when we want that,
            but the last of a constructor in Vector2 allows it to be present in
            unions.
        */
        Vec2( const PeGaSys::Math::Vector2 & vector2 )
        {
            x = vector2.x ; y = vector2.y ;
        }

        /// Construct vector from array of floats.
        /// \param fa - array of at least 2 floats
        Vec2( const float * fa ) { x = fa[0] ; y = fa[1] ; }

        // Assignment operators

        /// Add vector to self.
        Vec2 & operator += ( const Vec2 & rhs )          { x += rhs.x ; y += rhs.y ; return * this ; }

        /// Subtract vector from self.
        Vec2 & operator -= ( const Vec2 & rhs )          { x -= rhs.x ; y -= rhs.y ; return * this ; }

        /// Multiply scalar to self.
        Vec2 & operator *= ( float f )                   { x *= f ; y *= f ; return * this ; }

        /// Divide self by scalar.
        Vec2 & operator /= ( float f )                   { (*this) *= ( 1.0f / f )  ; return * this ; }

        // Unary operators

        /// Unary plus -- does nothing.
        Vec2 operator + () const                         { return * this ; }

        /// unary minus -- return negative of this vector.
        Vec2 operator - () const                         { return Vec2( -x , -y ) ; }

        // Binary operators

        /// add two vectors.
        Vec2 operator + ( const Vec2 & rhs ) const       { return Vec2( x + rhs.x , y + rhs.y ) ; }

        /// Subtract two vectors.
        Vec2 operator - ( const Vec2 & rhs ) const       { return Vec2( x - rhs.x , y - rhs.y ) ; }
        //Vec2 operator - ( const Vec2 & rhs ) const       ; // For CPU performance profiling. See util.cpp.

        /// Divide each component of this vector by each component of that vector.
        Vec2 operator / ( const Vec2 & rhs ) const       { return Vec2( x / rhs.x , y / rhs.y ) ; }

        /// Multiply vector by scalar.
        Vec2 operator * ( float f ) const                { return Vec2( x * f , y * f ) ; }

        /// Divide vector by scalar.
        Vec2 operator / ( float f ) const                { return (*this) * ( 1.0f / f ) ; }

        /// Multiply vector by scalar.
        friend Vec2 operator * ( float f , const Vec2 & v)  { return Vec2( v.x * f , v.y * f ) ; }

        /// Compare two vectors for equal values.
        bool operator == ( const Vec2 & rhs ) const      { return ( ( x == rhs.x ) && ( y == rhs.y ) ) ; }

        /// Compare two vectors for equal values and negate.
        bool operator != ( const Vec2 & rhs ) const      { return ( ( x != rhs.x ) || ( y != rhs.y ) ) ; }

        /// Determine whether all components of another vector strictly exceed those of this.
        bool operator < ( const Vec2 & rhs ) const       { return ( ( x < rhs.x ) && ( y < rhs.y ) ) ; }

        /// Determine whether all components of this vector strictly exceed those of another.
        bool operator > ( const Vec2 & rhs ) const       { return ( ( x > rhs.x ) && ( y > rhs.y ) ) ; }

        /// Determine whether all components of another vector exceed or equal those of this.
        bool operator <= ( const Vec2 & rhs ) const
        {
            return ( ( x <= rhs.x ) && ( y <= rhs.y ) ) ;
        }

        /// Determine whether all components of this vector exceed or equal those of another.
        bool operator >= ( const Vec2 & rhs ) const
        {
            return ( ( x >= rhs.x ) && ( y >= rhs.y ) ) ;
        }

        /// Compute inner (i.e. dot) product of two vectors.
        float operator*( const Vec2 & rhs ) const
        {
            return x * rhs.x + y * rhs.y ;
        }

        /// Compute outer (i.e. cross) product of two vectors.
        //Vec3  operator^( const Vec2 & rhs ) const       { return Vec3( 0.0f , 0.0f , x * rhs.y - y * rhs.x ) ; }

        /// Return magnitude squared of a vector.
        float Mag2() const                              { return x * x + y * y ; }

        /// Return magnitude.
        float Magnitude() const                         { return sqrtf( Mag2() ) ; }

        /// Return magnitude.
        float MagnitudeFast() const                     { return fsqrtf( Mag2() ) ; }

        /// Return reciprocal magnitude.
        float ReciprocalMagnitude() const               { return 1.0f / sqrtf( Mag2() ) ; }
        float ReciprocalMagnitudeFast() const           { return finvsqrtf   ( Mag2() ) ; }

        /// Return a unit vector in the same direction as this vector.
        Vec2  GetDir() const                            { return (*this) * 1.0f / sqrtf( Mag2() + FLT_MIN ) ; }
        Vec2  GetDirFast() const                        { return (*this) * finvsqrtf( Mag2() + FLT_MIN ) ; }
        static Vec2 GetDir( const Vec2 & v )            { return v.GetDir() ; }
        static Vec2 GetDirFast( const Vec2 & v )        { return v.GetDirFast() ; }

        /// Normalize this vector, i.e. make its length unity.
        void  Normalize()                               { * this = GetDir()     ; }
        void  NormalizeFast()                           { * this = GetDirFast() ; }

        /// Assign a vector to zero.
        void  Zero()                                    { x = y = 0.0f ; }


        /** Return whether this vector has unit length.

            \param tolerance    Threshold for length.

            \return Whether this vector has unit length.
        */
        bool IsNormalized( const float tolerance = 1.0e-6f ) const
        {
            const float mag2 = fabsf( 1.0f - Mag2() ) ;
            return ( mag2 < tolerance ) ;
        }


        /** Return whether this vector resembles another.

            \param other        Other vector.

            \param tolerance    Threshold for measure of similarity.

            \return Whether this vector resembles another (within tolerance)
        */
        bool Resembles( const Vec2 & other , const float tolerance = 1.0e-4f ) const
        {
            return  Math::Resembles( x , other.x , tolerance )
                &&  Math::Resembles( y , other.y , tolerance ) ;
        }
} ;


// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

/** Generate a pseudo-random Vec3 value whose components lie in the given range.

    \param vSpread  Range of values for each component.
                    The value for each component will lie between
                    -component/2 and +component/2.

*/
inline Vec2 RandomSpread( const Vec2 & vSpread )
{
    return  Vec2( RandomSpread( vSpread.x )
                , RandomSpread( vSpread.y ) ) ;
}

namespace Math
{
    inline bool Resembles( const Vec2 & v1, const Vec2 & v2 , const float fTolerance = 0.0001f )
    {
        return v1.Resembles( v2 , fTolerance ) ;
    }
} ;

inline bool IsNan( const Vec2 & v )
{
    return IsNan( v.x ) || IsNan( v.y ) ;
}

inline bool IsInf( const Vec2 & v )
{
    return IsInf( v.x ) || IsInf( v.y ) ;
}

#endif
