/** \file vec4.h

    \brief 4-vector class

    \author Copyright 2005-2012 MJG; All rights reserved.
*/
#ifndef VEC4_H
#define VEC4_H

#include <math.h>
#include <float.h>

#include "vec3.h"

// Macros ----------------------------------------------------------------------

/// Return square of given value.
#define POW2( x ) ( ( x ) * ( x ) )

// Types -----------------------------------------------------------------------

/** Plain-old-data struct for Vec4, but that can be in a union.
*/
struct Vector4
{
    float x ;   ///< x-component
    float y ;   ///< y-component
    float z ;   ///< z-component
    float w ;   ///< w-component
} ;




/** Utility class for 4-vectors.
*/
struct Vec4 : public Vector4
{
    public:
        Vec4()
        {
        }

        /** Construct vector from floats.
            \param fx - x component
            \param fy - y component
            \param fz - z component
            \param fw - w component
        */
        Vec4( float fx, float fy, float fz , float fw )     { x = fx ; y = fy ; z = fz ; w = fw ; }

        Vec4( const Vec3 & v3 , float fw )                  { x = v3.x ; y = v3.y ; z = v3.z ; w = fw ; }

        /// Construct vector from float array.
        Vec4( const float * pf )                            { x = pf[0] ; y = pf[1] ; z = pf[2] ; w = pf[3] ; }

        // Assignment operators.

        /// Add vector to self.
        Vec4 & operator += ( const Vec4 & rhs )             { x += rhs.x ; y += rhs.y ; z += rhs.z ; w += rhs.w ; return * this ; }
        Vec4 & operator -= ( const Vec4 & rhs )             { x -= rhs.x ; y -= rhs.y ; z -= rhs.z ; w -= rhs.w ; return * this ; }
        Vec4 & operator *= ( float f )                      { x *= f ; y *= f ; z *= f ; w *= f ; return * this ; }
        Vec4 & operator /= ( float f )                      { (*this) *= ( 1.0f / f )  ; return * this ; }

        // Unary operators.
        Vec4   operator + () const                          { return * this ; }
        Vec4   operator - () const                          { return Vec4( -x , -y , -z , -w ) ; }

        // Binary operators.
        Vec4   operator + ( const Vec4 & rhs ) const        { return Vec4( x + rhs.x , y + rhs.y , z + rhs.z , w + rhs.w ) ; }
        Vec4   operator - ( const Vec4 & rhs ) const        { return Vec4( x - rhs.x , y - rhs.y , z - rhs.z , w - rhs.w ) ; }

        Vec4   operator * ( float f ) const                 { return Vec4( x * f , y * f , z * f , w * f ) ; }
        Vec4   operator / ( float f ) const                 { return (*this) * ( 1.0f / f ) ; }

        friend Vec4   operator * ( float f , const Vec4 & v)    { return Vec4( v.x * f , v.y * f , v.z * f , v.w * f ) ; }

        bool   operator == ( const Vec4 & rhs ) const       { return ( ( x == rhs.x ) && ( y == rhs.y ) && ( z == rhs.z ) && ( w == rhs.w ) ) ; }
        bool   operator != ( const Vec4 & rhs ) const       { return ( ( x != rhs.x ) || ( y != rhs.y ) || ( z != rhs.z ) || ( w != rhs.w ) ) ; }

        float & operator[]( size_t i ) const                { return ((float*)(&x))[i] ; } ;

        /// Compute inner (i.e. dot) product of two vectors.
        float operator*( const Vec4 & rhs ) const           { return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w ; }

        /// Compute outer (i.e. cross) product of two vectors, treating both as 3-vectors.
        /// \note w-component is ignored and remains unchanged.
        Vec4  operator^( const Vec4 & rhs ) const           { return Vec4( y * rhs.z - z * rhs.y , z * rhs.x - x * rhs.z , x * rhs.y - y * rhs.x , 0.0f ) ; }

        /// Return magnitude squared of a vector.
        float Mag2( void ) const                            { return x * x + y * y + z * z + w * w ; }

        /// Return magnitude squared of a vector, treating it as a 3-vector.
        /// \note w-component is ignored.
        float Mag2v3( void ) const                          { return x * x + y * y + z * z ; }

        /// Return magnitude.
        float Magnitude( void ) const                       { return sqrt( Mag2() ) ; }

        /// Return magnitude of 3-vector.
        //! \note w-component is ignored
        float Magnitudev3( void ) const                     { return sqrt( Mag2v3() ) ; }

        /// Normalize a vector, i.e. make its length unity.
        void  Normalize( void )                             { const float len = Magnitude()   ; (*this) /= len ; }

        /// Normalize first 3 components of a 4-vector, i.e. make its length unity.
        /// \note w-component is ignored.
        void  Normalizev3( void )                           { const float len = Magnitudev3() + FLT_MIN ; x /= len ; y /= len ; z /= len ; }

        void  Zero( void )                                  { x = y = z = w = 0.0f ; }

        //static Vec4 ZERO( void )                         { return Vec4( 0.0f , 0.0f , 0.0f , 0.0f ) ; }

        bool  IsNormalized( void ) const
        {
            const float mag = Mag2() ;
            return ( ( mag < 1.00001f ) && ( mag > 0.99999f ) ) ;
        }

        bool  IsNormalizedv3( void ) const
        {
            const float mag = Mag2v3() ;
            return ( ( mag < 1.00001f ) && ( mag > 0.99999f ) ) ;
        }

        bool Resembles( const Vec4 & other , const float tolerance = 0.00001f ) const
        {
            return  Math::Resembles( x , other.x , tolerance )
                &&  Math::Resembles( y , other.y , tolerance )
                &&  Math::Resembles( z , other.z , tolerance )
                &&  Math::Resembles( w , other.w , tolerance ) ;
        }
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

/** Generate a pseudo-random Vec4 value whose components lie in the given range.

    \param vSpread  Range of values for each component.
                    The value for each component will lie between
                    -component/2 and +component/2.

*/
inline Vec4 RandomSpread( const Vec4 & vSpread )
{
    return  Vec4( RandomSpread( vSpread.x )
                , RandomSpread( vSpread.y )
                , RandomSpread( vSpread.z )
                , RandomSpread( vSpread.w ) ) ;
}

#endif
