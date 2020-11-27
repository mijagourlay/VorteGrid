/*! \file vec4.h

    \brief 4-vector class

    \author Copyright 2005-2006 UCF/FIEA/MJG; All rights reserved.
*/
#ifndef VEC4_H
#define VEC4_H

#include <math.h>
#include <float.h>

#include "vec3.h"

// Macros --------------------------------------------------------------

//! \brief square value
#define POW2( x ) ( ( x ) * ( x ) )

// Types --------------------------------------------------------------

/*! \brief Utility class for 4-vectors
*/
struct Vec4
{
    public:
        Vec4()
        {
        }

        /*! \brief Destruct a 4-vector
            \note   Even though this destructor body contains no code, this method is required
                    by the Vector (dynamic array) container, which explicitly calls the destructor
                    of the contained type.  If you remove this method, and create a container
                    Vector<Vec4>, the compiler will generate errors about unused variables in Vector::Clear.
                    The compiler might also generate warnings about other types such as "int", which are bogus,
                    and which disappear when this destructor exists.
        */
        // ~Vec4() { }

        //! \brief construct vector from floats
        //! \param fx - x component
        //! \param fy - y component
        //! \param fz - z component
        Vec4( float fx, float fy, float fz )             { x = fx ; y = fy ; z = fz ; }
        //! \brief construct vector from floats
        //! \param fx - x component
        //! \param fy - y component
        //! \param fz - z component
        //! \param fw - w component
        Vec4( float fx, float fy, float fz , float fw )  { x = fx ; y = fy ; z = fz ; w = fw ; }

        Vec4( const Vec3 & v3 )                          { x = v3.x ; y = v3.y ; z = v3.z ; }
        Vec4( const Vec3 & v3 , float fw )               { x = v3.x ; y = v3.y ; z = v3.z ; w = fw ; }

        //! \brief construct vector from float array
        Vec4( const float * pf )                         { x = pf[0] ; y = pf[1] ; z = pf[2] ; w = pf[3] ; }

        // assignment operators
        //! \brief add vector to self
        Vec4 & operator += ( const Vec4 & rhs )          { x += rhs.x ; y += rhs.y ; z += rhs.z ; w += rhs.w ; return * this ; }
        Vec4 & operator -= ( const Vec4 & rhs )          { x -= rhs.x ; y -= rhs.y ; z -= rhs.z ; w -= rhs.w ; return * this ; }
        Vec4 & operator *= ( float f )                   { x *= f ; y *= f ; z *= f ; w *= f ; return * this ; }
        Vec4 & operator /= ( float f )                   { (*this) *= ( 1.0f / f )  ; return * this ; }

        // unary operators
        Vec4   operator + () const                       { return * this ; }
        Vec4   operator - () const                       { return Vec4( -x , -y , -z , -w ) ; }

        // binary operators
        Vec4   operator + ( const Vec4 & rhs ) const     { return Vec4( x + rhs.x , y + rhs.y , z + rhs.z , w + rhs.w ) ; }
        Vec4   operator - ( const Vec4 & rhs ) const     { return Vec4( x - rhs.x , y - rhs.y , z - rhs.z , w - rhs.w ) ; }

        Vec4   operator * ( float f ) const              { return Vec4( x * f , y * f , z * f , w * f ) ; }
        Vec4   operator / ( float f ) const              { return (*this) * ( 1.0f / f ) ; }

        friend Vec4   operator * ( float f , const Vec4 & v)    { return Vec4( v.x * f , v.y * f , v.z * f , v.w * f ) ; }

        bool   operator == ( const Vec4 & rhs ) const    { return ( ( x == rhs.x ) && ( y == rhs.y ) && ( z == rhs.z ) && ( w == rhs.w ) ) ; }
        bool   operator != ( const Vec4 & rhs ) const    { return ( ( x != rhs.x ) || ( y != rhs.y ) || ( z != rhs.z ) || ( w != rhs.w ) ) ; }

        float & operator[]( int i ) const                { return ((float*)(&x))[i] ; } ;

        //! \brief compute inner (i.e. dot) product of two vectors
        float operator*( const Vec4 & rhs ) const        { return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w ; }
        //! \brief compute outer (i.e. cross) product of two vectors, treating both as 3-vectors
        //! \note w-component is ignored and remains unchanged.
        Vec4  operator^( const Vec4 & rhs ) const        { return Vec4( y * rhs.z - z * rhs.y , z * rhs.x - x * rhs.z , x * rhs.y - y * rhs.x , 0.0f ) ; }

        //! \brief return magnitude squared of a vector
        float Mag2( void ) const                         { return x * x + y * y + z * z + w * w ; }
        //! \brief return magnitude squared of a vector, treating it as a 3-vector
        //! \note w-component is ignored
        float Mag2v3( void ) const                       { return x * x + y * y + z * z ; }
        //! \brief return magnitude
        float Magnitude( void ) const                    { return sqrt( Mag2() ) ; }
        //! \brief return magnitude of 3-vector
        //! \note w-component is ignored
        float Magnitudev3( void ) const                  { return sqrt( Mag2v3() ) ; }
        //! \brief normalize a vector, i.e. make its length unity
        void  Normalize( void )                          { const float len = Magnitude()   ; (*this) /= len ; }
        //! \brief normalize first 3 components of a 4-vector, i.e. make its length unity
        //! \note w-component is ignored
        void  Normalizev3( void )                        { const float len = Magnitudev3() ; x /= len ; y /= len ; z /= len ; }
        void  Zero( void )                               { x = y = z = w = 0.0f ; }
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

        bool Resembles( const Vec4 & other , const float fTolerance = 0.00001f ) const
        {
            Vec4 diff = other - ( * this ) ;
            const float diffMag2 = diff.Mag2() ;
            return ( diffMag2 < fTolerance ) ;
        }

        float DistFromPlane( const Vec3 & vPoint ) const
        {
            const float distFromPlane = vPoint * (Vec3&) *this + w ;
            return distFromPlane ;
        }

        float x ;   ///< x-component
        float y ;   ///< y-component
        float z ;   ///< z-component
        float w ;   ///< w-component
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

inline Vec4 RandomSpread( const Vec4 & vSpread )
{
    return  Vec4( RandomSpread( vSpread.x )
                , RandomSpread( vSpread.y )
                , RandomSpread( vSpread.z )
                , RandomSpread( vSpread.w ) ) ;
}

#endif
