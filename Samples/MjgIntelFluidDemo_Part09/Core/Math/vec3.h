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

#if MEM_ALIGN_16
#define USE_SSE 1
#else
#define USE_SSE 0
#endif

#if USE_SSE

#pragma warning( disable: 4201 )

#include <fvec.h>

extern __declspec(align(16)) const __int32 __MASKSIGNs_[8] ;

#define _MASKSIGNs_ (*(F32vec4*)& __MASKSIGNs_ )

/*! \brief 3-vector that uses Streaming SIMD Extensions

    \note This code was inspired by code accompanying the article
          "Optimized Matrix Library for use with the Intel® Pentium® 4 Processor's SSE2 Instructions (SSE2)"
          by Zvi Devir & Ronen Zohar.

*/
struct Vec3
{
    public:
        Vec3() { }
        Vec3( const Vec3 & that ) : vec( that.vec ) { }
        Vec3( const __m128 & m ) : vec( m ) {}
        Vec3( const F32vec4 & m) : vec( m ) {}

        //! \brief construct vector from floats
        //! \param fx - x component
        //! \param fy - y component
        //! \param fz - z component
        Vec3(const float x, const float y, const float z) : vec( F32vec4( 0.0f , z , y , x ) ) {}

        Vec3(const float *arr) : vec( _mm_loadl_pi( _mm_movelh_ps( _mm_load_ss( arr + 2 ) , _mm_load_ss( arr + 2 ) ) , (__m64*) arr ) ) {}

        operator __m128()  const { return vec ; }
        operator F32vec4() const { return vec ; }

        // assignment operators
        Vec3 & operator = ( const Vec3 & that )             { vec = that.vec ; return * this ; }
        Vec3 & operator = ( const F32vec4 & a )             { vec = a ; return * this ; }
        Vec3 & operator = ( const __m128 & a )              { vec = a ; return * this ; }

        //      float & X( void )       { return * ( ( (float *) & vec )     ) ; }
        //      float & Y( void )       { return * ( ( (float *) & vec ) + 1 ) ; }
        //      float & Z( void )       { return * ( ( (float *) & vec ) + 2 ) ; }
        //const float & X( void ) const { return * ( ( (float *) & vec )     ) ; }
        //const float & Y( void ) const { return * ( ( (float *) & vec ) + 1 ) ; }
        //const float & Z( void ) const { return * ( ( (float *) & vec ) + 2 ) ; }

        float & operator () ( int i )
        {
            return * ( ( (float *) & vec ) + i ) ;
        }

        float & operator [] (int i)
        {
            return * ( ( (float *) & vec ) + i ) ;
        }

        float & operator [] (int i) const
        {
            return * ( ( (float *) & vec ) + i ) ;
        }


        //! \brief add vector to self
        Vec3 & operator += ( const Vec3 & rhs )             { vec = _mm_add_ps( vec , rhs.vec ) ; return * this ; }
        //! \brief subtract vector from self
        Vec3 & operator -= ( const Vec3 & rhs )             { vec = _mm_sub_ps( vec , rhs.vec ) ; return * this ; }
        //! \brief multiply scalar to self
        Vec3 & operator *= ( float f )                      { vec = vec * F32vec4( f ) ; return * this ; }
        //! \brief divide self by scalar
        Vec3 & operator /= ( float f )                      { (*this) *= ( 1.0f / f )  ; return * this ; }

        // unary operators
        //! \brief unary plus -- does nothing
        Vec3 operator + () const                            { return * this ; }
        //! \brief unary minus -- return negative of this vector
        Vec3 operator - () const                            { return _mm_xor_ps( _MASKSIGNs_ , vec ) ; }

        // binary operators
        //! \brief add two vectors
        Vec3 operator + ( const Vec3 & rhs ) const          { return _mm_add_ps( vec , rhs.vec ) ; }

        //! \brief subtract two vectors
        Vec3 operator - ( const Vec3 & rhs ) const          { return _mm_sub_ps( vec , rhs.vec ) ; }

        //! \brief Divide each component of this vector by each component of that vector.
        Vec3 operator / ( const Vec3 & rhs ) const          { return Vec3( x / rhs.x , y / rhs.y , z / rhs.z ) ; }

        //! \brief multiply vector by scalar
        Vec3 operator * ( float f ) const                   { return vec * F32vec4( f ) ; }
        //! \brief divide vector by scalar
        Vec3 operator / ( float f ) const                   { return (*this) * ( 1.0f / f ) ; }

        //! \brief multiply vector by scalar
        friend Vec3 operator * ( float f , const Vec3 & v)  { return v.vec * F32vec4( f ) ; }

        //! \brief compare two vectors for equal values
        bool operator == ( const Vec3 & rhs ) const         { return ( ( x == rhs.x ) && ( y == rhs.y ) && ( z == rhs.z ) ) ; }
        //! \brief compare two vectors for equal values and negate
        bool operator != ( const Vec3 & rhs ) const         { return ( ( x != rhs.x ) || ( y != rhs.y ) || ( z != rhs.z ) ) ; }

        //! \brief determine whether all components of another vector strictly exceed those of this
        bool operator < ( const Vec3 & rhs ) const          { return ( ( x < rhs.x ) && ( y < rhs.y ) && ( z < rhs.z ) ) ; }
        //! \brief determine whether all components of this vector strictly exceed those of another
        bool operator > ( const Vec3 & rhs ) const          { return ( ( x > rhs.x ) && ( y > rhs.y ) && ( z > rhs.z ) ) ; }

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
        float operator * ( const Vec3 & rhs ) const
        {
            F32vec4 r = _mm_mul_ps( vec , rhs.vec ) ;
            F32vec1 t = _mm_add_ss(_mm_shuffle_ps(r,r,1), _mm_add_ps(_mm_movehl_ps(r,r),r));
            return *(float *) & t ;
        }

        //! \brief compute outer (i.e. cross) product of two vectors
        Vec3 operator ^ ( const Vec3 & rhs ) const
        {
            F32vec4 l1, l2, m1, m2 ;
            l1 = _mm_shuffle_ps( vec , vec , _MM_SHUFFLE(3,1,0,2) ) ;
            l2 = _mm_shuffle_ps( rhs , rhs , _MM_SHUFFLE(3,0,2,1) ) ;
            m2 = l1 * l2 ;
            l1 = _mm_shuffle_ps( vec , vec , _MM_SHUFFLE(3,0,2,1) ) ;
            l2 = _mm_shuffle_ps( rhs , rhs , _MM_SHUFFLE(3,1,0,2) ) ;
            m1 = l1 * l2 ;
            return m1 - m2 ;
        }

        //! \brief return magnitude squared of a vector
        float Mag2() const
        {
            F32vec4 r = _mm_mul_ps( vec , vec ) ;
            F32vec1 t = _mm_add_ss( _mm_shuffle_ps( r , r , 1 ) , _mm_add_ss( _mm_movehl_ps( r , r ) , r ) ) ;
            return * (float *) & t ;
        }

        //! \brief return magnitude
        float Magnitude() const                         { return sqrtf( Mag2() ) ; }

        //! \brief return reciprocal magnitude
        float ReciprocalMagnitude() const               {  return finvsqrtf( Mag2() ) ; }

        //! \brief return a unit vector in the same direction as this vector.
        Vec3  Direction() const
        {
            F32vec4 r = _mm_mul_ps( vec , vec ) ;
            F32vec1 t = _mm_add_ss(_mm_shuffle_ps(r,r,1), _mm_add_ss(_mm_movehl_ps(r,r),r));
            t = _mm_cmpneq_ss(t, _mm_setzero_ps()) & rsqrt_nr(t) ;
            return _mm_mul_ps( vec , _mm_shuffle_ps(t,t,0x00) ) ;
        }

        //! \brief normalize this vector, i.e. make its length unity
        void  Normalize()
        {
            F32vec4 r = _mm_mul_ps(vec,vec);
            F32vec1 t = _mm_add_ss(_mm_shuffle_ps(r,r,1), _mm_add_ss(_mm_movehl_ps(r,r),r)) ;
            t = _mm_cmpneq_ss(t, _mm_setzero_ps()) & rsqrt_nr(t) ;
            vec = _mm_mul_ps(vec, _mm_shuffle_ps(t,t,0x00)) ;
        }

        //! \brief Assign a vector to zero
        void  Zero()                                    { x = y = z = 0.0f ; }

        bool IsNormalized( void ) const
        {
            const float mag2 = Mag2() ;
            if( ( mag2 < 1.00001 ) && ( mag2 > 0.99999 ) ) return true ;
            return false ;
        }

        bool Resembles( const Vec3 & other , const float fTolerance = 0.00001f ) const
        {
            const Vec3          vDiff       = other - ( * this ) ;
            const float         fDiffMag2   = vDiff.Mag2() ;
            if( fDiffMag2 >= fTolerance )
            {   // Absolute difference exceeds tolerance.
                const float fMag2 = this->Mag2() ;
                // Use relative difference.
                return ( fDiffMag2 / fMag2 < fTolerance ) ;
            }
            return ( fDiffMag2 < fTolerance ) ;
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

        union {
            __m128 vec ;
            struct {
                float x                 ;   ///< x-component
                float y                 ;   ///< y-component
                float z                 ;   ///< z-component
                //float w__FOR_ALIGNMENT  ;   ///< w-component, used to maintain 16-byte alignment.
            } ;
        } ;
} ;

#else

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

        //! \brief return magnitude
        float MagnitudeFast() const                     { return fsqrtf( Mag2() ) ; }

        //! \brief return reciprocal magnitude
        float ReciprocalMagnitude() const               { return finvsqrtf( Mag2() ) ; }

        //! \brief return a unit vector in the same direction as this vector.
        Vec3  Direction() const                         { return (*this) * finvsqrtf( Mag2() + FLT_MIN ) ; }

        //! \brief normalize this vector, i.e. make its length unity
        void  Normalize()                               { * this = Direction() ; }

        //! \brief Assign a vector to zero
        void  Zero()                                    { x = y = z = 0.0f ; }

        bool IsNormalized( void ) const
        {
            const float mag2 = Mag2() ;
            if( ( mag2 < 1.00001 ) && ( mag2 > 0.99999 ) ) return true ;
            return false ;
        }

        bool Resembles( const Vec3 & other , const float fTolerance = 0.00001f ) const
        {
            const Vec3          vDiff       = other - ( * this ) ;
            const float         fDiffMag2   = vDiff.Mag2() ;
            if( fDiffMag2 >= fTolerance )
            {   // Absolute difference exceeds tolerance.
                const float fMag2 = this->Mag2() ;
                // Use relative difference.
                return ( fDiffMag2 / fMag2 < fTolerance ) ;
            }
            return ( fDiffMag2 < fTolerance ) ;
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

        void Clamp( const Vec3 & min , const Vec3 & max )
        {
            x = CLAMP( x , min.x , max.x ) ;
            y = CLAMP( y , min.y , max.y ) ;
            z = CLAMP( z , min.z , max.z ) ;
        }

        float x                 ;   ///< x-component
        float y                 ;   ///< y-component
        float z                 ;   ///< z-component
        //float w__FOR_ALIGNMENT  ;   ///< w-component, used to maintain 16-byte alignment.
} ;

#endif

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

inline Vec3 RandomSpread( const Vec3 & vSpread )
{
    return  Vec3( RandomSpread( vSpread.x )
                , RandomSpread( vSpread.y )
                , RandomSpread( vSpread.z ) ) ;
}

#endif
