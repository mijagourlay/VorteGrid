/** \file vec3.h

    \brief 3-vector class.

    \author Copyright 2005-2012 MJG; All rights reserved.
*/
#ifndef VEC3_H
#define VEC3_H

#include <float.h>
#include <math.h>

#include "Core/Math/math.h"

// Macros ----------------------------------------------------------------------

/// square value.
#define POW2( x ) ( ( x ) * ( x ) )

#if MEM_ALIGN_16
/// Enable SSE -- Streaming SIMD Extensions for x86 architectures.
#define USE_SSE 1
#else
/// Disable SSE -- Streaming SIMD Extensions for x86 architectures.
#define USE_SSE 0
#endif

// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Math
    {
        /** Plain-old-data struct for Vec3, but that can be in a union.
        */
        struct Vector3
        {
            float x ;   ///< x-component
            float y ;   ///< y-component
            float z ;   ///< z-component
        } ;
    } ;
} ;



#if USE_SSE

#pragma warning( disable: 4201 )

#include <fvec.h>

extern __declspec(align(16)) const __int32 __MASKSIGNs_[8] ;

#define _MASKSIGNs_ (*(F32vec4*)& __MASKSIGNs_ )

/** 3-vector that uses Streaming SIMD Extensions.

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

        /// construct vector from floats.
        /// \param fx - x component
        /// \param fy - y component
        /// \param fz - z component
        Vec3(const float x, const float y, const float z) : vec( F32vec4( 0.0f , z , y , x ) ) {}

        Vec3(const float *arr) : vec( _mm_loadl_pi( _mm_movelh_ps( _mm_load_ss( arr + 2 ) , _mm_load_ss( arr + 2 ) ) , (__m64*) arr ) ) {}

        Vec3( const PeGaSys::Math::Vector3 & vector3 ) : vec( F32vec4( 0.0f , vector3.z , vector3.y , vector3.x ) ) {}

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
            ASSERT( (0 <= i) && ( i <= 2 ) ) ;
            return * ( ( (float *) & vec ) + i ) ;
        }

        float & operator [] (int i)
        {
            ASSERT( ( 0 <= i ) && ( i <= 2 ) ) ;
            return * ( ( (float *) & vec ) + i ) ;
        }

        float & operator [] (int i) const
        {
            ASSERT( ( 0 <= i ) && ( i <= 2 ) ) ;
            return * ( ( (float *) & vec ) + i ) ;
        }


        /// add vector to self.
        Vec3 & operator += ( const Vec3 & rhs )             { vec = _mm_add_ps( vec , rhs.vec ) ; return * this ; }
        /// subtract vector from self.
        Vec3 & operator -= ( const Vec3 & rhs )             { vec = _mm_sub_ps( vec , rhs.vec ) ; return * this ; }
        /// multiply scalar to self.
        Vec3 & operator *= ( float f )                      { vec = vec * F32vec4( f ) ; return * this ; }
        /// divide self by scalar.
        Vec3 & operator /= ( float f )                      { (*this) *= ( 1.0f / f )  ; return * this ; }

        // unary operators
        /// unary plus -- does nothing.
        Vec3 operator + () const                            { return * this ; }
        /// unary minus -- return negative of this vector.
        Vec3 operator - () const                            { return _mm_xor_ps( _MASKSIGNs_ , vec ) ; }

        // binary operators
        /// add two vectors.
        Vec3 operator + ( const Vec3 & rhs ) const          { return _mm_add_ps( vec , rhs.vec ) ; }

        /// subtract two vectors.
        Vec3 operator - ( const Vec3 & rhs ) const          { return _mm_sub_ps( vec , rhs.vec ) ; }

        /// Divide each component of this vector by each component of that vector.
        Vec3 operator / ( const Vec3 & rhs ) const          { return Vec3( x / rhs.x , y / rhs.y , z / rhs.z ) ; }

        /// multiply vector by scalar.
        Vec3 operator * ( float f ) const                   { return vec * F32vec4( f ) ; }
        /// divide vector by scalar.
        Vec3 operator / ( float f ) const                   { return (*this) * ( 1.0f / f ) ; }

        /// multiply vector by scalar.
        friend Vec3 operator * ( float f , const Vec3 & v)  { return v.vec * F32vec4( f ) ; }

        /// compare two vectors for equal values.
        bool operator == ( const Vec3 & rhs ) const         { return ( ( x == rhs.x ) && ( y == rhs.y ) && ( z == rhs.z ) ) ; }
        /// compare two vectors for equal values and negate.
        bool operator != ( const Vec3 & rhs ) const         { return ( ( x != rhs.x ) || ( y != rhs.y ) || ( z != rhs.z ) ) ; }

        /// determine whether all components of another vector strictly exceed those of this.
        bool operator < ( const Vec3 & rhs ) const          { return ( ( x < rhs.x ) && ( y < rhs.y ) && ( z < rhs.z ) ) ; }
        /// determine whether all components of this vector strictly exceed those of another.
        bool operator > ( const Vec3 & rhs ) const          { return ( ( x > rhs.x ) && ( y > rhs.y ) && ( z > rhs.z ) ) ; }

        /// determine whether all components of another vector exceed or equal those of this.
        bool operator <= ( const Vec3 & rhs ) const
        {
            return ( ( x <= rhs.x ) && ( y <= rhs.y ) && ( z <= rhs.z ) ) ;
        }
        /// determine whether all components of this vector exceed or equal those of another.
        bool operator >= ( const Vec3 & rhs ) const
        {
            return ( ( x >= rhs.x ) && ( y >= rhs.y ) && ( z >= rhs.z ) ) ;
        }


        /// Compute inner (i.e. dot) product of two vectors.
        float operator * ( const Vec3 & rhs ) const
        {
            F32vec4 r = _mm_mul_ps( vec , rhs.vec ) ;
            F32vec1 t = _mm_add_ss(_mm_shuffle_ps(r,r,1), _mm_add_ps(_mm_movehl_ps(r,r),r));
            return *(float *) & t ;
        }

        /// Compute outer (i.e. cross) product of two vectors.
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

        /// Return magnitude squared of a vector.
        float Mag2() const
        {
            F32vec4 r = _mm_mul_ps( vec , vec ) ;
            F32vec1 t = _mm_add_ss( _mm_shuffle_ps( r , r , 1 ) , _mm_add_ss( _mm_movehl_ps( r , r ) , r ) ) ;
            return * (float *) & t ;
        }

        /// Return magnitude.
        float Magnitude() const                         { return sqrtf( Mag2() ) ; }
        float MagnitudeFast() const                     { return fsqrtf( Mag2() ) ; }

        /// Return reciprocal magnitude.
        float ReciprocalMagnitude() const               { ASSERT( Mag2() > FLT_MIN ) ; return finvsqrtf( Mag2() ) ; }

        /// Return a unit vector in the same direction as this vector.
        Vec3  GetDir() const
        {
            F32vec4 r = _mm_mul_ps( vec , vec ) ;
            F32vec1 t = _mm_add_ss(_mm_shuffle_ps(r,r,1), _mm_add_ss(_mm_movehl_ps(r,r),r));
            t = _mm_cmpneq_ss(t, _mm_setzero_ps()) & rsqrt_nr(t) ;
            return _mm_mul_ps( vec , _mm_shuffle_ps(t,t,0x00) ) ;
        }

        /// Normalize this vector, i.e. make its length unity.
        void  Normalize()
        {
            F32vec4 r = _mm_mul_ps(vec,vec);
            F32vec1 t = _mm_add_ss(_mm_shuffle_ps(r,r,1), _mm_add_ss(_mm_movehl_ps(r,r),r)) ;
            t = _mm_cmpneq_ss(t, _mm_setzero_ps()) & rsqrt_nr(t) ;
            vec = _mm_mul_ps(vec, _mm_shuffle_ps(t,t,0x00)) ;
        }

        void  NormalizeFast() { Normalize() ; }

        /// Assign a vector to zero.
        void  Zero()                                    { x = y = z = 0.0f ; }

        bool IsNormalized( const float fTolerance = 0.00001f ) const
        {
            const float mag2 = Mag2() ;
            const float absDiff = fabsf( mag2 - 1.0f ) ;
            if( absDiff < fTolerance ) return true ;
            return false ;
        }

        bool Resembles( const Vec3 & other , const float fTolerance = 0.0001f ) const
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


        /** Return whether 3 points are colinear.

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


        /** Return whether 4 points are coplanar.

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

#else   // Generic version

/** Utility class for 3-vectors.
*/
struct Vec3 : public PeGaSys::Math::Vector3
{
    public:
        Vec3() {}

        /** Construct 3-vector from floats.
            \param fx - x component
            \param fy - y component
            \param fz - z component
        */
        Vec3( float fx , float fy , float fz ) { x = fx ; y = fy ; z = fz ; }

        /** Construct Vec3 from Vector3.

            \param vector3  Vector3 plain-old-data structure.

            Note that this is NOT "explicit" so this is a cast constructor.
            This allows Vector3 to be treated as a Vec3 when we want that,
            but the last of a constructor in Vector3 allows it to be present in
            unions.
        */
        Vec3( const PeGaSys::Math::Vector3 & vector3 )
        {
            x = vector3.x ; y = vector3.y ; z = vector3.z ;
        }

        /// Construct vector from array of floats.
        /// \param fa - array of at least 3 floats
        Vec3( const float * fa ) { x = fa[0] ; y = fa[1] ; z = fa[2] ; }

        // Assignment operators

        /// Add vector to self.
        Vec3 & operator += ( const Vec3 & rhs )          { x += rhs.x ; y += rhs.y ; z += rhs.z ; return * this ; }

        /// Subtract vector from self.
        Vec3 & operator -= ( const Vec3 & rhs )          { x -= rhs.x ; y -= rhs.y ; z -= rhs.z ; return * this ; }

        /// Multiply scalar to self.
        Vec3 & operator *= ( float f )                   { x *= f ; y *= f ; z *= f ; return * this ; }

        /// Divide self by scalar.
        Vec3 & operator /= ( float f )                   { (*this) *= ( 1.0f / f )  ; return * this ; }

        // Unary operators

        /// Unary plus -- does nothing.
        Vec3 operator + () const                         { return * this ; }

        /// unary minus -- return negative of this vector.
        Vec3 operator - () const                         { return Vec3( -x , -y , -z ) ; }

        // Binary operators

        /// add two vectors.
        Vec3 operator + ( const Vec3 & rhs ) const       { return Vec3( x + rhs.x , y + rhs.y , z + rhs.z ) ; }

        /// Subtract two vectors.
        Vec3 operator - ( const Vec3 & rhs ) const       { return Vec3( x - rhs.x , y - rhs.y , z - rhs.z ) ; }
        //Vec3 operator - ( const Vec3 & rhs ) const       ; // For CPU performance profiling. See util.cpp.

        /// Divide each component of this vector by each component of that vector.
        Vec3 operator / ( const Vec3 & rhs ) const       { return Vec3( x / rhs.x , y / rhs.y , z / rhs.z ) ; }

        /// Multiply vector by scalar.
        Vec3 operator * ( float f ) const                { return Vec3( x * f , y * f , z * f ) ; }

        /// Divide vector by scalar.
        Vec3 operator / ( float f ) const                { return (*this) * ( 1.0f / f ) ; }

        /// Multiply vector by scalar.
        friend Vec3 operator * ( float f , const Vec3 & v)  { return Vec3( v.x * f , v.y * f , v.z * f ) ; }

        /// Compare two vectors for equal values.
        bool operator == ( const Vec3 & rhs ) const      { return ( ( x == rhs.x ) && ( y == rhs.y ) && ( z == rhs.z ) ) ; }

        /// Compare two vectors for equal values and negate.
        bool operator != ( const Vec3 & rhs ) const      { return ( ( x != rhs.x ) || ( y != rhs.y ) || ( z != rhs.z ) ) ; }

        /// Determine whether all components of another vector strictly exceed those of this.
        bool operator < ( const Vec3 & rhs ) const       { return ( ( x < rhs.x ) && ( y < rhs.y ) && ( z < rhs.z ) ) ; }

        /// Determine whether all components of this vector strictly exceed those of another.
        bool operator > ( const Vec3 & rhs ) const       { return ( ( x > rhs.x ) && ( y > rhs.y ) && ( z > rhs.z ) ) ; }

        /// Determine whether all components of another vector exceed or equal those of this.
        bool operator <= ( const Vec3 & rhs ) const
        {
            return ( ( x <= rhs.x ) && ( y <= rhs.y ) && ( z <= rhs.z ) ) ;
        }
        /// Determine whether all components of this vector exceed or equal those of another.
        bool operator >= ( const Vec3 & rhs ) const
        {
            return ( ( x >= rhs.x ) && ( y >= rhs.y ) && ( z >= rhs.z ) ) ;
        }

        /// Compute inner (i.e. dot) product of two vectors.
        float operator*( const Vec3 & rhs ) const
        {
            return x * rhs.x + y * rhs.y + z * rhs.z ;
        }

        /// Compute outer (i.e. cross) product of two vectors.
        Vec3  operator^( const Vec3 & rhs ) const       { return Vec3( y * rhs.z - z * rhs.y , z * rhs.x - x * rhs.z , x * rhs.y - y * rhs.x ) ; }

        /// Return magnitude squared of a vector.
        float Mag2() const                              { return x * x + y * y + z * z ; }

        /// Return magnitude.
        float Magnitude() const                         { return sqrtf( Mag2() ) ; }

        /// Return magnitude.
        float MagnitudeFast() const                     { return fsqrtf( Mag2() ) ; }

        /// Return reciprocal magnitude.
        float ReciprocalMagnitude() const               { return 1.0f / sqrtf( Mag2() ) ; }
        float ReciprocalMagnitudeFast() const           { return finvsqrtf   ( Mag2() ) ; }

        /// Return a unit vector in the same direction as this vector.
        Vec3  GetDir() const                            { return (*this) * 1.0f / sqrtf( Mag2() + FLT_MIN ) ; }
        Vec3  GetDirFast() const                        { return (*this) * finvsqrtf( Mag2() + FLT_MIN ) ; }
        static Vec3 GetDir( const Vec3 & v )            { return v.GetDir() ; }
        static Vec3 GetDirFast( const Vec3 & v )        { return v.GetDirFast() ; }

        /// Normalize this vector, i.e. make its length unity.
        void  Normalize()                               { * this = GetDir()     ; }
        void  NormalizeFast()                           { * this = GetDirFast() ; }

        /// Assign a vector to zero.
        void  Zero()                                    { x = y = z = 0.0f ; }


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
        bool Resembles( const Vec3 & other , const float tolerance = 1.0e-4f ) const
        {
            return  Math::Resembles( x , other.x , tolerance )
                &&  Math::Resembles( y , other.y , tolerance )
                &&  Math::Resembles( z , other.z , tolerance ) ;
        }


        /** Return whether 3 points are colinear.

            \param v0 - point in space
            \param v1 - point in space
            \param v2 - point in space

            \return Whether 3 given points are colinear (within tolerance)
        */
        static bool IsColinear( const Vec3 & v0 , const Vec3 & v1 , const Vec3 & v2 , const float tolerance = 1.0e-6f )
        {
            // Use unit vectors since magnitudes are irrelevant yet scale the result, so tolerance would effectively depend on magnitude.
            const Vec3  v01     = GetDirFast( v1 - v0 ) ;
            const Vec3  v02     = GetDirFast( v2 - v0 ) ;
            const Vec3  cross   = v01 ^ v02 ;
            const float crosMag = cross.Mag2() ;    // area of parallelogram
            return crosMag < tolerance ;
        }


        /** Return whether 4 points are coplanar.

            \param v0 - point in space
            \param v1 - point in space
            \param v2 - point in space
            \param v3 - point in space

            \return Whether 4 given points are coplanar (within tolerance)
        */
        static bool IsCoplanar( const Vec3 & v0 , const Vec3 & v1 , const Vec3 & v2 , const Vec3 & v3 , const float tolerance = 1.0e-6f )
        {
            if( IsColinear( v0 , v1 , v2 ) )
            {   // If any 3 of the given points are colinear then these 4 points are colinear
                return true ;
            }
            // Use unit vectors since magnitudes are irrelevant yet scale the result, so tolerance would effectively depend on magnitude.
            const Vec3  v01     = GetDirFast( v1 - v0 ) ;
            const Vec3  v02     = GetDirFast( v2 - v0 ) ;
            const Vec3  v03     = GetDirFast( v3 - v0 ) ;
            const Vec3  cross   = v01 ^ v02 ;
            const float triple  = fabsf( cross * v03 ) ; // volume of parallelepiped
            return triple < tolerance ;
        }
} ;

#endif

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

/** Generate a pseudo-random Vec3 value whose components lie in the given range.

    \param vSpread  Range of values for each component.
                    The value for each component will lie between
                    -component/2 and +component/2.

*/
inline Vec3 RandomSpread( const Vec3 & vSpread )
{
    return  Vec3( RandomSpread( vSpread.x )
                , RandomSpread( vSpread.y )
                , RandomSpread( vSpread.z ) ) ;
}

namespace Math
{
    inline bool Resembles( const Vec3 & v1, const Vec3 & v2 , const float fTolerance = 0.0001f )
    {
        return v1.Resembles( v2 , fTolerance ) ;
    }
} ;

inline bool IsNan( const Vec3 & v )
{
    return IsNan( v.x ) || IsNan( v.y ) || IsNan( v.z ) ;
}

inline bool IsInf( const Vec3 & v )
{
    return IsInf( v.x ) || IsInf( v.y ) || IsInf( v.z ) ;
}

inline Vec3 Clamp0to1( const Vec3 & newValueToClamp )
{
    Vec3 returnValue ;
    returnValue.x = Clamp( newValueToClamp.x , 0.0f , 1.0f ) ;
    returnValue.y = Clamp( newValueToClamp.y , 0.0f , 1.0f ) ;
    returnValue.z = Clamp( newValueToClamp.z , 0.0f , 1.0f ) ;
    return returnValue ;
}

inline void AssignClamp0to1( Vec3 & varToAssign , const Vec3 & newValueToClamp )
{
    varToAssign.x = Clamp( newValueToClamp.x , 0.0f , 1.0f ) ;
    varToAssign.y = Clamp( newValueToClamp.y , 0.0f , 1.0f ) ;
    varToAssign.z = Clamp( newValueToClamp.z , 0.0f , 1.0f ) ;
}

#endif
