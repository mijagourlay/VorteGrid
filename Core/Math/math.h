/*! \file math.h

    \brief Mathematical utility routines

    \author Copyright 2005-2009 UCF/FIEA/MJG; All rights reserved.
*/
#ifndef MATH_H
#define MATH_H

#include <stdio.h>
#include "wrapperMacros.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------


/*! \brief Return whether given float is infinity
*/
inline bool IsInf( const float & x )
{
    return ( ( x - x ) != 0.0f ) ;
}


/*! \brief Return whether given float is not-a-number
*/
inline bool IsNan( const float & x )
{
    // If x is a real number then it must be either less-than 0 or greather-than-or-equal-to 0.
    // If x is neither then it is Not A Number (NAN).
    return ( ( ! ( x < 0.0f ) ) && ( ! ( x >= 0.0f ) ) ) ;
}


//! \brief Return sign bit of floating point value
inline int ISignBit( const float & f )
{
    return ( (int&) f & 0x80000000 ) >> 31 ;
}


//! \brief Return sign (-1 or 1) of floating point value
inline  float FSign( const float & f )
{
    float fReturnValue = 1.0f ;
    // Mask sign bit from f and set it in fReturnValue so that fReturnValue has the same sign
    (int&) fReturnValue |= ( ( (int&) f ) & 0x80000000 ) ;
    return fReturnValue ;
}


//! \brief Multiply floating point value by the sign of another floating point value
inline void FMultSign( float & dest , float const & source )
{
    // NOTE: For transfering sign from source to dest, it seems like this first line could be simpler:
    // int sign_mask = (int&) source & 0x80000000 ;
    int sign_mask = ((int&)dest ^ (int&)source) & 0x80000000; // XOR and mask
    (int&)dest &= 0x7FFFFFFF; // clear sign bit
    (int&)dest |= sign_mask; // set sign bit if necessary
}


//! \brief Assign sign of floating point value from the sign of another floating point value
inline void FSetSign( float & dest , float const & source )
{
    int sign_mask = (int&) source & 0x80000000 ;
    (int&)dest &= 0x7FFFFFFF; // clear sign bit
    (int&)dest |= sign_mask; // set sign bit if necessary
}


//! \brief Return absolute value of floating point value
inline float FAbs( const float & f )
{
    int iReturnValue = (int&) f & 0x7FFFFFFF ; // clear sign bit
    return (float&) iReturnValue ; // clear sign bit
}


//! \brief Assign floating point value in-place
inline void FAbsInPlace( float & f )
{
    (int&) f &= 0x7FFFFFFF ; // clear sign bit
}




/*! \brief Fast reciprocal square root

    \note This assumes "float" uses IEEE 754 format.

    \see Paul Hsieh's Square Root page: http://www.azillionmonkeys.com/qed/sqroot.html

    \see Charles McEniry (2007): The mathematics behind the fast inverse square root function code

    \see Chris Lomont: Fast inverse square root
*/
inline float finvsqrtf(const float & val )
{
    long    i   = (long&) val ;             // Exploit IEEE 754 inner workings.
    i  = 0x5f3759df - ( i >> 1 ) ;          // From Taylor's theorem and IEEE 754 format.
    float   y   = (float&) i ;              // Estimate of 1/sqrt(val) close enough for convergence using Newton's method.
    static const float  f   = 1.5f ;        // Derived from Newton's method.
    const float         x   = val * 0.5f ;  // Derived from Newton's method.
    y  = y * ( f - ( x * y * y ) ) ;        // Newton's method for 1/sqrt(val)
    y  = y * ( f - ( x * y * y ) ) ;        // Another iteration of Newton's method
    return y ;
}




/*! \brief Fast square root

    This computes val/sqrt(val) (which is sqrt(val)) so uses the 1/sqrt formula of finvsqrtf.

    \note This assumes "float" uses IEEE 754 format.

    \see Paul Hsieh's Square Root page: http://www.azillionmonkeys.com/qed/sqroot.html

    \see Charles McEniry (2007): The mathematics behind the fast inverse square root function code

    \see Chris Lomont: Fast inverse square root
*/
inline float fsqrtf( const float & val )
{
    long    i   = (long&) val ;             // Exploit IEEE 754 inner workings.
    i  = 0x5f3759df - ( i >> 1 ) ;          // From Taylor's theorem and IEEE 754 format.
    float   y   = (float&) i ;              // Estimate of 1/sqrt(val) close enough for convergence using Newton's method.
    static const float  f   = 1.5f ;        // Derived from Newton's method.
    const float         x   = val * 0.5f ;  // Derived from Newton's method.
    y  = y * ( f - ( x * y * y ) ) ;        // Newton's method for 1/sqrt(val)
    y  = y * ( f - ( x * y * y ) ) ;        // Another iteration of Newton's method
    return val * y ;                        // Return val / sqrt(val) which is sqrt(val)
}




/*! \brief compute nearest power of 2 greater than or equal to the given value
*/
inline unsigned int NearestPowerOfTwoExponent( unsigned int iVal )
{
    if( 0 == iVal )
    {
        return 0 ;
    }
    -- iVal ; // exact powers of two otherwise produce the wrong result below
    unsigned int shift = 0 ;
    while( ( iVal >> shift ) != 0 )
    {
        ++ shift ;
    }
    return shift ;
}




/*! \brief Compute nearest power of 2 greater than or equal to the given value
*/
inline unsigned int NearestPowerOfTwo( unsigned int iVal )
{
    return 1 << NearestPowerOfTwoExponent( iVal ) ;
}




/*! \brief  Generate a pseudo-random floating-point number
*/
inline float RandomSpread( float fSpread )
{
    const float randomFloat = fSpread * ( float( rand() ) / float( RAND_MAX ) - 0.5f ) ;
    return randomFloat ;
}



#if defined( _DEBUG )
extern void Math_UnitTest( void ) ;
#endif

#endif
