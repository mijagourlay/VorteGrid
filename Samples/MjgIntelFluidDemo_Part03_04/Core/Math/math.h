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

typedef unsigned short WORD  ;

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
    return (float&) iReturnValue ;
}


//! \brief Assign floating point value in-place
inline void FAbsInPlace( float & f )
{
    (int&) f &= 0x7FFFFFFF ; // clear sign bit
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




/*! \brief Fast reciprocal square root

    \note Careful profiling tests in optimized builds
            show that this actually runs slower than 1/sqrt(x).

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
    //y  = y * ( f - ( x * y * y ) ) ;        // Another iteration of Newton's method
    return y ;
}




#define FINVSQRT( y , val )                 \
{                                           \
    long    i   = (long&) val ;             \
    i  = 0x5f3759df - ( i >> 1 ) ;          \
    y   = (float&) i ;                      \
    static const float  f   = 1.5f ;        \
    const float         x   = val * 0.5f ;  \
    y  = y * ( f - ( x * y * y ) ) ;        \
}




/*! \brief Fast reciprocal square root

    \see Gerber (2002): The Software Optimization Cookbook

*/
inline float FastInvSqrt( const float & val )
{
    static const unsigned OneAsInt          = 0x3f800000    ; // The value 1.0 in IEEE 754 format.
    static const unsigned OneAsIntShifted   = OneAsInt << 1 ; // ...shifted left
    static const unsigned MagicNumber       = OneAsIntShifted + OneAsInt ; // ...magic
    long tmp = ( MagicNumber - *(long*) & val ) >> 1 ;
    float y = *(float*) & tmp ;
    return y * ( 1.47f - 0.47f * val * y * y ) ;
}




/*! \brief Fast reciprocal square root

    \see Gerber (2002): The Software Optimization Cookbook

    \note This is meant for use in comparison with FastInvSqrt, which is inlined.
            Theoretically, the inlined version and this macro should perform about the same.

*/
#define FAST_INV_SQRT( y , val )                                            \
{                                                                           \
    static const unsigned OneAsInt          = 0x3f800000    ;               \
    static const unsigned OneAsIntShifted   = OneAsInt << 1 ;               \
    static const unsigned MagicNumber       = OneAsIntShifted + OneAsInt ;  \
    long tmp = ( MagicNumber - *(long*) & val ) >> 1 ;                      \
    y = *(float*) & tmp ;                                                   \
    y = y * ( 1.47f - 0.47f * val * y * y ) ;                               \
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
    //y  = y * ( f - ( x * y * y ) ) ;        // Another iteration of Newton's method
    return val * y ;                        // Return val / sqrt(val) which is sqrt(val)
}




/*! \brief Flush Denormals to Zero
*/
inline unsigned int XMM_SetFlushToZeroDenormalAreZero( void )
{
    unsigned int oldMxcsrVal , newMxcsrVal ;
    _asm {
        STMXCSR oldMxcsrVal
        mov eax , oldMxcsrVal
        // flush-to-zero = bit 15
        // mask underflow = bit 11
        // denormals are zero = bit 6
        or eax , 08840h
        mov newMxcsrVal , eax
        LDMXCSR newMxcsrVal
    }
    return oldMxcsrVal ;
}




/*! \brief Change the x87 floating-point coprocessor mode to truncation

    By default, the x87 coprocessor rounds floats to ints
    when executing the store-float-as-int instruction (fistp).
    That behavior is incompatible with standard C behavior,
    so this instruction changes the floating-point control
    register such that fistp truncates instead of rounding.

    \return Value of floating-point control register prior to calling this routine

    \see Setx87ControlWord to restore floating-point settings
*/
inline WORD Changex87FpToTrunc( void )
{
    WORD NewCtrlWord , OldCtrlWord ;
    _asm {
        fnstcw OldCtrlWord
        mov    ax          , OldCtrlWord
        or     ax          , 0c00h
        mov    NewCtrlWord , ax
        fldcw  NewCtrlWord
    }
    return OldCtrlWord ;
}




/*! \brief Enable many floating-point exceptions

    Use this to find floating-point exceptions that
    are normally ignored.  These can indicate problems,
    including performance and accuracy.

    \return Original value of floating-point control register.

    \see Setx87ControlWord to restore floating-point control to what it was before calling this routine
*/
inline WORD Unmaskx87FpExceptions( void )
{
    //WORD exceptionsMask = EM_OVERFLOW | EM_UNDERFLOW | EM_ZERODIVIDE | EM_DENORMAL | EM_INVALID  ;
    WORD OldCtrlWord ;
    WORD NewCtrlWord ;
    __asm {
        FSTCW   OldCtrlWord
        mov     ax          , OldCtrlWord
        and     ax          , 0ffe1h
        mov     NewCtrlWord , ax
        FLDCW   NewCtrlWord
    }
    return OldCtrlWord ;
}




/*! \brief Set floating-point control word

    \return Value of floating-point control word from before calling this routine

*/
inline WORD Setx87ControlWord( WORD NewCtrlWord )
{
    WORD OldCtrlWord ;
    __asm {
        fnstcw OldCtrlWord
        fldcw  NewCtrlWord
    }
    return OldCtrlWord ;
}




/*! \brief Set floating-point precision

    \return Value of floating-point control word from before calling this routine

*/
static const WORD PRECISION_SINGLE   = 0x0000 ;
static const WORD PRECISION_DOUBLE   = 0x0200 ;
static const WORD PRECISION_EXTENDED = 0x0400 ;
inline WORD Setx87Precision( WORD precision )
{
    WORD OldCtrlWord , NewCtrlWord ;
    __asm {
        FSTCW OldCtrlWord
        mov ax , OldCtrlWord
        and ax , 0fcffh
        or ax , precision
        mov NewCtrlWord , ax
        FLDCW NewCtrlWord
    }
    return OldCtrlWord ;
}




/*! \brief Store a floating-point value as an integer

    This runs faster than using the C-style float-to-int cast,
    but to obtain identical behavior requires calling Changex87FpToTrunc
    first.

*/
inline int StoreFloatAsInt( float a )
{
    int retVal ;
    __asm fld   a      ;
    __asm fistp retVal ;
    return retVal ;
}




/*! \brief Approximate float-to-int conversion

    Supposedly faster than C-style cast, but
    MJG found the results incompatible for his needs.

*/
inline int FastIntFromFloatApproximate( float f )
{
#define FLOAT_FTOI_MAGIC_NUM (float) (3<<21)
#define IT_FTOI_MAGIC_NUM 0x4ac00000
    f += FLOAT_FTOI_MAGIC_NUM ;
    return (*((int*)&f) - IT_FTOI_MAGIC_NUM) >> 1 ;
}




	#define FCOMI_ST0	_asm	_emit	0xdb	_asm	_emit	0xf0
	#define FCOMIP_ST0	_asm	_emit	0xdf	_asm	_emit	0xf0
	#define FCMOVB_ST0	_asm	_emit	0xda	_asm	_emit	0xc0
	#define FCMOVNB_ST0	_asm	_emit	0xdb	_asm	_emit	0xc0

	#define FCOMI_ST1	_asm	_emit	0xdb	_asm	_emit	0xf1
	#define FCOMIP_ST1	_asm	_emit	0xdf	_asm	_emit	0xf1
	#define FCMOVB_ST1	_asm	_emit	0xda	_asm	_emit	0xc1
	#define FCMOVNB_ST1	_asm	_emit	0xdb	_asm	_emit	0xc1

	#define FCOMI_ST2	_asm	_emit	0xdb	_asm	_emit	0xf2
	#define FCOMIP_ST2	_asm	_emit	0xdf	_asm	_emit	0xf2
	#define FCMOVB_ST2	_asm	_emit	0xda	_asm	_emit	0xc2
	#define FCMOVNB_ST2	_asm	_emit	0xdb	_asm	_emit	0xc2

	#define FCOMI_ST3	_asm	_emit	0xdb	_asm	_emit	0xf3
	#define FCOMIP_ST3	_asm	_emit	0xdf	_asm	_emit	0xf3
	#define FCMOVB_ST3	_asm	_emit	0xda	_asm	_emit	0xc3
	#define FCMOVNB_ST3	_asm	_emit	0xdb	_asm	_emit	0xc3

	#define FCOMI_ST4	_asm	_emit	0xdb	_asm	_emit	0xf4
	#define FCOMIP_ST4	_asm	_emit	0xdf	_asm	_emit	0xf4
	#define FCMOVB_ST4	_asm	_emit	0xda	_asm	_emit	0xc4
	#define FCMOVNB_ST4	_asm	_emit	0xdb	_asm	_emit	0xc4

	#define FCOMI_ST5	_asm	_emit	0xdb	_asm	_emit	0xf5
	#define FCOMIP_ST5	_asm	_emit	0xdf	_asm	_emit	0xf5
	#define FCMOVB_ST5	_asm	_emit	0xda	_asm	_emit	0xc5
	#define FCMOVNB_ST5	_asm	_emit	0xdb	_asm	_emit	0xc5

	#define FCOMI_ST6	_asm	_emit	0xdb	_asm	_emit	0xf6
	#define FCOMIP_ST6	_asm	_emit	0xdf	_asm	_emit	0xf6
	#define FCMOVB_ST6	_asm	_emit	0xda	_asm	_emit	0xc6
	#define FCMOVNB_ST6	_asm	_emit	0xdb	_asm	_emit	0xc6

	#define FCOMI_ST7	_asm	_emit	0xdb	_asm	_emit	0xf7
	#define FCOMIP_ST7	_asm	_emit	0xdf	_asm	_emit	0xf7
	#define FCMOVB_ST7	_asm	_emit	0xda	_asm	_emit	0xc7
	#define FCMOVNB_ST7	_asm	_emit	0xdb	_asm	_emit	0xc7

	//! A global function to find MAX(a,b) using FCOMI/FCMOV
	inline float FCMax2(float a, float b)
	{
#ifdef _MSC_VER
		float Res;
		_asm	fld		[a]
		_asm	fld		[b]
        _asm fcomi st,st(1)
        _asm fcmovb      st,st(1)
		_asm	fstp	[Res]
		_asm	fcomp
		return Res;
#else
		return (a > b) ? a : b;
#endif
	}

	//! A global function to find MIN(a,b) using FCOMI/FCMOV
	inline float FCMin2(float a, float b)
	{
#ifdef _MSC_VER
		float Res;
		_asm	fld		[a]
		_asm	fld		[b]
		FCOMI_ST1
		FCMOVNB_ST1
		_asm	fstp	[Res]
		_asm	fcomp
		return Res;
#else
		return (a < b) ? a : b;
#endif
	}




#if defined( _DEBUG )
extern void Math_UnitTest( void ) ;
#endif

#endif
