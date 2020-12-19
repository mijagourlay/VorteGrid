/** \file math.h

    \brief Mathematical utility routines

    \author Written and Copyright 2005-2013 MJG; All rights reserved.
*/
#ifndef MATH_H
#define MATH_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "Core/Utility/macros.h" // for ASSERT

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

typedef unsigned short WORD  ;

// Public variables ------------------------------------------------------------

#ifdef __cplusplus
static const float PI       = 3.1415926535897932384626433832795f ;
static const float TWO_PI   = 2.0f * PI ;
static const float HALF_PI  = 0.5f * 3.1415926535897932384626433832795f ;
#else
#define PI      3.1415926535897932384626433832795f
#define TWO_PI  ( 2.0f * PI )
#endif

static const float RAD2DEG  = (180.0f / PI) ;
static const float DEG2RAD  = (PI / 180.0f) ;

// Public functions ------------------------------------------------------------


/** Return whether given float is infinity.
*/
inline bool IsInf( const float & x )
{
#if defined( _MSCVER )
    return ! _finite( x ) ;
#else
    return ( ( x == x ) && ( ( x - x ) != 0.0f ) ) ;
#endif
}


/** Return whether given float is not-a-number.
*/
inline bool IsNan( const float & x )
{
#if defined( _MSCVER )
    return _isnan( x ) ;
#else
    return x != x ;
#endif
}


/// Return sign bit of floating point value.
inline int ISignBit( const float & f )
{
    return ( (int&) f & 0x80000000 ) >> 31 ;
}


/// Return sign (-1 or 1) of floating point value.
inline  float FSign( const float & f )
{
    float fReturnValue = 1.0f ;
    // Mask sign bit from f and set it in fReturnValue so that fReturnValue has the same sign
    (int&) fReturnValue |= ( ( (int&) f ) & 0x80000000 ) ;
    return fReturnValue ;
}


/// Assign sign of floating point value from the sign of another floating point value.
inline void FSetSign( float & dest , float const & source )
{
    int sign_mask = (int&) source & 0x80000000 ;
    (int&)dest &= 0x7FFFFFFF; // clear sign bit
    (int&)dest |= sign_mask; // set sign bit if necessary
}


/// Copy sign of floating point value from the sign of another floating point value.
inline float FCopySign( const float & absSource , const float & signSource )
{
    float returnValue = absSource ;
    FSetSign( returnValue , signSource ) ;
    return returnValue ;
}


/// Return absolute value of floating point value.
inline float FAbs( const float & f )
{
    int iReturnValue = (int&) f & 0x7FFFFFFF ; // clear sign bit
    return (float&) iReturnValue ;
}


/// Assign floating point value in-place.
inline void FAbsInPlace( float & f )
{
    (int&) f &= 0x7FFFFFFF ; // clear sign bit
}


/// Return the value raised to the second power.
template <class T> T Pow2( T x ) { return x * x ; }


/// Return the value raised to the third power.
template < class T >
T Pow3( const T & x )
{
    return x * x * x ;
}


namespace Math
{

    inline bool Resembles( const float f1 , const float f2 , const float tolerance = 1.0e-4f )
    {
        const float diff    = f1 - f2 ;
        const float absDiff = fabs( diff ) ;
        if( absDiff >= tolerance )
        {   // Absolute difference exceeds tolerance.
            // Use relative difference.
            return ( absDiff / fabs( f1 ) < tolerance ) ;
        }
        return ( absDiff < tolerance ) ;
    }

} ;




/** Compute nearest power of 2 greater than or equal to the given value.
    \param iVal - integer value
    \return Something like the logarithm-base-2 of iVal rounded up to the next integer
*/
inline size_t NearestPowerOfTwoExponent( size_t iVal )
{
    ASSERT( iVal >= 0 ) ;
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




/** Compute nearest power of 2 greater than or equal to the given value.

    \param iVal - integer value
*/
inline size_t NearestPowerOfTwo( size_t iVal )
{
    return 1U << NearestPowerOfTwoExponent( iVal ) ;
}




/** Generate a pseudo-random floating-point number.
    \param fSpread - range of output values from -fSpread/2 to +fSpread/2
*/
inline float RandomSpread( float fSpread )
{
    static const float  sOneOverRandMax = 1.0f / float( RAND_MAX ) ;
    const float         randomFloat     = fSpread * ( float( rand() ) * sOneOverRandMax - 0.5f ) ;
    return randomFloat ;
}




/** Fast reciprocal square root.

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




/// Macro version of fast inverse square root.
#define FINVSQRT( y , val )                 \
{                                           \
    long    i   = (long&) val ;             \
    i  = 0x5f3759df - ( i >> 1 ) ;          \
    y   = (float&) i ;                      \
    static const float  f   = 1.5f ;        \
    const float         x   = val * 0.5f ;  \
    y  = y * ( f - ( x * y * y ) ) ;        \
}




/** Fast reciprocal square root.

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




/** Fast reciprocal square root.

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




/** Fast square root.

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





/** Compute cosine of given angle in radians.
*/
inline float ImpreciseCosF_worker( float angleInRadiansZeroToHalfPi )
{
    static const float c1 =  0.99940307f ;
    static const float c2 = -0.49558072f ;
    static const float c3 =  0.03679168f ;
    const float x2 = angleInRadiansZeroToHalfPi * angleInRadiansZeroToHalfPi ;
    return c1 + x2 * ( c2 + c3 * x2 ) ;
}




inline float ImpreciseCosF( float angleInRadians )
{
    static float PI            = 3.1415926535897932384626433832795f ;
    static float TWO_PI        = 6.283185307179586476925286766559f  ;
    const float  angleInTwoPi  = fmod( angleInRadians , TWO_PI ) ;
    const float  positiveAngle = FAbs( angleInTwoPi ) ;
    static float HALF_PI       = 1.5707963267948966192313216916398f ;
    const int    quadrant      = int( positiveAngle / HALF_PI ) ;
    switch( quadrant )
    {
        case 0: return  ImpreciseCosF_worker( positiveAngle ) ;
        case 1: return -ImpreciseCosF_worker( PI - positiveAngle ) ;
        case 2: return -ImpreciseCosF_worker( positiveAngle - PI ) ;
        case 3: return  ImpreciseCosF_worker( TWO_PI - positiveAngle ) ;
    }
    FAIL() ;
    return  ImpreciseCosF_worker( positiveAngle ) ;
}




inline float ImpreciseSinF( float angleInRadians )
{
    static float HALF_PI = 1.5707963267948966192313216916398f ;
    return ImpreciseCosF( HALF_PI - angleInRadians ) ;
}




#if defined( WIN32 ) // && defined( __MACHINEI ) // Accelerated for x86

/** Set multimedia extended control status register.

    \return Previous value of MMX Control Status Register.
*/
inline unsigned SetMmxControlStatusRegister( unsigned newMxcsrVal )
{
    unsigned int oldMxcsrVal ;
    _asm {
        STMXCSR oldMxcsrVal
        LDMXCSR newMxcsrVal
    }
    return oldMxcsrVal ;
}




/** Get MultiMedia eXtension control status register.
*/
inline unsigned int GetMmxControlStatusRegister()
{
    unsigned mxcsrVal ;
    _asm {
        STMXCSR mxcsrVal
    }
    return mxcsrVal ;
}




/** Flush denormals to zero.

    This can make floating point operations much faster when
    values approach very close to zero.
*/
inline unsigned int XMM_Set_FlushToZero_DenormalAreZero( void )
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




/** Get floating-point control word.

    \return Value of floating-point control word.
*/
inline WORD GetFloatingPointControlWord()
{
    WORD ctrlWord ;
    __asm {
        fnstcw ctrlWord
    }
    return ctrlWord ;
}




/** Return whether the floating point control word is set to truncate when converting float to int.

    \see Changex87FloatingPointToTruncate.
*/
inline bool FpcwTruncates()
{
    static const WORD FPU_CW_ROUND_MASK = 0x0c00 ;
    static const WORD FPU_CW_ROUND_NEAR = 0x0000 ;
    static const WORD FPU_CW_ROUND_DOWN = 0x0400 ;
    static const WORD FPU_CW_ROUND_UP   = 0x0800 ;
    static const WORD FPU_CW_ROUND_CHOP = 0x0c00 ;

    return ( GetFloatingPointControlWord() & FPU_CW_ROUND_MASK ) == FPU_CW_ROUND_CHOP ;
}




/** Change the x87 floating-point coprocessor mode to truncation.

    By default, the x87 coprocessor rounds floats to ints
    when executing the store-float-as-int instruction (fistp).
    That behavior is incompatible with standard C behavior,
    so this instruction changes the floating-point control
    register such that fistp truncates instead of rounding.

    \return Value of floating-point control register prior to calling this routine

    \see Comments in StoreFloatAsInt for an explanation of when and why
            you would want to call this routine.

    \see SetFloatingPointControlWord to restore floating-point settings

    \see FpcwTruncates.
*/
inline WORD Changex87FloatingPointToTruncate( void )
{
    WORD NewCtrlWord , OldCtrlWord ;
    _asm {
        fnstcw OldCtrlWord
        mov    ax          , OldCtrlWord
        or     ax          , 0c00h
        mov    NewCtrlWord , ax
        fldcw  NewCtrlWord
    }
    ASSERT( FpcwTruncates() ) ;
    return OldCtrlWord ;
}




/** Enable many floating-point exceptions.

    Use this to find floating-point exceptions that
    are normally ignored.  These can indicate problems,
    including performance and accuracy.

    \return Original value of floating-point control register.

    \see SetFloatingPointControlWord to restore floating-point control to what it was before calling this routine
*/
inline WORD Unmaskx87FpExceptions( void )
{
    //const unsigned exceptionsMask = EM_OVERFLOW | EM_UNDERFLOW | EM_ZERODIVIDE | EM_DENORMAL | EM_INVALID  ;
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




/** Set floating-point control word.

    \return Value of floating-point control word from before calling this routine

*/
inline WORD SetFloatingPointControlWord( WORD NewCtrlWord )
{
    WORD OldCtrlWord ;
    __asm {
        fnstcw OldCtrlWord
        fldcw  NewCtrlWord
    }
    return OldCtrlWord ;
}




static const WORD PRECISION_SINGLE   = 0x0000 ; ///< Bit mask for enabling single precision via floating point control word.
static const WORD PRECISION_DOUBLE   = 0x0200 ; ///< Bit mask for enabling double precision via floating point control word.
static const WORD PRECISION_EXTENDED = 0x0400 ; ///< Bit mask for enabling extended precision via floating point control word.

/** Set floating-point precision.

    \return Value of floating-point control word from before calling this routine

*/
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




/** Store a floating-point value as an integer.

    This runs faster than using the C-style float-to-int cast, but to obtain
    identical behavior requires calling Changex87FloatingPointToTruncate first.

    Compiler-generated float-to-int runs slower because C-style float-to-int has
    a policy of truncating whereas the CPU typically has the policy of rounding,
    so the compiler emits fnstcw instructions before and after each fistp instruction.
    fnstcw causes the instruction pipeline to stall so all instructions in the
    pipeline flush out, because changing the Floating Point Control Word (FPCW)
    with instructions in flight would cause nonsensical results.
    The fistp instruction isn't slow -- the pipeline flushes surrounding it are.

    This routine simply performs the same float-to-int instructions that the compiler
    would generate, minus the fnstcw.  The caller must therefore set the FPWC
    to whatever mode it wants, before calling this.

    The intended use case is that this routine would get called inside a loop,
    whereas the fnstcw would get called outside the loop:

    \example
        const WORD OldCtrlWord = Changex87FloatingPointToTruncate() ; // Save off original FPCW and change mode to "truncate".
        for( each floating point value )
        {
            StoreFloatAsInt( value ) ;  // Convert float to int without changing FPCW.
        }
        SetFloatingPointControlWord( OldCtrlWord ) ;  // Restore FPCW.

    \see Changex87FloatingPointToTruncate, SetFloatingPointControlWord
*/
inline int StoreFloatAsInt( float a )
{
    int retVal ;
    __asm fld   a      ;
    __asm fistp retVal ;
    return retVal ;
}




/** Approximate float-to-int conversion.

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

    /// A global function to find MAX(a,b) using FCOMI/FCMOV.
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

    /// A global function to find MIN(a,b) using FCOMI/FCMOV.
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

#endif

#if defined( _DEBUG )
extern void Math_UnitTest() ;
#endif


#endif
