/*! \file mat4.h

    \brief 4x4 matrix class

    \author Copyright 2005-2006 UCF/FIEA/MJG; All rights reserved.
*/
#ifndef MAT4_H
#define MAT4_H

#include <math.h>

#include "float.h"
#include "vec4.h"
#include "vec3.h"

// Macros --------------------------------------------------------------

//! \brief square value
#define POW2( x ) ( ( x ) * ( x ) )

// Types --------------------------------------------------------------

/*! \brief Utility class for 4x4 matrices
*/
struct Mat4
{
    public:
        Mat4() {}
        Mat4(    float m0, float m1, float m2 , float m3 ,
                        float m4, float m5, float m6 , float m7 ,
                        float m8, float m9, float ma , float mb ,
                        float mc, float md, float me , float mf )
        {
            m[0][0] = m0 ; m[0][1] = m1 ; m[0][2] = m2 ; m[0][3] = m3 ;
            m[1][0] = m4 ; m[1][1] = m5 ; m[1][2] = m6 ; m[1][3] = m7 ;
            m[2][0] = m8 ; m[2][1] = m9 ; m[2][2] = ma ; m[2][3] = mb ;
            m[3][0] = mc ; m[3][1] = md ; m[3][2] = me ; m[3][3] = mf ;
        }



        // binary operators
        Mat4 operator + ( const Mat4 & rhs ) const
        {
            return Mat4(    m[0][0] + rhs.m[0][0] , m[0][1] + rhs.m[0][1] , m[0][2] + rhs.m[0][2] , m[0][3] + rhs.m[0][3] ,
                            m[1][0] + rhs.m[1][0] , m[1][1] + rhs.m[1][1] , m[1][2] + rhs.m[1][2] , m[1][3] + rhs.m[1][3] ,
                            m[2][0] + rhs.m[2][0] , m[2][1] + rhs.m[2][1] , m[2][2] + rhs.m[2][2] , m[2][3] + rhs.m[2][3] ,
                            m[3][0] + rhs.m[3][0] , m[3][1] + rhs.m[3][1] , m[3][2] + rhs.m[3][2] , m[3][3] + rhs.m[3][3] ) ;
        }


        Mat4 operator * ( float f ) const
        {
            return Mat4(    m[0][0] * f , m[0][1] * f , m[0][2] * f , m[0][3] * f ,
                            m[1][0] * f , m[1][1] * f , m[1][2] * f , m[1][3] * f ,
                            m[2][0] * f , m[2][1] * f , m[2][2] * f , m[2][3] * f ,
                            m[3][0] * f , m[3][1] * f , m[3][2] * f , m[3][3] * f ) ;
        }


        bool operator == ( const Mat4 & rhs ) const
        {
            return (    ( m[0][0] == rhs.m[0][0] ) && ( m[0][1] == rhs.m[0][1] ) && ( m[0][2] == rhs.m[0][2] ) && ( m[0][3] == rhs.m[0][3] ) &&
                        ( m[1][0] == rhs.m[1][0] ) && ( m[1][1] == rhs.m[1][1] ) && ( m[1][2] == rhs.m[1][2] ) && ( m[1][3] == rhs.m[1][3] ) &&
                        ( m[2][0] == rhs.m[2][0] ) && ( m[2][1] == rhs.m[2][1] ) && ( m[2][2] == rhs.m[2][2] ) && ( m[2][3] == rhs.m[2][3] ) &&
                        ( m[3][0] == rhs.m[3][0] ) && ( m[3][1] == rhs.m[3][1] ) && ( m[3][2] == rhs.m[3][2] ) && ( m[3][3] == rhs.m[3][3] )    ) ;
        }

        bool operator != ( const Mat4 & rhs ) const      { return ! ( (*this) == rhs ) ; }

        float m[4][4] ;
} ;

// Public variables --------------------------------------------------------------

static const Mat4 Mat4_xIdentity(   1.0f , 0.0f , 0.0f , 0.0f ,
                                    0.0f , 1.0f , 0.0f , 0.0f ,
                                    0.0f , 0.0f , 1.0f , 0.0f ,
                                    0.0f , 0.0f , 0.0f , 1.0f ) ;

// Public functions --------------------------------------------------------------

#endif
