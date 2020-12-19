/*! \file mat4.h

    \brief 4x4 matrix class

    \author Copyright 2005-2006 MJG; All rights reserved.
*/
#ifndef MAT4_H
#define MAT4_H

#include <math.h>

//#include "Core/Utility/macros.h"
#include "float.h"
#include "vec4.h"
#include "vec3.h"
#include "mat33.h"

// Macros ----------------------------------------------------------------------

//! \brief square value
#define POW2( x ) ( ( x ) * ( x ) )

//! \brief Cast an array of floats to a Mat4
#define M4( m4 ) (*(Mat4*)m4)

// Types -----------------------------------------------------------------------

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

        // assignment operators
        Mat4 & operator += ( const Mat4 & rhs )          { (*this) = (*this) + rhs ; return * this ; }
        Mat4 & operator -= ( const Mat4 & rhs )          { (*this) = (*this) + (-1.0f * rhs) ; return * this ; }
        Mat4 & operator *= ( float f )                   { (*this) = (*this) * f ; return * this ; }

        // unary operators
        Mat4 operator + () const                         { return * this ; }
        Mat4 operator - () const                         { return -1.0f * (*this) ; }

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


        //  Mat4 operator / ( float f ) const                { return f * ( ~(*this) ) ; }

        Mat4 operator - ( const Mat4 & rhs ) const       { return (*this) + ( -1.0f * rhs ) ; }

        Mat4 operator / ( float f ) const                { return (*this) * ( 1.0f / f ) ; }

        friend Mat4 operator * ( float f , const Mat4 & m4 )    { return m4 * f ; }


        /*! \brief Transform a vector by this matrix
            \param v4 - Vector to transform
        */
        Vec4 operator* ( const Vec4 & v4 ) const
        {
            return Vec4(    v4.x * m[0][0] + v4.y * m[1][0] + v4.z * m[2][0] + v4.w * m[3][0] ,
                            v4.x * m[0][1] + v4.y * m[1][1] + v4.z * m[2][1] + v4.w * m[3][1] ,
                            v4.x * m[0][2] + v4.y * m[1][2] + v4.z * m[2][2] + v4.w * m[3][2] ,
                            v4.x * m[0][3] + v4.y * m[1][3] + v4.z * m[2][3] + v4.w * m[3][3] ) ;
        }


        /*! \brief Transform a vector by the transpose of a matrix
            \param v4 - Vector to transform
            \param m4 - Transformation matrix
        */
        static friend Vec4 operator* ( const Vec4 & v4 , const Mat4 & m4 )
        {
            return Vec4(    v4.x * m4.m[0][0] + v4.y * m4.m[0][1] + v4.z * m4.m[0][2] + v4.w * m4.m[0][3] ,
                            v4.x * m4.m[1][0] + v4.y * m4.m[1][1] + v4.z * m4.m[1][2] + v4.w * m4.m[1][3] ,
                            v4.x * m4.m[2][0] + v4.y * m4.m[2][1] + v4.z * m4.m[2][2] + v4.w * m4.m[2][3] ,
                            v4.x * m4.m[3][0] + v4.y * m4.m[3][1] + v4.z * m4.m[3][2] + v4.w * m4.m[3][3] ) ;
        }


        /*! \brief Transform a vector by the upper 3x3 (rotation and scale part) of a transformation matrix.
        */
        static Vec4 TransformNormal( const Vec4 & v4 , const Mat4 & m4 )
        {
            return Vec4(    v4.x * m4.m[0][0] + v4.y * m4.m[1][0] + v4.z * m4.m[2][0] ,
                            v4.x * m4.m[0][1] + v4.y * m4.m[1][1] + v4.z * m4.m[2][1] ,
                            v4.x * m4.m[0][2] + v4.y * m4.m[1][2] + v4.z * m4.m[2][2] ,
                            0.0f ) ;
        }


        /*! \brief Transform a vector by the transpose of the upper 3x3 (rotation and scale part) of a transformation matrix.
        */
        static Vec4 TransformNormalByTranspose( const Vec4 & v4 , const Mat4 & m4 )
        {
            return Vec4(    v4.x * m4.m[0][0] + v4.y * m4.m[0][1] + v4.z * m4.m[0][2] ,
                            v4.x * m4.m[1][0] + v4.y * m4.m[1][1] + v4.z * m4.m[1][2] ,
                            v4.x * m4.m[2][0] + v4.y * m4.m[2][1] + v4.z * m4.m[2][2] ,
                            0.0f ) ;
        }


        /*! \brief Transform a vector by the transpose of the upper 3x3 (rotation and scale part) of a transformation matrix.
        */
        static Vec3 TransformNormalByTranspose( const Vec3 & v3 , const Mat4 & m4 )
        {
            return Vec3(    v3.x * m4.m[0][0] + v3.y * m4.m[0][1] + v3.z * m4.m[0][2] ,
                            v3.x * m4.m[1][0] + v3.y * m4.m[1][1] + v3.z * m4.m[1][2] ,
                            v3.x * m4.m[2][0] + v3.y * m4.m[2][1] + v3.z * m4.m[2][2]  ) ;
        }


        /*! \brief Transform a vector by the upper 3x3 (rotation and scale part) of a transformation matrix.
        */
        static Vec3 TransformNormal( const Vec3 & v3 , const Mat4 & m4 )
        {
            return Vec3(    v3.x * m4.m[0][0] + v3.y * m4.m[1][0] + v3.z * m4.m[2][0] ,
                            v3.x * m4.m[0][1] + v3.y * m4.m[1][1] + v3.z * m4.m[2][1] ,
                            v3.x * m4.m[0][2] + v3.y * m4.m[1][2] + v3.z * m4.m[2][2] ) ;
        }




        bool operator == ( const Mat4 & rhs ) const
        {
            return (    ( m[0][0] == rhs.m[0][0] ) && ( m[0][1] == rhs.m[0][1] ) && ( m[0][2] == rhs.m[0][2] ) && ( m[0][3] == rhs.m[0][3] ) &&
                        ( m[1][0] == rhs.m[1][0] ) && ( m[1][1] == rhs.m[1][1] ) && ( m[1][2] == rhs.m[1][2] ) && ( m[1][3] == rhs.m[1][3] ) &&
                        ( m[2][0] == rhs.m[2][0] ) && ( m[2][1] == rhs.m[2][1] ) && ( m[2][2] == rhs.m[2][2] ) && ( m[2][3] == rhs.m[2][3] ) &&
                        ( m[3][0] == rhs.m[3][0] ) && ( m[3][1] == rhs.m[3][1] ) && ( m[3][2] == rhs.m[3][2] ) && ( m[3][3] == rhs.m[3][3] )    ) ;
        }

        bool operator != ( const Mat4 & rhs ) const      { return ! ( (*this) == rhs ) ; }

        Mat4 operator*( const Mat4 & rhs ) const    // Matrix-matrix multiplication
        {
            return Mat4(    RowDotCol(0, rhs, 0) , RowDotCol(0, rhs, 1) , RowDotCol(0, rhs, 2) , RowDotCol(0, rhs, 3) ,
                            RowDotCol(1, rhs, 0) , RowDotCol(1, rhs, 1) , RowDotCol(1, rhs, 2) , RowDotCol(1, rhs, 3) ,
                            RowDotCol(2, rhs, 0) , RowDotCol(2, rhs, 1) , RowDotCol(2, rhs, 2) , RowDotCol(2, rhs, 3) ,
                            RowDotCol(3, rhs, 0) , RowDotCol(3, rhs, 1) , RowDotCol(3, rhs, 2) , RowDotCol(3, rhs, 3) ) ;
        }


        bool Resembles( const Mat4 & that , float tolerance = 1.0e-4f ) const
        {
            const float * thisM = (const float *)   this ;
            const float * thatM = (const float *) & that ;
            for( size_t i = 0 ; i < 16 ; ++ i )
            {
                const float elementDiff = thisM[ i ] - thatM[ i ] ;
                const float absDiff = fabsf( elementDiff ) ;
                if( absDiff > tolerance )
                {
                    return false ;
                }
            }
            return true ;
        }

        Mat4 GetTranspose() const
        {
            return Mat4(    m[0][0] , m[1][0] , m[2][0] , m[3][0] ,
                            m[0][1] , m[1][1] , m[2][1] , m[3][1] ,
                            m[0][2] , m[1][2] , m[2][2] , m[3][2] ,
                            m[0][3] , m[1][3] , m[2][3] , m[3][3] ) ;
        }


        /*! \brief Assign scaling matrix
            \param scaleX - x component of scale
            \param scaleY - y component of scale
            \param scaleZ - z component of scale
        */
        void SetScale( float scaleX , float scaleY , float scaleZ )
        {
            m[0][0] = scaleX ; m[0][1] =   0.0f ; m[0][2] =   0.0f ; m[0][3] = 0.0f ;
            m[1][0] =   0.0f ; m[1][1] = scaleY ; m[1][2] =   0.0f ; m[1][3] = 0.0f ;
            m[2][0] =   0.0f ; m[2][1] =   0.0f ; m[2][2] = scaleZ ; m[2][3] = 0.0f ;
            m[3][0] =   0.0f ; m[3][1] =   0.0f ; m[3][2] =   0.0f ; m[3][3] = 1.0f ;
        }


        /*! \brief Assign scaling matrix
            \param scale - scale vector
        */
        void SetScale( const Vec3 & vScale )
        {
            m[0][0] = vScale.x ; m[0][1] =     0.0f ; m[0][2] =     0.0f ; m[0][3] = 0.0f ;
            m[1][0] =     0.0f ; m[1][1] = vScale.y ; m[1][2] =     0.0f ; m[1][3] = 0.0f ;
            m[2][0] =     0.0f ; m[2][1] =     0.0f ; m[2][2] = vScale.z ; m[2][3] = 0.0f ;
            m[3][0] =     0.0f ; m[3][1] =     0.0f ; m[3][2] =     0.0f ; m[3][3] = 1.0f ;
        }


        /*! \brief Apply uniform scale to upper 3x3 part of matrix
            \param scale - scale factor
        */
        void ApplyScaleToUpper3x3( float scale )
        {
            m[0][0] *= scale ; m[0][1] *= scale ; m[0][2] *= scale ;
            m[1][0] *= scale ; m[1][1] *= scale ; m[1][2] *= scale ;
            m[2][0] *= scale ; m[2][1] *= scale ; m[2][2] *= scale ;
        }


        /*! \brief Assign rightward unit vector part of transformation matrix
            \param vRight - vector pointing along transformed space +x axis
        */
        void SetRight( Vec4 vRight )
        {
            m[0][0] = vRight.x ; m[1][0] = vRight.y ; m[2][0] = vRight.z ;
        }


        /*! \brief Assign upward unit vector part of transformation matrix
            \param vUp - vector pointing along transformed space +z axis
        */
        void SetUp( Vec4 vUp )
        {
            m[0][1] = vUp.x ; m[1][1] = vUp.y ; m[2][1] = vUp.z ;
        }


        /*! \brief Assign forward unit vector part of transformation matrix
            \param vForward - vector pointing along transformed space +y axis
        */
        void SetForward( Vec4 vForward )
        {
            m[0][2] = vForward.x ; m[1][2] = vForward.y ; m[2][2] = vForward.z ;
        }


        /*! \brief Assign translation part of transformation matrix
            \param trans - translation vector
        */
        void SetTranslationPart( Vec4 trans )
        {
            m[3][0] = trans.x ; m[3][1] = trans.y ; m[3][2] = trans.z ; m[3][3] = trans.w ;
        }


        /*! \brief Assign a transformation matrix to be a translation.
            \param trans - translation vector
        */
        void SetTranslation( Vec4 trans )
        {
            m[0][0] =    1.0f ; m[0][1] =    0.0f ; m[0][2] =    0.0f ; m[0][3] = 0.0f ;
            m[1][0] =    0.0f ; m[1][1] =    1.0f ; m[1][2] =    0.0f ; m[1][3] = 0.0f ;
            m[2][0] =    0.0f ; m[2][1] =    0.0f ; m[2][2] =    1.0f ; m[2][3] = 0.0f ;
            m[3][0] = trans.x ; m[3][1] = trans.y ; m[3][2] = trans.z ; m[3][3] = 1.0f ;
        }


        /*! \brief Obtain "right" part of matrix
            \return Rightward vector
        */
        Vec3 GetRight( void ) const
        {
            return Vec3( m[0][0] , m[1][0] , m[2][0] ) ;
        }


        /*! \brief Obtain "up" part of matrix
            \return Upward vector
        */
        Vec3 GetUp( void ) const
        {
            return Vec3( m[0][1] , m[1][1] , m[2][1] ) ;
        }


        /*! \brief Obtain "forward" part of matrix
            \return Forward vector
        */
        Vec3 GetForward( void ) const
        {
            return Vec3( m[0][2] , m[1][2] , m[2][2] ) ;
        }


        /*! \brief Obtain translation part of matrix
            \return Translation vector
        */
        Vec3 GetTranslation( void ) const
        {
            return Vec3( m[3][0] , m[3][1] , m[3][2] ) ;
        }


        /*! \brief Assign rotation matrix through angle about X axis
            \param angle - angle (in radians) to rotate
        */
        void SetRotationX( float angle )
        {
            const float c = cos( angle ) ;
            const float s = sin( angle ) ;
            m[0][0] =   1.0f ; m[0][1] =   0.0f ; m[0][2] =   0.0f ; m[0][3] =    0.0f ;
            m[1][0] =   0.0f ; m[1][1] =    c   ; m[1][2] =    s   ; m[1][3] =    0.0f ;
            m[2][0] =   0.0f ; m[2][1] =   -s   ; m[2][2] =    c   ; m[2][3] =    0.0f ;
            m[3][0] =   0.0f ; m[3][1] =   0.0f ; m[3][2] =   0.0f ; m[3][3] =    1.0f ;
        }


        /*! \brief Assign rotation matrix through angle about Y axis
            \param angle - angle (in radians) to rotate
        */
        void SetRotationY( float angle )
        {
            const float c = cos( angle ) ;
            const float s = sin( angle ) ;
            m[0][0] =    c   ; m[0][1] =   0.0f ; m[0][2] =   -s   ; m[0][3] =    0.0f ;
            m[1][0] =   0.0f ; m[1][1] =   1.0f ; m[1][2] =   0.0f ; m[1][3] =    0.0f ;
            m[2][0] =    s   ; m[2][1] =   0.0f ; m[2][2] =    c   ; m[2][3] =    0.0f ;
            m[3][0] =   0.0f ; m[3][1] =   0.0f ; m[3][2] =   0.0f ; m[3][3] =    1.0f ;
        }


        /*! \brief Assign rotation matrix through angle about Z axis
            \param angle - angle (in radians) to rotate
        */
        void SetRotationZ( float angle )
        {
            const float c = cos( angle ) ;
            const float s = sin( angle ) ;
            m[0][0] =    c   ; m[0][1] =    s   ; m[0][2] =   0.0f ; m[0][3] =    0.0f ;
            m[1][0] =   -s   ; m[1][1] =    c   ; m[1][2] =   0.0f ; m[1][3] =    0.0f ;
            m[2][0] =   0.0f ; m[2][1] =   0.0f ; m[2][2] =   1.0f ; m[2][3] =    0.0f ;
            m[3][0] =   0.0f ; m[3][1] =   0.0f ; m[3][2] =   0.0f ; m[3][3] =    1.0f ;
        }


        /*! \brief Assign rotation matrix through angle about X, Y and Z axes
            \param ax - angle (in radians) to rotate about X axis
            \param ay - angle (in radians) to rotate about Y axis
            \param az - angle (in radians) to rotate about Z axis
        */
        void SetRotationXYZ( float ax , float ay , float az )
        {
            const float A  =   cos(ax);
            const float B  =   sin(ax);
            const float C  =   cos(-ay);
            const float D  =   sin(-ay);
            const float E  =   cos(az);
            const float F  =   sin(az);
            const float AD =   A * D;
            const float BD =   B * D;

            m[0][0] =   C * E;
            m[1][0] = - C * F;
            m[2][0] = - D;

            m[0][1] = - BD * E + A * F;
            m[1][1] =   BD * F + A * E;
            m[2][1] = - B * C;

            m[0][2] =   AD * E + B * F;
            m[1][2] = - AD * F + B * E;
            m[2][2] =   A * C;

            m[0][3] =
            m[1][3] =
            m[2][3] =
            m[3][0] =
            m[3][1] =
            m[3][2] = 0.0;

            m[3][3] = 1.0;
        }



        /*! \brief Assign view matrix for given camera parameters
            \param vEye             Eye position in world space
            \param vTarget          Look-at position in world space
            \param vUpApproximate   Approximate direction, in world space, toward which camera top points
        */
        void SetView( const Vec4 & vEye , const Vec4 & vTarget , const Vec4 & vUpApproximate )
        {

            // Compute camera coordinate system unit vectors.
            Vec4 vForward = vEye - vTarget ;
            vForward.Normalizev3() ;
            Vec4 vRight = vUpApproximate ^ vForward ;
            vRight.w = 0.0f ;
            vRight.Normalizev3() ;
            Vec4 vUpOrthoNormal = vForward ^ vRight ; // this up direction has unit length and is mutually orthogonal to vForward and vRight
            vUpOrthoNormal.w = 0.0f ;


            // Set up this matrix as the rotation-only portion of the view matrix.
            SetRight  ( vRight         ) ;
            SetUp     ( vUpOrthoNormal ) ;
            SetForward( vForward       ) ;
            m[0][3] = 0.0f ;
            m[1][3] = 0.0f ;
            m[2][3] = 0.0f ;
            SetTranslationPart( Vec4( 0.0f , 0.0f , 0.0f , 1.0f ) ) ;

            // Compute the translation column of the view matrix.
		    const Vec4 trans = - *this * vEye ;

            SetTranslationPart( Vec4( trans.x , trans.y , trans.z , 1.0f ) ) ;
        }




        /*! \brief Assign rotation matrix from axis and angle
            \param axis - axis about which to rotate. Must have unit length.
            \param angle - angle (in radians) to rotate

            \note   Beware that the formulae given in the Matrix and Quaternion FAQ
                    for the diagonal terms appear to be nonsense.  Those formulae
                    result in non-unitary transforms.  (That FAQ often has errors.)

        */
        void SetRotation( const Vec3 & axis , float angle )
        {
	        const float c    = cos( angle ) ;
	        const float omc  = 1.0f - c ;
	        const float s    = sin( angle ) ;
	        const float xs   = axis.x * s ;
	        const float ys   = axis.y * s ;
	        const float zs   = axis.z * s ;
	        const float x1mc = axis.x * omc ;
	        const float y1mc = axis.y * omc ;
	        const float z1mc = axis.z * omc ;
	        m[0][0] =  c + axis.x * x1mc ;
	        m[0][1] = axis.y * x1mc + zs ;
	        m[0][2] = axis.z * x1mc - ys ;

	        m[1][0] =  axis.x * y1mc - zs ;
	        m[1][1] = c + axis.y * y1mc ;
	        m[1][2] = axis.z * y1mc + xs ;

	        m[2][0] =  axis.x * z1mc + ys ;
	        m[2][1] = axis.y * z1mc - xs ;
	        m[2][2] = c + axis.z * z1mc ;

            m[0][3] =
            m[1][3] =
            m[2][3] =
            m[3][0] =
            m[3][1] =
            m[3][2] = 0.0f ;

            m[3][3] = 1.0f ;
        }


        void SetRotation( const Mat33 & o33 )
        {
            m[0][0] = o33.x.x ; m[0][1] = o33.x.y ; m[0][2] = o33.x.z ; m[0][3] = 0.0f ;
            m[1][0] = o33.y.x ; m[1][1] = o33.y.y ; m[1][2] = o33.y.z ; m[1][3] = 0.0f ;
            m[2][0] = o33.z.x ; m[2][1] = o33.z.y ; m[2][2] = o33.z.z ; m[2][3] = 0.0f ;
            m[3][0] =    0.0f ; m[3][1] =    0.0f ; m[3][2] =    0.0f ; m[3][3] = 1.0f ;
        }


        /*! \brief Extract camera position from view matrix
        */
        static Vec4 CameraPositionFromViewMatrix( const Mat4 & xView )
        {
            Vec4 vPos( - xView.GetTranslation() ) ;
            vPos = Mat4::TransformNormalByTranspose( (Vec3&) vPos , xView ) ;
            vPos.w = 1.0f ;
            return vPos ;
        }

        union
        {
            float m[4][4] ;
            struct V
            {
                Vector4 x, y, z, w ;
            } v ;
        } ;

    private:
        // float Det2( int row1, int row2, int col1, int col2 ) const ;

        float RowDotCol( int row, const Mat4 & m4 , int col ) const
        {
            return m[row][0] * m4.m[0][col]
                 + m[row][1] * m4.m[1][col]
                 + m[row][2] * m4.m[2][col]
                 + m[row][3] * m4.m[3][col] ;
        }
} ;

// Public variables ------------------------------------------------------------

static const Mat4 Mat4_xIdentity(   1.0f , 0.0f , 0.0f , 0.0f ,
                                    0.0f , 1.0f , 0.0f , 0.0f ,
                                    0.0f , 0.0f , 1.0f , 0.0f ,
                                    0.0f , 0.0f , 0.0f , 1.0f ) ;

// Public functions ------------------------------------------------------------


#endif
