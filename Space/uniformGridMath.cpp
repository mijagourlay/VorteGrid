/*! \file uniformGridMath.cpp

    \brief Mathematical routines for UniformGrids of vectors or matrices

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "uniformGridMath.h"




/*! \brief Compute curl of a vector field, from its Jacobian

    \param curl - (output) UniformGrid of 3-vector values.

    \param jacobian - UniformGrid of 3x3 matrix values.

    \see ComputeJacobian.

*/
void ComputeCurlFromJacobian( UniformGrid< Vec3 > & curl , const UniformGrid< Mat33 > & jacobian )
{
    const unsigned  dims[3]     = { jacobian.GetNumPoints( 0 ) , jacobian.GetNumPoints( 1 ) , jacobian.GetNumPoints( 2 ) } ;
    const unsigned  numXY       = dims[0] * dims[1] ;
    unsigned        index[3] ;

    // Compute curl from Jacobian
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        const unsigned offsetZ = numXY * index[2]       ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            const unsigned offsetYZ = dims[ 0 ] * index[1] + offsetZ ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                const unsigned offsetXYZ = index[0] + offsetYZ ;
                const Mat33 & j     = jacobian[ offsetXYZ ] ;
                Vec3        & rCurl = curl[ offsetXYZ ] ;
                // Meaning of j.i.k is the derivative of the kth component with respect to i, i.e. di/dk.
                rCurl = Vec3( j.y.z - j.z.y , j.z.x - j.x.z , j.x.y - j.y.x ) ;
            }
        }
    }
}




/*! \brief Compute statistics of data in a uniform grid of 3-by-3-matrices

    \param min - minimum of all values in grid.

    \param max - maximum of all values in grid.

*/
void UniformGrid<Mat33>::ComputeStatistics( Mat33 & min , Mat33 & max ) const
{
    const Vec3 vMax = Vec3( FLT_MAX , FLT_MAX , FLT_MAX ) ;
    min = Mat33( vMax , vMax , vMax ) ;
    max = Mat33( -min ) ;
    const unsigned dims[3] = { GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) } ;
    const unsigned numXY   = dims[0] * dims[1] ;
    unsigned index[3] ;

    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        const unsigned offsetPartialZ = numXY * index[2]       ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            const unsigned offsetPartialYZ = dims[ 0 ] * index[1] + offsetPartialZ ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                const unsigned offset = index[0]     + offsetPartialYZ  ;
                const Mat33 & rVal = (*this)[ offset ] ;
                min.x.x = MIN2( min.x.x , rVal.x.x ) ;
                min.y.x = MIN2( min.y.x , rVal.y.x ) ;
                min.z.x = MIN2( min.z.x , rVal.z.x ) ;
                max.x.x = MAX2( max.x.x , rVal.x.x ) ;
                max.y.x = MAX2( max.y.x , rVal.y.x ) ;
                max.z.x = MAX2( max.z.x , rVal.z.x ) ;

                min.x.y = MIN2( min.x.y , rVal.x.y ) ;
                min.y.y = MIN2( min.y.y , rVal.y.y ) ;
                min.z.y = MIN2( min.z.y , rVal.z.y ) ;
                max.x.y = MAX2( max.x.y , rVal.x.y ) ;
                max.y.y = MAX2( max.y.y , rVal.y.y ) ;
                max.z.y = MAX2( max.z.y , rVal.z.y ) ;

                min.x.z = MIN2( min.x.z , rVal.x.z ) ;
                min.y.z = MIN2( min.y.z , rVal.y.z ) ;
                min.z.z = MIN2( min.z.z , rVal.z.z ) ;
                max.x.z = MAX2( max.x.z , rVal.x.z ) ;
                max.y.z = MAX2( max.y.z , rVal.y.z ) ;
                max.z.z = MAX2( max.z.z , rVal.z.z ) ;
            }
        }
    }
}




#pragma warning(disable: 4189)  // Disable "local variable is initialized but not referenced" because macros below define variables that are sometimes not used.




/*! \brief Compute Jacobian of a vector field

    \param jacobian - (output) UniformGrid of 3x3 matrix values.
                        The matrix is a vector of vectors.
                        Each component is a partial derivative with
                        respect to some direction:
                            j.a.b = d v.b / d a
                        where a and b are each one of {x,y,z}.
                        So j.x contains the partial derivatives with respect to x, etc.

    \param vec - UniformGrid of 3-vector values

*/
void ComputeJacobian( UniformGrid< Mat33 > & jacobian , const UniformGrid< Vec3 > & vec )
{
    const Vec3      spacing                 = vec.GetCellSpacing() ;
    // Avoid divide-by-zero when z size is effectively 0 (for 2D domains)
    const Vec3      reciprocalSpacing( 1.0f / spacing.x , 1.0f / spacing.y , spacing.z > FLT_EPSILON ? 1.0f / spacing.z : 0.0f ) ;
    const Vec3      halfReciprocalSpacing( 0.5f * reciprocalSpacing ) ;
    const unsigned  dims[3]                 = { vec.GetNumPoints( 0 )   , vec.GetNumPoints( 1 )   , vec.GetNumPoints( 2 )   } ;
    const unsigned  dimsMinus1[3]           = { vec.GetNumPoints( 0 )-1 , vec.GetNumPoints( 1 )-1 , vec.GetNumPoints( 2 )-1 } ;
    const unsigned  numXY                   = dims[0] * dims[1] ;
    unsigned        index[3] ;

#define ASSIGN_Z_OFFSETS                                    \
    const unsigned offsetZM = numXY * ( index[2] - 1 ) ;    \
    const unsigned offsetZ0 = numXY *   index[2]       ;    \
    const unsigned offsetZP = numXY * ( index[2] + 1 ) ;

#define ASSIGN_YZ_OFFSETS                                                   \
    const unsigned offsetYMZ0 = dims[ 0 ] * ( index[1] - 1 ) + offsetZ0 ;   \
    const unsigned offsetY0Z0 = dims[ 0 ] *   index[1]       + offsetZ0 ;   \
    const unsigned offsetYPZ0 = dims[ 0 ] * ( index[1] + 1 ) + offsetZ0 ;   \
    const unsigned offsetY0ZM = dims[ 0 ] *   index[1]       + offsetZM ;   \
    const unsigned offsetY0ZP = dims[ 0 ] *   index[1]       + offsetZP ;

#define ASSIGN_XYZ_OFFSETS                                      \
    const unsigned offsetX0Y0Z0 = index[0]     + offsetY0Z0 ;   \
    const unsigned offsetXMY0Z0 = index[0] - 1 + offsetY0Z0 ;   \
    const unsigned offsetXPY0Z0 = index[0] + 1 + offsetY0Z0 ;   \
    const unsigned offsetX0YMZ0 = index[0]     + offsetYMZ0 ;   \
    const unsigned offsetX0YPZ0 = index[0]     + offsetYPZ0 ;   \
    const unsigned offsetX0Y0ZM = index[0]     + offsetY0ZM ;   \
    const unsigned offsetX0Y0ZP = index[0]     + offsetY0ZP ;

    // Compute derivatives for interior (i.e. away from boundaries).
    for( index[2] = 1 ; index[2] < dimsMinus1[2] ; ++ index[2] )
    {
        ASSIGN_Z_OFFSETS ;
        for( index[1] = 1 ; index[1] < dimsMinus1[1] ; ++ index[1] )
        {
            ASSIGN_YZ_OFFSETS ;
            for( index[0] = 1 ; index[0] < dimsMinus1[0] ; ++ index[0] )
            {
                ASSIGN_XYZ_OFFSETS ;

                Mat33 & rMatrix = jacobian[ offsetX0Y0Z0 ] ;
                /* Compute d/dx */
                rMatrix.x = ( vec[ offsetXPY0Z0 ] - vec[ offsetXMY0Z0 ] ) * halfReciprocalSpacing.x ;
                /* Compute d/dy */
                rMatrix.y = ( vec[ offsetX0YPZ0 ] - vec[ offsetX0YMZ0 ] ) * halfReciprocalSpacing.y ;
                /* Compute d/dz */
                rMatrix.z = ( vec[ offsetX0Y0ZP ] - vec[ offsetX0Y0ZM ] ) * halfReciprocalSpacing.z ;
            }
        }
    }

    // Compute derivatives for boundaries: 6 faces of box.
    // In some situations, these macros compute extraneous data.
    // A tiny bit more efficiency could be squeezed from this routine,
    // but it turns out to be well under 1% of the total expense.

#define COMPUTE_FINITE_DIFF                                                                                                             \
    Mat33 & rMatrix = jacobian[ offsetX0Y0Z0 ] ;                                                                                        \
    if( index[0] == 0 )                     { rMatrix.x = ( vec[ offsetXPY0Z0 ] - vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing.x ;     }   \
    else if( index[0] == dimsMinus1[0] )    { rMatrix.x = ( vec[ offsetX0Y0Z0 ] - vec[ offsetXMY0Z0 ] ) * reciprocalSpacing.x ;     }   \
    else                                    { rMatrix.x = ( vec[ offsetXPY0Z0 ] - vec[ offsetXMY0Z0 ] ) * halfReciprocalSpacing.x ; }   \
    if( index[1] == 0 )                     { rMatrix.y = ( vec[ offsetX0YPZ0 ] - vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing.y ;     }   \
    else if( index[1] == dimsMinus1[1] )    { rMatrix.y = ( vec[ offsetX0Y0Z0 ] - vec[ offsetX0YMZ0 ] ) * reciprocalSpacing.y ;     }   \
    else                                    { rMatrix.y = ( vec[ offsetX0YPZ0 ] - vec[ offsetX0YMZ0 ] ) * halfReciprocalSpacing.y ; }   \
    if( index[2] == 0 )                     { rMatrix.z = ( vec[ offsetX0Y0ZP ] - vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing.z ;     }   \
    else if( index[2] == dimsMinus1[2] )    { rMatrix.z = ( vec[ offsetX0Y0Z0 ] - vec[ offsetX0Y0ZM ] ) * reciprocalSpacing.z ;     }   \
    else                                    { rMatrix.z = ( vec[ offsetX0Y0ZP ] - vec[ offsetX0Y0ZM ] ) * halfReciprocalSpacing.z ; }

    // Compute derivatives for -X boundary.
    index[0] = 0 ;
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        ASSIGN_Z_OFFSETS ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            ASSIGN_YZ_OFFSETS ;
            {
                ASSIGN_XYZ_OFFSETS ;
                COMPUTE_FINITE_DIFF ;
            }
        }
    }

    // Compute derivatives for -Y boundary.
    index[1] = 0 ;
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        ASSIGN_Z_OFFSETS ;
        {
            ASSIGN_YZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XYZ_OFFSETS ;
                COMPUTE_FINITE_DIFF ;
            }
        }
    }

    // Compute derivatives for -Z boundary.
    index[2] = 0 ;
    {
        ASSIGN_Z_OFFSETS ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            ASSIGN_YZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XYZ_OFFSETS ;
                COMPUTE_FINITE_DIFF ;
            }
        }
    }

    // Compute derivatives for +X boundary.
    index[0] = dimsMinus1[0] ;
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        ASSIGN_Z_OFFSETS ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            ASSIGN_YZ_OFFSETS ;
            {
                ASSIGN_XYZ_OFFSETS ;
                COMPUTE_FINITE_DIFF ;
            }
        }
    }


    // Compute derivatives for +Y boundary.
    index[1] = dimsMinus1[1] ;
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        ASSIGN_Z_OFFSETS ;
        {
            ASSIGN_YZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XYZ_OFFSETS ;
                COMPUTE_FINITE_DIFF ;
            }
        }
    }

    // Compute derivatives for +Z boundary.
    index[2] = dimsMinus1[2] ;
    {
        ASSIGN_Z_OFFSETS ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            ASSIGN_YZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XYZ_OFFSETS ;
                COMPUTE_FINITE_DIFF ;
            }
        }
    }

#undef COMPUTE_FINITE_DIFF
#undef ASSIGN_XYZ_OFFSETS
#undef ASSIGN_YZ_OFFSETS
#undef ASSIGN_Z_OFFSETS

}
