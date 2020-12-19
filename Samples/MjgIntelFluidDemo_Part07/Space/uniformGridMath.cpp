/*! \file uniformGridMath.cpp

    \brief Mathematical routines for UniformGrids of vectors or matrices

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "useTbb.h"
#include "uniformGridMath.h"



#pragma warning(disable: 4189)  // Disable "local variable is initialized but not referenced" because macros below define variables that are sometimes not used.




/*! \brief Techniques for implementing the Gauss-Seidel method

    When USE_TBB is enabled (i.e. when using Intel Threading Building Blocks)
    for parallel execution, then SolveVectorPoisson always uses red-black,
    regardless of the value of POISSON_TECHNIQUE.  This is only used when
    USE_TBB is false, i.e. when built for serial execution.
    The POISSON_TECHNIQUE_GAUSS_SEIDEL_RED_BLACK technique mimics the operation
    of the parallelized routine, whereas POISSON_TECHNIQUE_GAUSS_SEIDEL
    implements the traditional straightforward Gauss-Seidel technique.

    \see GaussSeidelPortion
*/
#define POISSON_TECHNIQUE_GAUSS_SEIDEL              1
#define POISSON_TECHNIQUE_GAUSS_SEIDEL_RED_BLACK    2

#define POISSON_TECHNIQUE                           POISSON_TECHNIQUE_GAUSS_SEIDEL_RED_BLACK




/*! \brief Which portion of the linear algebraic equation to solve

    The Gauss-Seidel method for solving a system of linear equations
    operates "in-place" meaning that the updated solution to a particular
    element overwrites the previous value for that same element.
    This contrasts with the Jacobi method, which stores the results of
    a given iteration in a separate location from the values from
    the previous iteration.  The Gauss-Seidel method has 2 advantages
    over Jacobi: Faster convergence and lower storage requirements.
    Unfortunately, when distributed across multiple processors,
    the traditional Gauss-Seidel method is not thread-safe, since the inputs
    used by one thread are the outputs written by another thread.
    Synchronizing across threads by element would cost too much overhead,
    so instead, we partition the elements into "red" and "black",
    analogous to squares in a checkerboard.  The inputs for red squares
    are all black, and vice-versa.  During one pass, the algorithm
    operates on (i.e. writes to) a single color, then in a second pass,
    the algorithm operates on the other color.  All threads operate on
    a single color, therefore there is no contention for data; all threads
    are reading from one color and writing to the other.

    \note   Logic elsewhere depends on these symbols having the values they have.
            Specifically, a boolean expression depends on red & black being 0 or 1,
            and both not being either 0 or 1.
*/
enum GaussSeidelPortion
{
    GS_RED      = 0         ,   ///< Operate only on "red" elements
    GS_MIN      = GS_RED    ,
    GS_BLACK    = 1         ,   ///< Operate only on "black" elements
    GS_BOTH     = 2         ,   ///< Operate on all matrix elements
    GS_NUM
} ;




#if USE_TBB
    static void StepTowardVectorPoissonSolution( UniformGrid< Vec3 > & soln , const UniformGrid< Vec3 > & lap , size_t izStart , size_t izEnd , GaussSeidelPortion redOrBlack ) ;
    extern unsigned gNumberOfProcessors ;  ///< Number of processors this machine has.

    /*! \brief Function object to solve vector Poisson equation using Threading Building Blocks
    */
    class UniformGrid_StepTowardVectorPoissonSolution_TBB
    {
            UniformGrid< Vec3 > &       mSolution   ;   ///< Address of object containing solution
            const UniformGrid< Vec3 > & mLaplacian  ;   ///< Address of object containing Laplacian
            const GaussSeidelPortion    mRedOrBlack ;   ///< Whether this pass operates on red or black portion of grid
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of velocity grid.
                StepTowardVectorPoissonSolution( mSolution , mLaplacian , r.begin() , r.end() , mRedOrBlack ) ;
            }
            UniformGrid_StepTowardVectorPoissonSolution_TBB( UniformGrid< Vec3 > & pSolution , const UniformGrid< Vec3 > & pLaplacian , GaussSeidelPortion redOrBlack )
                : mSolution( pSolution ) , mLaplacian( pLaplacian ) , mRedOrBlack( redOrBlack ) {}
    } ;
#endif




#define ASSIGN_Z_OFFSETS                                \
    const size_t offsetZM  = numXY * ( index[2] - 1 ) ; \
    const size_t offsetZ0  = numXY *   index[2]       ; \
    const size_t offsetZP  = numXY * ( index[2] + 1 ) ; \

#define ASSIGN_YZ_OFFSETS                                               \
    const size_t offsetYMZ0 = dims[ 0 ] * ( index[1] - 1 ) + offsetZ0 ; \
    const size_t offsetY0Z0 = dims[ 0 ] *   index[1]       + offsetZ0 ; \
    const size_t offsetYPZ0 = dims[ 0 ] * ( index[1] + 1 ) + offsetZ0 ; \
    const size_t offsetY0ZM = dims[ 0 ] *   index[1]       + offsetZM ; \
    const size_t offsetY0ZP = dims[ 0 ] *   index[1]       + offsetZP ;

#define ASSIGN_XYZ_OFFSETS                                  \
    const size_t offsetX0Y0Z0 = index[0]     + offsetY0Z0 ; \
    const size_t offsetXMY0Z0 = index[0] - 1 + offsetY0Z0 ; \
    const size_t offsetXPY0Z0 = index[0] + 1 + offsetY0Z0 ; \
    const size_t offsetX0YMZ0 = index[0]     + offsetYMZ0 ; \
    const size_t offsetX0YPZ0 = index[0]     + offsetYPZ0 ; \
    const size_t offsetX0Y0ZM = index[0]     + offsetY0ZM ; \
    const size_t offsetX0Y0ZP = index[0]     + offsetY0ZP ;


#define ASSIGN_ZZ_OFFSETS                               \
    ASSIGN_Z_OFFSETS ;                                  \
    const size_t offsetZMM = numXY * ( index[2] - 2 ) ; \
    const size_t offsetZPP = numXY * ( index[2] + 2 ) ;

#define ASSIGN_YYZZ_OFFSETS                                                 \
    ASSIGN_YZ_OFFSETS ;                                                     \
    const size_t offsetYMMZ0 = dims[ 0 ] * ( index[1] - 2 ) + offsetZ0  ;   \
    const size_t offsetYPPZ0 = dims[ 0 ] * ( index[1] + 2 ) + offsetZ0  ;   \
    const size_t offsetY0ZMM = dims[ 0 ] *   index[1]       + offsetZMM ;   \
    const size_t offsetY0ZPP = dims[ 0 ] *   index[1]       + offsetZPP ;

#define ASSIGN_XXYYZZ_OFFSETS                                   \
    ASSIGN_XYZ_OFFSETS                                          \
    const size_t offsetXMMY0Z0 = index[0] - 2 + offsetY0Z0  ;   \
    const size_t offsetXPPY0Z0 = index[0] + 2 + offsetY0Z0  ;   \
    const size_t offsetX0YMMZ0 = index[0]     + offsetYMMZ0 ;   \
    const size_t offsetX0YPPZ0 = index[0]     + offsetYPPZ0 ;   \
    const size_t offsetX0Y0ZMM = index[0]     + offsetY0ZMM ;   \
    const size_t offsetX0Y0ZPP = index[0]     + offsetY0ZPP ;



/*! \brief Compute curl of a vector field, from its Jacobian

    \param curl - (output) UniformGrid of 3-vector values.

    \param jacobian - UniformGrid of 3x3 matrix values.

    \see ComputeJacobian.

*/
void ComputeCurlFromJacobian( UniformGrid< Vec3 > & curl , const UniformGrid< Mat33 > & jacobian )
{
    const size_t    dims[3]     = { jacobian.GetNumPoints( 0 ) , jacobian.GetNumPoints( 1 ) , jacobian.GetNumPoints( 2 ) } ;
    const size_t    numXY       = dims[0] * dims[1] ;
    size_t          index[3] ;

    // Compute curl from Jacobian
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        const size_t offsetZ = numXY * index[2]       ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            const size_t offsetYZ = dims[ 0 ] * index[1] + offsetZ ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                const size_t  offsetXYZ = index[0] + offsetYZ ;
                const Mat33 & j         = jacobian[ offsetXYZ ] ;
                Vec3        & rCurl     = curl[ offsetXYZ ] ;
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
    const Vec3      vMax    = Vec3( FLT_MAX , FLT_MAX , FLT_MAX ) ;
    min = Mat33( vMax , vMax , vMax ) ;
    max = Mat33( -min ) ;
    const size_t    dims[3] = { GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) } ;
    const size_t    numXY   = dims[0] * dims[1] ;
    size_t          index[3] ;

    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        const size_t offsetPartialZ = numXY * index[2]       ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            const size_t offsetPartialYZ = dims[ 0 ] * index[1] + offsetPartialZ ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                const size_t offset = index[0]     + offsetPartialYZ  ;
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
    const size_t    dims[3]                 = { vec.GetNumPoints( 0 )   , vec.GetNumPoints( 1 )   , vec.GetNumPoints( 2 )   } ;
    const size_t    dimsMinus1[3]           = { vec.GetNumPoints( 0 )-1 , vec.GetNumPoints( 1 )-1 , vec.GetNumPoints( 2 )-1 } ;
    const size_t    numXY                   = dims[0] * dims[1] ;
    size_t          index[3] ;

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

#define COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES                                                                              \
    Mat33 & rMatrix = jacobian[ offsetX0Y0Z0 ] ;                                                                                        \
    if     ( index[0] == 0             )    { rMatrix.x = ( vec[ offsetXPY0Z0 ] - vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing.x ;     }   \
    else if( index[0] == dimsMinus1[0] )    { rMatrix.x = ( vec[ offsetX0Y0Z0 ] - vec[ offsetXMY0Z0 ] ) * reciprocalSpacing.x ;     }   \
    else                                    { rMatrix.x = ( vec[ offsetXPY0Z0 ] - vec[ offsetXMY0Z0 ] ) * halfReciprocalSpacing.x ; }   \
    if     ( index[1] == 0             )    { rMatrix.y = ( vec[ offsetX0YPZ0 ] - vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing.y ;     }   \
    else if( index[1] == dimsMinus1[1] )    { rMatrix.y = ( vec[ offsetX0Y0Z0 ] - vec[ offsetX0YMZ0 ] ) * reciprocalSpacing.y ;     }   \
    else                                    { rMatrix.y = ( vec[ offsetX0YPZ0 ] - vec[ offsetX0YMZ0 ] ) * halfReciprocalSpacing.y ; }   \
    if     ( index[2] == 0             )    { rMatrix.z = ( vec[ offsetX0Y0ZP ] - vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing.z ;     }   \
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
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
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
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
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
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
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
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
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
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
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
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
            }
        }
    }
#undef COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES
}




/*! \brief Compute Laplacian of a vector field

    \param laplacian - (output) UniformGrid of 3-vector values, the vector Laplacian of "vec"

    \param vec - UniformGrid of 3-vector values

    \see ComputeJacobian.

*/
void ComputeLaplacian( UniformGrid< Vec3 > & laplacian , const UniformGrid< Vec3 > & vec )
{
    // This routine currently only supports fully 3D domains.
    // To compute a Laplacian, which is a second derivative, requires at least 3 gridpoints in each direction.

    const Vec3      spacing                 = vec.GetCellSpacing() ;
    // Avoid divide-by-zero when z size is effectively 0 (for 2D domains)
    const Vec3      reciprocalSpacing( 1.0f / spacing.x , 1.0f / spacing.y , spacing.z > FLT_EPSILON ? 1.0f / spacing.z : 0.0f ) ;
    const Vec3      reciprocalSpacing2( POW2( reciprocalSpacing.x ) , POW2( reciprocalSpacing.y ) , POW2( reciprocalSpacing.z ) ) ;
    const size_t    dims[3]                 = { vec.GetNumPoints( 0 )   , vec.GetNumPoints( 1 )   , vec.GetNumPoints( 2 )   } ;
    const size_t    dimsMinus1[3]           = { vec.GetNumPoints( 0 )-1 , vec.GetNumPoints( 1 )-1 , vec.GetNumPoints( 2 )-1 } ;
    const size_t    numXY                   = dims[0] * dims[1] ;
    size_t          index[3] ;

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

                Vec3 & rLaplacian = laplacian[ offsetX0Y0Z0 ] ;
                // Compute ( d2/dx2 + d2/dy2 + d2/dz2 ) vec
            #if 1
                rLaplacian =    ( vec[ offsetXPY0Z0 ] + vec[ offsetXMY0Z0 ] - 2.0f * vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing2.x
                            +   ( vec[ offsetX0YPZ0 ] + vec[ offsetX0YMZ0 ] - 2.0f * vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing2.y
                            +   ( vec[ offsetX0Y0ZP ] + vec[ offsetX0Y0ZM ] - 2.0f * vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing2.z ;
            #else   // Mathematically equivalent, computationally more expensive.  This form is better suited to writing a Poisson solver.
                rLaplacian =    ( vec[ offsetXPY0Z0 ] + vec[ offsetXMY0Z0 ] ) * reciprocalSpacing2.x
                            +   ( vec[ offsetX0YPZ0 ] + vec[ offsetX0YMZ0 ] ) * reciprocalSpacing2.y
                            +   ( vec[ offsetX0Y0ZP ] + vec[ offsetX0Y0ZM ] ) * reciprocalSpacing2.z
                            - 2.0f * vec[ offsetX0Y0Z0 ] * ( reciprocalSpacing2.x + reciprocalSpacing2.y + reciprocalSpacing2.z ) ;
            #endif
            }
        }
    }

    // Compute derivatives for boundaries: 6 faces of box.
    // In some situations, these macros compute extraneous data.
    // A tiny bit more efficiency could be squeezed from this routine,
    // but it turns out to be well under 1% of the total expense.

#define COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES                                                                                                          \
    Vec3 & rLaplacian = laplacian[ offsetX0Y0Z0 ] ;                                                                                                                 \
    if     ( index[0] == 0             )    { rLaplacian  = ( vec[ offsetXPPY0Z0 ] + vec[ offsetX0Y0Z0  ] - 2.0f * vec[ offsetXPY0Z0 ] ) * reciprocalSpacing2.x ; } \
    else if( index[0] == dimsMinus1[0] )    { rLaplacian  = ( vec[ offsetX0Y0Z0  ] + vec[ offsetXMMY0Z0 ] - 2.0f * vec[ offsetXMY0Z0 ] ) * reciprocalSpacing2.x ; } \
    else                                    { rLaplacian  = ( vec[ offsetXPY0Z0  ] + vec[ offsetXMY0Z0  ] - 2.0f * vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing2.x ; } \
    if     ( index[1] == 0             )    { rLaplacian += ( vec[ offsetX0YPPZ0 ] + vec[ offsetX0Y0Z0  ] - 2.0f * vec[ offsetX0YPZ0 ] ) * reciprocalSpacing2.y ; } \
    else if( index[1] == dimsMinus1[1] )    { rLaplacian += ( vec[ offsetX0Y0Z0  ] + vec[ offsetX0YMMZ0 ] - 2.0f * vec[ offsetX0YMZ0 ] ) * reciprocalSpacing2.y ; } \
    else                                    { rLaplacian += ( vec[ offsetX0YPZ0  ] + vec[ offsetX0YMZ0  ] - 2.0f * vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing2.y ; } \
    if     ( index[2] == 0             )    { rLaplacian += ( vec[ offsetX0Y0ZPP ] + vec[ offsetX0Y0Z0  ] - 2.0f * vec[ offsetX0Y0ZP ] ) * reciprocalSpacing2.z ; } \
    else if( index[2] == dimsMinus1[2] )    { rLaplacian += ( vec[ offsetX0Y0Z0  ] + vec[ offsetX0Y0ZMM ] - 2.0f * vec[ offsetX0Y0ZM ] ) * reciprocalSpacing2.z ; } \
    else                                    { rLaplacian += ( vec[ offsetX0Y0ZP  ] + vec[ offsetX0Y0ZM  ] - 2.0f * vec[ offsetX0Y0Z0 ] ) * reciprocalSpacing2.z ; } \

    // Compute derivatives for -X boundary.
    index[0] = 0 ;
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        ASSIGN_ZZ_OFFSETS ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            ASSIGN_YYZZ_OFFSETS ;
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for -Y boundary.
    index[1] = 0 ;
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        ASSIGN_ZZ_OFFSETS ;
        {
            ASSIGN_YYZZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for -Z boundary.
    index[2] = 0 ;
    {
        ASSIGN_ZZ_OFFSETS ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            ASSIGN_YYZZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for +X boundary.
    index[0] = dimsMinus1[0] ;
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        ASSIGN_ZZ_OFFSETS ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            ASSIGN_YYZZ_OFFSETS ;
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for +Y boundary.
    index[1] = dimsMinus1[1] ;
    for( index[2] = 0 ; index[2] < dims[2] ; ++ index[2] )
    {
        ASSIGN_ZZ_OFFSETS ;
        {
            ASSIGN_YYZZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for +Z boundary.
    index[2] = dimsMinus1[2] ;
    {
        ASSIGN_ZZ_OFFSETS ;
        for( index[1] = 0 ; index[1] < dims[1] ; ++ index[1] )
        {
            ASSIGN_YYZZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                COMPUTE_FINITE_PARTIAL_DIFFERENTIALS_AT_BOUNDARIES ;
            }
        }
    }
}




/*! \brief Shift Y based on z index and red-black value

    Red   elements start at (iy=0, iz=0) and both iy and iz have the same parity.
    Black elements start at (iy=1, iz=0) and iy and iz have opposite parity.

    This implementation exploits the fact that redOrBlack is either 0 or 1 for red or black,
    and 2 (neither 0 nor 1) for both.

*/
#define ASSIGN_Y_SHIFT const size_t idxYShift = ( index[2] % 2 == redOrBlack ) ? 1 : 0 ;

#define ASSIGN_Z_OFFSETS_AND_Y_SHIFT  ASSIGN_Z_OFFSETS  ; ASSIGN_Y_SHIFT ;
#define ASSIGN_ZZ_OFFSETS_AND_Y_SHIFT ASSIGN_ZZ_OFFSETS ; ASSIGN_Y_SHIFT ;




/*! \brief Make one step toward solving the discretized vector Poisson equation

    This routine takes a step toward solving the discretized form of the Poisson equation,
        D soln = lap ,
    where D is the Laplacian partial differential operator.

    This routine uses a finite difference representation of the Laplacian operator,
    and uses the Gauss-Seidel method, augmented with successive over-relaxation,
    to solve the resulting linear algebraic equation that replaces the partial differential equation.

    This routine should be invoked multiple times.  In the simplest case (that is, when NOT using
    this routine inside a multi-grid algorithm), invoke this routine approximately N times where
    N is the largest dimension of the grid.  Each step of this routine transfers information
    between adjacent cells.  But the Poisson equation require a global solution, meaning that
    each cell must feel the influence of all cells in the grid.  It therefore takes at least N
    steps to propagate information between the cells separated by N cells.

    \param soln - (output) UniformGrid of 3-vector values, the solution to the vector Poisson equation

    \param lap - (input) UniformGrid of 3-vector values.

    \param izStart - starting value for z index

    \param izEnd - one past final value for z index

    \param redOrBlack - When running this routine serially, pass "GS_BOTH" for this value.
                    When running this routine in parallel with others accessing the same UniformGrid,
                    call this routine twice per thread, alternatively passing "GS_RED"
                    and "GS_BLACK" for this value, both calls with the same range for izStart and izEnd.
                    This implements the so-called "red-black Gauss-Seidel" algorithm.

                    red squares start at (0,0).

    \see ComputeJacobian.

*/
static void StepTowardVectorPoissonSolution( UniformGrid< Vec3 > & soln , const UniformGrid< Vec3 > & lap , size_t izStart , size_t izEnd , GaussSeidelPortion redOrBlack )
{
    const Vec3      spacing                 = lap.GetCellSpacing() ;
    // Avoid divide-by-zero when z size is effectively 0 (for 2D domains)
    const Vec3      reciprocalSpacing( 1.0f / spacing.x , 1.0f / spacing.y , spacing.z > FLT_EPSILON ? 1.0f / spacing.z : 0.0f ) ;
    const Vec3      reciprocalSpacing2( POW2( reciprocalSpacing.x ) , POW2( reciprocalSpacing.y ) , POW2( reciprocalSpacing.z ) ) ;
    const float     HalfSpacing2Sum         = 0.5f / ( reciprocalSpacing2.x + reciprocalSpacing2.y + reciprocalSpacing2.z ) ;
    const size_t    dims[3]                 = { lap.GetNumPoints( 0 )   , lap.GetNumPoints( 1 )   , lap.GetNumPoints( 2 )   } ;
    const size_t    dimsMinus1[3]           = { lap.GetNumPoints( 0 )-1 , lap.GetNumPoints( 1 )-1 , lap.GetNumPoints( 2 )-1 } ;
    const size_t    numXY                   = dims[0] * dims[1] ;
    const size_t    gridDimMax              = MAX3( dims[0] , dims[1] , dims[2] ) ;
    // Crudely approximate optimal relaxation parameter.
    // Note: Setting "relax" to 1 would yield the canonical Gauss-Seidel algorithm.
    //const float     relax                   = 2.0f / ( 1.0f + sin( PI / float( gridDimMax ) ) ) ;
    const float     relax                   = 1.0f ;
    const float     oneMinusRelax           = 1.0f - relax ;
    // To make this routine work in red-black mode, the index range for the interior depends on redOrBlack.
    const size_t    idxZMinInterior         = MAX2( 1             , izStart ) ;
    const size_t    idxZMaxInterior         = MIN2( dimsMinus1[2] , izEnd   ) ;
    const size_t    iyStep                  = ( GS_BOTH == redOrBlack ) ? 1 : 2 ;
    size_t          index[3] ;

    // Solve equation for interior (i.e. away from boundaries).
    for( index[2] = idxZMinInterior ; index[2] < idxZMaxInterior ; ++ index[2] )
    {
        ASSIGN_Z_OFFSETS_AND_Y_SHIFT ;
        for( index[1] = 1 + idxYShift ; index[1] < dimsMinus1[1] ; index[1] += iyStep )
        {
            ASSIGN_YZ_OFFSETS ;
            for( index[0] = 1 ; index[0] < dimsMinus1[0] ; ++ index[0] )
            {
                ASSIGN_XYZ_OFFSETS ;
                Vec3   vSolution =  (   ( soln[ offsetXPY0Z0 ] + soln[ offsetXMY0Z0 ] ) * reciprocalSpacing2.x
                                    +   ( soln[ offsetX0YPZ0 ] + soln[ offsetX0YMZ0 ] ) * reciprocalSpacing2.y
                                    +   ( soln[ offsetX0Y0ZP ] + soln[ offsetX0Y0ZM ] ) * reciprocalSpacing2.z
                                    -   lap[ offsetX0Y0Z0 ]
                                    ) * HalfSpacing2Sum ;
                soln[ offsetX0Y0Z0 ] = oneMinusRelax * soln[ offsetX0Y0Z0 ] + relax * vSolution ;
            }
        }
    }

#define NATURAL_BOUNDARY_CONDITION 1
#if NATURAL_BOUNDARY_CONDITION
    // Assign solution on boundaries: 6 faces of box.
    // Assign boundary values to equal those of adjacent cells (already solved).
    // This contrasts with "Dirichlet" boundary conditions, where the value at the
    // boundary is prescribed.  To implement that, disable this block of code,
    // and prescribe the values on the boundaries prior to calling this routine.
    #define SOLVE_POISSON_AT_BOUNDARIES                                                         \
        Vec3 & rSolution = soln[ offsetX0Y0Z0 ] ;                                                \
        size_t idxProxy[3] = {  CLAMP( index[0] , 1 , dimsMinus1[0] - 1 )                       \
                             ,  CLAMP( index[1] , 1 , dimsMinus1[1] - 1 )                       \
                             ,  CLAMP( index[2] , 1 , dimsMinus1[2] - 1 )  } ;                  \
        const size_t offsetProxy = index[0] + dims[ 0 ] * index[1] + numXY * index[2] ;         \
        rSolution = soln[ offsetProxy ] ;                                                        \

    // Compute derivatives for -X boundary.
    index[0] = 0 ;
    for( index[2] = izStart ; index[2] < izEnd ; ++ index[2] )
    {
        ASSIGN_ZZ_OFFSETS_AND_Y_SHIFT ;
        for( index[1] = idxYShift ; index[1] < dims[1] ; index[1] += iyStep )
        {
            ASSIGN_YYZZ_OFFSETS ;
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                SOLVE_POISSON_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for -Y boundary.
    for( index[2] = izStart ; index[2] < izEnd ; ++ index[2] )
    {
        ASSIGN_ZZ_OFFSETS_AND_Y_SHIFT ;
        index[1] = idxYShift ;
        {
            ASSIGN_YYZZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XXYYZZ_OFFSETS ;
            }
        }
    }

    // Compute derivatives for -Z boundary.
    index[2] = izStart ;
    {
        ASSIGN_ZZ_OFFSETS_AND_Y_SHIFT ;
        for( index[1] = idxYShift ; index[1] < dims[1] ; index[1] += iyStep )
        {
            ASSIGN_YYZZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                SOLVE_POISSON_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for +X boundary.
    index[0] = dimsMinus1[0] ;
    for( index[2] = izStart ; index[2] < izEnd ; ++ index[2] )
    {
        ASSIGN_ZZ_OFFSETS_AND_Y_SHIFT ;
        for( index[1] = idxYShift ; index[1] < dims[1] ; index[1] += iyStep )
        {
            ASSIGN_YYZZ_OFFSETS ;
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                SOLVE_POISSON_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for +Y boundary.
    for( index[2] = izStart ; index[2] < izEnd ; ++ index[2] )
    {
        ASSIGN_ZZ_OFFSETS_AND_Y_SHIFT ;
        index[1] = idxYShift ;
        if( index[1] <= dimsMinus1[1] )
        {
            ASSIGN_YYZZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                SOLVE_POISSON_AT_BOUNDARIES ;
            }
        }
    }

    // Compute derivatives for +Z boundary.
    index[2] = izEnd - 1 ;
    {
        ASSIGN_ZZ_OFFSETS_AND_Y_SHIFT ;
        for( index[1] = idxYShift ; index[1] < dims[1] ; index[1] += iyStep )
        {
            ASSIGN_YYZZ_OFFSETS ;
            for( index[0] = 0 ; index[0] < dims[0] ; ++ index[0] )
            {
                ASSIGN_XXYYZZ_OFFSETS ;
                SOLVE_POISSON_AT_BOUNDARIES ;
            }
        }
    }
#endif
}

#undef ASSIGN_XYZ_OFFSETS
#undef ASSIGN_YZ_OFFSETS
#undef ASSIGN_Z_OFFSETS

#undef ASSIGN_XXYYZZ_OFFSETS
#undef ASSIGN_YYZZ_OFFSETS
#undef ASSIGN_ZZ_OFFSETS
#undef ASSIGN_ZZ_OFFSETS_AND_Y_SHIFT




/*! \brief Solve the discretized vector Poisson equation

    This routine solves the discretized form of the Poisson equation,
        D soln = lap ,
    where D is the Laplacian partial differential operator.

    \param soln - (output) UniformGrid of 3-vector values, the solution to the vector Poisson equation

    \param lap - (input) UniformGrid of 3-vector values.

*/
void SolveVectorPoisson( UniformGrid< Vec3 > & soln , const UniformGrid< Vec3 > & lap , size_t numSteps )
{
    soln.Init( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
    const size_t  gridDimMax    = MAX3( soln.GetNumPoints( 0 ) , soln.GetNumPoints( 1 ) , soln.GetNumPoints( 2 ) ) ;
    const size_t  maxIters      = ( numSteps > 0 ) ? numSteps : gridDimMax ;
    for( size_t iter = 0 ; iter < maxIters ; ++ iter )
    {
    #if USE_TBB
        {
            const size_t numZ = soln.GetNumPoints( 2 ) ;
            // Estimate grain size based on size of problem and number of processors.
            const size_t grainSize =  MAX2( 1 , numZ / gNumberOfProcessors ) ;
            parallel_for( tbb::blocked_range<size_t>( 0 , numZ , grainSize ) , UniformGrid_StepTowardVectorPoissonSolution_TBB( soln , lap , GS_RED   ) ) ;
            parallel_for( tbb::blocked_range<size_t>( 0 , numZ , grainSize ) , UniformGrid_StepTowardVectorPoissonSolution_TBB( soln , lap , GS_BLACK ) ) ;
        }
    #elif POISSON_TECHNIQUE == POISSON_TECHNIQUE_GAUSS_SEIDEL_RED_BLACK
        StepTowardVectorPoissonSolution( soln , lap , 0 , soln.GetNumPoints( 2 ) , GS_RED   ) ;
        StepTowardVectorPoissonSolution( soln , lap , 0 , soln.GetNumPoints( 2 ) , GS_BLACK ) ;
    #elif POISSON_TECHNIQUE == POISSON_TECHNIQUE_GAUSS_SEIDEL
        StepTowardVectorPoissonSolution( soln , lap , 0 , soln.GetNumPoints( 2 ) , GS_BOTH  ) ;
    #else
        #error Invalid or undefined POISSON_TECHNIQUE.  Either define POISSON_TECHNIQUE appropriately or change this code.
    #endif
    }
}
