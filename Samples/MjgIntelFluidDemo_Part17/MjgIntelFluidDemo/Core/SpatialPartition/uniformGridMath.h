/** \file uniformGridMath.h

    \brief Mathematical routines for UniformGrids of vectors or matrices

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
       http://www.mijagourlay.com/

*/
#ifndef UNIFORM_GRID_MATH_H
#define UNIFORM_GRID_MATH_H

#include "Core/Math/mat33.h"
#include "uniformGrid.h"


// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

extern void FindValueRange( const UniformGrid< float > & vec , float & valMin , float & valMax ) ;
extern void FindValueStats( const UniformGrid< float > & vec , float & valMin , float & valMax , float & valMean , float & valStdDev ) ;
extern void FindMagnitudeRange( const UniformGrid< Vec3 > & vec , float & magMin , float & magMax ) ;
extern void ComputeGradient( UniformGrid< Vec3 > & gradient , const UniformGrid< float > & val ) ;
extern void ComputeGradientConditionally( UniformGrid< Vec3 > & gradient , const UniformGrid< float > & val ) ;
extern void ComputeJacobian( UniformGrid< Mat33 > & jacobian , const UniformGrid< Vec3 > & vec ) ;
extern void ComputeCurlFromJacobian( UniformGrid< Vec3 > & curl , const UniformGrid< Mat33 > & jacobian ) ;
extern void ComputeLaplacian( UniformGrid< Vec3 > & laplacian , const UniformGrid< Vec3 > & vec ) ;
extern void SolveVectorPoisson( UniformGrid< Vec3 > & soln , const UniformGrid< Vec3 > & lap , size_t numSteps = 0 ) ;

#endif
