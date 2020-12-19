/*! \file uniformGridMath.h

    \brief Mathematical routines for UniformGrids of vectors or matrices

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef UNIFORM_GRID_MATH_H
#define UNIFORM_GRID_MATH_H

#include "Core/Math/mat33.h"
#include "uniformGrid.h"


// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

extern void ComputeJacobian( UniformGrid< Mat33 > & jacobian , const UniformGrid< Vec3 > & vec ) ;
extern void ComputeCurlFromJacobian( UniformGrid< Vec3 > & curl , const UniformGrid< Mat33 > & jacobian ) ;

#endif
