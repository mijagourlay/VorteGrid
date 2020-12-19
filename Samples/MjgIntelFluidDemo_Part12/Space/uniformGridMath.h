/*! \file uniformGridMath.h

    \brief Mathematical routines for UniformGrids of vectors or matrices

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef UNIFORM_GRID_MATH_H
#define UNIFORM_GRID_MATH_H

#include "Core/Math/mat33.h"
#include "uniformGrid.h"


// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

extern void FindMagnitudeRange( const UniformGrid< Vec3 > & vec , float & magMin , float & magMax ) ;
extern void ComputeGradient( UniformGrid< Vec3 > & gradient , const UniformGrid< float > & val ) ;
extern void ComputeJacobian( UniformGrid< Mat33 > & jacobian , const UniformGrid< Vec3 > & vec ) ;
extern void ComputeCurlFromJacobian( UniformGrid< Vec3 > & curl , const UniformGrid< Mat33 > & jacobian ) ;
extern void ComputeLaplacian( UniformGrid< Vec3 > & laplacian , const UniformGrid< Vec3 > & vec ) ;
extern void SolveVectorPoisson( UniformGrid< Vec3 > & soln , const UniformGrid< Vec3 > & lap , size_t numSteps = 0 ) ;

#endif
