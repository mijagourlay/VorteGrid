/*! \file rbSphere.h

    \brief Spherical rigid body

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "Core/Math/vec3.h"

#include "rbSphere.h"

/*! \brief Update rigid bodies
*/
/* static */ void RbSphere::UpdateSystem( Vector< RbSphere > & rbSpheres , float timeStep , unsigned uFrame )
{
    const size_t numBodies = rbSpheres.Size() ;

    for( unsigned uBody = 0 ; uBody < numBodies ; ++ uBody )
    {   // For each body in the simulation...
        RbSphere & rBody = rbSpheres[ uBody ] ;

        // Update body physical state
        rBody.Update( timeStep ) ;
    }
}
