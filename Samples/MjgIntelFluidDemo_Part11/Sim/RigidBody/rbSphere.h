/*! \file rbSphere.h

    \brief Spherical rigid body

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef RB_SPHERE_H
#define RB_SPHERE_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "wrapperMacros.h"
#include "rigidBody.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Spherical rigid body
*/
class RbSphere : public RigidBody
{
    public:

        typedef RigidBody Parent ;

        /*! \brief Construct a spherical rigid body
        */
        RbSphere()
            : RigidBody()
            , mRadius( 0.0f )
        {
        }

        RbSphere( const Vec3 & vPos , const Vec3 & vVelocity , const float & fMass , const float & fRadius )
            : Parent( vPos , vVelocity , fMass )
            , mRadius( fRadius )
        {
            // Moments of inertia for a sphere are 2 M R^2 / 5.
            // So the inverse of that is 5/(2 M R^2)
            // which is 5 (1/M) / (2 R^2) = 2.5 (1/M) / R^2
            mInertiaInv = Mat33_xIdentity * 2.5f * mInverseMass / ( fRadius * fRadius ) ;
            mVolume = 4.0f * PI * POW3( fRadius ) / 3.0f ;
        }

        float	mRadius		    ;	///< Radius of vortex particle
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
