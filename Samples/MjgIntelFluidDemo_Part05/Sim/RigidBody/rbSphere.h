/*! \file rbSphere.h

    \brief Spherical rigid body

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
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
            mInertiaInv = Mat33_xIdentity * 5.0f * mInverseMass / ( 2.0f * fRadius * fRadius ) ;
        }

        static void UpdateSystem( Vector< RbSphere > & rbSpheres , float timeStep , unsigned uFrame ) ;

        float	mRadius		    ;	///< Radius of vortex particle
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
