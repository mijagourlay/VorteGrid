/** \file rbSphere.h

    \brief Spherical rigid body.

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef RB_SPHERE_H
#define RB_SPHERE_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "Core/wrapperMacros.h"
#include "Collision/sphereShape.h"
#include "rigidBody.h"
#include "physicalObject.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/** Spherical rigid body.
*/
class RbSphere : public Impulsion::PhysicalObject
{
    public:
        /// Construct a spherical rigid body.
        //RbSphere() : PhysicalObject( & mRigidBody , & mSphereShape ) , mVolume( -1.0f ) {  }

        RbSphere( const Vec3 & vPos , const Vec3 & vVelocity , const float & fMass , const float & fRadius )
            : PhysicalObject( & mRigidBody , & mSphereShape )
            , mRigidBody( vPos , vVelocity , fMass )
        {
            // Moments of inertia for a sphere are 2 M R^2 / 5.
            // So the inverse of that is 5/(2 M R^2)
            // which is 5 (1/M) / (2 R^2) = 2.5 (1/M) / R^2
            GetBody()->SetInverseInertiaTensor( Mat33_xIdentity * 2.5f * mBody->GetReciprocalMass() / ( fRadius * fRadius ) ) ;

            mSphereShape.SetBoundingSphereRadius( fRadius ) ;

            GetThermalProperties().SetTemperature( sAmbientTemperature ) ;
            GetThermalProperties().SetThermalConductivity( 500.0f ) ;
            GetThermalProperties().SetOneOverHeatCapacity( 0.00001f );

            mVolume = 4.0f * PI * POW3( fRadius ) / 3.0f ;
        }

        RbSphere & operator=( const RbSphere & that )
        {
            // Assign parent class members.
            mBody               = & mRigidBody ;
            mCollisionShape     = & mSphereShape ;
            mFrictionProperties = that.GetFrictionProperties() ;
            mThermalProperties  = that.GetThermalProperties() ;

            // Copy members of this class.
            mRigidBody          = that.mRigidBody ;
            mSphereShape        = that.mSphereShape ;
            mVolume             = that.mVolume ;
            return * this ;
        }

        RbSphere( const RbSphere & that )
            : PhysicalObject( & mRigidBody , & mSphereShape , that.mFrictionProperties , that.mThermalProperties )
            , mRigidBody( that.mRigidBody )
            , mSphereShape( that.mSphereShape )
        {
            this->operator=( that ) ;
        }

    private:
        Impulsion::RigidBody    mRigidBody      ;
        Collision::SphereShape  mSphereShape    ;
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
