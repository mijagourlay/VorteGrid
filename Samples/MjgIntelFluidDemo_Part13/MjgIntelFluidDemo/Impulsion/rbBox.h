/** \file rbBox.h

    \brief Right rectangular rigid body.

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef RB_BOX_H
#define RB_BOX_H

#include <math.h>

#include "Core/Math/vec3.h"
#include "Core/wrapperMacros.h"
#include "Collision/sphereShape.h"
#include "rigidBody.h"
#include "physicalObject.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/** Right rectangular rigid body.
*/
class RbBox : public Impulsion::PhysicalObject
{
    public:
        /// Construct a right rectangular rigid body.
        //RbBox() : PhysicalObject( & mRigidBody , & mSphereShape ) , mVolume( -1.0f ) {  }

        RbBox( const Vec3 & vPos , const Vec3 & vVelocity , const float & fMass , const Vec3 & dimensions )
            : PhysicalObject( & mRigidBody , & mPolytope )
            , mRigidBody( vPos , vVelocity , fMass )
            , mDimensions( dimensions )
        {
            const Vec3 dims2( POW2( dimensions.x ) , POW2( dimensions.y ) , POW2( dimensions.z ) ) ;
            Mat33 inertiaTensor( Mat33_xIdentity ) ;
            inertiaTensor.x.x = fMass * ( dims2.y + dims2.z ) / 12.0f ;
            inertiaTensor.y.y = fMass * ( dims2.x + dims2.z ) / 12.0f ;
            inertiaTensor.z.z = fMass * ( dims2.x + dims2.y ) / 12.0f ;
            GetBody()->SetInverseInertiaTensor( inertiaTensor.Inverse() ) ;

            ConvexPolytope_MakeBox( mPolytope , dimensions ) ;

            GetThermalProperties().SetTemperature( sAmbientTemperature ) ;
            GetThermalProperties().SetThermalConductivity( 500.0f ) ;
            GetThermalProperties().SetOneOverHeatCapacity( 0.00001f );
        }

        RbBox & operator=( const RbBox & that )
        {
            // Assign parent class members.
            mBody               = & mRigidBody ;
            mCollisionShape     = & mPolytope ; // TODO: FIXME
            mFrictionProperties = that.GetFrictionProperties() ;
            mThermalProperties  = that.GetThermalProperties() ;

            // Copy members of this class.
            mRigidBody          = that.mRigidBody ;
            mPolytope           = that.mPolytope ;
            mVolume             = that.mVolume ;
            mDimensions         = that.mDimensions ;
            return * this ;
        }

        RbBox( const RbBox & that )
            : PhysicalObject( & mRigidBody , & mPolytope , that.mFrictionProperties , that.mThermalProperties )
            , mRigidBody( that.mRigidBody )
            , mPolytope( that.mPolytope )
            , mDimensions( that.mDimensions )
        {
            this->operator=( that ) ;
        }

        const Vec3 & GetDimensions() const { return mDimensions ; }

    private:
        Impulsion::RigidBody        mRigidBody  ;
        Collision::ConvexPolytope   mPolytope   ;
        Vec3                        mDimensions ;
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
