/** \file physicalObject.h

    \brief Representation of a physical object that can collide and move.

    \author Copyright 2012 MJG; All rights reserved.
*/
#ifndef IMPULSION_PHYSICAL_OBJECT_H
#define IMPULSION_PHYSICAL_OBJECT_H

#include "Impulsion/rigidBody.h"
#include "Collision/convexPolytope.h"

namespace Impulsion
{

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

/** Friction properties of a physical object.

    Technically, friction is a property of pairs of objects in contact, not of
    individual objects.  This is a simplification.

*/
struct FrictionProperties
{
    FrictionProperties()
        : mSlidingFriction( 0.0f )
        , mStaticFriction( 0.0f )
        , mRollingFriction( 0.0f )
        , mRestitution( 1.0f )
    {}
    float   mSlidingFriction    ;
    float   mStaticFriction     ;
    float   mRollingFriction    ;
    float   mRestitution        ;
} ;




struct ThermalProperties
{
    ThermalProperties( float temperature = 0.0f , float thermalConductivity = 0.0f , float oneOverHeatCapacity = 1.0f )
        : mTemperature( temperature )
        , mThermalConductivity( thermalConductivity )
        , mOneOverHeatCapacity( oneOverHeatCapacity )
    {}

    void SetTemperature( float temperature )
    {
        ASSERT( temperature > 0.0f ) ;
        mTemperature = temperature ;
    }

    const float & GetTemperature() const { return mTemperature ; }

    void SetThermalConductivity( float thermalConductivity )
    {
        ASSERT( thermalConductivity >= 0.0f ) ;
        mThermalConductivity = thermalConductivity ;
    }

    const float & GetThermalConductivity() const { return mThermalConductivity ; }

    void SetOneOverHeatCapacity( float oneOverHeatCapacity )
    {
        ASSERT( oneOverHeatCapacity > 0.0f ) ;
        mOneOverHeatCapacity = oneOverHeatCapacity ;
    }

    const float & GetOneOverHeatCapacity() const { return mOneOverHeatCapacity ; }

private:

    float   mTemperature            ;   ///< Temperature of body -- heat per mass.
    float   mThermalConductivity    ;   ///< Ability to transfer heat by contact.
    float   mOneOverHeatCapacity    ;   ///< Reciprocal of heat capacity, where heat capacity is specific heat times mass.
} ;




class PhysicalObject
{
    public:
        static const float sAmbientTemperature ;

        /** Construct a physical object.
        */
        PhysicalObject( Impulsion::RigidBody * body , Collision::ShapeBase * collisionShape )
            : mBody( body )
            , mCollisionShape( collisionShape )
            , mVolume( 0.0f )
        {}

        PhysicalObject( Impulsion::RigidBody * body , Collision::ShapeBase * collisionShape , const FrictionProperties & FrictionProperties , const ThermalProperties & thermalProperties )
            : mBody( body )
            , mCollisionShape( collisionShape )
            , mFrictionProperties( FrictionProperties )
            , mThermalProperties( thermalProperties )
            , mVolume( 0.0f )
        {}

        Impulsion::RigidBody *          GetBody()                       { return mBody ; }
        const Impulsion::RigidBody *    GetBody() const                 { return mBody ; }

        Collision::ShapeBase *          GetCollisionShape()             { return mCollisionShape ; }
        const Collision::ShapeBase *    GetCollisionShape() const       { return mCollisionShape ; }
        void SetCollisionShape( Collision::ShapeBase * collisionShape ) ;

        const FrictionProperties &  GetFrictionProperties() const   { return mFrictionProperties ; }
              ThermalProperties &   GetThermalProperties()          { return mThermalProperties ; }
        const ThermalProperties &   GetThermalProperties() const    { return mThermalProperties ; }

        const float & GetVolume() const { return mVolume ; }

    protected:
        RigidBody *             mBody               ;
        Collision::ShapeBase *  mCollisionShape     ;
        FrictionProperties      mFrictionProperties ;
        ThermalProperties       mThermalProperties  ;
        float                   mVolume             ;
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

extern void PhysicalObject_UpdateSystem( const VECTOR< PhysicalObject * > & physicalObjects , float timeStep , unsigned uFrame ) ;
extern Vec3 PhysicalObject_ComputeAngularMomentum( const VECTOR< PhysicalObject * > & physicalObjects ) ;

#if defined( UNIT_TEST )

extern void UnitTests() ;

#endif

} ;

#endif
