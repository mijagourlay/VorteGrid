/** \file dynaWorld.cpp

    \brief Container for physically dynamic objects

    \author Copyright 2011 MJG; All rights reserved.
*/

#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#include "Impulsion/dynaWorld.h"

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace Impulsion {

/** Construct container for physically dynamic objects.
*/
DynaWorld::DynaWorld()
{
}




/** Destruct container for physically dynamic objects.
*/
DynaWorld::~DynaWorld()
{
}




void DynaWorld::UpdateBodies( float timeStep )
{
    const BodyIterator end = mBodies.End() ;
    for( BodyIterator iter = mBodies.Begin() ; iter != end ; ++ iter )
    {
        RigidBody * & body = * iter ;
        body->Update( timeStep ) ;
    }
}


} ;


#if defined( UNIT_TEST )

void PeGaSys_Impulsion_World_UnitTest( void )
{
    DebugPrintf( "Impulsion::DynaWorld::UnitTest ----------------------------------------------\n" ) ;


    DebugPrintf( "Impulsion::DynaWorld::UnitTest: THE END ----------------------------------------------\n" ) ;
}

#endif
