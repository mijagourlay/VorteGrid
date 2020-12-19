/** \file dynaWorld.h

    \brief Container for physically dynamic objects.

    \author Copyright 2011 MJG; All rights reserved.
*/
#ifndef IMPULSION_DYNAMICS_WORLD_H
#define IMPULSION_DYNAMICS_WORLD_H

#include "Core/Containers/vector.h"

#include "Impulsion/rigidBody.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace Impulsion
{
    /** Container for physically dynamic objects.
    */
    class DynaWorld
    {
        public:
            DynaWorld() ;
            ~DynaWorld() ;

            void    AddBody( RigidBody * body )
            {
                mBodies.PushBack( body ) ;
            }
            void    UpdateBodies( float timeStep ) ;

        private:
            typedef VECTOR< RigidBody * >   BodyContainer   ;
            typedef BodyContainer::Iterator BodyIterator    ;
            BodyContainer                   mBodies         ;   ///< Rigid bodies
    } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

} ;

#endif
