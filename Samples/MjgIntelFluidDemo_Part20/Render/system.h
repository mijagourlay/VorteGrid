/** \file system.h

    \brief Render system

    \author Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_SYSTEM_H
#define PEGASYS_RENDER_SYSTEM_H

#include "Core/Containers/slist.h"

#include "Render/Device/target.h"
#include "Render/Device/api.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Render system.
        */
        class System
        {
            public:
                System( ApiBase * renderApi ) ;
                ~System() ;

                /// Return address of Render API object.
                ApiBase * GetApi() { return mApi ; }

                /** Add a Render Target such as a window or texture.
                    \param target   Render Target
                */
                void    AddTarget( Target * target )
                {
                    mTargets.PushBack( target ) ;
                }

                void    UpdateTargets( const double & currentVirtualTimeInSeconds ) ;

            private:
                typedef SLIST< Target * >               TargetContainer     ;
                typedef TargetContainer::Iterator       TargetIterator      ;
                typedef TargetContainer::ConstIterator  TargetConstIterator ;
                TargetContainer                         mTargets            ;   /// Render targets
                ApiBase *                               mApi                ;   /// Address of low-level render system device, which this object owns.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

#if defined( _DEBUG ) && ! defined( _XBOX )
extern void System_UnitTest() ;
#endif

    } ;
} ;

#endif
