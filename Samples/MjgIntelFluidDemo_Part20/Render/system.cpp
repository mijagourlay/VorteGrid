/** \file system.cpp

    \brief Render system

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/system.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct render system.

            \param  renderApi   Address of platform-dependent API object that this object takes ownership of.
        */
        System::System( ApiBase * renderApi )
            : mApi( renderApi )
        {
            PERF_BLOCK( Render__System__System ) ;
        }




        /** Destruct render system.
        */
        System::~System()
        {
            PERF_BLOCK( Render__System__dtor ) ;

            while( ! mTargets.Empty() )
            {   // For each target...
                Target * target = mTargets.Front() ;    // Get the address of the first target in the list.
                delete target ;                         // Delete the object.
                mTargets.PopFront() ;                   // Remove its address from the list.
            }
            delete mApi ;
        }




        /** Update all Render Targets in this System.

            Each Target has Viewports, which this routine renders by calling
            RenderViewports.
        */
        void System::UpdateTargets( const double & currentVirtualTimeInSeconds )
        {
            PERF_BLOCK( Render__System__UpdateTargets ) ;

            for( TargetIterator iter = mTargets.Begin() ; iter != mTargets.End() ; ++ iter )
            {   // For each render target...
                Target * const & target = * iter ;

                target->RenderViewports( currentVirtualTimeInSeconds ) ;
            }
        }




#if defined( _DEBUG )

        class TestSystem : public Render::System
        {
        public:
            TestSystem( PeGaSys::Render::ApiBase * renderApi )
                : System( renderApi )
            {
            }

            ~TestSystem() {}

        private:
        } ;

        void System_UnitTest()
        {
            DebugPrintf( "System::UnitTest ----------------------------------------------\n" ) ;

            {
                TestSystem system( 0 ) ;

                // Update render system.
                system.UpdateTargets( 0.0 ) ;
            }

            DebugPrintf( "System::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
