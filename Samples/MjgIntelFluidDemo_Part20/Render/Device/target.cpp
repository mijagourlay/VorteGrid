/** \file target.cpp

    \brief Canvas on which to draw

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Device/target.h"

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

        /** Construct canvas on which to draw.
        */
        Target::Target( System * renderSystem , TypeId typeId )
            : mRenderSystem( renderSystem )
            , mTypeId( typeId )
            , mLeft  ( 0 )
            , mTop   ( 0 )
            , mWidth ( 640 )
            , mHeight( 480 )
        {
            PERF_BLOCK( Target__Target ) ;
        }




        /** Destruct canvas on which to draw.
        */
        Target::~Target()
        {
            PERF_BLOCK( Target__dtor ) ;

            Clear() ;
        }





        /** Erase all viewports from this target.
        */
        void Target::Clear()
        {
            PERF_BLOCK( Target__Clear ) ;

            while( ! mViewports.Empty() )
            {   // For each viewport this target owns...
                Viewport * viewport = mViewports.Front() ;  // Get viewport address from front of list.
                delete viewport ;                           // Delete object.
                mViewports.PopFront() ;                     // Remove address from list.
            }
        }




        /** Add a viewport for this target.
        */
        void Target::AddViewport( Camera * camera , const Vec4 & viewport )
        {
            PERF_BLOCK( Target__AddViewport ) ;

            Viewport * pViewport = NEW Viewport( camera , this , viewport ) ;
            // TODO: Set setAspectRatio on camera given target/viewport shape
            mViewports.PushBack( pViewport ) ;
        }




        /** Perform type-specific operations, such as DetectAndHandleResize, and set current window.
        */
        /* virtual */ void Target::PreRenderViewports()
        {
            PERF_BLOCK( Target__PreRenderViewports ) ;
        }




        /** Render viewports associated with this target.
        */
        void Target::RenderViewports( const double & currentVirtualTimeInSeconds )
        {
            PERF_BLOCK( Target__RenderViewports ) ;

            // See comment in System::UpdateTargets.
            PreRenderViewports() ;

            for( ViewportIterator iter = GetViewports().Begin() ; iter != GetViewports().End() ; ++ iter )
            {
                Viewport * const & viewport = * iter ;
                viewport->TriggerCamera( currentVirtualTimeInSeconds ) ;
            }
        }




#if defined( _DEBUG )




        class TestTarget : public PeGaSys::Render::Target
        {
        public:
            static const TypeId sTypeId = 'TsTg' ; ///< Type identifier for this class

            TestTarget( class Render::System * renderSystem )
                : Target( renderSystem , sTypeId )
            {
            }

            ~TestTarget() {}

        private:
        } ;

        void PeGaSys_Render_Target_UnitTest( void )
        {
            DebugPrintf( "Target::UnitTest ----------------------------------------------\n" ) ;

            {
                TestTarget target( 0 ) ;

                target.SetWidth( 640 ) ;
                target.SetHeight( 480 ) ;
                target.SetDepth( true ) ;

                target.RenderViewports( 0.0 ) ;
            }

            DebugPrintf( "Target::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
