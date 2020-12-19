/** \file renderState.cpp

    \brief Render state

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/renderState.h"

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

        /** Construct render state object.
        */
        RenderState::RenderState()
        {
            PERF_BLOCK( RenderState__RenderState ) ;

            mShadeMode = SHADE_MODE_SMOOTH ;
        }




        /** Destruct render state object.
        */
        //RenderState::~RenderState()
        //{
        //}




        /** Release reference to this object and delete it if that was the last reference.
        */
        void RenderState::ReleaseReference()
        {
            PERF_BLOCK( RenderState__ReleaseReference ) ;

            if( _ReleaseReference() )
            {   // That was the last reference to this object.
                delete this ;
            }
        }



#if defined( _DEBUG )

        class TestRenderState : public RenderState
        {
        public:
            virtual void Apply()
            {
            }
        } ;

        void PeGaSys::Render::RenderState::UnitTest()
        {
            DebugPrintf( "RenderState::UnitTest ----------------------------------------------\n" ) ;

            {
                TestRenderState renderState ;
            }

            DebugPrintf( "RenderState::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
