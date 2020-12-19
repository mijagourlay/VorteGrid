/** \file technique.cpp

    \brief Technique to render a mesh

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/technique.h"

#include "Render/Resource/pass.h"
#include "Render/Resource/material.h"

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

        /** Construct render technique.
        */
        Technique::Technique( Effect * parentEffect )
            : mParentEffect( parentEffect )
        {
            PERF_BLOCK( Technique__Technique ) ;

            AddNewPass() ;
        }




        /** Destruct render technique.
        */
        Technique::~Technique()
        {
            PERF_BLOCK( Technique__dtor ) ;

            ClearPasses() ;
        }




        /** Add reference to this object through its parent.
        */
        void Technique::AddReference()
        {
            PERF_BLOCK( Technique__AddReference ) ;

            mParentEffect->AddReference() ;
        }




        /** Release reference to this object and delete it if that was the last reference.
        */
        void Technique::ReleaseReference()
        {
            PERF_BLOCK( Technique__ReleaseReference ) ;

            if( mParentEffect->_ReleaseReference() )
            {   // That was the last reference to the parent object.
                delete mParentEffect ;
            }
        }






        /** Delete and remove all passes from this technique.
        */
        void Technique::ClearPasses()
        {
            PERF_BLOCK( Technique__ClearPasses ) ;

            while( ! mPasses.Empty() )
            {   // For each pass in this technique...
                Pass * pass = mPasses.Back() ;
                delete pass ;
                mPasses.PopBack() ;
            }
        }




        /** Add a new render pass to this technique.

            \return Address of new pass just created and added.
        */
        Pass * Technique::AddNewPass()
        {
            PERF_BLOCK( Technique__AddNewPass ) ;

            Pass * newPass = NEW Pass( this ) ;
            mPasses.PushBack( newPass ) ;
            return newPass ;
        }




    } ;
} ;




#if defined( _DEBUG )

void PeGaSys::Render::Technique::UnitTest()
{
    DebugPrintf( "Technique::UnitTest ----------------------------------------------\n" ) ;

    {
        Technique technique( 0 ) ;
    }

    DebugPrintf( "Technique::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif