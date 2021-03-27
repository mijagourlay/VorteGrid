/** \file effect.cpp

    \brief Rendering effect.

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/material.h"

#include "Render/Resource/textureSampler.h"
#include "Render/Resource/technique.h"

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

        /** Construct render effect.

            This automatically adds the first Technique.
        */
        Effect::Effect( size_t numTechniquesToReserve , size_t numPassesToReserveForFirstTechnique )
        {
            PERF_BLOCK( Effect__Effect ) ;

            mTechniques.Reserve( numTechniquesToReserve ) ;

            AddNewTechnique( numPassesToReserveForFirstTechnique ) ;
        }




        /** Destruct render effect.
        */
        Effect::~Effect()
        {
            PERF_BLOCK( Effect__dtor ) ;

            ClearTechniques() ;
        }




        /** Delete and remove all techniques from this effect.
        */
        void Effect::ClearTechniques()
        {
            PERF_BLOCK( Effect__ClearTechniques ) ;

            while( ! mTechniques.Empty() )
            {   // For each technique in this effect...
                Technique * technique = mTechniques.Back() ;
                delete technique ;
                mTechniques.PopBack() ;
            }
        }




        /** Add a new technique to this effect.
        */
        Technique * Effect::AddNewTechnique( size_t numPassesToReserve )
        {
            PERF_BLOCK( Effect__AddNewTechnique ) ;

            Technique * newTechnique = NEW Technique( this , numPassesToReserve ) ;
            mTechniques.PushBack( newTechnique ) ;
            return newTechnique ;
        }




        /** Release reference to this object and delete it if that was the last reference.
        */
        void Effect::ReleaseReference()
        {
            PERF_BLOCK( Effect__ReleaseReference ) ;

            if( _ReleaseReference() )
            {   // That was the last reference to this object.
                delete this ;
            }
        }




    } ;
} ;




#if defined( _DEBUG )

void PeGaSys::Render::Effect::UnitTest( void )
{
    DebugPrintf( "Effect::UnitTest ----------------------------------------------\n" ) ;

    {
        Effect effect ;
    }

    DebugPrintf( "Effect::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif