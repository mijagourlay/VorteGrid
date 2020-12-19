/** \file technique.h

    \brief Technique to render a mesh.

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_TECHNIQUE_H
#define PEGASYS_RENDER_TECHNIQUE_H

#include "Core/Containers/vector.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        // Forward declarations
        class Effect ;
        class Pass  ;

        /** Technique to render a mesh.
        */
        class Technique
        {
            public:
                typedef VECTOR< Pass * >                PassContainer       ;
                typedef PassContainer::Iterator         PassIterator        ;
                typedef PassContainer::ConstIterator    PassConstIterator   ;

                explicit Technique( Effect * parentEffect ) ;
                ~Technique() ;

                void AddReference()     ; // Used by IntrusivePtr (indirectly, through global ReleaseReference)
                void ReleaseReference() ; // Used by IntrusivePtr (indirectly, through global ReleaseReference)

                Pass * AddNewPass() ;
                const PassContainer & GetPasses() const { return mPasses ; }

            #if defined( _DEBUG )
                static void UnitTest() ;
            #endif

            private:
                Effect  *       mParentEffect   ;
                PassContainer   mPasses         ;   ///< List of passes when rendering mesh

                void ClearPasses() ;
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

        /// Add reference to a Technique, for use by IntrusivePtr.
        inline void AddReference( Technique * technique )     { technique->AddReference() ; }

        /// Release reference to a Technique, for use by IntrusivePtr.
        inline void ReleaseReference( Technique * technique ) { technique->ReleaseReference() ; }

    } ;
} ;

#endif
