/** \file effect.h

    \brief Rendering effect.

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_EFFECT_H
#define PEGASYS_RENDER_EFFECT_H

#include "Core/Containers/vector.h"
#include "Core/Containers/intrusivePtr.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class TextureSampler    ; // Forward declaration
        class Technique         ; // Forward declaration

        /** Rendering effect.
        */
        class Effect : public RefCountedMixin
        {
            public:
                typedef TextureSampler *                    TextureSamplerPtr       ;
                typedef VECTOR< TextureSamplerPtr >         TextureSamplerContainer ;
                typedef TextureSamplerContainer::Iterator   TextureSamplerIterator  ;

                typedef Technique *                         TechniquePtr            ;
                typedef VECTOR< TechniquePtr >              TechniqueContainer      ;
                typedef TechniqueContainer::Iterator        TechniqueIterator       ;

                Effect() ;
                ~Effect() ;

                Technique * AddNewTechnique() ;

                const TechniqueContainer & GetTechniques() const { return mTechniques ; }

                void ReleaseReference() ; // Used by IntrusivePtr (indirectly, through global ReleaseReference)

            #if defined( _DEBUG )
                static void UnitTest() ;
            #endif

            private:
                TextureSamplerContainer mTextureSampler ;   ///< Samplers used to access textures
                TechniqueContainer      mTechniques     ;   ///< Rendering techniques associated with this material

                friend class Technique ; // To grant access to _ReleaseReference so they can share refcount.

                void ClearTechniques() ;
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

        /// Add reference to a Effect, for use by IntrusivePtr.
        inline void AddReference( Effect * effect )     { effect->AddReference() ; }

        /// Release reference to a Effect, for use by IntrusivePtr.
        inline void ReleaseReference( Effect * effect ) { effect->ReleaseReference() ; }

    } ;
} ;

#endif
