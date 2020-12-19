/** \file viewport.h

    \brief Rectangular region of a render target through which a camera looks.

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/
#ifndef PEGASYS_RENDER_VIEWPORT_H
#define PEGASYS_RENDER_VIEWPORT_H

#include "Render/Device/diagnosticTextOverlay.h"

#include "Render/Scene/camera.h"

#include "Core/Containers/vector.h"


// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class Target ;
        class ApiBase ;

        /** Rectangular region of a render target through which a camera looks.
        */
        class Viewport
        {
        public:
            Viewport( Camera * camera , Target * target , const Vec4 & viewport ) ;
            virtual ~Viewport() ;

            void    TriggerCamera() ;

            const Camera * GetCamera() const    { return mCamera     ; }
            const Target * GetTarget() const    { return mTarget     ; }
            const float & GetRelLeft() const    { return mRelLeft    ; }
            const float & GetRelTop() const     { return mRelTop     ; }
            const float & GetRelWidth() const   { return mRelWidth   ; }
            const float & GetRelHeight() const  { return mRelHeight  ; }
            const Vec4  & GetClearColor() const { return mClearColor ; }

            DiagnosticTextOverlay & GetDiagnosticTextOverlay() { return mDiagnosticTextOverlay ; }

        private:
            Camera  *               mCamera                 ;   /// Camera used to view scene through this viewport.
            Target  *               mTarget                 ;   /// Render target this viewport belongs to
            float                   mRelLeft                ;   /// Position relative to target (in [0,1]) of left edge of this viewport
            float                   mRelTop                 ;   /// Position relative to target (in [0,1]) of top edge of this viewport
            float                   mRelWidth               ;   /// Width relative to target (in [0,1]) of this viewport
            float                   mRelHeight              ;   /// Height relative to target (in [0,1]) of this viewport
            Vec4                    mClearColor             ;   /// Color used to clear this viewport before rendering into it.
            DiagnosticTextOverlay   mDiagnosticTextOverlay  ;   /// Field of text used for display diagnostic messages
        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

    } ;
} ;

#endif
