/** \file diagnosticText.h

    \brief Diagnostic text overlay.

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/
#ifndef PEGASYS_RENDER_DIAGNOSTIC_TEXT_H
#define PEGASYS_RENDER_DIAGNOSTIC_TEXT_H

#include "Core/Containers/vector.h"

#include "Render/Scene/iRenderable.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        // Forward declarations
        class ApiBase   ;

        /** Field of text used to display diagnostic messages.

            Technically, this is an Overlay, so it could (should?) be managed as an overlay,
            but (a) I have not yet implemented Overlays and (b) even if/when I do, diagnostic text
            would be a special overlay in that it would be the last one rendered, to guarantee it
            appears overtop everything else on the screen.  Still, it should eventually follow the same
            recipe (e.g. implement the same interface) as a regular Overlay.

        */
        class DiagnosticTextOverlay : public IRenderable
        {
        public:
            DiagnosticTextOverlay( ApiBase * renderApi , size_t numLines = 256 , size_t maxCharactersPerLine = 256 ) ;
            ~DiagnosticTextOverlay() ;

            void AppendLine( const char * format , ... ) ;
            void Clear() ;

            bool GetDisplayEnabled() const                  { return mDisplayEnabled ; }
            void SetDisplayEnabled( bool displayEnabled )   { mDisplayEnabled = displayEnabled ; }

            void Render() ;

        private:
            ApiBase *           mRenderApi          ;   /// Non-owned address of low-level render system device
            VECTOR< char * >    mTextLines          ;   /// Text buffer
            size_t              mMaxCharsPerLine    ;   /// Maximum number of characters per line
            size_t              mNumLinesPopulated  ;   /// Number of text lines that have data
            bool                mDisplayEnabled     ;   /// Whether to display diagnostic text
        } ;
    } ;
} ;

#endif
