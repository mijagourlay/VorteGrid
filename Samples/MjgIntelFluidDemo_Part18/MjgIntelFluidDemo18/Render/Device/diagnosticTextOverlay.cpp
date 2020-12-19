/** \file diagnosticText.cpp

    \brief Diagnostic text overlay.

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/

#include "Render/system.h"
#include "Render/Device/target.h"
#include "Render/Device/diagnosticTextOverlay.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#include <stdarg.h> // For va_start

#include <algorithm>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct field of text used to display diagnostic messages.
        */
        DiagnosticTextOverlay::DiagnosticTextOverlay( ApiBase * renderApi , size_t numLines , size_t maxCharactersPerLine )
            : mRenderApi( renderApi )
            , mMaxCharsPerLine( maxCharactersPerLine )
            , mNumLinesPopulated( 0 )
            , mDisplayEnabled( true )
        {
            PERF_BLOCK( DiagnosticTextOverlay__DiagnosticTextOverlay ) ;

            mTextLines.Resize( numLines ) ;
            for( size_t line = 0 ; line < numLines ; ++ line )
            {   // For each line in the text buffer...
                // Allocate a string.
                mTextLines[ line ] = new char[ maxCharactersPerLine ] ;
            }
        }




        /** Destruct field of text used to display diagnostic messages.
        */
        DiagnosticTextOverlay::~DiagnosticTextOverlay()
        {
            PERF_BLOCK( DiagnosticTextOverlay__dtor ) ;
        }




        /** Append line to end of text buffer.
        */
        void DiagnosticTextOverlay::AppendLine( const char * format , ... )
        {
            PERF_BLOCK( DiagnosticTextOverlay__AppendLine ) ;

            ASSERT( mNumLinesPopulated < mTextLines.Size() ) ; // Make sure text buffer has room for another line.

            char * stringBuffer = mTextLines[ mNumLinesPopulated ] ; // Select line from text buffer.

            // Write formatted string into text buffer line.
            va_list args ;
            va_start( args , format ) ;
            _vsnprintf( stringBuffer , mMaxCharsPerLine , format , args ) ;
            va_end( args ) ;

            ++ mNumLinesPopulated ; // Count new text line.
        }




        /** Clear text buffer.
        */
        void DiagnosticTextOverlay::Clear()
        {
            PERF_BLOCK( DiagnosticTextOverlay__Clear ) ;

            for( size_t line = 0 ; line < mNumLinesPopulated ; ++ line )
            {   // For each populated line in the text buffer...
                // Empty the line.
                mTextLines[ line ][ 0 ] = '\0' ;
            }
            mNumLinesPopulated = 0 ; // Indicate text buffer has no populated lines.
        }




        /** Render field of text used to display diagnostic messages.
        */
        void DiagnosticTextOverlay::Render()
        {
            PERF_BLOCK( DiagnosticTextOverlay__Render ) ;

            ASSERT( mNumLinesPopulated <= mTextLines.Size() ) ;
            static const float verticalSpacing = 10.0f ;
            for( size_t line = 0 ; line < mNumLinesPopulated ; ++ line )
            {   // For each line of text in this overlay...
                float yPos = verticalSpacing * (float) ( line + 1 ) ;
                // Render line by calling api->RenderSimpleText
                mRenderApi->RenderSimpleText( mTextLines[ line ] , Vec3( 0.0f , yPos , 0.0f ) , /* use screen space */ true ) ;
            }
            // TODO (maybe): Restore previous projection. -- if above routine removes screen-space parameter
        }




    } ;
} ;




#if defined( _DEBUG )

void PeGaSys_Render_DiagnosticTextOverlay_UnitTest()
{
    DebugPrintf( "DiagnosticTextOverlay::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::DiagnosticTextOverlay diagnosticTextOverlay( NULL /* TODO: Supply API object */ ) ;

        diagnosticTextOverlay.Render() ;
    }

    DebugPrintf( "DiagnosticTextOverlay::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif
