/** \file window.h

    \brief Window render target

    \author Written and copyright 2010-2014 Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PEGASYS_RENDER_WINDOW_H
#define PEGASYS_RENDER_WINDOW_H

#include "Core/Containers/vector.h"

#include "Render/Device/target.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Window render target.
        */
        class Window : public Target
        {
        public:
            static const unsigned sTypeId = 'WNDW' ; ///< Type identifier for this class

            enum Parameters
            {
                sName           = Target::NUM_PARAMETERS    ,   ///< Name of window
                bFullScreen     ,   ///< Whether window should be full-screen
                iLeft           ,   ///< Position, in pixels, of left edge of window
                iTop            ,   ///< Position, in pixels, of top edge of window
                NUM_PARAMETERS
            } ;

            enum StateE
            {
                POSITION_X  ,
                POSITION_Y  ,
                WIDTH       ,
                HEIGHT      ,
                NUM_STATES
            } ;

            Window( class Render::System * renderSystem ) ;
            virtual ~Window() ;

            virtual /* override */ void PreRenderViewports() ;

            void SetName( const char * name ) ;

            void SetFullScreen( bool fullScreen )
            {
                ASSERT( ! fullScreen ) ; // Full-screen support not yet implemented.
                mFullScreen = fullScreen ;
            }

            virtual void    Create() = 0 ;
            virtual void    Change() = 0 ;
            virtual int     GetState( StateE state ) = 0 ;

#       if defined( _DEBUG )
            static void UnitTest() ;
#       endif

        protected:
            void FreeName() ;

            char *  mName       ;
            bool    mFullScreen ;

        private:
            void DetectAndHandleResize() ;
            virtual void    SetWindow() = 0 ;

        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

    } ;
} ;

#endif
