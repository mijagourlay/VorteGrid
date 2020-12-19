/** \file target.h

    \brief Canvas on which to draw.

    \author Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_TARGET_H
#define PEGASYS_RENDER_TARGET_H

#include "Core/Containers/slist.h"

#include "Render/Device/viewport.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class System ;  // Forward declaration

        /** Canvas on which to draw.

            A Target contains one or more Viewports, each of which has its own Camera.
            A Window is a kind of Target, which usually maps to some on screen.
            Targets can also be off-screen textures.
        */
        class Target
        {
            public:
                typedef SLIST< Viewport * >         ViewportContainer ;
                typedef ViewportContainer::Iterator ViewportIterator ;

                typedef unsigned TypeId ;   ///< Type identifier, used for simple runtime type information.

                static const TypeId sTypeId = 'TRGT' ; ///< Type identifier for this class

                enum Parameters
                {
                    iType           ,   ///< Type identifier
                    iWidth          ,   ///< Width, in pixels
                    iHeight         ,   ///< Height, in pixels
                    bDepth          ,   ///< Whether this target has a depth buffer
                    NUM_PARAMETERS
                } ;

                Target( System * renderSystem , TypeId typeId ) ;
                virtual ~Target() ;

                void Clear() ;

                virtual void PreRenderViewports() ;

                const TypeId & GetTypeId() { return mTypeId ; }

                void AddViewport( Camera * camera , const Vec4 & viewport = Vec4( 0.0f , 0.0f , 1.0f , 1.0f ) ) ;
                void RenderViewports() ;

                ViewportContainer & GetViewports() { return mViewports ; }

                System * GetRenderSystem() { return mRenderSystem ; }

                void SetLeft( int left )        { mLeft   = left    ; }
                void SetTop( int top )          { mTop    = top     ; }
                void SetWidth( int width )      { mWidth  = width   ; }
                void SetHeight( int height )    { mHeight = height  ; }
                void SetDepth( bool depth )     { mDepth  = depth   ; }

                const int & GetLeft() const     { return mLeft   ; }
                const int & GetTop() const      { return mTop    ; }
                const int & GetWidth() const    { return mWidth  ; }
                const int & GetHeight() const   { return mHeight ; }
                const bool & GetDepth() const   { return mDepth  ; }

            protected:
                TypeId  mTypeId ;   ///< Type identifier
                int     mLeft   ;   ///< Position, in pixels, of left edge this target
                int     mTop    ;   ///< Position, in pixels, of top edge this target
                int     mWidth  ;   ///< Width, in pixels, of this target
                int     mHeight ;   ///< Height, in pixels, of this target
                bool    mDepth  ;   ///< Whether this target has a depth buffer

            private:
                System  *           mRenderSystem   ;   ///< Render system that owns this target
                ViewportContainer   mViewports      ;   ///< Viewports within this render target
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
