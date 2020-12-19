/** \file iRenderable.h

    \brief Interface for a renderable object.

    \author Copyright 2009-2014 MJG; All rights reserved.
*/
#ifndef I_RENDERABLE_H
#define I_RENDERABLE_H

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class IRenderable
        {
        public:
            IRenderable () {}
            virtual ~IRenderable() {}

            /// Render this object.
            virtual void Render() = 0 ;
        } ;
    } ;
} ;

#endif
