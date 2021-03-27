/** \file iSceneNode.h

    \brief Interface for a node in a scene graph.

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/
#ifndef I_SCENE_NODE_H
#define I_SCENE_NODE_H

#include "Render/Scene/iRenderable.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Interface for a node in a scene graph.

            \see ISceneManager

        */
        class ISceneNode : public IRenderable
        {
        public:
            class IVisitor
            {
            public:
                /** Operation to perform on each SceneNode visited.
                    \param sceneNode    ISceneNode to visit.
                */
                virtual void operator()( ISceneNode & sceneNode ) = 0 ;
            } ;

            enum ClearDeletePolicy { DO_NOT_DELETE_NOTES , DELETE_NODES } ;

            virtual ~ISceneNode() {}

            /// Return parent of this node, or NULL if it is the root.
            virtual ISceneNode *    GetParent() = 0 ;

            /// Add the given node as a child to this one.
            virtual void            AdoptChild( ISceneNode * ) = 0 ;

            /// Remove the given node, which must be an immediate child of this one.
            virtual void            OrphanChild( ISceneNode * ) = 0 ;

            /** Visit this node, then recursively visit each descendant node in the scene.
                \param visitor  Visitor to apply to each scene node.
            */
            virtual void            Visit( IVisitor * visitor ) = 0 ;

            /// Remove all child nodes this node parents.
            virtual void            Clear( ClearDeletePolicy deleteNode = DELETE_NODES ) = 0 ;
        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

    } ;
} ;

#endif
