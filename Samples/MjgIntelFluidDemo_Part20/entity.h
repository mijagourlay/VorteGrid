/** \file entity.h

    \brief Simulation entity that binds a physical object to its render model.
*/
#ifndef ENTITY_H
#define ENTITY_H

// Forward declarations
namespace Impulsion
{
    class PhysicalObject ;
} ;

namespace PeGaSys
{
    namespace Render
    {
        class SceneNodeBase ;
        class ModelNode ;
    } ;
} ;

class QdModel ;




/** Simulation entity that binds a physical object to its render model.
*/
class Entity
{
    public:
        Entity( Impulsion::PhysicalObject * physicalObject , PeGaSys::Render::ModelNode * modelNode = 0 )
            : mPhysicalObject( physicalObject )
            , mSceneNode( modelNode )
        {}

        ~Entity() {}

        void Update() ;

        Impulsion::PhysicalObject  *        mPhysicalObject ;   /// Physics simulation object associated with this entity, but this entity does not own it.
        PeGaSys::Render::ModelNode  *       mSceneNode      ;   /// Scene associated with this entity, but this entity does not own it; a SceneManager does.
} ;

#endif
