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

class QdModel ;




/** Simulation entity that binds a physical object to its render model.
*/
class Entity
{
    public:
        Entity( Impulsion::PhysicalObject * physicalObject , QdModel * renderModel )
            : mPhysicalObject( physicalObject )
            , mRenderModel( renderModel )
        {}

        ~Entity() {}

        void Update() ;

        Impulsion::PhysicalObject  *    mPhysicalObject ;
        QdModel                    *    mRenderModel    ;
} ;

#endif
