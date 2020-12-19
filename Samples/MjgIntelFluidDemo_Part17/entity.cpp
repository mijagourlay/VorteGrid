/** \file entity.cpp

    \brief Simulation entity that binds a physical object to its render model.
*/
#include "entity.h"

#include "Render/qdModel.h"
#include "Impulsion/physicalObject.h"

void Entity::Update()
{
    mRenderModel->SetPositionOrientation( mPhysicalObject->GetBody()->GetPosition() , mPhysicalObject->GetBody()->GetOrientation() ) ;
}
