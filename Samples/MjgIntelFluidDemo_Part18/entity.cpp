/** \file entity.cpp

    \brief Simulation entity that binds a physical object to its render model.
*/
#include "entity.h"

#include "QdRender/qdModel.h"
#include "Impulsion/physicalObject.h"
#include "Render/Scene/model.h"

#include <Core/Performance/perfBlock.h>

void Entity::Update()
{
    PERF_BLOCK( Entity__Update ) ;

    ASSERT( mSceneNode ) ;
    if( mSceneNode )
    {
        mSceneNode->SetPosition( mPhysicalObject->GetBody()->GetPosition() ) ;
        mSceneNode->SetOrientation( mPhysicalObject->GetBody()->GetOrientation() ) ;
    }
}
