/** \file renderScene.cpp

    \brief Scene rendering routines for InteSiVis.

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "inteSiVis.h"

#include "Core/Performance/perfBlock.h"



/** Perform initializations that cannot happen before render device is initialized.

    For example, the device cannot generate a texture until OpenGL has been initialized.
*/
void InteSiVis::InitializeQdRendering()
{
    PERF_BLOCK( InteSiVis__InitializeQdRendering ) ;

    ASSERT( ! mInitialized ) ; // This must be the first time this app has displayed anything.

    CreateQdMaterials() ;

    mInitialized = true ;
}




/** Render vortex particles.
*/
void InteSiVis::QdRenderVortonDiagnostics()
{
    PERF_BLOCK( InteSiVis__QdRenderVortonDiagnostics ) ;

    ASSERT( mFluidScene.GetVortonRenderingStyle() != FluidScene::VORTON_RENDER_NONE ) ;

    if( mDiagnosticText == DIAG_TEXT_FULL )
    {
        QdRenderVortonDiagnosticText() ;
    }

    if(     ( mFluidScene.GetVortonRenderingStyle() == FluidScene::VORTON_RENDER_VECTORS                 )
        ||  ( mFluidScene.GetVortonRenderingStyle() == FluidScene::VORTON_RENDER_PARTICLES_AND_VECTORS   )
        ||  ( mFluidScene.GetVortonRenderingStyle() == FluidScene::VORTON_RENDER_ALL                     ) )
    {
        QdRenderVortonDiagnosticVectors() ;
    }
}




#if USE_ORIENTED_LIT_PARTICLES
/** Cheap lighting hack to make an internal glow effect inside smoke.

    \param vGlowPos - position of point light

*/
static void SetGlow( const Vec3 vGlowPos )
{
    PERF_BLOCK( SetGlow ) ;

    glEnable( GL_LIGHTING ) ;

    // Code below here belongs in "QdLight"
    // Simplify (and speed up) specular computation
    glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE ) ;

    // Global ambient light
    {
        static const int iLight = GL_LIGHT0 ;
        float globalAmbientLight[] = { 1.0f , 1.0f , 1.0f , 1.0f } ;
        glLightModelfv( GL_LIGHT_MODEL_AMBIENT , globalAmbientLight ) ;
        glEnable( iLight ) ;
    }

    // Glow point light
    {
        glPushMatrix() ;
        static const int iLight = GL_LIGHT1 ;
        const Vec4 diffuse( 18.0 , 9.0 , 0.0 , 1.0 ) ;
        glLightfv( iLight , GL_DIFFUSE , (float*) & diffuse ) ;
        Vec4 pos( vGlowPos , 1.0 ) ;
        glLightfv( iLight , GL_POSITION , (float*) & pos ) ;
        glLightf( iLight , GL_CONSTANT_ATTENUATION , 0.0f ) ;
        glLightf( iLight , GL_LINEAR_ATTENUATION , 0.0f ) ;
        glLightf( iLight , GL_QUADRATIC_ATTENUATION , 10.0 ) ;
        glEnable( iLight ) ;
        glPopMatrix() ;
    }

    // Directional light from above
    {
        glPushMatrix() ;
        static const int iLight = GL_LIGHT2 ;
        const Vec4 ambient( 0.05f , 0.05f , 0.05f , 1.0f ) ;
        glLightfv( iLight , GL_AMBIENT , (float*) & ambient ) ;
        const Vec4 diffuse( 1.0f , 1.0f , 1.0f , 1.0f ) ;
        glLightfv( iLight , GL_DIFFUSE , (float*) & diffuse ) ;
        const Vec4 position( 0.0f , 0.0f , -1.0f , 0.0f ) ;
        glLightfv( iLight , GL_POSITION , (float*) & position ) ;
        glEnable( iLight ) ;
        glPopMatrix() ;
    }
}
#endif




/** Render particle diagnostics.
*/
void InteSiVis::QdRenderParticleDiagnostics()
{
    PERF_BLOCK( InteSiVis__QdRenderParticleDiagnostics ) ;

//#if USE_ORIENTED_LIT_PARTICLES
//
//    switch( mScenario )
//    {
//        case 0: case 1:
//            const Vec3      vCoM        = Particles::ComputeGeometricCenter( mTracerPclGrpInfo.mParticleGroup->GetParticles() ) ;
//            const Vec3 &    vMin        = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid().GetMinCorner() ;
//            const Vec3 &    vExtent     = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid().GetExtent() ;
//            const Vec3      vMax        = vMin + vExtent ;
//            const Vec3      vFireball   = Vec3( 0.5f * ( vCoM.x + vMax.x ) , vCoM.y , vCoM.z ) ;
//            SetGlow( vFireball ) ;
//        break ;
//    }
//#else
    QdLight::DisableLights() ;
//#endif

    if( mFluidScene.GetVortonRenderingStyle() != FluidScene::VORTON_RENDER_NONE )
    {   // Vorton rendering is enabled.
        QdRenderVortonDiagnostics() ;
    }

    if(     ( mFluidScene.GetVortonRenderingStyle() == FluidScene::VORTON_RENDER_PATHLINES )
        ||  ( mFluidScene.GetVortonRenderingStyle() == FluidScene::VORTON_RENDER_ALL       )  )
    {   // Vorton Pathline rendering is enabled.
#if ENABLE_PARTICLE_POSITION_HISTORY
        QdRenderDiagnosticPathlines( reinterpret_cast< VECTOR< Particle > & >( * mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortons() ) ) ;
#endif
    }
}
