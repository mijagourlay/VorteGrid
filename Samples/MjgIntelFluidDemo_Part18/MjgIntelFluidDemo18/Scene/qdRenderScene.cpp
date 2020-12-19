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
void InteSiVis::QdRenderVortons()
{
    PERF_BLOCK( InteSiVis__QdRenderVortons ) ;

    ASSERT( mVortonRendering != VORTON_RENDER_NONE ) ;

    VortonSim &         rVortonSim  = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
    VECTOR< Vorton > &  vortons     = * rVortonSim.GetVortons() ;

    if(     ( VORTON_RENDER_DIAGNOSTIC_PARTICLES  == mVortonRendering )
        ||  ( VORTON_RENDER_PARTICLES_AND_VECTORS == mVortonRendering )
        ||  ( VORTON_RENDER_PATHLINES             == mVortonRendering )
        ||  ( VORTON_RENDER_ALL                   == mVortonRendering ) )
    {   // Render vortons as diagnostic particles.
        // ASSERT( vortons.empty() || vortons.data() == & vortons[0] ) ; // Modern STL supports vector.data() but not older versions like MSVS7.
        mDiagnosticVortonRenderer.SetParticleData( reinterpret_cast< char * >( & vortons.operator[]( 0 ) ) ) ; // Modern STL supports vector.data() but not older versions like MSVS7.
        mDiagnosticVortonRenderer.Render( mTimeNow , mTimeStep , vortons.Size() ) ;
        mSimpleVortonRenderer.SetParticleData( 0 ) ;
    }
    else if( VORTON_RENDER_SIMPLE_PARTICLES == mVortonRendering )
    {   // Render vortons as simple particles.
        mSimpleVortonRenderer.SetParticleData( reinterpret_cast< char * >( & vortons.operator[]( 0 ) ) ) ; // Modern STL supports vector.data() but not older versions like MSVS7.
        mSimpleVortonRenderer.Render( mTimeNow , mTimeStep , vortons.Size() ) ;
        mDiagnosticVortonRenderer.SetParticleData( 0 ) ;
    }
    else
    {   // Do not render vortons as particles.
        mDiagnosticVortonRenderer.SetParticleData( 0 ) ;
        mSimpleVortonRenderer.SetParticleData( 0 ) ;
    }

    if( mDiagnosticText == DIAG_TEXT_FULL )
    {
        QdRenderVortonDiagnosticText() ;
    }

    if(     ( mVortonRendering == VORTON_RENDER_VECTORS                 )
        ||  ( mVortonRendering == VORTON_RENDER_PARTICLES_AND_VECTORS   )
        ||  ( mVortonRendering == VORTON_RENDER_ALL                     ) )
    {
        QdRenderVortonDiagnosticVectors() ;
    }
}




/** Render tracer particles.
*/
void InteSiVis::QdRenderTracers()
{
#if ENABLE_FIRE
    PERF_BLOCK( InteSiVis__QdRenderTracers ) ;

    VECTOR< Particle > & tracers = mTracerPclGrpInfo.mParticleGroup->GetParticles() ;

    if( TRACER_RENDER_DIAGNOSTIC == mTracerRendering )
    {
        // Render tracers as diagnostic particles:
        mDiagnosticTracerRenderer.SetParticleData( (char*) & tracers[0] ) ;
        mDiagnosticTracerRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
    }
    else if( TRACER_RENDER_DYE == mTracerRendering )
    {
        // Render tracers as dye:
        mDyeRenderer.SetParticleData( (char*) & tracers[0] ) ;
        mDyeRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
    }
    else
    {
        if( ( TRACER_RENDER_FIRE == mTracerRendering ) || ( TRACER_RENDER_FUEL == mTracerRendering ) )
        {
            // Render tracers as fuel:
            mFuelRenderer.SetParticleData( (char*) & tracers[0] ) ;
            mFuelRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
        }

        if( ( TRACER_RENDER_FIRE == mTracerRendering ) || ( TRACER_RENDER_SMOKE == mTracerRendering ) )
        {
            // Render tracers as smoke:
            mSmokeRenderer.SetParticleData( (char*) & tracers[0] ) ;
            mSmokeRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
        }

        if( ( TRACER_RENDER_FIRE == mTracerRendering ) || ( TRACER_RENDER_FLAME == mTracerRendering ) )
        {
            // Render tracers as flame:
            mFlameRenderer.SetParticleData( (char*) & tracers[0] ) ;
            mFlameRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
        }
    }
#else
    {
        // Render tracers with texture that varies according to buoyancy:
        mDyeRenderer.SetParticleData( (char*) & tracer[0] ) ;
        mDyeRenderer.Render( mTimeNow , mTimeStep , tracer.Size() ) ;
    }
#endif
}




/** Render NonFluid particles.
*/
void InteSiVis::QdRenderNonFluidParticles()
{
    PERF_BLOCK( InteSiVis__QdRenderNonFluidParticles ) ;

    for( ParticleSystemManager::Iterator ips = mPclSysMgr.Begin() ; ips != mPclSysMgr.End() ; ++ ips )
    {
        ParticleSystem * pclSys = * ips ;
        for( ParticleSystem::Iterator ipg = pclSys->Begin() ; ipg != pclSys->End() ; ++ ipg )
        {
            ParticleGroup * pclGrp = * ipg ;
            VECTOR< Particle > & pcls = pclGrp->GetParticles() ;

            mSmokeRenderer.SetParticleData( reinterpret_cast< char * >( & pcls.operator[]( 0 ) ) ) ; // Modern STL supports vector.data() but not older versions like MSVS7.
            mSmokeRenderer.Render( mTimeNow , mTimeStep , pcls.Size() ) ;
        }
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




/** Render particles.
*/
void InteSiVis::QdRenderParticles()
{
#if USE_ORIENTED_LIT_PARTICLES
    PERF_BLOCK( InteSiVis__QdRenderParticles ) ;

    switch( mScenario )
    {
        case 0: case 1:
            const Vec3      vCoM        = Particles::ComputeGeometricCenter( mTracerPclGrpInfo.mParticleGroup->GetParticles() ) ;
            const Vec3 &    vMin        = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid().GetMinCorner() ;
            const Vec3 &    vExtent     = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid().GetExtent() ;
            const Vec3      vMax        = vMin + vExtent ;
            const Vec3      vFireball   = Vec3( 0.5f * ( vCoM.x + vMax.x ) , vCoM.y , vCoM.z ) ;
            SetGlow( vFireball ) ;
        break ;
    }
#else
    QdLight::DisableLights() ;
#endif

    // TODO: FIXME: Make particle rendering its own PclOp
    // HACK CODE START: Particle rendering should be another ParticleOperation but it is not.
    //                  Meanwhile, rendering non-fluid and fluid particles is essentially mutually exclusive.
    //                  One way to sort of repair this would be to make tracer and non-fluid rendering identical.
    //                  Even better, however, would be to make particle rendering a PclOp.
    if( ! mTracerPclGrpInfo.mParticleGroup->HasParticles() )
    {   // There is no fluid particle system.
        QdRenderNonFluidParticles() ;
    }
    else
    {   // There is a fluid particle system.
        if( mVortonRendering != VORTON_RENDER_NONE )
        {   // Vorton rendering is enabled.
            QdRenderVortons() ;
        }

        if(     ( mVortonRendering == VORTON_RENDER_PATHLINES )
            ||  ( mVortonRendering == VORTON_RENDER_ALL       )  )
        {   // Vorton Pathline rendering is enabled.
        #if ENABLE_PARTICLE_POSITION_HISTORY
            QdRenderDiagnosticPathlines( reinterpret_cast< VECTOR< Particle > & >( * mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortons() ) ) ;
        #endif
        }

        if( mTracerRendering != TRACER_RENDER_NONE )
        {   // Tracer rendering is enabled.
            QdRenderTracers() ;
        }
    }
}
