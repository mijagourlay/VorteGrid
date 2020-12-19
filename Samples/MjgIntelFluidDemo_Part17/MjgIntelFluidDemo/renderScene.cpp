/** \file renderScene.cpp

    \brief Scene rendering routines for InteSiVis.

*/

#include "inteSiVis.h"

#include "Core/Performance/perf.h"



/** Perform initializations that cannot happen before render device is initialized.

    For example, the device cannot generate a texture until OpenGL has been initialized.
*/
void InteSiVis::InitializeRendering()
{
    ASSERT( ! mInitialized ) ; // This must be the first time this app has displayed anything.

    CreateMaterials() ;

    mBallDisplayList = MakeSphere( 1.0f ) ;

    mInitialized = true ;
}




/** Render sky.
*/
void InteSiVis::RenderSky()
{
    QUERY_PERFORMANCE_ENTER ;
    mSkyMaterial.UseMaterial() ;
    QdLight::DisableLights() ;
    {
        glCullFace(GL_FRONT) ; // Render /inside/ of sphere
        glPushMatrix() ;
        static const float skyRadius = 500.0f ;
        const Vec3 & rEye    = mCamera.GetEye() ;
        glTranslatef( rEye.x , rEye.y , rEye.z ) ; // translate sky sphere to camera eye.
        glScalef( skyRadius , skyRadius , skyRadius ) ;
        glCallList( mBallDisplayList ) ;
        glPopMatrix() ;
    }
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_Sky ) ;
}




/** Render rigid bodies.
*/
void InteSiVis::RenderRigidBodies()
{
    const size_t numEntities = mEntities.Size() ;
    for( size_t idxEntity = 0 ; idxEntity < numEntities ; ++ idxEntity )
    {
        Entity & entity = mEntities[ idxEntity ] ;
        entity.Update() ;
        entity.mRenderModel->Render( & mLights , static_cast< float >( mTimeNow ) ) ;
    }
}




/** Render vortex particles.
*/
void InteSiVis::RenderVortons()
{
    ASSERT( mVortonRendering != VORTON_RENDER_NONE ) ;

    QUERY_PERFORMANCE_ENTER ;

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
        RenderVortonDiagnosticText() ;
    }

    if(     ( mVortonRendering == VORTON_RENDER_VECTORS                 )
        ||  ( mVortonRendering == VORTON_RENDER_PARTICLES_AND_VECTORS   )
        ||  ( mVortonRendering == VORTON_RENDER_ALL                     ) )
    {
        RenderVortonDiagnosticVectors() ;
    }

    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderVortons ) ;
}




/** Render tracer particles.
*/
void InteSiVis::RenderTracers()
{
#if ENABLE_FIRE

    VECTOR< Particle > & tracers = mTracerPclGrpInfo.mParticleGroup->GetParticles() ;

    if( TRACER_RENDER_DIAGNOSTIC == mTracerRendering )
    {
        // Render tracers as diagnostic particles:
        QUERY_PERFORMANCE_ENTER ;
        mDiagnosticTracerRenderer.SetParticleData( (char*) & tracers[0] ) ;
        mDiagnosticTracerRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
        QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
    }
    else if( TRACER_RENDER_DYE == mTracerRendering )
    {
        // Render tracers as dye:
        QUERY_PERFORMANCE_ENTER ;
        mDyeRenderer.SetParticleData( (char*) & tracers[0] ) ;
        mDyeRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
        QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
    }
    else
    {
        if( ( TRACER_RENDER_FIRE == mTracerRendering ) || ( TRACER_RENDER_FUEL == mTracerRendering ) )
        {
            // Render tracers as fuel:
            QUERY_PERFORMANCE_ENTER ;
            mFuelRenderer.SetParticleData( (char*) & tracers[0] ) ;
            mFuelRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
            QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
        }

        if( ( TRACER_RENDER_FIRE == mTracerRendering ) || ( TRACER_RENDER_SMOKE == mTracerRendering ) )
        {
            // Render tracers as smoke:
            QUERY_PERFORMANCE_ENTER ;
            mSmokeRenderer.SetParticleData( (char*) & tracers[0] ) ;
            mSmokeRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
            QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
        }

        if( ( TRACER_RENDER_FIRE == mTracerRendering ) || ( TRACER_RENDER_FLAME == mTracerRendering ) )
        {
            // Render tracers as flame:
            QUERY_PERFORMANCE_ENTER ;
            mFlameRenderer.SetParticleData( (char*) & tracers[0] ) ;
            mFlameRenderer.Render( mTimeNow , mTimeStep , tracers.Size() ) ;
            QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
        }
    }

#else
    // Render tracers with texture that varies according to buoyancy:
    QUERY_PERFORMANCE_ENTER ;
    mDyeRenderer.SetParticleData( (char*) & tracer[0] ) ;
    mDyeRenderer.Render( mTimeNow , mTimeStep , tracer.Size() ) ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderTracers ) ;
#endif
}




/** Render NonFluid particles.
*/
void InteSiVis::RenderNonFluidParticles()
{
    QUERY_PERFORMANCE_ENTER ;

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

    QUERY_PERFORMANCE_EXIT( InteSiVis_Render_RenderNonFluidParticles ) ;
}




#if USE_ORIENTED_SORTED_PARTICLES
/** Cheap lighting hack to make an internal glow effect inside smoke.

    \param vGlowPos - position of point light

*/
static void SetGlow( const Vec3 vGlowPos )
{
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
void InteSiVis::RenderParticles()
{
#if USE_ORIENTED_SORTED_PARTICLES
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
    QUERY_PERFORMANCE_ENTER ;
    QdLight::DisableLights() ;
    QUERY_PERFORMANCE_EXIT( InteSiVis_RenderParticles_DisableLightingForParticles ) ;
#endif

    // TODO: FIXME: Make particle rendering its own PclOp
    // HACK CODE START: Particle rendering should be another ParticleOperation but it is not.
    //                  Meanwhile, rendering non-fluid and fluid particles is essentially mutually exclusive.
    //                  One way to sort of repair this would be to make tracer and non-fluid rendering identical.
    //                  Even better, however, would be to make particle rendering a PclOp.
    if( ! mTracerPclGrpInfo.mParticleGroup->HasParticles() )
    {   // There is no fluid particle system.
        RenderNonFluidParticles() ;
    }
    else
    {   // There is a fluid particle system.
        if( mVortonRendering != VORTON_RENDER_NONE )
        {   // Vorton rendering is enabled.
            RenderVortons() ;
        }

        if(     ( mVortonRendering == VORTON_RENDER_PATHLINES )
            ||  ( mVortonRendering == VORTON_RENDER_ALL       )  )
        {   // Vorton Pathline rendering is enabled.
        #if ENABLE_PARTICLE_POSITION_HISTORY
            RenderDiagnosticPathlines( reinterpret_cast< VECTOR< Particle > & >( * mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortons() ) ) ;
        #endif
        }

        if( mTracerRendering != TRACER_RENDER_NONE )
        {   // Tracer rendering is enabled.
            RenderTracers() ;
        }
    }
}
