/** \file D3D9_window.cpp

    \brief Window render target for Direct3D 9

    \author Written and Copyright 2010-2014 Michael Jason Gourlay; All rights reserved.
*/

#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#if _MSC_VER && ( _MSC_VER < 1700 ) // Older compiler...
#   pragma warning( disable: 4995 ) // cstdio uses deprecated functions
#endif

#include <stdio.h>

#include <windows.h>
#include <d3d9.h>

// SAL annotations: As of the June 2010 DXSDK, including DxErr.h generates errors about "__in" being undeclared, probably because SAL annotations hadn't been invented or made public by then.
#ifndef __in
#   define __in
#   define __in_z
#   define __in_z_opt
#endif

// TODO: FIXME: Replace DXGetErrorDescription with WinSDK FormatMessage.
//#include <dxerr.h>
//#pragma comment( lib, "dxerr.lib" )

#include <strsafe.h>

#include "Render/Platform/DirectX9/D3D9_api.h" // for HROK

#include "Render/Platform/DirectX9/D3D9_window.h"


// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------

extern LPDIRECT3D9         g_pD3D       ; // Used to create the D3DDevice
extern LPDIRECT3DDEVICE9   g_pd3dDevice ; // Our rendering device

// Private functions -----------------------------------------------------------

// Formats a message string using the specified message and variable
// list of arguments.
//Example usage:
//{
//    LPWSTR pBuffer = NULL;
//    LPWSTR pMessage = L"%1!*.*s! %3 %4!*s!";
//    // The variable length arguments correspond directly to the format
//    // strings in pMessage.
//    pBuffer = GetFormattedMessage(pMessage, 4, 2, L"Bill", L"Bob", 6, L"Bill");
//    if (pBuffer)
//    {
//        // Buffer contains "  Bi Bob   Bill".
//        wprintf(L"Formatted message: %s\n", pBuffer);
//        LocalFree(pBuffer);
//    }
//    else
//    {
//        wprintf(L"Format message failed with 0x%x\n", GetLastError());
//    }
//}
LPWSTR GetFormattedMessage(LPWSTR pMessage, ...)
{
    LPWSTR pBuffer = NULL;

    va_list args = NULL;
    va_start(args, pMessage);

    FormatMessage(FORMAT_MESSAGE_FROM_STRING |
        FORMAT_MESSAGE_ALLOCATE_BUFFER,
        pMessage, 
        0,
        0,
        (LPWSTR)&pBuffer, 
        0, 
        &args);

    va_end(args);

    return pBuffer;
}

// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct D3D window render target.
        */
        D3D9_Window::D3D9_Window( class Render::System * renderSystem )
            : Window( renderSystem )
        {
        }




        /** Destruct D3D window render target.
        */
        D3D9_Window::~D3D9_Window()
        {
            if( mWindowHandle )
            {
                DestroyWindow( mWindowHandle ) ;
            }

            if( mRenderTarget )
            {
                mRenderTarget->Release() ;
            }
        }




        /** Set D3D "present" parameters.

            Set up the structure used to create the D3DDevice. Most parameters are
            zeroed out. We set Windowed to TRUE, since we want to do D3D in a
            window, and then set the SwapEffect to "discard", which is the most
            efficient method of presenting the back buffer to the display.  And 
            we request a back buffer format that matches the current desktop display 
            format.

        */
        static void SetPresentParameters( D3DPRESENT_PARAMETERS & d3dpp )
        {
            // Set up the structure used to create the D3DDevice. Most parameters are
            // zeroed out. We set Windowed to TRUE, since we want to do D3D in a
            // window, and then set the SwapEffect to "discard", which is the most
            // efficient method of presenting the back buffer to the display.  And 
            // we request a back buffer format that matches the current desktop display 
            // format.
            ZeroMemory( &d3dpp, sizeof( d3dpp ) );
            d3dpp.Windowed                  = TRUE ;
            d3dpp.SwapEffect                = D3DSWAPEFFECT_DISCARD ;
            d3dpp.BackBufferFormat          = D3DFMT_UNKNOWN ;
            d3dpp.PresentationInterval      = D3DPRESENT_INTERVAL_ONE ;    // Enable waiting for vsync.
            d3dpp.EnableAutoDepthStencil    = TRUE ; // Enable depth buffer
            d3dpp.AutoDepthStencilFormat    = D3DFMT_D16 ;
        }




        //-----------------------------------------------------------------------------
        // Desc: Initializes Direct3D
        //-----------------------------------------------------------------------------
        static HRESULT InitD3D( HWND hWnd )
        {
            // Create the D3D object, which is needed to create the D3DDevice.
            if( NULL == ( g_pD3D = Direct3DCreate9( D3D_SDK_VERSION ) ) )
                return E_FAIL;

            D3DPRESENT_PARAMETERS d3dpp ;
            SetPresentParameters( d3dpp ) ;

            // Create the Direct3D device. Here we are using the default adapter (most
            // systems only have one, unless they have multiple graphics hardware cards
            // installed) and requesting the HAL (which is saying we want the hardware
            // device rather than a software one). Software vertex processing is 
            // specified since we know it will work on all cards. On cards that support 
            // hardware vertex processing, though, we would see a big performance gain 
            // by specifying hardware vertex processing.
            // NOTE: Using D3DCREATE_SOFTWARE_VERTEXPROCESSING with debug D3D9 will lead to
            //      ASSERTION FAILED! File s:\gfx_aug09\windows\directx\dxg\inactive\d3d9\d3d\fw\lhbatchfilter.cpp Line 3466: pArgs->Flags.Discard
            HRESULT hResult = g_pD3D->CreateDevice( D3DADAPTER_DEFAULT
                , D3DDEVTYPE_HAL
                , hWnd
                , D3DCREATE_HARDWARE_VERTEXPROCESSING // D3DCREATE_SOFTWARE_VERTEXPROCESSING
                , & d3dpp
                , & g_pd3dDevice ) ;
            if( FAILED( hResult ) )
            {
                //MessageBox( NULL, DXGetErrorString( hResult ), L"Error", MB_OK | MB_ICONERROR );
                //MessageBox( NULL, DXGetErrorDescription( hResult ), L"D3D Error Creating Device", MB_OK | MB_ICONERROR );

                return E_FAIL;
            }

            // Get device capabilities.
            D3DCAPS9 deviceCapbilities ;
            HROK( g_pd3dDevice->GetDeviceCaps( & deviceCapbilities ) ) ;

            // Device state would normally be set here

            return S_OK;
        }




        //-----------------------------------------------------------------------------
        // Desc: Releases all previously initialized objects
        //-----------------------------------------------------------------------------
        static VOID Cleanup()
        {
            if( g_pd3dDevice != NULL )
                g_pd3dDevice->Release() ;

            if( g_pD3D != NULL )
                g_pD3D->Release() ;
        }




        //-----------------------------------------------------------------------------
        // Desc: Draws the scene
        //-----------------------------------------------------------------------------
        static VOID Render()
        {
            if( NULL == g_pd3dDevice )
                return;

            // Begin the scene
            if( SUCCEEDED( g_pd3dDevice->BeginScene() ) )
            {
                // Rendering of scene objects can happen here

                // End the scene
                HROK( g_pd3dDevice->EndScene() ) ;
            }

            // Present the backbuffer contents to the display
            HROK( g_pd3dDevice->Present( NULL, NULL, NULL, NULL ) ) ;
        }




        //-----------------------------------------------------------------------------
        // Desc: The window's message handler
        //-----------------------------------------------------------------------------
        static LRESULT WINAPI MsgProc( HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam )
        {
            switch( msg )
            {
            case WM_DESTROY:
                Cleanup();
                PostQuitMessage( 0 );
                return 0;

            case WM_PAINT:
                Render();
                ValidateRect( hWnd, NULL );
                return 0;

            case WM_SIZE:
#if 0   // TODO: Support window resizing properly.
                // Without the code below, resizing the on-screen window will reuse the existing render target (at its original resolution).
                // So if the on-screen window is larger than the original render target, the render target will just be scaled up and hence look blocky.
                // The code below would fix that problem but it requires resetting the D3D9 device, which requires releasing
                // all (well, most) D3D9 resources (textures, meshes, vertex buffers, index buffers, etc.).
                // This would basically entail destroying the scene graph, releasing all assets,
                // reloading all assets and rebuilding the scene graph, which is basically tantamount
                // to shutting down and restarting.  So we might as well disallow resizing the window, for D3D.
                // Note that OpenGL does not have this problem.
                if( g_pd3dDevice != NULL && wParam != SIZE_MINIMIZED )
                {   // The device is not NULL and the WM_SIZE message is not a SIZE_MINIMIZED event.
                    // Resize the device's swap buffers to match the new window size.
                    //invalidateDeviceObjects(); Need to release all vertex buffers, textures, meshes -- all D3D resources.
                    D3DPRESENT_PARAMETERS d3dpp ;
                    SetPresentParameters( d3dpp ) ;
                    d3dpp.BackBufferWidth  = LOWORD( lParam ) ;
                    d3dpp.BackBufferHeight = HIWORD( lParam ) ;
                    HRESULT hr = g_pd3dDevice->Reset( & d3dpp ) ;
                    if( hr == D3DERR_INVALIDCALL )
                    {   // This is a catastrophic failure.  The process will be terminated.
                        // All user created D3DPOOL_DEFAULT surfaces must be freed before ResetEx can succeed. ResetEx Fails.
                        // ResetEx failed and ResetEx/TestCooperativeLevel/Release are the only legal APIs to be called subsequently.

                        DebugPrintf( "MsgProc: Call to D3D9 Reset() failed with D3DERR_INVALIDCALL!\n") ;
                        DEBUG_BREAK() ;
                        MessageBox( NULL
                            , (LPCWSTR)L"Call to Reset() failed with D3DERR_INVALIDCALL! "
                            , (LPCWSTR)L"ERROR"
                            , MB_OK | MB_ICONEXCLAMATION ) ;
                    }
                    //restoreDeviceObjects();
                }
#endif
                break;

            }

            return DefWindowProc( hWnd, msg, wParam, lParam );
        }




        void D3D9_Window::Create()
        {
            //static const size_t winNameMaxSize = 256 ;
            //wchar_t             winNameWide[ winNameMaxSize ] ;
            //mbstowcs( winNameWide , mName , winNameMaxSize ) ;

            // Register the window class
            WNDCLASSEX wc =
            {
                sizeof( WNDCLASSEX ), CS_CLASSDC, MsgProc, 0L, 0L,
                    GetModuleHandle( NULL ), NULL, NULL, NULL, NULL,
                    L"PeGaSys", NULL
            };
            RegisterClassEx( &wc );

            // Create the window.
            // Note that this window has borders, so the "client area" that corresponds to the render surface will have a different position and size.
            mWindowHandle = CreateWindow( L"PeGaSys"
                , L"PeGaSys"
                , WS_OVERLAPPEDWINDOW
                , GetLeft()
                , GetTop()
                , GetWidth()
                , GetHeight()
                , NULL, NULL, wc.hInstance, NULL );

            if( SUCCEEDED( InitD3D( mWindowHandle ) ) )
            {
                // Show the window
                ShowWindow( mWindowHandle, SW_SHOWDEFAULT );
                UpdateWindow( mWindowHandle );
                //// Clear the backbuffer to a blue color
#ifdef Clear // Wrapper macros defined Clear
#   undef Clear
#endif
                HROK( g_pd3dDevice->Clear( 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB( 0, 0, 255 ), 1.0f, 0 ) ) ;
                HROK( g_pd3dDevice->Present( NULL, NULL, NULL, NULL ) ) ;

                // For later use, get D3D render target associated with this window.
                HROK( g_pd3dDevice->GetRenderTarget( 0 /* render target index */ , & mRenderTarget ) ) ;

                // Paranoid sanity checks.
                {
                    D3DSURFACE_DESC surfaceDesc ;
                    HROK( mRenderTarget->GetDesc( & surfaceDesc ) ) ;
                    ASSERT( static_cast< LONG >( surfaceDesc.Width  ) == GetState( WIDTH )  ) ;
                    ASSERT( static_cast< LONG >( surfaceDesc.Height ) == GetState( HEIGHT ) ) ;
                }

                // Update this window object to reflect "client" (renderable surface) geometry.
                SetTop( GetState( POSITION_X ) ) ;
                SetLeft( GetState( POSITION_Y ) ) ;
                SetWidth( GetState( WIDTH ) ) ;
                SetHeight( GetState( HEIGHT ) ) ;
            }
        }




        void D3D9_Window::Change()
        {
        }




        /* virtual */ void D3D9_Window::SetWindow()
        {
        }




        /** Obtain state information about this window.
        */
        int D3D9_Window::GetState( StateE state )
        {
            WINDOWINFO windowInfo ;
            DEBUG_ONLY( BOOL gwiOk = ) GetWindowInfo( mWindowHandle , & windowInfo ) ;
            ASSERT( gwiOk ) ;

            switch( state )
            {
            case POSITION_X : return windowInfo.rcClient.top                                ; break ;
            case POSITION_Y : return windowInfo.rcClient.left                               ; break ;
            case WIDTH      : return windowInfo.rcClient.right  - windowInfo.rcClient.left  ; break ;
            case HEIGHT     : return windowInfo.rcClient.bottom - windowInfo.rcClient.top   ; break ;
            }

            FAIL() ;    // state has invalid value.
            return -1 ; // Return invalid value.
        }




    } ;
} ;



#if defined( _DEBUG )




void PeGaSys::Render::D3D9_Window::UnitTest( void )
{
    DebugPrintf( "D3D9_Window::UnitTest ----------------------------------------------\n" ) ;

    {
        D3D9_Window window( 0 ) ;

        window.GetTypeId() ;
        window.SetWidth( 640 ) ;
        window.SetHeight( 480 ) ;
        window.SetDepth( true ) ;
        window.SetName( "MyWindow_D3D9" ) ;
        window.SetFullScreen( false ) ;
        window.SetLeft( 20 ) ;
        window.SetTop( 20 ) ;

        window.Create() ;
        window.Change() ;
        window.RenderViewports( 0.0 ) ;
    }

    DebugPrintf( "D3D9_Window::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif