



// --------------------------

// Tests below assume grid spacing h==1/N+1 (i.e. there are NUM_PTS cells/intervals).

static const unsigned NX            = 64 ;
static const unsigned NY            = 16 ;
static const float    h[2]          = { 1.0f / float( NX - 1 ) , 1.0f / float( NY - 1 ) } ;
static const float    h2[2]         = { h[0] * h[0] , h[1] * h[1] } ;
static const float    oneOverH2[2]  = { 1.0f / h2[0] , 1.0f / h2[1] } ;
static const float    halfH2Sum     = 0.5f / ( oneOverH2[0] + oneOverH2[1] ) ;


inline float X( int i ) { return float( i ) * h[0] ; }
inline float Y( int i ) { return float( i ) * h[1] ; }


void ComputeLaplacian1D( float laplacian[ NX ] , const float func[ NX ] )
{
    for( unsigned i = 1 ; i < NX - 1 ; ++ i )
    {
        laplacian[ i ] = ( func[ i+1 ] + func[ i-1 ] - 2.0f * func[ i ] ) * oneOverH2[0] ;
    }

#if 1
    // Use Dirichlet boundary conditions:
    // Assume func[boundary]==0.
    laplacian[ 0      ] = ( func[ 1      ] - 2.0f * func[ 0      ] ) * oneOverH2[0] ;
    laplacian[ NX - 1 ] = ( func[ NX - 2 ] - 2.0f * func[ NX - 1 ] ) * oneOverH2[0] ;
#elif 0
    // Use Neumann-ish boundary conditions:
    // -1 instead of -2 because we assume func[i]==func[boundary].
    laplacian[ 0      ] = ( func[ 1      ] - 1.0f * func[ 0      ] ) * oneOverH2[0] ;
    laplacian[ NX - 1 ] = ( func[ NX - 2 ] - 1.0f * func[ NX - 1 ] ) * oneOverH2[0] ;
#else
    laplacian[ 0      ] = laplacian[ 1      ] ;
    laplacian[ NX - 1 ] = laplacian[ NX - 2 ] ;
#endif
}




void ComputeLaplacian2D( float laplacian[ NY ][ NX ] , const float func[ NY ][ NX ] )
{
    for( unsigned j = 1 ; j < NY - 1 ; ++ j )
    {
        for( unsigned i = 1 ; i < NX - 1 ; ++ i )
        {
            laplacian[ j ][ i ] =   ( func[ j   ][ i+1 ] + func[ j   ][ i-1 ] - 2.0f * func[ j ][ i ] ) * oneOverH2[0]
                                +   ( func[ j+1 ][ i   ] + func[ j-1 ][ i   ] - 2.0f * func[ j ][ i ] ) * oneOverH2[1]
                                ;
        }
    }

#if 1
    // Use Dirichlet boundary conditions:
    // Assume func[boundary]==0.
    for( unsigned i = 0 ; i < NX ; ++ i )
    {
        laplacian[ 0      ][i] = ( func[ 1      ][i] - 2.0f * func[ 0      ][i] ) * oneOverH2[1] ;
        laplacian[ NY - 1 ][i] = ( func[ NY - 2 ][i] - 2.0f * func[ NY - 1 ][i] ) * oneOverH2[1] ;
    }
    for( unsigned j = 0 ; j < NY ; ++ j )
    {
        laplacian[j][ 0      ] = ( func[j][ 1      ] - 2.0f * func[j][ 0      ] ) * oneOverH2[0] ;
        laplacian[j][ NX - 1 ] = ( func[j][ NX - 2 ] - 2.0f * func[j][ NX - 1 ] ) * oneOverH2[0] ;
    }
#elif 0
    // Use Neumann-ish boundary conditions:
    // -1 instead of -2 because we assume func[i]==func[boundary].
    laplacian[ 0      ] = ( func[ 1      ] - 1.0f * func[ 0      ] ) * oneOverH2[0] ;
    laplacian[ NX - 1 ] = ( func[ NX - 2 ] - 1.0f * func[ NX - 1 ] ) * oneOverH2[0] ;
#else
    // "Gourlay" boundaries: second derivative is constant.
    for( unsigned i = 0 ; i < NX ; ++ i )
    {
        laplacian[ 0      ][ i ] = laplacian[ 1      ][ i ] ;
        laplacian[ NY - 1 ][ i ] = laplacian[ NY - 2 ][ i ] ;
    }
    for( unsigned j = 0 ; j < NY ; ++ j )
    {
        laplacian[ j ][ 0      ] = laplacian[ j ][ 1      ] ;
        laplacian[ j ][ NX - 1 ] = laplacian[ j ][ NX - 2 ] ;
    }
#endif
}




void SolvePoisson1D( float phi[ NX ] , const float laplacian[ NX ] )
{
    for( unsigned i = 1 ; i < NX - 1 ; ++ i )
    {
        // phi[ i ] = ( phi[ i+1 ] + phi[ i-1 ] ) * oneOverH2[0] - laplacian[ i ] ) * 0.5f * h2[0] ;
        phi[ i ] = ( phi[ i+1 ] + phi[ i-1 ] - laplacian[ i ] * h2[0] ) * 0.5f ;
    }
}




void SolvePoisson2D( float phi[ NY ][ NX ] , const float laplacian[ NY ][ NX ] )
{
    for( unsigned j = 1 ; j < NY - 1 ; ++ j )
    {
        for( unsigned i = 1 ; i < NX - 1 ; ++ i )
        {
            phi[ j ][ i ] = (   ( phi[ j   ][ i+1 ] + phi[ j   ][ i-1 ] ) * oneOverH2[0]
                            +   ( phi[ j+1 ][ i   ] + phi[ j-1 ][ i   ] ) * oneOverH2[1]
                            -   laplacian[ j ][ i ]
                            ) * halfH2Sum
                            ;
        }
    }
}




void PrintFunc1D( const float func[ NX ] , const char * filename )
{
    FILE * fp = fopen( filename , "w" ) ;
    for( unsigned i = 0 ; i < NX ; ++ i )
    {
        fprintf( fp , "%g\t%g\n" , X( i ) , func[ i ] ) ;
    }
    fclose( fp ) ;
}




void PrintFunc2D( const float func[ NY ][ NX ] , const char * filename )
{
    FILE * fp = fopen( filename , "w" ) ;

#define EXCEL_STYLE 1

#if EXCEL_STYLE
    fprintf( fp , "\t" ) ;
    for( unsigned i = 0 ; i < NX ; ++ i )
    {
        fprintf( fp , "%g\t" , X( i ) ) ;
    }
    fprintf( fp , "\n" ) ;
#endif

    for( unsigned j = 0 ; j < NY ; ++ j )
    {
        #if EXCEL_STYLE
            fprintf( fp , "%g\t" , Y( j ) ) ;
        #endif
        for( unsigned i = 0 ; i < NX ; ++ i )
        {
            #if EXCEL_STYLE
                fprintf( fp , "%g\t" , func[ j ][ i ] ) ;
            #else
                fprintf( fp , "%g\t%g\t%g\n" , X( i ) , Y( i ) , func[ j ][ i ] ) ;
            #endif
        }
        #if EXCEL_STYLE
            fprintf( fp , "\n" ) ;
        #endif
    }
    fclose( fp ) ;
}




inline float testFunc1D( float x           ) { return x * ( 4.0f - x * 4.0f ) ; }

inline float testFunc2D( float x , float y ) { return testFunc1D( x ) * testFunc1D( y ) ; }




void UnitTestPoisson1D( void )
{
    float func[ NX ] ;

    for( unsigned i = 0 ; i < NX ; ++ i )
    {
        func[ i ] = testFunc1D( X( i ) ) ; // effectively, func( x ) = x^2
    }
    PrintFunc1D( func , "func.dat" ) ;

    float laplacian[ NX ] ;
    ComputeLaplacian1D( laplacian , func ) ;
    PrintFunc1D( laplacian , "laplacian.dat" ) ;

    float funcSolved[ NX ] ;
    for( unsigned i = 0 ; i < NX ; ++ i )
    {
        funcSolved[ i ] = 0.0f ;    // Initialize the trial solution
    }
    for( unsigned iter = 0 ; iter < NX * NX ; ++ iter )
    {
        //char filename[ 256 ] ;
        //sprintf( filename , "funcSoln%04i.dat" , iter ) ;
        //PrintFunc1D( funcSolved , filename ) ;
        SolvePoisson1D( funcSolved , laplacian ) ;
    }
    PrintFunc1D( funcSolved , "funcSoln.dat" ) ;
}



void UnitTestPoisson2D( void )
{
    float func[ NY ][ NX ] ;

    for( unsigned j = 0 ; j < NY ; ++ j )
    {
        for( unsigned i = 0 ; i < NX ; ++ i )
        {
            func[ j ][ i ] = testFunc2D( X( i ) , Y( j ) ) ;
        }
    }
    PrintFunc2D( func , "func2d.dat" ) ;

    float laplacian[ NY ][ NX ] ;
    ComputeLaplacian2D( laplacian , func ) ;
    PrintFunc2D( laplacian , "laplacian2.dat" ) ;

    float funcSolved[ NY ][ NX ] ;
    for( unsigned j = 0 ; j < NY ; ++ j )
    {
        for( unsigned i = 0 ; i < NX ; ++ i )
        {
            funcSolved[ j ][ i ] = 0.0f ;    // Initialize the trial solution
        }
    }
    for( unsigned iter = 0 ; iter < NX * NY ; ++ iter )
    {
        //char filename[ 256 ] ;
        //sprintf( filename , "funcSoln%04i.dat" , iter ) ;
        //PrintFunc2D( funcSolved , filename ) ;
        SolvePoisson2D( funcSolved , laplacian ) ;
    }
    PrintFunc2D( funcSolved , "funcSoln2.dat" ) ;
}
