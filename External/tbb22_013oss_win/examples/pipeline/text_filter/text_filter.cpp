/*
    Copyright 2005-2010 Intel Corporation.  All Rights Reserved.

    This file is part of Threading Building Blocks.

    Threading Building Blocks is free software; you can redistribute it
    and/or modify it under the terms of the GNU General Public License
    version 2 as published by the Free Software Foundation.

    Threading Building Blocks is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty
    of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Threading Building Blocks; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    As a special exception, you may use this file as part of a free software
    library without restriction.  Specifically, if other files instantiate
    templates or use macros or inline functions from this file, or you compile
    this file and link it with other files to produce an executable, this
    file does not by itself cause the resulting executable to be covered by
    the GNU General Public License.  This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
*/

//
// Example program that reads a file of text and changes the first letter
// of each word to upper case.
// 
#include "tbb/pipeline.h"
#include "tbb/tick_count.h"
#include "tbb/task_scheduler_init.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cctype>

using namespace std;

//! Buffer that holds block of characters and last character of previous buffer.
class MyBuffer {
    static const size_t buffer_size = 10000;
    char* my_end;
    //! storage[0] holds the last character of the previous buffer.
    char storage[1+buffer_size];
public:
    //! Pointer to first character in the buffer
    char* begin() {return storage+1;}
    const char* begin() const {return storage+1;}
    //! Pointer to one past last character in the buffer
    char* end() const {return my_end;}
    //! Set end of buffer.
    void set_end( char* new_ptr ) {my_end=new_ptr;}
    //! Number of bytes a buffer can hold
    size_t max_size() const {return buffer_size;}
    //! Number of bytes appended to buffer.
    size_t size() const {return my_end-begin();}
};

static const char* InputFileName = "input.txt";
static const char* OutputFileName = "output.txt";

class MyInputFilter: public tbb::filter {
public:
    static const size_t n_buffer = 8;
    MyInputFilter( FILE* input_file_ );
private:
    FILE* input_file;
    size_t next_buffer;
    char last_char_of_previous_buffer;
    MyBuffer buffer[n_buffer];
    /*override*/ void* operator()(void*);
};

MyInputFilter::MyInputFilter( FILE* input_file_ ) : 
    filter(serial_in_order),
    next_buffer(0),
    input_file(input_file_),
    last_char_of_previous_buffer(' ')
{ 
}

void* MyInputFilter::operator()(void*) {
    MyBuffer& b = buffer[next_buffer];
    next_buffer = (next_buffer+1) % n_buffer;
    size_t n = fread( b.begin(), 1, b.max_size(), input_file );
    if( !n ) {
        // end of file
        return NULL;
    } else {
        b.begin()[-1] = last_char_of_previous_buffer;
        last_char_of_previous_buffer = b.begin()[n-1];
        b.set_end( b.begin()+n );
        return &b;
    }
}

//! Filter that changes the first letter of each word from lower case to upper case.
class MyTransformFilter: public tbb::filter {
public:
    MyTransformFilter();
    /*override*/void* operator()( void* item );
};

MyTransformFilter::MyTransformFilter() : 
    tbb::filter(parallel) 
{}  

/*override*/void* MyTransformFilter::operator()( void* item ) {
    MyBuffer& b = *static_cast<MyBuffer*>(item);
    int prev_char_is_space = b.begin()[-1]==' ';
    for( char* s=b.begin(); s!=b.end(); ++s ) {
        if( prev_char_is_space && islower((unsigned char)*s) )
            *s = toupper(*s);
        prev_char_is_space = isspace((unsigned char)*s);
    }
    return &b;  
}
         
//! Filter that writes each buffer to a file.
class MyOutputFilter: public tbb::filter {
    FILE* my_output_file;
public:
    MyOutputFilter( FILE* output_file );
    /*override*/void* operator()( void* item );
};

MyOutputFilter::MyOutputFilter( FILE* output_file ) : 
    tbb::filter(serial_in_order),
    my_output_file(output_file)
{
}

void* MyOutputFilter::operator()( void* item ) {
    MyBuffer& b = *static_cast<MyBuffer*>(item);
    int n = (int) fwrite( b.begin(), 1, b.size(), my_output_file );
    if( n<=0 ) {
        fprintf(stderr,"Can't write into %s file\n", OutputFileName);
        exit(1);
    }
    return NULL;
}

static int NThread = tbb::task_scheduler_init::automatic;
static bool is_number_of_threads_set = false;

void Usage()
{
    fprintf( stderr, "Usage:\ttext_filter [input-file [output-file [nthread]]]\n");
}

int ParseCommandLine(  int argc, char* argv[] ) {
    // Parse command line
    if( argc> 4 ){
        Usage();
        return 0;
    }
    if( argc>=2 ) InputFileName = argv[1];
    if( argc>=3 ) OutputFileName = argv[2];
    if( argc>=4 ) {
        NThread = strtol(argv[3],0,0);
        if( NThread<1 ) {
            fprintf(stderr,"nthread set to %d, but must be at least 1\n",NThread);
            return 0;
        }
        is_number_of_threads_set = true; //Number of threads is set explicitly
    }
    return 1;
}

int run_pipeline( int nthreads )
{
    FILE* input_file = fopen(InputFileName,"r");
    if( !input_file ) {
        perror( InputFileName );
        Usage();
        return 0;
    }
    FILE* output_file = fopen(OutputFileName,"w");
    if( !output_file ) {
        perror( OutputFileName );
        return 0;
    }

    // Create the pipeline
    tbb::pipeline pipeline;

    // Create file-reading writing stage and add it to the pipeline
    MyInputFilter input_filter( input_file );
    pipeline.add_filter( input_filter );

    // Create capitalization stage and add it to the pipeline
    MyTransformFilter transform_filter; 
    pipeline.add_filter( transform_filter );

    // Create file-writing stage and add it to the pipeline
    MyOutputFilter output_filter( output_file );
    pipeline.add_filter( output_filter );

    // Run the pipeline
    tbb::tick_count t0 = tbb::tick_count::now();
    pipeline.run( MyInputFilter::n_buffer );
    tbb::tick_count t1 = tbb::tick_count::now();

    fclose( output_file );
    fclose( input_file );

    if (is_number_of_threads_set) {
        printf("threads = %d time = %g\n", nthreads, (t1-t0).seconds());
    } else {
        if ( nthreads == 1 ){
            printf("single thread run time = %g\n", (t1-t0).seconds());
        } else {
            printf("parallel run time = %g\n", (t1-t0).seconds());
        }
    }
    return 1;
}

int main( int argc, char* argv[] ) {
    if(!ParseCommandLine( argc, argv ))
        return 1;
    if (is_number_of_threads_set) {
        // Start task scheduler
        tbb::task_scheduler_init init( NThread );
        if(!run_pipeline (NThread))
            return 1;
    } else { // Number of threads wasn't set explicitly. Run single-thread and fully subscribed parallel versions
        { // single-threaded run
            tbb::task_scheduler_init init_serial(1);
            if(!run_pipeline (1))
                return 1;
        }
        { // parallel run (number of threads is selected automatically)
            tbb::task_scheduler_init init_parallel;
            if(!run_pipeline (0))
                return 1;
        }
    }
    return 0;
}
