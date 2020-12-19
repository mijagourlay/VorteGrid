/** \file intrusivePtr.cpp

    \brief Lightweight reference-counted pointer templated class

    \author Copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <cassert>

#include "Core/Utility/macros.h"
#include "Core/File/debugPrint.h"

#include "intrusivePtr.h"

#if 0 && defined( UNIT_TEST )

#include "talliedInt.h"

void IntrusivePtr_UnitTest(void)
{
    DEBUG_ONLY( DebugPrintf( "IntrusivePtr::UnitTest: ----------------------------------------------\n" ) ) ;

    const size_t talliedIntCountBefore = TalliedInt::GetCount() ;

    {
        TalliedInt * talliedInt1 = new TalliedInt( 1 ) ;
        ASSERT( TalliedInt::GetCount() == talliedIntCountBefore + 1 ) ;

        IntrusivePtr<TalliedInt> intrusivePtrTalliedInt1a( talliedInt1 ) ;
        ASSERT( intrusivePtrTalliedInt1a->GetRefCount() == 1 ) ;
        ASSERT( * intrusivePtrTalliedInt1a == 1 ) ;
        * intrusivePtrTalliedInt1a = 2 ;
        ASSERT( intrusivePtrTalliedInt1a->GetRefCount() == 1 ) ;
        ASSERT( * intrusivePtrTalliedInt1a == 2 ) ;

        {
            IntrusivePtr<TalliedInt> intrusivePtrTalliedInt1b( intrusivePtrTalliedInt1a ) ;
            ASSERT( intrusivePtrTalliedInt1a->GetRefCount() == 2 ) ;
            ASSERT( intrusivePtrTalliedInt1b->GetRefCount() == 2 ) ;
            ASSERT( intrusivePtrTalliedInt1a == intrusivePtrTalliedInt1b ) ;
            ASSERT( * intrusivePtrTalliedInt1a == * intrusivePtrTalliedInt1b ) ;
            ASSERT( * intrusivePtrTalliedInt1b == 2 ) ;

            ASSERT( TalliedInt::GetCount() == talliedIntCountBefore + 1 ) ;

            {
                IntrusivePtr<TalliedInt> intrusivePtrTalliedInt1c =  intrusivePtrTalliedInt1b ;
                ASSERT( intrusivePtrTalliedInt1a->GetRefCount() == 3 ) ;
                ASSERT( intrusivePtrTalliedInt1b->GetRefCount() == 3 ) ;
                ASSERT( intrusivePtrTalliedInt1c->GetRefCount() == 3 ) ;
                ASSERT( intrusivePtrTalliedInt1a == intrusivePtrTalliedInt1b ) ;
                ASSERT( intrusivePtrTalliedInt1a == intrusivePtrTalliedInt1c ) ;
                ASSERT( intrusivePtrTalliedInt1b == intrusivePtrTalliedInt1c ) ;
                ASSERT( * intrusivePtrTalliedInt1a == * intrusivePtrTalliedInt1c ) ;
                ASSERT( * intrusivePtrTalliedInt1c == 2 ) ;

                ASSERT( TalliedInt::GetCount() == talliedIntCountBefore + 1 ) ;
            }

            ASSERT( intrusivePtrTalliedInt1a->GetRefCount() == 2 ) ;
            ASSERT( intrusivePtrTalliedInt1b->GetRefCount() == 2 ) ;

            {
                IntrusivePtr<TalliedInt> intrusivePtrTalliedInt1d ;
                intrusivePtrTalliedInt1d = intrusivePtrTalliedInt1a ;
                ASSERT( intrusivePtrTalliedInt1a->GetRefCount() == 3 ) ;
                ASSERT( intrusivePtrTalliedInt1b->GetRefCount() == 3 ) ;
                ASSERT( intrusivePtrTalliedInt1d->GetRefCount() == 3 ) ;
                ASSERT( intrusivePtrTalliedInt1a == intrusivePtrTalliedInt1b ) ;
                ASSERT( intrusivePtrTalliedInt1a == intrusivePtrTalliedInt1d ) ;
                ASSERT( * intrusivePtrTalliedInt1a == * intrusivePtrTalliedInt1d ) ;
                ASSERT( * intrusivePtrTalliedInt1d == 2 ) ;

                ASSERT( TalliedInt::GetCount() == talliedIntCountBefore + 1 ) ;
            }

            ASSERT( intrusivePtrTalliedInt1a->GetRefCount() == 2 ) ;
            ASSERT( intrusivePtrTalliedInt1b->GetRefCount() == 2 ) ;
        }
        ASSERT( intrusivePtrTalliedInt1a->GetRefCount() == 1 ) ;
    }

    // Make sure SharedPtr deleted the TalliedInt.
    ASSERT( talliedIntCountBefore == TalliedInt::GetCount() ) ;

    return ;
}

#endif
