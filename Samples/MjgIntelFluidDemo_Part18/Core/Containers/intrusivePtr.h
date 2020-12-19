/** \file intrusivePtr.h

    \brief Lightweight reference-counted pointer templated class.

    \author Copyright 2005-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef INTRUSIVE_PTR_H
#define INTRUSIVE_PTR_H

#include "Core/Utility/macros.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

/** Reference-counted mix-in class that provides functionality used by IntrusivePtr.

    This class provides a reference count and accessors that help implement
    the members that IntrusivePtr uses.  It is not necessary for classes that
    want to be used by IntrsivePtr to derived from RefCountedMixin; it is merely
    provided for convenience.

    In addition to inheriting from this class, the derived class must also
    implement ReleaseReference that calls _ReleaseReference and calls
    "delete this" of _ReleaseReference returns true.

    Furthermore, IntrusivePtr also expects there to be helper functions
    that are NOT members of this class, which implement AddReference
    and ReleaseReference, which can simply be facades to the member functions
    AddReference and ReleaseReference.

    Again, the member functions are not required; they simply maintain encapsulation
    of the reference count and easy implementing the functions that IntrusivePtr
    needs.

    \see IntrusivePtr.
*/
class RefCountedMixin
{
    public:

        /// Increment the number of references to this object.
        void AddReference()
        {
            ASSERT( mRefCount >= 0 ) ;
            ++ mRefCount ;
        }

        /** Get number of references to this object.
            IntrusivePtr does not require this method (or any member function), but it
            is often useful for the derived class to have access to the reference count.
        */
        int GetRefCount() const
        {
            ASSERT( mRefCount >= 0 ) ;
            return mRefCount ;
        }

    protected:
        /** Initialize the reference count to zero.

            \note   Constructor is private to prevent instantiating RefCountedMixin objects.
                    This class is meant to be inherited, not instantiated.
        */
        RefCountedMixin()
            : mRefCount( 0 )
        {}

        /// Destruct a reference-counted object.
        ~RefCountedMixin()
        {
            ASSERT( 0 == mRefCount ) ;
        }

        /// Decrement the number of references to this object, and return true if this was the last one.
        /// Derived class must call _ReleaseReference and "delete this" if it returns true.
        bool _ReleaseReference()
        {
            ASSERT( mRefCount > 0 ) ;
            -- mRefCount ;
            return 0 == mRefCount ;
        }

    private:
        int mRefCount   ;   ///< Number of references to this object.
} ;




/** Smart pointer that uses intrusive reference counting.

    Calls these functions:

    -   void AddReference( ReferentT * ) ;
    -   void ReleaseReference( ReferentT * ) ;

    The IntrusivePtr class template stores a pointer to an object with an embedded reference count.
    Every new IntrusivePtr instance increments the reference count by using an unqualified call to
    AddReference.  When an IntrusivePtr is destroyed, it calls ReleaseReference;
    this function is responsible for destroying the object when its reference count drops to zero.
    The user shall provide definitions of these two functions.

    \see RefCountedMixin.

*/
template< class ReferentT > class IntrusivePtr
{
    private:

        typedef IntrusivePtr ThisType ; ///< Nickname for IntrusivePtr< ReferentT >.

    public:

        typedef ReferentT ReferentType ;    ///< Provide access to the referent type even when the IntrusivePtr is itself typedef'd.

        /// Initialize this IntrusivePtr to null_ptr.
        IntrusivePtr()
            : mPointer( 0 )
        {
        }


        /// Initialize this IntrusivePtr to the given raw pointer.
        IntrusivePtr( ReferentT * p , bool addReference = true )
            : mPointer( p )
        {
            if( ( mPointer != 0 ) && addReference )
            {
                AddReference( mPointer ) ;
            }
        }


        /// Initialize this IntrusivePtr with the given IntrusivePtr, whose referent has a different type than this one's.
        template< class OtherReferentT > IntrusivePtr( IntrusivePtr< OtherReferentT > const & that )
            : mPointer( that.Get() )
        {
            if( mPointer != 0 )
            {
                AddReference( mPointer ) ;
            }
        }


        /// Initialize this IntrusivePtr with the given IntrusivePtr.
        IntrusivePtr( IntrusivePtr const & that )
            : mPointer( that.mPointer )
        {
            if( mPointer != 0 )
            {
                AddReference( mPointer ) ;
            }
        }

        ~IntrusivePtr()
        {
            if( mPointer != 0 )
            {
                ReleaseReference( mPointer ) ;
            }
        }

        /// Assign another IntrusivePtr, with a different referent type, to this one.
        template< class OtherReferentT > IntrusivePtr & operator=( IntrusivePtr< OtherReferentT > const & that )
        {
            ThisType( that ).Swap( * this ) ;
            return * this ;
        }


    #if defined( HAVE_RVALUE_REFS )

        // Move support

        IntrusivePtr( IntrusivePtr && that )
            : mPointer( that.mPointer )
        {
            that.mPointer = 0 ;
        }

        IntrusivePtr & operator=( IntrusivePtr && that )
        {
            ThisType( static_cast< IntrusivePtr && >( that ) ).Swap( * this ) ;
            return * this ;
        }

    #endif

        /// Assign another IntrusivePtr to this one.
        IntrusivePtr & operator=( IntrusivePtr const & that )
        {
            ThisType( that ).Swap( * this ) ;
            return * this ;
        }


        /// Assign a raw pointer to this IntrusivePtr.
        IntrusivePtr & operator=( ReferentT * that )
        {
            ThisType( that ).Swap( * this ) ;
            return * this ;
        }


        /// Release the owned pointer.
        void Reset()
        {
            ThisType().Swap( * this ) ;
        }


        /// Release the owned pointer and own the given pointer.
        void Reset( ReferentT * that )
        {
            ThisType( that ).Swap( * this ) ;
        }


        /// Return the address of the contained object.
        ReferentT * Get() const
        {
            return mPointer ;
        }


        /// Dereference the contained object.
        ReferentT & operator*() const
        {
            ASSERT( mPointer != 0 ) ;
        #pragma warning( push )
        #pragma warning( disable: 4616 6011 ) // Dereferencing NULL pointer 'mPointer'
            return * mPointer ;
        #pragma warning( pop)
        }


        /// Dereference a member of the contained object.
        ReferentT * operator->() const
        {
            ASSERT( mPointer != 0 ) ;
        #pragma warning( push )
        #pragma warning( disable: 4616 6011 ) // Dereferencing NULL pointer 'mPointer'
            return mPointer ;
        #pragma warning( pop)
        }


        /// Exchange the contents of the given smart pointer with this one.
        void Swap( IntrusivePtr & that )
        {
            ReferentT * tmp = mPointer ;
            mPointer        = that.mPointer ;
            that.mPointer   = tmp ;
        }


        ///////////////////////////////////////////////////////////////////////////////////////
        // implicit conversion to "bool"

        typedef ReferentT * ThisType::*unspecified_bool_type ;

        /// Cast to an unspecified boolean type to allow for constructs like "if( ptr )..." without problematic implicit cast to bool.
        operator unspecified_bool_type() const // never throws
        {
            return mPointer == 0 ? 0 : & ThisType::mPointer ;
        }


        /// operator! is redundant, but some compilers need it
        bool operator! () const // never throws
        {
            return mPointer == 0 ;
        }
        ///////////////////////////////////////////////////////////////////////////////////////

    private:
        ReferentT * mPointer ;  ///< Pointer to reference-counted object.
} ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

/// Compare two IntrusivePtr values whose referent types differ, for equality of their addresses.
template< class ReferentT , class OtherReferentT > inline bool operator==( IntrusivePtr< ReferentT > const & lhs , IntrusivePtr< OtherReferentT > const & rhs )
{
    return lhs.Get() == rhs.Get() ;
}

/// Compare two IntrusivePtr values, whose referent types differ, for inequality of their addresses
template< class ReferentT , class OtherReferentT > inline bool operator!=( IntrusivePtr< ReferentT > const & lhs , IntrusivePtr< OtherReferentT > const & rhs )
{
    return lhs.Get() != rhs.Get() ;
}

/// Compare an IntrusivePtr to lhs raw pointer whose referent type differs, for equality of their addresses.
template< class ReferentT , class OtherReferentT > inline bool operator==( IntrusivePtr< ReferentT > const & lhs , OtherReferentT * rhs )
{
    return lhs.Get() == rhs ;
}

/// Compare an IntrusivePtr to lhs raw pointer whose referent type differs, for inequality of their addresses.
template< class ReferentT , class OtherReferentT > inline bool operator!=( IntrusivePtr< ReferentT > const & lhs , OtherReferentT * rhs )
{
    return lhs.Get() != rhs ;
}

/// Compare lhs raw pointer to an IntrusivePtr whose referent type differs, for equality of their addresses.
template< class ReferentT , class OtherReferentT > inline bool operator==( ReferentT * lhs , IntrusivePtr< OtherReferentT > const & rhs )
{
    return lhs == rhs.Get() ;
}

/// Compare lhs raw pointer to an IntrusivePtr whose referent type differs, for inequality of their addresses.
template< class ReferentT , class OtherReferentT > inline bool operator!=( ReferentT * lhs , IntrusivePtr< OtherReferentT > const & rhs )
{
    return lhs != rhs.Get() ;
}

/// Compare two IntrusivePtr values, with identical referent types, for order of their addresses.
template< class ReferentT > inline bool operator<( IntrusivePtr< ReferentT > const & lhs , IntrusivePtr< ReferentT > const & rhs )
{
    return lhs.Get() < rhs.Get() ;
}

//template< class ReferentT > void Swap( IntrusivePtr< ReferentT > & lhs , IntrusivePtr< ReferentT > & rhs )
//{
//    lhs.Swap( rhs ) ;
//}
//
//template< class ReferentT > ReferentT * GetPointer( IntrusivePtr< ReferentT > const & p )
//{
//    return p.Get() ;
//}

/// StaticPointerCast overload for raw pointers.
template<class ReferentT, class OtherReferentT> inline ReferentT* StaticPointerCast(OtherReferentT *ptr)
{  
    return static_cast<ReferentT*>(ptr);
}

/// ConstPointerCast overload for raw pointers.
template<class ReferentT, class OtherReferentT> inline ReferentT* ConstPointerCast(OtherReferentT *ptr)
{  
    return const_cast<ReferentT*>(ptr);
}

/// DynamicPointerCast overload for raw pointers.
template<class ReferentT, class OtherReferentT> inline ReferentT* DynamicPointerCast(OtherReferentT *ptr)
{  
    return dynamic_cast<ReferentT*>(ptr);
}

/// StaticPointerCast overload for IntrusivePtr.
template< class ReferentT , class OtherReferentT > IntrusivePtr< ReferentT > StaticPointerCast( IntrusivePtr< OtherReferentT > const & p )
{
    return static_cast< ReferentT * >( p.Get() ) ;
}

/// ConstPointerCast overload for IntrusivePtr.
template< class ReferentT , class OtherReferentT > IntrusivePtr< ReferentT > ConstPointerCast( IntrusivePtr< OtherReferentT > const & p )
{
    return const_cast< ReferentT * >( p.Get() ) ;
}

/// DynamicPointerCast overload for IntrusivePtr.
template< class ReferentT , class OtherReferentT > IntrusivePtr< ReferentT > DynamicPointerCast( IntrusivePtr< OtherReferentT > const & p )
{
    return dynamic_cast< ReferentT * >( p.Get() ) ;
}

/// ReinterpretPointerCast overload for raw pointers
//template<class ReferentT, class OtherReferentT> inline ReferentT* ReinterpretPointerCast(OtherReferentT *ptr)
//{  
//    return reinterpret_cast<ReferentT*>(ptr);
//}

#if defined( UNIT_TEST )
    extern void IntrusivePtr_UnitTest(void) ;
#endif

#endif
