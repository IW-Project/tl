#pragma once

#include <tl_system.h>
#include <tl_thread.h>
#include "phys_mem_new.h"
#include "phys_transient_allocator.h"

template <typename T>
class phys_link_list_base
{
public:
  T *m_next_link;

  inline void set_next_link( T *p ) { m_next_link = p; }
  inline T *get_next_link() { return m_next_link; }
};

template <typename T>
class phys_link_list
{
private:
  T *m_first;
  T **m_last_next_ptr;
  int m_allocCount;

public:
  inline phys_link_list()
  {
    m_first = NULL;
    m_last_next_ptr = &m_first;
    m_allocCount = 0;
  }
  inline void add( T *p )
  {
    tlAssert( m_last_next_ptr );
    p->m_next_link = NULL;
    ++m_allocCount;
    *m_last_next_ptr = p;
    m_last_next_ptr = &p->m_next_link;
  }
  inline void add_mt( T *p )
  {
    tlAssert( m_last_next_ptr );

    p->set_next_link( NULL );
    tlAtomicIncrement( (volatile u32 *)&m_allocCount );

    T **cur_last_next_ptr;
    do
    {
      cur_last_next_ptr = m_last_next_ptr;
    } while ( !tlAtomicCompareAndSwap( (volatile unsigned int *)&m_last_next_ptr, (unsigned int)p, (unsigned int)cur_last_next_ptr ) );
    *cur_last_next_ptr = p;
  }
  inline void remove_all()
  {
    m_first = NULL;
    m_last_next_ptr = &m_first;
    m_allocCount = 0;
  }
  inline const int get_count() const { return m_allocCount; }

  inline T *get_first() { return m_first; }

  inline void set_last_next_ptr( T *p )
  {
    tlAssert( m_last_next_ptr );
    *m_last_next_ptr = p;
    m_last_next_ptr = NULL;
  }

  __forceinline void atomic_prepend_to( T **output )
  {
    do
      *m_last_next_ptr = *output;
    while ( !tlAtomicCompareAndSwap( (volatile u32 *)output, (u32)m_first, (u32)*m_last_next_ptr ) );
  }

  class iterator
  {
  private:
    T *m_ptr;

  public:
    inline iterator( T *ptr )
      : m_ptr( ptr )
    {
    }

    inline void operator++() { m_ptr = m_ptr->get_next_link(); }

    inline bool operator!=( const iterator &other ) const { return m_ptr != other.m_ptr; }

    inline T& operator*() const { return *m_ptr; }
  };

  inline iterator begin() const { return iterator( m_first ); }

  inline iterator end() const { return iterator( NULL ); }
};

template <typename T>
class phys_simple_link_list
{
private:
  T *m_first;

public:
  inline phys_simple_link_list() { set_first( NULL ); }

  inline void add( T *p )
  {
    p->set_next_link( m_first );
    set_first( p );
  }

  inline void remove_all() { set_first( NULL ); }

  inline T *get_first() { return m_first; }

  inline T *get_last_added() { return m_first; }

  inline void set_first( T *p ) { m_first = p; }

  class iterator
  {
  private:
    T *m_ptr;

  public:
    inline iterator( T *ptr )
      : m_ptr( ptr )
    {
    }
    inline void operator++() { m_ptr = m_ptr->get_next_link(); }
    inline bool operator!=( const iterator &other ) const { return m_ptr != other.m_ptr; }
    inline T& operator*() { return *m_ptr; }
  };
  inline iterator begin() { return iterator( m_first ); }
  inline iterator end() { return iterator( NULL ); }
};

template <typename T>
class phys_simple_allocator
{
private:
  int m_count;

public:
  inline phys_simple_allocator()
    : m_count( 0 )
  {
  }
  T *allocate()
  {
    PHYS_MEM_ALLOC( sizeof( T ), T, slot );
    if ( slot )
    {
      ++m_count;
      return slot;
    }
    else
    {
      return NULL;
    }
  }
  void free( T *slot )
  {
    if ( slot )
    {
      PHYS_MEM_VALIDATE( slot, sizeof( T ), T );
      --m_count;
      slot->~T();
      PHYS_MEM_FREE( slot, sizeof( T ), T );
    }
  }
  const int get_count() const { return m_count; }
};

template <typename T>
class phys_free_list
{
  class T_internal_base
  {
  public:
    T_internal_base *m_prev_T_internal;
    T_internal_base *m_next_T_internal;
  };

  class T_internal : public T_internal_base
  {
  public:
    T m_data;
    int m_ptr_list_index;
    inline T_internal()
      : m_ptr_list_index( -1 )
    {
    }
  };

private:
  T_internal *get_T_internal( T *p ) const { return (T_internal *)( (char *)p - sizeof( T_internal ) ); }
  T_internal_base m_dummy_head;
  int m_list_count;
  int m_list_count_high_water;
  enum
  {
    PTR_LIST_SIZE = 256
  };
  T *m_ptr_list[PTR_LIST_SIZE];
  int m_ptr_list_count;
  void debug_init()
  {
    m_list_count_high_water = 0;
    m_ptr_list_count = 0;
  }
  void debug_shutdown()
  {
    m_list_count_high_water = 0;
    m_ptr_list_count = 0;
  }
  void debug_add( T_internal *T_i )
  {
    m_list_count_high_water = tl_max( m_list_count_high_water, m_list_count );
    if ( m_ptr_list_count >= PTR_LIST_SIZE )
    {
      T_i->m_ptr_list_index = -1;
    }
    else
    {
      T_i->m_ptr_list_index = m_ptr_list_count;
      m_ptr_list[m_ptr_list_count] = &T_i->m_data;
      ++m_ptr_list_count;
    }
  }
  void debug_remove( T_internal *T_i )
  {
    if ( T_i->m_ptr_list_index != -1 )
    {
      tlAssert( T_i->m_ptr_list_index >= 0 && T_i->m_ptr_list_index < PTR_LIST_SIZE );
      tlAssert( m_ptr_list[T_i->m_ptr_list_index] == &T_i->m_data );

      --m_ptr_list_count;
      this->get_T_internal( m_ptr_list[m_ptr_list_count] )->m_ptr_list_index = T_i->m_ptr_list_index;
      m_ptr_list[T_i->m_ptr_list_index] = m_ptr_list[m_ptr_list_count];
    }
  }

public:
  phys_free_list()
  {
    m_dummy_head.m_prev_T_internal = &m_dummy_head;
    m_dummy_head.m_next_T_internal = &m_dummy_head;
    m_list_count = 0;
    debug_init();
  }
  ~phys_free_list()
  {
    remove_all();
    tlAssert( m_list_count == 0 );
    tlAssert( m_dummy_head.m_next_T_internal == &m_dummy_head );
    tlAssert( m_dummy_head.m_prev_T_internal == &m_dummy_head );
    debug_shutdown();
  }
  T *add( const int no_error = false, const char *error_msg = "phys_free_list error: out of memory." )
  {
    T_internal *ptr = (T_internal *)PMM_ALLOC( sizeof( T_internal ), tl_max( __alignof( T ), 4 ) );
    if ( ptr )
    {
      ptr->m_prev_T_internal = &m_dummy_head;
      ptr->m_next_T_internal = m_dummy_head.m_next_T_internal;
      m_dummy_head.m_next_T_internal->m_prev_T_internal = ptr;
      m_dummy_head.m_next_T_internal = ptr;
      ++m_list_count;
      debug_add( (T_internal *)ptr );
      return &ptr->m_data;
    }
    else
    {
      tlAssertMsg( no_error, error_msg );
      return NULL;
    }
  }
  void remove( T *data_ )
  {
    if ( data_ )
    {
      T_internal *ptr = get_T_internal( data_ );
      PMM_VALIDATE( ptr, sizeof( T_internal ), tl_max( __alignof( T ), PHYS_MEM_MIN_ALIGNMENT ) );
      remove( ptr );
    }
  }
  void remove( T_internal *data )
  {
    tlAssert( data );
    --m_list_count;
    debug_remove( data );
    T_internal_base *next = data->m_next_T_internal;
    T_internal_base *prev = data->m_prev_T_internal;
    prev->m_next_T_internal = next;
    next->m_prev_T_internal = prev;
    PMM_FREE( data, sizeof( T_internal ), tl_max( __alignof( T ), PHYS_MEM_MIN_ALIGNMENT ) );
  }
  void remove_all()
  {
    while ( m_dummy_head.m_next_T_internal != &m_dummy_head )
    {
      remove( static_cast<T_internal *>( m_dummy_head.m_next_T_internal ) );
    }
  }
  const int get_count() const { return m_list_count; }
  const int get_alloc_count() const { return m_ptr_list_count; }
  const int get_high_water() const { return m_list_count_high_water; }
  void validate_member( const T *data )
  {
    T_internal *ptr = get_T_internal( const_cast<T *>( data ) );
    PMM_VALIDATE( ptr, sizeof( T_internal ), tl_max( __alignof( T ), PHYS_MEM_MIN_ALIGNMENT ) );
  }
  void ptr_array_read( T **ptr_array, const int ptr_array_size )
  {
    tlAssert( ptr_array_size == m_list_count );
    T_internal_base *cur_i = m_dummy_head.m_next_T_internal;
    while ( cur_i != &m_dummy_head )
    {
      T_internal *node = static_cast<T_internal *>( cur_i );
      *ptr_array = &node->m_data;

      cur_i = cur_i->m_next_T_internal;
      ++ptr_array;
    }
  }
  void ptr_array_write( T **ptr_array, const int ptr_array_size )
  {
    tlAssert( ptr_array_size == m_list_count );
    if ( ptr_array_size > 0 )
    {
      T **ptr_i = ptr_array;
      T_internal *cur_i = get_T_internal( *ptr_i );
      cur_i->m_prev_T_internal = &m_dummy_head;
      m_dummy_head.m_next_T_internal = cur_i;
      while ( ptr_i < &ptr_array[ptr_array_size - 1] )
      {
        T_internal *next_i = get_T_internal( *++ptr_i );
        cur_i->m_next_T_internal = next_i;
        next_i->m_prev_T_internal = cur_i;
        cur_i = next_i;
      }
      cur_i->m_next_T_internal = &m_dummy_head;
      m_dummy_head.m_prev_T_internal = cur_i;
    }
  }

  class iterator
  {
  public:
    T_internal_base *m_ptr;
    inline iterator( T_internal_base *ptr )
      : m_ptr( ptr )
    {
    }
    void operator++() { m_ptr = m_ptr->m_next_T_internal; }
    bool operator!=( const iterator &other ) const { return m_ptr != other.m_ptr; }
    T &operator*() const { return static_cast<T_internal *>( m_ptr )->m_data; }
    iterator next() const { return iterator( m_ptr->m_next_T_internal ); }
    iterator next_after_remove() const { return iterator( m_ptr->m_next_T_internal ); }
  };

  iterator begin() { return iterator( m_dummy_head.m_next_T_internal ); }

  iterator end() { return iterator( &m_dummy_head ); }
};

class phys_memory_heap
{
protected:
  char *m_buffer_start;
  char *m_buffer_end;
  char *m_buffer_cur;
  char *m_user_start;

public:
  inline phys_memory_heap()
  {
    m_buffer_start = NULL;
    m_buffer_end = NULL;
    m_buffer_cur = NULL;
    m_user_start = NULL;
  }
  inline void nullify_buffer()
  {
    m_buffer_start = NULL;
    m_buffer_end = NULL;
    m_buffer_cur = NULL;
    m_user_start = NULL;
  }
  inline void set_buffer( const void *start, const int size, const int alignment )
  {
    tlAssert( m_buffer_start == NULL );
    tlAssert( m_buffer_end == NULL );
    tlAssert( m_buffer_cur == NULL );
    tlAssert( size > 0 );
    tlAssert( ( (size_t)start ) % alignment == 0 );
    m_buffer_start = (char *)start;
    m_buffer_end = m_buffer_start + size;
    m_buffer_cur = m_buffer_start;
    m_user_start = m_buffer_start;
  }
  inline void set_buffer_no_complain( const void *start, const int size, const int alignment )
  {
    tlAssert( size > 0 );
    tlAssert( ( (size_t)start ) % alignment == 0 );
    m_buffer_start = (char *)start;
    m_buffer_end = m_buffer_start + size;
    m_buffer_cur = m_buffer_start;
    m_user_start = m_buffer_start;
  }
  inline void capture_user_start() { m_user_start = m_buffer_cur; }
  inline void reset() { m_buffer_cur = m_buffer_start; }
  inline void reset_to_user_start() { m_buffer_cur = m_user_start; }
  inline void *allocate( const int size, const int alignment, const int no_error, char *error_msg = "phys_memory_heap overflow." )
  {
    char *addr = (char *)PHYS_ALIGN( m_buffer_cur, alignment );
    char *new_cur = addr + size;
    if ( new_cur <= m_buffer_end )
    {
      m_buffer_cur = new_cur;
      return addr;
    }
    else
    {
      tlAssertMsg( no_error, error_msg );
      return NULL;
    }
  }
  inline void *mt_allocate( const int size, const int alignment, const int no_error, char *error_msg = "phys_memory_heap overflow." )
  {
    char *aligned_cur;
    char *new_cur;
    do
    {
      aligned_cur = (char *)PHYS_ALIGN( m_buffer_cur, alignment );
      new_cur = aligned_cur + size;
    } while ( tlAtomicCompareAndSwap( (volatile unsigned int *)&m_buffer_cur, (unsigned int)new_cur, (unsigned int)aligned_cur ) == 0 );

    if ( new_cur <= m_buffer_end )
    {
      return aligned_cur;
    }
    else
    {
      tlAssertMsg( no_error, error_msg );
      return NULL;
    }
  }
  inline void align_cur( const int alignment )
  {
    m_buffer_cur = (char *)PHYS_ALIGN( m_buffer_cur, alignment );
    tlAssert( m_buffer_cur < m_buffer_end );
  }
  inline void *fast_align_start( const int alignment, const char *error_msg )
  {
    m_buffer_cur = (char *)PHYS_ALIGN( m_buffer_cur, alignment );
    tlAssert( m_buffer_cur < m_buffer_end );
    return m_buffer_cur;
  }
  inline void *fast_allocate( const int size, const int alignment, const char *error_msg )
  {
    char *addr = (char *)PHYS_ALIGN( m_buffer_cur, alignment );
    m_buffer_cur = addr + size;
    tlAssert( m_buffer_cur <= m_buffer_end );
    return addr;
  }
  inline void *fast_allocate( const int size, const char *error_msg )
  {
    char *addr = m_buffer_cur;
    m_buffer_cur = addr + size;
    tlAssert( m_buffer_cur <= m_buffer_end );
    return addr;
  }
  inline bool fast_is_within_buffer_limits( void *ptr, const int size )
  {
    if ( ptr >= m_buffer_start )
    {
      return (char *)( (int)ptr + size ) <= this->m_buffer_end;
    }
    else
    {
      return false;
    }
  }
  inline char **get_buffer_cur_ptr() { return &m_buffer_cur; }
  inline char *get_buffer_start() { return m_buffer_start; }
  inline char *get_buffer_cur() { return m_buffer_cur; }
  inline char *get_buffer_end() { return m_buffer_end; }
  inline const int get_buffer_size() { return m_buffer_end - m_buffer_start; }
  inline const int get_current_alloc_size() { return m_buffer_cur - m_buffer_start; }
  inline const int get_remaining_alloc_size() { return m_buffer_end - m_buffer_cur; }
  inline void lock_buffer()
  {
    tlAssert( m_buffer_start );
    tlAssert( m_buffer_end );
    tlAssert( m_buffer_cur );
    m_buffer_cur = m_buffer_end;
  }
  inline const phys_memory_heap capture_state()
  {
    phys_memory_heap state;
    state.m_buffer_start = m_buffer_start;
    state.m_buffer_end = m_buffer_end;
    state.m_buffer_cur = m_buffer_cur;
    state.m_user_start = m_user_start;
    return state;
  }
  inline void reset_to_state( phys_memory_heap &state )
  {
    m_buffer_start = state.m_buffer_start;
    m_buffer_end = state.m_buffer_end;
    m_buffer_cur = state.m_buffer_cur;
    m_user_start = state.m_user_start;
  }
  inline const int is_empty() { return m_buffer_cur == m_buffer_start; }
};

template <class T, const int m_slot_array_size>
class phys_static_array
{
private:
  ALIGN( 16 ) char m_buffer[m_slot_array_size * sizeof( T )];
  T *const m_slot_array;
  int m_alloc_count;

public:
  phys_static_array();
  ~phys_static_array();

private:
  void call_destructors();
  void reset_buffer();

public:
  T *add( const bool no_error = false, const char *error_msg = "phys array add overflow." );
  T *add_nc( const bool no_error = false, const char *error_msg = "phys array add_nc overflow." );
  T *add_fast( const bool no_error = false, const char *error_msg = "phys array add_fast overflow." );
  T *add_block_nc( const int size, const bool no_error = false, const char *error_msg = "phys array add_block_nc overflow." );
  void remove( T *data );
  void remove_slow( T *data );
  void remove_all();
  void remove_all_ndc();
  bool is_member( T *data ) const;
  T *find_by_val( const T &data ) const;
  void remove_by_val( const T &data );
  bool is_member_by_val( const T &data ) const;
  const T &operator[]( const int i ) const;
  T &operator[]( const int i );
  T *const get_list_head();
  T *const get_list_head_wo_assert();
  const int get_max_slots() const;
  const int get_available_slots() const;
  const int get_used_slots() const;
  const int get_count() const;

  class iterator
  {
  private:
    T *m_ptr;

  public:
    iterator( T *ptr );
    void operator++( int );
    bool operator!=( const iterator &i ) const;
    T &operator*();
  };

  iterator begin();
  iterator end();
  void push_back( const T &data );
  T &back();

private:
  phys_static_array &operator=( const phys_static_array & );
};

#include "phys_array_base.inl"
