#pragma once

#include "phys_mutex.h"

#define TRANSIENT_ALLOCATE_ERROR "transient allocation too large, increase block_size"

class phys_transient_allocator
{
public:
  struct block_header
  {
    unsigned int m_block_size;
    unsigned int m_block_alignment;
    block_header *m_next_block;
  };

  struct allocator_state
  {
    block_header *m_first_block;
    char *m_cur;
    char *m_end;
    unsigned int m_total_memory_allocated;
  };

  enum
  {
    BLOCK_SIZE = 16384,
    BLOCK_ALIGNMENT = 4
  };

  phys_transient_allocator()
    : m_first_block( NULL ),
      m_cur( NULL ),
      m_end( NULL ),
      m_total_memory_allocated( 0 ),
      m_slot_pool( NULL )
  {
  }
  ~phys_transient_allocator() { tlAssert( m_first_block == NULL ); }

  void *allocate( const int size, const int alignment, const int no_error, char *error_msg = "" );
  void *mt_allocate( const int size, const int alignment, const int no_error, char *error_msg = "" );
  void reset();

  const allocator_state capture_state();
  void reset_to_state( allocator_state &state );
  const int is_empty();
  const int get_current_alloc_size();

private:
  void resize();
  void *allocate_internal( const int size, const int alignment );
  void *mt_allocate_internal( const int size, const int alignment );

  block_header *m_first_block;
  char *m_cur;
  char *m_end;
  unsigned int m_total_memory_allocated;
  minspec_read_write_mutex m_mutex;
  void *m_slot_pool;
};

#define TRANSIENT_ALLOCATE( name, allocator, size, type )                                       \
  type *name = (type *)allocator.allocate( size * sizeof( type ),                               \
                                           tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ), \
                                           0,                                                   \
                                           "phys_transient_allocator out of memory." );

#define TRANSIENT_ALLOCATE_CONSTRUCT( name, allocator, size, type )                             \
  type *name = (type *)allocator.allocate( size * sizeof( type ),                               \
                                           tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ), \
                                           0,                                                   \
                                           "phys_transient_allocator out of memory." );         \
  if ( name )                                                                                   \
  {                                                                                             \
    new ( name ) type();                                                                        \
  }

#define TRANSIENT_ALLOCATE_CONSTRUCT_ALIGNED( name, allocator, size, type )                                      \
  type *name = (type *)allocator.allocate(                                                                       \
      sizeof( type ) * size + PHYS_ALIGN( sizeof( type ), tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ) ), \
      tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ),                                                       \
      0,                                                                                                         \
      "phys_transient_allocator out of memory." );                                                               \
  if ( name )                                                                                                    \
  {                                                                                                              \
    new ( name ) type();                                                                                         \
  }