#pragma once

#include "phys_mutex.h"
#include "phys_hash_table.h"

#define PHYS_MEM_MIN_ALIGNMENT 4

volatile struct tagged_void_pointer_t
{
  u32 m_ptr;
  u32 m_tag;

  void set( const volatile tagged_void_pointer_t *value )
  {
    m_ptr = value->m_ptr;
    m_tag = value->m_tag;
  }
};

class phys_slot_pool
{
public:
  volatile tagged_void_pointer_t m_first_free_slot;
  unsigned int m_map_key;
  phys_slot_pool *m_hash_next;
  int m_total_slot_count;
  int m_allocated_slot_count;

  struct extra_info
  {
    phys_slot_pool *m_slot_pool_owner;
    void *m_allocation_owner;
  };
  static const u32 encode_size_alignment( const u32 size, const u32 alignment );
  static void decode_size_alignment( const u32 encoded_size, u32 &size, u32 &alignment );
  u32 get_slot_size( const u32 count );
  u32 get_slot_alignment( const u32 count );
  phys_slot_pool *get_this_ptr();
  u32 get_hash_key();
  void set_hash_key( const u32 key );
  phys_slot_pool *get_hash_next();
  void set_hash_next( phys_slot_pool *hash );
  u32 get_map_key();
  void init( const u32 slot_size, const u32 slot_alignment );
  void *allocate_slot();
  void free_slot( void *slot );
  extra_info *get_ei( void *slot, const u32 count );
  void extra_info_init( void *slot );
  void extra_info_allocate( void *slot );
  void extra_info_free( void *slot );
  void validate_slot( void *slot );
};

class phys_memory_manager
{
public:
  char *m_buffer_start;
  char *m_buffer_end;
  char *m_buffer_cur;
  int m_list_slot_pool_count;
  minspec_read_write_mutex m_slot_pool_map_mutex;
  minspec_hash_table<phys_slot_pool, 64> m_slot_pool_map;
  minspec_mutex m_slot_pool_allocate_mutex;
  __declspec( align( 8 ) ) phys_slot_pool m_list_preallocated_slot_pools[28];
  int m_list_preallocated_slot_pools_count;

  void *allocate( unsigned int size, unsigned int alignment );
  phys_slot_pool *allocate_slot_pool();
  phys_slot_pool *get_slot_pool( unsigned int slot_size, unsigned int slot_alignment );
  phys_memory_manager( void *memory_buffer, int memory_buffer_size );
};

void PSP_FREE( void *slot_pool, void *slot );
void *PMM_PERM_ALLOCATE( const size_t size, const u32 alignment );
void *PMM_ALLOC( const size_t size, const u32 alignment );
void PMM_FREE( void *ptr, const size_t size, const u32 alignment );
void *PSP_ALLOC( void *slot_pool );
void PMM_VALIDATE( void *ptr, const size_t size, const u32 alignment );
char *PHYS_ALIGN( char *pos, int alignment );
phys_slot_pool *GET_PHYS_SLOT_POOL( unsigned int size, unsigned int alignment );
void phys_memory_manager_init( void *memory_buffer, const int memory_buffer_size );
void phys_memory_manager_term();
void transient_allocator_update_largest_size( const int size );

#define PHYS_ALIGNOF( type ) tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT )

#define PHYS_MEM_ALLOC_PERM( size, type, name )                                                           \
  type *name = (type *)PMM_PERM_ALLOCATE( size, tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ) );   \
  if ( name )                                                                                             \
  {                                                                                                       \
    new ( name ) type();                                                                                   \
  }

#define PHYS_MEM_ALLOC( size, type, name ) \
  type *name = (type *)PMM_ALLOC( sizeof( type ), tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ) )

#define PHYS_MEM_VALIDATE( ptr, size, type ) PMM_VALIDATE( ptr, size, tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ) )

#define PHYS_MEM_FREE( ptr, size, type ) PMM_FREE( ptr, size, tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ) )
