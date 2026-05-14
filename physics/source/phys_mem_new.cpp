#include "phys_mem_new.h"

void *g_phys_memory_buffer;
int g_phys_memory_buffer_size;
phys_memory_manager *g_phys_memory_manager;

#define allocation_owner ( (void *)0xFEDCBA98 )

template <typename T>
inline T GetStuff32( volatile const T *ptr )
{
  return *ptr;
}

const u32 phys_slot_pool::encode_size_alignment( const u32 size, const u32 alignment )
{
  tlAssert( size <= 0xFFFF );
  tlAssert( alignment <= 0xFFFF );

  return size | ( alignment << 16 );
}

void phys_slot_pool::decode_size_alignment( const u32 encoded_size, u32 &size, u32 &alignment )
{
  size = encoded_size & 0xFFFF;
  alignment = ( encoded_size >> 16 ) & 0xFFFF;
}

u32 phys_slot_pool::get_slot_size( const u32 map_key )
{
  return map_key & 0xFFFF;
}

u32 phys_slot_pool::get_slot_alignment( const u32 map_key )
{
  return ( map_key >> 16 ) & 0xFFFF;
}

phys_slot_pool *phys_slot_pool::get_this_ptr()
{
  return this;
}

u32 phys_slot_pool::get_hash_key()
{
  return m_map_key;
}

void phys_slot_pool::set_hash_key( const u32 key )
{
  m_map_key = key;
}

phys_slot_pool *phys_slot_pool::get_hash_next()
{
  return m_hash_next;
}

void phys_slot_pool::set_hash_next( phys_slot_pool *hash )
{
  m_hash_next = hash;
}

u32 phys_slot_pool::get_map_key()
{
  return m_map_key;
}

void phys_slot_pool::init( const u32 slot_size, const u32 slot_alignment )
{
  m_first_free_slot.m_ptr = 0;
  m_first_free_slot.m_tag = 0;
  m_map_key = encode_size_alignment( slot_size, slot_alignment );
  m_total_slot_count = 0;
  m_allocated_slot_count = 0;
}

void *phys_slot_pool::allocate_slot()
{
  void *return_ptr = NULL;
  tagged_void_pointer_t next_slot;
  tagged_void_pointer_t first_slot;
  u32 slot_offset;

  do
  {
    // Get current state of the free list
    first_slot.set( &m_first_free_slot );
    slot_offset = first_slot.m_ptr;

    // If no free slots available, allocate a new one
    if ( slot_offset == 0 )
    {
      // Get size and alignment requirements
      u32 map_key = get_map_key();
      u32 slot_size, slot_alignment;
      decode_size_alignment( map_key, slot_size, slot_alignment );

      // Allocate directly from the memory manager
      return_ptr = g_phys_memory_manager->allocate( slot_size, slot_alignment );

      // Initialize the extra info for this new slot
      extra_info_init( return_ptr );
      return return_ptr;
    }

    // Memory barrier to ensure proper ordering of operations
    tlMemoryFence();

    // Get the next free slot pointer from the current free slot
    next_slot.m_ptr = *(u32 *)&g_phys_memory_manager->m_buffer_start[slot_offset];
    next_slot.m_tag = first_slot.m_tag + 1;

  } while ( !tlAtomicCompareAndSwap( (volatile u64 *)&m_first_free_slot, *(u64 *)&next_slot, *(u64 *)&first_slot ) );

  // Calculate the actual pointer from the buffer offset
  return_ptr = &g_phys_memory_manager->m_buffer_start[slot_offset];

  // Update allocation tracking
  extra_info_allocate( return_ptr );

  return return_ptr;
}

void phys_slot_pool::free_slot( void *_slot )
{
  u64 next_slot;
  u32 slot;
  tagged_void_pointer_t first_slot;

  slot = (u32)_slot - (u32)g_phys_memory_manager->m_buffer_start;
  tlAssertMsg( _slot, "no support support for freeing NULL slots." );
  extra_info_free( _slot );
  do
  {
    first_slot.set( &this->m_first_free_slot );
    *(u32 *)&g_phys_memory_manager->m_buffer_start[slot] = first_slot.m_ptr;
    tlMemoryFence();
    ( *(tagged_void_pointer_t *)&next_slot ).m_ptr = slot;
    ( *(tagged_void_pointer_t *)&next_slot ).m_tag = first_slot.m_tag + 1;

  } while ( !tlAtomicCompareAndSwap( (volatile u64 *)&this->m_first_free_slot, next_slot, *(u64 *)&first_slot ) );
}

phys_slot_pool::extra_info *phys_slot_pool::get_ei( void *slot, const u32 map_key )
{
  return (phys_slot_pool::extra_info *)( (char *)slot + get_slot_size( map_key ) - sizeof( phys_slot_pool::extra_info ) );
}

void phys_slot_pool::extra_info_init( void *slot )
{
  u32 map_key = get_map_key();
  if ( slot )
  {
    extra_info *ei = get_ei( slot, map_key );
    ei->m_slot_pool_owner = get_this_ptr();
    ei->m_allocation_owner = (void *)allocation_owner;
    tlAtomicIncrement( (volatile u32 *)&m_total_slot_count );
    tlAtomicIncrement( (volatile u32 *)&m_allocated_slot_count );
    tlMemoryFence();
    tlAssert( GetStuff32( &m_allocated_slot_count ) <= GetStuff32( &m_total_slot_count ) );
  }
}

void phys_slot_pool::extra_info_allocate( void *slot )
{
  tlMemoryFence();
  u32 map_key = get_map_key();
  tlAssert( slot );
  extra_info *ei = get_ei( slot, map_key );
  phys_slot_pool* this_ = get_this_ptr();
  tlAssert( GetStuff32( &ei->m_slot_pool_owner ) == this_ );
  tlAssert( tlAtomicCompareAndSwap( (volatile u32 *)&ei->m_slot_pool_owner->m_first_free_slot.m_tag, (u32)allocation_owner, (u32)this_ ) );
  tlAtomicIncrement( (volatile u32 *)&m_allocated_slot_count );
  tlMemoryFence();
  tlAssert( GetStuff32( &m_allocated_slot_count ) <= GetStuff32( &m_total_slot_count ) );
}

void phys_slot_pool::extra_info_free( void *slot )
{
  tlMemoryFence();
  u32 map_key = get_map_key();
  tlAssert( slot );
  u32 slot_size = get_slot_size( map_key );
  memset( slot, 0xFF, slot_size - sizeof( extra_info ) );
  extra_info *ei = get_ei( slot, map_key );
  phys_slot_pool* this_ = get_this_ptr();

  tlAssert( GetStuff32( &ei->m_slot_pool_owner ) == this_ );
  tlAssert( tlAtomicCompareAndSwap( (volatile u32 *)&ei->m_allocation_owner, (u32)this_, (u32)allocation_owner ) );
  tlAtomicDecrement( (volatile u32 *)&m_allocated_slot_count );
  tlMemoryFence();
  tlAssert( GetStuff32( &m_allocated_slot_count ) <= GetStuff32( &m_total_slot_count ) );
  tlAssert( GetStuff32( &m_allocated_slot_count ) >= 0 );
}

void phys_slot_pool::validate_slot( void *slot )
{
  tlMemoryFence();
  u32 map_key = get_map_key();
  extra_info *ei = get_ei( slot, map_key );
  phys_slot_pool* this_ = get_this_ptr();
  tlAssert( GetStuff32( &ei->m_slot_pool_owner ) == this_ );
  tlAssert( GetStuff32( &ei->m_allocation_owner ) == allocation_owner );
}

//
void *phys_memory_manager::allocate( unsigned int size, unsigned int alignment )
{
  char *alignedPos;

  while ( 1 )
  {
    alignedPos = PHYS_ALIGN( m_buffer_cur, alignment );
    if ( alignedPos + size > m_buffer_end )
    {
      return 0;
    }
    if ( InterlockedCompareExchange( (volatile LONG *)m_buffer_cur, (int)( alignedPos + size ), *m_buffer_cur ) == *m_buffer_cur )
    {
      return alignedPos;
    }
  }
}

phys_slot_pool *phys_memory_manager::allocate_slot_pool()
{
  phys_slot_pool *slotPool;

  m_slot_pool_allocate_mutex.Lock();
  if ( m_list_preallocated_slot_pools_count >= 28 )
  {
    slotPool = (phys_slot_pool *)allocate( sizeof( phys_slot_pool ), 8 );
  }
  else
  {
    slotPool = &m_list_preallocated_slot_pools[m_list_preallocated_slot_pools_count++];
  }
  ++m_list_slot_pool_count;
  m_slot_pool_allocate_mutex.Unlock();
  return slotPool;
}

phys_slot_pool *phys_memory_manager::get_slot_pool( unsigned int slot_size, unsigned int slot_alignment )
{
  unsigned int key;
  phys_slot_pool *slotPool;

  tlAssert( slot_alignment >= 4 );
  tlAssert( slot_size % slot_alignment == 0 );

  key = phys_slot_pool::encode_size_alignment( slot_size + 8, slot_alignment );
  m_slot_pool_map_mutex.ReadLock();
  slotPool = m_slot_pool_map.find( key );
  m_slot_pool_map_mutex.ReadUnlock();

  if ( !slotPool )
  {
    m_slot_pool_map_mutex.WriteLock();
    slotPool = m_slot_pool_map.find( key );
    if ( !slotPool )
    {
      slotPool = allocate_slot_pool();
      slotPool->init( slot_size + 8, slot_alignment );
      m_slot_pool_map.add( key, slotPool );
    }
    m_slot_pool_map_mutex.WriteUnlock();
  }
  return slotPool;
}

phys_memory_manager::phys_memory_manager( void *memory_buffer, int memory_buffer_size )
{
  m_slot_pool_map_mutex.m_count = 1;
  memset( &m_slot_pool_map, 0, 256 );
  m_slot_pool_map.m_mod = 1;
  m_slot_pool_map.m_highest_collision = 0;
  m_slot_pool_map.m_total_collisions = 0;
  m_slot_pool_allocate_mutex.m_token = 0;
  m_buffer_start = (char *)memory_buffer;
  m_buffer_cur = m_buffer_start;
  m_buffer_end = &m_buffer_start[memory_buffer_size];
  m_list_slot_pool_count = 0;
  memset( &m_slot_pool_map, 0, 256 );
  m_slot_pool_map.m_highest_collision = 0;
  m_slot_pool_map.m_total_collisions = 0;
  m_slot_pool_map.m_mod = 1;
  m_list_preallocated_slot_pools_count = 0;
}

char *PHYS_ALIGN( char *pos, int alignment )
{
  return (char *)tl_align( (int)pos, alignment );
}

phys_slot_pool *GET_PHYS_SLOT_POOL( unsigned int size, unsigned int alignment )
{
  tlAssertMsg( size >= sizeof( uintptr_t ), "Allocations smaller than pointer size are not supported." );
  return g_phys_memory_manager->get_slot_pool( size, alignment );
}

void phys_memory_manager_init( void *memory_buffer, const int memory_buffer_size )
{
  phys_memory_manager *memManager;

  tlAssert( g_phys_memory_buffer == NULL );
  tlAssert( g_phys_memory_buffer_size == 0 );
  tlAssert( g_phys_memory_manager == NULL );

  memManager = (phys_memory_manager *)( ( (unsigned int)memory_buffer + 7 ) & 0xFFFFFFF8 );
  g_phys_memory_buffer = memory_buffer;
  g_phys_memory_buffer_size = memory_buffer_size;
  g_phys_memory_manager = memManager;
  if ( memManager )
  {
    memManager = &phys_memory_manager( memory_buffer, memory_buffer_size );
  }
}

void phys_memory_manager_term()
{
  g_phys_memory_buffer = 0;
  g_phys_memory_buffer_size = 0;
  g_phys_memory_manager = 0;
}

void transient_allocator_update_largest_size( const int size )
{
  // In each binary this is actually empty not sure why.
  ;
}

void PSP_FREE( void *slot_pool, void *slot )
{
  reinterpret_cast<phys_slot_pool *>( slot_pool )->free_slot( slot );
}

void *PMM_PERM_ALLOCATE( const size_t size, const u32 alignment )
{
  void *ptr = g_phys_memory_manager->allocate( size, alignment );
  tlAssertMsg( ptr, "physics memory manager error: out of memory." );
  return ptr;
}

void *PMM_ALLOC( const size_t size, const u32 alignment )
{
  phys_slot_pool *psp;

  tlAssert( size );
  psp = g_phys_memory_manager->get_slot_pool( size, alignment );
  return psp->allocate_slot();
}

void PMM_FREE( void *ptr, const size_t size, const u32 alignment )
{
  phys_slot_pool *slot_pool = g_phys_memory_manager->get_slot_pool( size, alignment );
  slot_pool->free_slot( ptr );
}

void *PSP_ALLOC( void *slot_pool )
{
  return reinterpret_cast<phys_slot_pool *>( slot_pool )->allocate_slot();
}

void PMM_VALIDATE( void *ptr, const size_t size, const u32 alignment )
{
  phys_slot_pool *slot_pool = g_phys_memory_manager->get_slot_pool( size, alignment );
  slot_pool->validate_slot( ptr );
}
