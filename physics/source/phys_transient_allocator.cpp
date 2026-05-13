#include "phys_transient_allocator.h"

#include "phys_mem_new.h"

void *phys_transient_allocator::allocate( const int size, const int alignment, const int no_error, char *error_msg )
{
  transient_allocator_update_largest_size( size );
  void *ptr = allocate_internal( size, alignment );
  if ( !ptr )
  {
    resize();
    ptr = allocate_internal( size, alignment );
    tlAssertMsg( ptr, TRANSIENT_ALLOCATE_ERROR );
  }
  tlAssert( ptr || no_error );
  return ptr;
}

void *phys_transient_allocator::mt_allocate( const int size, const int alignment, const int no_error, char *error_msg )
{
  transient_allocator_update_largest_size( size );
  m_mutex.ReadLock();
  void *ptr = mt_allocate_internal( size, alignment );
  m_mutex.ReadUnlock();

  if ( !ptr )
  {
    m_mutex.WriteLock();
    ptr = allocate_internal( size, alignment );
    if ( !ptr )
    {
      resize();
      ptr = allocate_internal( size, alignment );
      tlAssertMsg( ptr, TRANSIENT_ALLOCATE_ERROR );
    }
    m_mutex.WriteUnlock();
  }
  tlAssertMsg( ptr || no_error, error_msg );
  return ptr;
}

void phys_transient_allocator::reset()
{
  block_header *next_block;
  for ( block_header *block = m_first_block; block; block = block->m_next_block )
  {
    next_block = block->m_next_block;
    PSP_FREE( m_slot_pool, block );
  }
  m_first_block = NULL;
  m_cur = NULL;
  m_end = NULL;
  m_total_memory_allocated = 0;
}

const phys_transient_allocator::allocator_state phys_transient_allocator::capture_state()
{
  allocator_state state;
  state.m_first_block = m_first_block;
  state.m_cur = m_cur;
  state.m_end = m_end;
  state.m_total_memory_allocated = m_total_memory_allocated;
  return state;
}

void phys_transient_allocator::reset_to_state( allocator_state &as )
{
  block_header *next_block;
  for ( block_header *block = m_first_block; block != as.m_first_block; block = block->m_next_block )
  {
    next_block = block->m_next_block;
    PSP_FREE( m_slot_pool, block );
  }
  m_first_block = as.m_first_block;
  m_cur = as.m_cur;
  m_end = as.m_end;
  m_total_memory_allocated = as.m_total_memory_allocated;
}

const int phys_transient_allocator::is_empty()
{
  return m_cur == NULL;
}

const int phys_transient_allocator::get_current_alloc_size()
{
  return m_total_memory_allocated;
}

void phys_transient_allocator::resize()
{
  void* slot_pool = m_slot_pool;
  if ( !slot_pool )
  {
    slot_pool = GET_PHYS_SLOT_POOL( BLOCK_SIZE, 4 );
    m_slot_pool = slot_pool;
  }
  void *ptr = PSP_ALLOC( slot_pool );
  if ( ptr )
  {
    block_header *block = (block_header *)ptr;
    block->m_block_alignment = BLOCK_ALIGNMENT;
    block->m_block_size = BLOCK_SIZE;
    block->m_next_block = m_first_block;
    m_first_block = block;
    m_cur = (char *)ptr + sizeof( block_header );
    m_end = (char *)ptr + BLOCK_SIZE;
    m_total_memory_allocated += BLOCK_SIZE;
  }
}

void *phys_transient_allocator::allocate_internal( const int size, const int alignment )
{
  char *ptr = PHYS_ALIGN( m_cur, alignment );
  if ( &ptr[size] > m_end )
  {
    return 0;
  }
  m_cur = &ptr[size];
  return ptr;
}

void *phys_transient_allocator::mt_allocate_internal( const int size, const int alignment )
{
  char *ptr;
  char *cur;
  do
  {
    cur = m_cur;
    ptr = (char *)PHYS_ALIGN( cur, alignment );
    if ( &ptr[size] > m_end )
    {
      return 0;
    }
  } while ( !tlAtomicCompareAndSwap( (u32 *)&m_cur, (u32)&ptr[size], (u32)cur ) );
  return ptr;
}
