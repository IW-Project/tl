
template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::call_destructors()
{
  for ( int i = 0; i < m_alloc_count; ++i )
  {
    m_slot_array[i].~T();
  }
}

template <class T, const int m_slot_array_size>
phys_static_array<T, m_slot_array_size>::phys_static_array()
  : m_slot_array( (T *)m_buffer ),
    m_alloc_count( 0 )
{
}

template <class T, const int m_slot_array_size>
phys_static_array<T, m_slot_array_size>::~phys_static_array()
{
  call_destructors();
}

template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::reset_buffer()
{
  call_destructors();
  m_alloc_count = 0;
}

template <class T, const int m_slot_array_size>
T *phys_static_array<T, m_slot_array_size>::add( const bool no_error, const char *error_msg )
{
  if ( m_alloc_count >= m_slot_array_size )
  {
    if ( no_error == false )
      tlFatal( error_msg );
    return NULL;
  }
  else
  {
    T *rbc = &m_slot_array[m_alloc_count++];
    new ( (void *)rbc ) T;
    return rbc;
  }
}

template <class T, const int m_slot_array_size>
T *phys_static_array<T, m_slot_array_size>::add_nc( const bool no_error, const char *error_msg )
{
  if ( m_alloc_count >= m_slot_array_size )
  {
    if ( no_error == false )
      tlFatal( error_msg );
    return 0;
  }
  else
  {
    return &m_slot_array[m_alloc_count++];
  }
}

template <class T, const int m_slot_array_size>
T *phys_static_array<T, m_slot_array_size>::add_fast( const bool no_error, const char *error_msg )
{
  return &m_slot_array[m_alloc_count++];
}

template <class T, const int m_slot_array_size>
T *phys_static_array<T, m_slot_array_size>::add_block_nc( const int size, const bool no_error, const char *error_msg )
{
  if ( m_alloc_count + size > m_slot_array_size )
  {
    if ( no_error == false )
      tlFatal( error_msg );
    return 0;
  }
  else
  {
    T *rbc = &m_slot_array[m_alloc_count];
    m_alloc_count += size;
    return rbc;
  }
}

template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::remove( T *data )
{
  tlAssert( sizeof( T ) == 4 || sizeof( T ) == 8 );
  tlAssert( is_member( data ) );
  data->~T();
  *data = back();
  m_alloc_count--;
}

template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::remove_slow( T *data )
{
  tlAssert( is_member( data ) );
  *data = m_slot_array[--m_alloc_count];
}

template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::remove_all()
{
  reset_buffer();
}

template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::remove_all_ndc()
{
  m_alloc_count = 0;
}

template <class T, const int m_slot_array_size>
bool phys_static_array<T, m_slot_array_size>::is_member( T *data ) const
{
  if ( ( (char *)data - (char *)m_slot_array ) % sizeof( T ) != 0 )
    return false;
  if ( !( data >= m_slot_array && data < m_slot_array + m_alloc_count ) )
    return false;
  return true;
}

template <class T, const int m_slot_array_size>
T *phys_static_array<T, m_slot_array_size>::find_by_val( const T &data ) const
{
  for ( int i = 0; i < m_alloc_count; i++ )
    if ( m_slot_array[i] == data )
      return m_slot_array + i;
  return 0;
}

template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::remove_by_val( const T &data )
{
  T *i = find_by_val( data );
  remove( i );
}

template <class T, const int m_slot_array_size>
bool phys_static_array<T, m_slot_array_size>::is_member_by_val( const T &data ) const
{
  return ( find_by_val( data ) != 0 );
}

template <class T, const int m_slot_array_size>
const T &phys_static_array<T, m_slot_array_size>::operator[]( const int i ) const
{
  tlAssert( i >= 0 && i < m_alloc_count );
  return m_slot_array[i];
}

template <class T, const int m_slot_array_size>
T &phys_static_array<T, m_slot_array_size>::operator[]( const int i )
{
  tlAssert( i >= 0 && i < m_alloc_count );
  return m_slot_array[i];
}

template <class T, const int m_slot_array_size>
inline T *const phys_static_array<T, m_slot_array_size>::get_list_head()
{
  tlAssert( m_alloc_count > 0 );
  return m_slot_array;
}

template <class T, const int m_slot_array_size>
inline T *const phys_static_array<T, m_slot_array_size>::get_list_head_wo_assert()
{
  return m_slot_array;
}

template <class T, const int m_slot_array_size>
const int phys_static_array<T, m_slot_array_size>::get_max_slots() const
{
  return m_slot_array_size;
}

template <class T, const int m_slot_array_size>
const int phys_static_array<T, m_slot_array_size>::get_available_slots() const
{
  return m_slot_array_size - m_alloc_count;
}

template <class T, const int m_slot_array_size>
const int phys_static_array<T, m_slot_array_size>::get_used_slots() const
{
  return m_alloc_count;
}

template <class T, const int m_slot_array_size>
const int phys_static_array<T, m_slot_array_size>::get_count() const
{
  return m_alloc_count;
}

template <class T, const int m_slot_array_size>
phys_static_array<T, m_slot_array_size>::iterator::iterator( T *ptr )
  : m_ptr( ptr )
{
}

template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::iterator::operator++( int )
{
  m_ptr++;
}

template <class T, const int m_slot_array_size>
bool phys_static_array<T, m_slot_array_size>::iterator::operator!=( const iterator &i ) const
{
  return i.m_ptr != m_ptr;
}

template <class T, const int m_slot_array_size>
T &phys_static_array<T, m_slot_array_size>::iterator::operator*()
{
  return *m_ptr;
}

template <class T, const int m_slot_array_size>
typename phys_static_array<T, m_slot_array_size>::iterator phys_static_array<T, m_slot_array_size>::begin()
{
  return iterator( m_slot_array );
}

template <class T, const int m_slot_array_size>
typename phys_static_array<T, m_slot_array_size>::iterator phys_static_array<T, m_slot_array_size>::end()
{
  return iterator( m_slot_array + m_alloc_count );
}

template <class T, const int m_slot_array_size>
void phys_static_array<T, m_slot_array_size>::push_back( const T &data )
{
  *add() = data;
}

template <class T, const int m_slot_array_size>
T &phys_static_array<T, m_slot_array_size>::back()
{
  tlAssert( m_alloc_count > 0 );
  return m_slot_array[m_alloc_count - 1];
}