#pragma once

#include "phys_math.h"
#include "phys_mem.h"
#include "phys_avl_tree.h"

typedef u32 phys_gjk_geom_id;

struct cached_simplex_info
{
  phys_vec3 m_indices[3];
  cached_simplex_info();
};

class phys_gjk_geom_id_pair_key
{
private:
  phys_gjk_geom_id m_id1;
  phys_gjk_geom_id m_id2;

public:
  inline const phys_gjk_geom_id &get_id1() const { return m_id1; }
  inline const phys_gjk_geom_id &get_id2() const { return m_id2; }
  inline phys_gjk_geom_id_pair_key( const phys_gjk_geom_id &id1, const phys_gjk_geom_id &id2 )
  {
    tlAssert( id1 < id2 );
    m_id1 = id1;
    m_id2 = id2;
  }
  inline phys_gjk_geom_id_pair_key()
  {
    m_id1 = 1;
    m_id2 = 2;
  }
  inline const int operator<( const phys_gjk_geom_id_pair_key &k ) const
  {
    if ( m_id1 == k.m_id1 )
    {
      return m_id2 > k.m_id2;
    }
    else
    {
      return m_id1 > k.m_id1;
    }
  }
  inline const int operator<=( const phys_gjk_geom_id_pair_key &k ) const
  {
    if ( m_id1 == k.m_id1 )
    {
      return m_id2 >= k.m_id2;
    }
    else
    {
      return m_id1 >= k.m_id1;
    }
  }
  inline const int operator>( const phys_gjk_geom_id_pair_key &k ) const
  {
    if ( m_id1 == k.m_id1 )
    {
      return m_id2 < k.m_id2;
    }
    else
    {
      return m_id1 < k.m_id1;
    }
  }
  inline const int operator>=( const phys_gjk_geom_id_pair_key &k ) const
  {
    if ( m_id1 == k.m_id1 )
    {
      return m_id2 <= k.m_id2;
    }
    else
    {
      return m_id1 <= k.m_id1;
    }
  }
  inline const int operator==( const phys_gjk_geom_id_pair_key &k ) const { return ( m_id1 == k.m_id1 ) && ( m_id2 == k.m_id2 ); }
  inline const int operator!=( const phys_gjk_geom_id_pair_key &k ) const { return !( *this == k ); }
};

class phys_gjk_cache_info
{
public:
  phys_vec3 m_support_dir;
  cached_simplex_info m_support_a;
  cached_simplex_info m_support_b;
  int m_support_count;
  phys_gjk_geom_id_pair_key m_key;
  uint m_flags;

  enum phys_gjk_cache_info_e
  {
    FLAG_WAS_TOUCHED = 1,
    FLAG_IS_SWAPPED = 2,
    FLAG_IS_SUPPORT_DIR_VALID = 4,
    FLAG_IS_SIMPLEX_VALID = 8
  };

  void constructor( const phys_gjk_geom_id_pair_key &key, const bool swapped )
  {
    m_key = key;
    m_flags = 0;
    set_flag( FLAG_IS_SWAPPED, swapped );
  }
  const phys_gjk_geom_id_pair_key &get_key() const { return m_key; }
  inline const uint get_flag( const uint f ) const { return m_flags & f; }
  void set_flag( const uint f, const bool b )
  {
    if ( b )
      m_flags |= f;
    else
      m_flags &= ~f;
  }
  void update_swapped( const bool swapped )
  {
    if ( swapped != ( get_flag( FLAG_IS_SWAPPED ) != 0 ) )
    {
      set_flag( FLAG_IS_SWAPPED, swapped );
      if ( get_flag( FLAG_IS_SUPPORT_DIR_VALID ) )
      {
        m_support_dir = -m_support_dir;
      }
      if ( get_flag( FLAG_IS_SIMPLEX_VALID ) )
      {
        cached_simplex_info temp = m_support_a;
        m_support_a = m_support_b;
        m_support_b = temp;
      }
    }
  }
  const phys_vec3 &get_support_dir() const
  {
    tlAssert( get_flag( FLAG_IS_SUPPORT_DIR_VALID ) );
    return m_support_dir;
  }
  const int get_num_simplex_verts() const
  {
    tlAssert( get_flag( FLAG_IS_SIMPLEX_VALID ) );
    return m_support_count;
  }
  inline void set_flag_was_touched() { set_flag( FLAG_WAS_TOUCHED, true ); }
  inline const int get_flag_was_touched() const { return get_flag( FLAG_WAS_TOUCHED ); }
  void set_support_dir( const phys_vec3 &support_dir )
  {
    set_flag( FLAG_IS_SUPPORT_DIR_VALID, true );
    m_support_dir = support_dir;
  }
  inline void invalidate_support_dir() { set_flag( FLAG_IS_SUPPORT_DIR_VALID, false ); }
  inline void invalidate_simplex() { set_flag( FLAG_IS_SIMPLEX_VALID, false ); }
  inline const int is_support_dir_valid() { return get_flag( FLAG_IS_SUPPORT_DIR_VALID ); }
  inline const int is_simplex_valid() { return get_flag( FLAG_IS_SIMPLEX_VALID ); }
  phys_gjk_cache_info();
};

class phys_heap_gjk_cache_system_avl_tree
{
  struct phys_gjk_cache_info_internal : public phys_gjk_cache_info
  {
    phys_inplace_avl_tree_node<phys_gjk_cache_info_internal> m_avl_tree_node;
    phys_gjk_cache_info_internal *m_next_gjk_ci;
    struct avl_tree_accessor
    {
      static phys_inplace_avl_tree_node<phys_gjk_cache_info_internal> *get_avl_node( phys_gjk_cache_info_internal *gjk_ci )
      {
        return &gjk_ci->m_avl_tree_node;
      }
      static const phys_gjk_geom_id_pair_key &get_avl_key( phys_gjk_cache_info_internal *gjk_ci ) { return gjk_ci->m_key; }
      static void set_avl_key( phys_gjk_cache_info_internal *gjk_ci, const phys_gjk_geom_id_pair_key &avl_key ) { gjk_ci->m_key = avl_key; }
    };
    phys_gjk_cache_info_internal() {}
  };
  phys_simple_allocator<phys_gjk_cache_info_internal> m_list_phys_gjk_cache_info_internal;
  phys_inplace_avl_tree<phys_gjk_geom_id_pair_key, phys_gjk_cache_info_internal, phys_gjk_cache_info_internal::avl_tree_accessor>
      m_search_tree;
  int m_max_num_gjk_ci;
  phys_gjk_cache_info_internal *m_list_head;

public:
  void initialize( const int max_object_count )
  {
    m_max_num_gjk_ci = max_object_count;
    m_list_head = NULL;
  }
  void shutdown()
  {
    while ( m_list_head )
    {
      phys_gjk_cache_info_internal *next = m_list_head->m_next_gjk_ci;
      m_search_tree.remove( m_list_head->get_key() );
      m_list_phys_gjk_cache_info_internal.free( m_list_head );
      m_list_head = next;
    }
    tlAssert( m_list_phys_gjk_cache_info_internal.get_count() == 0 );
  }
  ~phys_heap_gjk_cache_system_avl_tree() { shutdown(); }
  int has_cache_info( phys_gjk_geom_id id1, phys_gjk_geom_id id2 )
  {
    tlAssert( id1 != id2 );

    if ( id1 > id2 )
    {
      phys_gjk_geom_id temp = id1;
      id1 = id2;
      id2 = temp;
    }

    phys_gjk_geom_id_pair_key key( id1, id2 );
    phys_gjk_cache_info_internal *gjk_ci = m_search_tree.find( key );
    return gjk_ci != NULL;
  }
  phys_gjk_cache_info *perm_allocate()
  {
    phys_gjk_cache_info_internal *gjk_ci = m_list_phys_gjk_cache_info_internal.allocate();
    if ( gjk_ci )
    {
      gjk_ci->constructor( phys_gjk_geom_id_pair_key(), false );
    }
    return gjk_ci;
  }
  void perm_free( phys_gjk_cache_info *gjk_ci )
  {
    m_list_phys_gjk_cache_info_internal.free( static_cast<phys_gjk_cache_info_internal *>( gjk_ci ) );
  }
  phys_gjk_cache_info *get_gjk_cache_info( phys_gjk_geom_id id1, phys_gjk_geom_id id2, const bool )
  {
    tlAssert( id1 != id2 );

    bool swapped;
    if ( id1 <= id2 )
    {
      swapped = false;
    }
    else
    {
      swapped = true;
      phys_gjk_geom_id temp = id1;
      id1 = id2;
      id2 = temp;
    }

    phys_gjk_geom_id_pair_key key( id1, id2 );
    phys_gjk_cache_info_internal *gjk_ci = m_search_tree.find( key );
    if ( gjk_ci )
    {
      gjk_ci->update_swapped( swapped );
      return gjk_ci;
    }
    else
    {
      PHYS_ASSERT( pai_gjk_cache_system_max_num_gjk_ci,
                   m_list_phys_gjk_cache_info_internal.get_count() < m_max_num_gjk_ci,
                   "max num gjk_ci reached." );

      if ( m_list_phys_gjk_cache_info_internal.get_count() < m_max_num_gjk_ci )
      {
        gjk_ci = m_list_phys_gjk_cache_info_internal.allocate();
        PHYS_ASSERT( pai_gjk_cache_system_max_num_gjk_ci, gjk_ci, "gjk_ci memory allocation failed." );
        if ( gjk_ci )
        {
          gjk_ci->constructor( key, swapped );
          m_search_tree.add( key, gjk_ci );
          gjk_ci->m_next_gjk_ci = m_list_head;
          m_list_head = gjk_ci;
        }
      }
    }
    return gjk_ci;
  }
  phys_gjk_cache_info *
  get_gjk_cache_info_mutex( phys_gjk_geom_id id1, phys_gjk_geom_id id2, tlAtomicReadWriteMutex &query_mutex, const bool )
  {
    tlAssert( id1 != id2 );

    bool swapped;
    if ( id1 <= id2 )
    {
      swapped = false;
    }
    else
    {
      swapped = true;
      phys_gjk_geom_id temp = id1;
      id1 = id2;
      id2 = temp;
    }

    phys_gjk_geom_id_pair_key key( id1, id2 );
    query_mutex.ReadLock();
    phys_gjk_cache_info_internal *gjk_ci = m_search_tree.find( key );
    query_mutex.ReadUnlock();
    if ( gjk_ci )
    {
      gjk_ci->update_swapped( swapped );
      return gjk_ci;
    }
    else
    {
      PHYS_ASSERT( pai_gjk_cache_system_max_num_gjk_ci,
                   m_list_phys_gjk_cache_info_internal.get_count() < m_max_num_gjk_ci,
                   "max num gjk_ci reached." );

      if ( m_list_phys_gjk_cache_info_internal.get_count() < m_max_num_gjk_ci )
      {
        query_mutex.WriteLock();
        tlAssert( m_search_tree.find( key ) == NULL );
        gjk_ci = m_list_phys_gjk_cache_info_internal.allocate();
        PHYS_ASSERT( pai_gjk_cache_system_max_num_gjk_ci, gjk_ci, "gjk_ci memory allocation failed." );
        if ( gjk_ci )
        {
          gjk_ci->constructor( key, swapped );
          m_search_tree.add( key, gjk_ci );
          gjk_ci->m_next_gjk_ci = m_list_head;
          m_list_head = gjk_ci;
        }
        query_mutex.WriteUnlock();
      }
    }
    return gjk_ci;
  }
  void update_cache()
  {
    phys_gjk_cache_info_internal **gjk_ci_i = &m_list_head;
    for ( phys_gjk_cache_info_internal *gjk_ci = m_list_head; gjk_ci; gjk_ci = gjk_ci->m_next_gjk_ci )
    {
      if ( gjk_ci->get_flag( phys_gjk_cache_info::FLAG_WAS_TOUCHED ) )
      {
        gjk_ci->set_flag( phys_gjk_cache_info::FLAG_WAS_TOUCHED, false );
      }
      else
      {
        *gjk_ci_i = gjk_ci->m_next_gjk_ci;
        m_search_tree.remove( gjk_ci->get_key() );
        m_list_phys_gjk_cache_info_internal.free( gjk_ci );
      }
    }
  }
  phys_heap_gjk_cache_system_avl_tree()
    : m_list_phys_gjk_cache_info_internal(),
      m_search_tree()
  {
  }
};
