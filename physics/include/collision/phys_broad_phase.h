#pragma once

#include "phys_broad_phase_base.h"
#include "rbc_defs//rbc_def_vehicle.h"

class rb_vehicle_model;

class broad_phase_memory_info
{
public:
  int m_max_num_gjk_ci;
  int m_max_num_sap_active_pairs;
  int m_max_num_surface_types;

  broad_phase_memory_info()
    : m_max_num_gjk_ci( 0 ),
      m_max_num_sap_active_pairs( 0 ),
      m_max_num_surface_types( 0 )
  {
  }
};

class broad_phase_collision_pair
{
public:
  broad_phase_info *m_bpi1;
  broad_phase_info *m_bpi2;
  broad_phase_collision_pair *m_next_bpcp;
  void set( broad_phase_info *bpi1, broad_phase_info *bpi2 )
  {
    m_bpi1 = bpi1;
    m_bpi2 = bpi2;
  }
  inline void list_bpcp_init( broad_phase_collision_pair **list_bpcp ) { *list_bpcp = NULL; }
  void list_bpcp_add( broad_phase_collision_pair **list_bpcp, broad_phase_collision_pair *bpcp )
  {
    bpcp->m_next_bpcp = *list_bpcp;
    *list_bpcp = bpcp;
  }
};

class gjk_unique_id_database_t
{
public:
  typedef phys_gjk_geom_id unique_id;
  unique_id m_counter;
  gjk_unique_id_database_t()
    : m_counter( 1 )
  {
  }
  const unique_id get_unique_id()
  {
    unique_id id;
    do
    {
      id = m_counter;
      tlAssertMsg( id != 0, "gjk unique_id wrapped around." );
    } while ( !tlAtomicCompareAndSwap( &m_counter, id + 1, id ) );
    return id;
  }
  void release_unique_id( const unique_id )
  {
    ; // No-op
  };
};

extern gjk_unique_id_database_t g_gjk_unique_id_database;

class bpei_database_id
{
private:
  uint m_id1;
  uint m_id2;

public:
  bpei_database_id( const uint id1, const uint id2 )
  {
    m_id1 = id1;
    m_id2 = id2;
  }
  bpei_database_id() {}
  void set( const uint id1, const uint id2 )
  {
    m_id1 = id1;
    m_id2 = id2;
  }
  const int operator<( const bpei_database_id &id ) const
  {
    if ( m_id1 == id.m_id1 )
    {
      return m_id2 < id.m_id2;
    }
    else
    {
      return m_id1 < id.m_id1;
    }
  }
  const int operator>( const bpei_database_id &id ) const
  {
    if ( m_id1 == id.m_id1 )
    {
      return m_id2 > id.m_id2;
    }
    else
    {
      return m_id1 > id.m_id1;
    }
  }
  const int operator>=( const bpei_database_id &id ) const
  {
    if ( m_id1 == id.m_id1 )
    {
      return m_id2 >= id.m_id2;
    }
    else
    {
      return m_id1 >= id.m_id1;
    }
  }
  const int operator==( const bpei_database_id &id ) const { return ( m_id1 == id.m_id1 ) && ( m_id2 == id.m_id2 ); }
};

class broad_phase_environment_info
{
public:
  void *m_data;
  minspec_mutex m_mutex;
  unsigned int m_gjk_geom_id;
  broad_phase_environment_info *m_next_bpei;
  phys_inplace_avl_tree_node<broad_phase_environment_info> m_avl_tree_node;
  bpei_database_id m_database_id;
  struct avl_tree_accessor
  {
    static phys_inplace_avl_tree_node<broad_phase_environment_info> *get_avl_node( broad_phase_environment_info *bpei )
    {
      return &bpei->m_avl_tree_node;
    }
    static const bpei_database_id &get_avl_key( broad_phase_environment_info *bpei ) { return bpei->m_database_id; }
    static inline void set_avl_key( broad_phase_environment_info *bpei, const bpei_database_id &avl_key ) { bpei->m_database_id = avl_key; }
  };
  inline void Lock() { m_mutex.Lock(); }
  inline void Unlock() { m_mutex.Unlock(); }
  broad_phase_environment_info() {}
};

class broad_phase_base_list
{
public:
  class node
  {
  public:
    broad_phase_base *m_bpb;
    node *m_next;
  };

  node *m_list;
  node **m_list_cur;
  inline broad_phase_base_list()
  {
    m_list = NULL;
    m_list_cur = &m_list;
  }
  inline void reset() { m_list_cur = &m_list; }
  inline void add( broad_phase_base *bpb );
};

class broad_phase_environment_query_input
{
public:
  phys_vec3 trace_aabb_min_wace;
  phys_vec3 trace_aabb_max_wace;
  phys_vec3 trace_translation;
  unsigned int env_collision_flags;
};

class broad_phase_environement_query_results
{
public:
  broad_phase_base_list m_list_bpi_env;
  int m_list_bpi_env_count;
  int m_thread_id;
  unsigned int m_env_collision_flags;
  inline const unsigned int get_env_collision_flags() const { return m_env_collision_flags; }
  inline broad_phase_environement_query_results( const int thread_id ) { m_thread_id = thread_id; }
  inline void reset()
  {
    m_list_bpi_env.reset();
    m_list_bpi_env_count = 0;
    m_env_collision_flags = 0;
  }
  inline void add( broad_phase_base *bpb )
  {
    tlAssert( bpb->is_bpi_env() );
    add( bpb->get_bpi_env() );
  }
  inline void add( broad_phase_info *bpi_env )
  {
    m_list_bpi_env.add( bpi_env );
    m_list_bpi_env_count++;
    m_env_collision_flags |= bpi_env->get_env_collision_flags();
  }
};

class bpei_database_t
{
public:
  phys_inplace_avl_tree<bpei_database_id, broad_phase_environment_info, broad_phase_environment_info::avl_tree_accessor> m_bpei_map;
  phys_simple_allocator<broad_phase_environment_info> m_bpei_allocator;
  broad_phase_environment_info *m_bpei_list;
  minspec_read_write_mutex m_mutex;

private:
  broad_phase_environment_info *create_bpei( const bpei_database_id database_id )
  {
    broad_phase_environment_info *bpei = m_bpei_allocator.allocate();
    m_bpei_map.add( database_id, bpei );
    bpei->m_data = NULL;
    bpei->m_gjk_geom_id = 0;
    return bpei;
  }
  void destroy_bpei( broad_phase_environment_info *bpei )
  {
    g_gjk_unique_id_database.release_unique_id( bpei->m_gjk_geom_id );
    m_bpei_map.remove( bpei->m_database_id );
    m_bpei_allocator.free( bpei );
  }

public:
  broad_phase_environment_info *get_bpei( const bpei_database_id database_id )
  {
    broad_phase_environment_info *bpei = m_bpei_map.find( database_id );
    if ( !bpei )
    {
      bpei = create_bpei( database_id );
      bpei->m_next_bpei = m_bpei_list;
      m_bpei_list = bpei;
    }
    tlAssert( bpei );
    return bpei;
  }
  broad_phase_environment_info *get_bpei_mt( const bpei_database_id database_id )
  {
    m_mutex.ReadLock();
    broad_phase_environment_info *bpei = m_bpei_map.find( database_id );
    m_mutex.ReadUnlock();
    if ( !bpei )
    {
      m_mutex.WriteLock();
      bpei = m_bpei_map.find( database_id );
      if ( !bpei )
      {
        bpei = create_bpei( database_id );
        bpei->m_next_bpei = m_bpei_list;
        m_bpei_list = bpei;
      }
      m_mutex.WriteUnlock();
    }
    tlAssert( bpei );
    return bpei;
  }
  void update_database()
  {
    broad_phase_environment_info **prev_next = &m_bpei_list;
    for ( broad_phase_environment_info *bpei = m_bpei_list; bpei; bpei = *prev_next )
    {
      if ( bpei->m_data )
      {
        prev_next = &bpei->m_next_bpei;
        bpei->m_data = NULL;
      }
      else
      {
        *prev_next = bpei->m_next_bpei;
        destroy_bpei( bpei );
      }
    }
  }
  void purge_database()
  {
    broad_phase_environment_info *next_bpei;
    for ( broad_phase_environment_info *bpei = m_bpei_list; bpei; bpei = next_bpei )
    {
      next_bpei = bpei->m_next_bpei;
      destroy_bpei( bpei );
    }
    m_bpei_list = NULL;
  }
  bpei_database_t()
    : m_bpei_map(),
      m_bpei_allocator(),
      m_mutex()
  {
    m_bpei_list = NULL;
  }
  ~bpei_database_t() { purge_database(); }
};

class broad_phase_terrain_query_callback;

class broad_phase_memory
{
public:
  tlAtomicMutex g_bp_auto_activate_mutex;
  tlAtomicReadWriteMutex g_bp_gjk_cache_mutex;
  phys_heap_gjk_cache_system_avl_tree g_phys_gjk_cache_system;
  phys_free_list<broad_phase_info> g_list_broad_phase_info;
  phys_free_list<broad_phase_group> g_list_broad_phase_group;
  phys_free_list<broad_phase_collision_pair> g_list_broad_phase_collision_pair;
  bpei_database_t g_bpei_database;
  broad_phase_terrain_query_callback *g_broad_phase_terrain_query_callback;
  broad_phase_base *g_list_bpb;
  int g_list_bpb_count;
  broad_phase_info *m_list_bpi_env;
  int m_bpi_env_count;
  int m_bpg_env_count;
  int m_bpg_env_bpi_count;
  int m_bpi_env_no_database_count;
  int m_memory_high_water;
  void reset_statistics();
  void increment_bpi_env_count();
  void increment_bpg_env_count();
  void increment_bpg_env_bpi_count();
  void increment_bpi_env_no_database_count();
  void list_bpb_add( broad_phase_base *bpb_to_add );
  void list_bpb_remove( broad_phase_base *bpb_to_remove );
  phys_link_list<phys_collision_pair> g_list_phys_collide_data;
  phys_transient_allocator g_collision_memory_buffer;
  phys_surface_type_info *g_surface_type_info_database;
  int m_max_num_surface_types;
  int m_max_num_surface_type_infos;
  void init_system();
  void update_memory_high_water();
  static broad_phase_memory *allocate_buffer( const broad_phase_memory_info &bpmi );
};

extern broad_phase_memory *G_BPM;

#define BPM_ALLOCATE( name, buffer, type, no_error, error_msg )                                                                            \
  type *name = (type *)G_BPM->buffer.allocate( sizeof( type ), tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ), no_error, error_msg ); \
  if ( name )                                                                                                                              \
  {                                                                                                                                        \
    new ( name ) type();                                                                                                                   \
  }

#define BPM_ALLOCATE_MT( name, buffer, type, no_error, error_msg )                                                                   \
  type *name =                                                                                                                       \
      (type *)G_BPM->buffer.mt_allocate( sizeof( type ), tl_max( __alignof( type ), PHYS_MEM_MIN_ALIGNMENT ), no_error, error_msg ); \
  if ( name )                                                                                                                        \
  {                                                                                                                                  \
    new ( name ) type();                                                                                                             \
  }

class broad_phase_terrain_query_callback
{
public:
  inline void query( broad_phase_environment_query_input &, broad_phase_environement_query_results * ) {}
  broad_phase_terrain_query_callback( broad_phase_terrain_query_callback &terrain_query_callback )
  {
    G_BPM->g_broad_phase_terrain_query_callback = &terrain_query_callback;
  }
  broad_phase_terrain_query_callback() {}
};

class phys_wheel_collide_info
{
public:
  phys_vec3 m_ray_pos;
  phys_vec3 m_ray_dir;
  phys_vec3 m_hitn;
  float m_hit_t;
  broad_phase_info *m_hit_bpi;
  phys_vec3 &get_trace_aabb_min_whace() { return m_hit_bpi->get_trace_aabb_min_whace(); }
  phys_vec3 &get_trace_aabb_max_whace() { return m_hit_bpi->get_trace_aabb_max_whace(); }
  phys_vec3 &get_trace_translation() { return m_hit_bpi->get_trace_translation(); }
  inline const phys_vec3 get_trace_end_aabb_min_whace() { return m_hit_bpi->get_trace_end_aabb_min_whace(); }
  inline const phys_vec3 get_trace_end_aabb_max_whace() { return m_hit_bpi->get_trace_end_aabb_max_whace(); }
  void collision_prolog( rigid_body_constraint_wheel *rbc_wheel, phys_mat44 &b1_mat );
  void collision_process( broad_phase_info *bpi );
  void collision_epilog( rigid_body_constraint_wheel *rbc_wheel );
};

void environment_collision_list_remove( broad_phase_base *bpb );
void environment_collision_list_add( broad_phase_base *bpb );

class broad_phase_group : public broad_phase_base
{
public:
  enum
  {
    FLAG_DO_INITIAL_TUNNEL_TEST = 512,
    BPG_FIRST_UNUSED_FLAG = 1024
  };

  broad_phase_info *m_list_bpi_head;
  int m_bpi_count;
  rb_vehicle_model *m_rbvm;
  phys_wheel_collide_info *m_list_wci;
  void set()
  {
    m_list_bpi_head = NULL;
    m_bpi_count = 0;
    m_flags = 0;
    m_rbvm = NULL;
    m_list_wci = NULL;
    m_sap_node = NULL;
    m_list_bpb_next = NULL;
    m_user_data = NULL;
    set_flag( FLAG_IS_BPG, 1 );
    m_env_collision_flags = 0;
    m_my_collision_type_flags = 0;
  }
  inline void set_rb_vehicle_model( rb_vehicle_model *rbvm ) { m_rbvm = rbvm; }
  void add_bpi( broad_phase_info *bpi )
  {
    tlAssert( !bpi->get_flag( broad_phase_info::FLAG_ON_BPG_LIST ) );
    environment_collision_list_remove( bpi );
  }
  void collision_prolog();
  void collision_epilog();
};

typedef class phys_free_list<broad_phase_collision_pair> list_broad_phase_collision_pair;
typedef bool (*phys_should_collide_callback_t)(const broad_phase_base *, const broad_phase_base *);

extern broad_phase_base **g_bpb_ptr_list;
extern int g_bpb_list_index;
extern int g_bpb_list_max_index;
extern tlAtomicMutex g_prolog_task_mutex;
extern int g_bpb_cluster_sort_axis;
extern broad_phase_base * g_bpb_list_cur;
extern int g_thread_id;

void calc_largest_vel_sq( broad_phase_info *bpi );
void aasap_list_add( broad_phase_base *bpb );
void aasap_list_remove( broad_phase_base *bpb );
broad_phase_environment_info* get_bpei(const bpei_database_id database_id);
broad_phase_info_env* allocate_bpi_env();
void broad_phase_system_init(const broad_phase_memory_info& bpmi, phys_should_collide_callback_t should_collide_callback);
broad_phase_group* create_broad_phase_group();
broad_phase_info* create_broad_phase_info();
broad_phase_collision_pair* create_broad_phase_collision_pair();
void destroy_broad_phase_group(broad_phase_group* bpg);
void destroy_broad_phase_info(broad_phase_info* bpi);
void destroy_broad_phase_info_list(broad_phase_info* list_bpi);
void destroy_broad_phase_collision_pair(broad_phase_collision_pair* bpcp);
const phys_surface_type_info* surface_type_info_database_get(const int surface_type_1, const int surface_type_2);
int surface_type_info_database_get_index(const int surface_type_1, const int surface_type_2);
void surface_type_info_database_set(const int surface_type_1, const int surface_type_2, const phys_surface_type_info& pst);
void broad_phase_process();
void broad_phase_nullify_buffer();
void broad_phase_reset_buffer();

#include "phys_broad_phase_inline.h"
