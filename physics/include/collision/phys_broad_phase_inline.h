#include "phys_broad_phase.h"

inline void broad_phase_info::collision_prolog()
{
  if ( get_flag( BPB_FIRST_UNUSED_FLAG ) )
  {
    phys_full_multiply_mat( const_cast<phys_mat44 &>( *m_cg_to_world_xform ), *m_rb_to_world_xform, *m_cg_to_rb_xform );
  }
  phys_calc_aabb_whace( m_gjk_geom, *m_cg_to_world_xform, &m_trace_aabb_min_whace, &m_trace_aabb_max_whace );
  m_trace_translation = m_rb->get_col_moved_dist();
  if ( !m_rb->get_flag( rigid_body::FLAG_ENVIRONMENT_RIGID_BODY | rigid_body::FLAG_USER_RIGID_BODY ) )
  {
    calc_largest_vel_sq( this );
  }
}

inline void environment_collision_list_remove( broad_phase_base *bpb )
{
  tlAssert( bpb->is_bpi() || bpb->is_bpg() );
  tlAssert( !bpb->get_flag( broad_phase_base::FLAG_ON_BPG_LIST ) );

  if ( bpb->get_flag( broad_phase_base::FLAG_ON_ENV_LIST ) )
  {
    G_BPM->list_bpb_remove( bpb );
    bpb->m_list_bpb_next = NULL;
    bpb->set_flag( broad_phase_base::FLAG_ON_ENV_LIST, 0 );
  }
}

inline void environment_collision_list_add( broad_phase_base *bpb )
{
  tlAssert( bpb->is_bpi() || bpb->is_bpg() );
  tlAssert( !bpb->get_flag( broad_phase_base::FLAG_ON_BPG_LIST ) );

  if ( !bpb->get_flag( broad_phase_base::FLAG_ON_ENV_LIST ) )
  {
    tlAssert( bpb->m_list_bpb_next == NULL );
    G_BPM->list_bpb_add( bpb );
    bpb->set_flag( broad_phase_base::FLAG_ON_ENV_LIST, 1 );
  }
}

inline void broad_phase_memory::reset_statistics()
{
  m_list_bpi_env = NULL;
  m_bpi_env_count = 0;
  m_bpg_env_count = 0;
  m_bpg_env_bpi_count = 0;
  m_bpi_env_no_database_count = 0;
}

inline void broad_phase_memory::increment_bpi_env_count()
{
  tlAtomicIncrement( (volatile u32 *)&m_bpi_env_count );
}

inline void broad_phase_memory::increment_bpg_env_count()
{
  tlAtomicIncrement( (volatile u32 *)&m_bpg_env_count );
}

inline void broad_phase_memory::increment_bpg_env_bpi_count()
{
  tlAtomicIncrement( (volatile u32 *)&m_bpg_env_bpi_count );
}

inline void broad_phase_memory::increment_bpi_env_no_database_count()
{
  tlAtomicIncrement( (volatile u32 *)&m_bpi_env_no_database_count );
}

inline void broad_phase_memory::list_bpb_add( broad_phase_base *bpb_to_add )
{
  bpb_to_add->m_list_bpb_next = g_list_bpb;
  g_list_bpb = bpb_to_add;
  ++g_list_bpb_count;
}

inline void broad_phase_memory::list_bpb_remove( broad_phase_base *bpb_to_remove )
{
  broad_phase_base *bpb;

  broad_phase_base **bpb_cur = &g_list_bpb;
  for ( bpb = g_list_bpb; bpb != bpb_to_remove; bpb = bpb->m_list_bpb_next )
  {
    tlAssert( bpb );
    bpb_cur = &bpb->m_list_bpb_next;
  }
  *bpb_cur = bpb->m_list_bpb_next;
  --g_list_bpb_count;
}

inline void broad_phase_memory::init_system()
{
  g_broad_phase_terrain_query_callback = NULL;
  g_list_bpb = NULL;
  g_list_bpb_count = 0;
  g_list_broad_phase_info.remove_all();
  g_list_broad_phase_group.remove_all();
  g_list_broad_phase_collision_pair.remove_all();
  g_bpei_database.purge_database();
  m_memory_high_water = 0;
}

inline void broad_phase_memory::update_memory_high_water()
{
  u64 b = g_collision_memory_buffer.get_current_alloc_size();
  m_memory_high_water = tl_max( m_memory_high_water, b );
}

inline broad_phase_memory *broad_phase_memory::allocate_buffer( const broad_phase_memory_info &bpmi )
{
  PHYS_MEM_ALLOC_PERM( sizeof( broad_phase_memory ), broad_phase_memory, bpm );

  if ( bpm )
  {
    new ( bpm ) broad_phase_memory();
  }

  // Initialize the GJK cache system with the sum of max SAP active pairs and GJK collisions
  bpm->g_phys_gjk_cache_system.initialize( bpmi.m_max_num_sap_active_pairs + bpmi.m_max_num_gjk_ci );

  // Copy and calculate surface type info database parameters
  bpm->m_max_num_surface_types = bpmi.m_max_num_surface_types;
  bpm->m_max_num_surface_type_infos = ( bpmi.m_max_num_surface_types + 1 ) * bpmi.m_max_num_surface_types / 2;

  // Allocate and initialize the surface type info database
  if ( bpm->m_max_num_surface_type_infos > 0 )
  {
    size_t db_size = bpm->m_max_num_surface_type_infos * sizeof( phys_surface_type_info );
    phys_surface_type_info *surface_type_db = (phys_surface_type_info *)PMM_PERM_ALLOCATE( db_size,
                                                                                           PHYS_ALIGNOF( phys_surface_type_info ) );
    if ( !surface_type_db )
    {
      return bpm;
    }

    bpm->g_surface_type_info_database = surface_type_db;

    // Initialize each surface type info entry
    for ( int i = 0; i < bpm->m_max_num_surface_type_infos; ++i )
    {
      bpm->g_surface_type_info_database[i].set( 0.0f, 0.0f, 0, 0 );
    }
  }
  else
  {
    bpm->g_surface_type_info_database = NULL;
  }

  return bpm;
}

inline void phys_wheel_collide_info::collision_prolog( rigid_body_constraint_wheel *rbc_wheel, phys_mat44 &b1_mat )
{
  phys_vec3 p0;
  phys_vec3 p1;

  rbc_wheel->get_wheel_collide_segment( b1_mat, &p0, &p1 );
  m_ray_pos = p0;
  m_ray_dir = p1 - p0;
  m_hit_t = 1.0f;
  m_hit_bpi = NULL;
  m_hitn = phys_vec3( 0.0f );
}

inline void phys_wheel_collide_info::collision_process( broad_phase_info *bpi )
{
  const phys_mat44 *cg_to_world_xform = bpi->m_cg_to_world_xform;
  const phys_gjk_geom *gjk_geom = bpi->m_gjk_geom;
  phys_vec3 ray_pos_loc = phys_full_inv_multiply( *cg_to_world_xform, m_ray_pos );
  phys_vec3 ray_dir_loc = phys_inv_multiply( *cg_to_world_xform, m_ray_dir );
  phys_vec3 hitn;
  float hit_t = 0.0f;

  if ( gjk_geom->ray_cast( ray_pos_loc, ray_dir_loc, m_hit_t, &hit_t, &hitn ) )
  {
    tlAssert( hit_t <= m_hit_t );
    m_hit_t = hit_t;
    m_hitn = hitn;
    m_hit_bpi = bpi;
  }
}

inline void phys_wheel_collide_info::collision_epilog( rigid_body_constraint_wheel *rbc_wheel )
{
  rbc_wheel->set_no_collision();

  if ( m_hit_bpi )
  {
    float hitn_len = Abs( m_hitn );

    if ( hitn_len > 0.000001f )
    {
      m_hitn = m_hitn / hitn_len;
      m_hitn = phys_multiply( *m_hit_bpi->m_cg_to_world_xform, m_hitn );

      const phys_vec3 hit_pos = m_ray_pos + ( m_ray_dir * m_hit_t );
      const phys_mat44 *rb_to_world_xform = m_hit_bpi->m_rb_to_world_xform;
      const phys_vec3 hitp_loc = phys_full_inv_multiply( *rb_to_world_xform, hit_pos );
      const phys_vec3 hitn_loc = phys_inv_multiply( *rb_to_world_xform, m_hitn );

      rbc_wheel->set_collision( m_hit_bpi->m_rb, hitp_loc, hitn_loc );
    }
  }
}

inline void broad_phase_group::collision_prolog()
{
  tlAssert( m_list_bpi_head );
  m_list_bpi_head->collision_prolog();
  phys_vec3 aabb1_min = m_list_bpi_head->get_trace_aabb_min_whace();
  phys_vec3 aabb1_max = m_list_bpi_head->get_trace_aabb_max_whace();
  phys_vec3 aabb2_min = m_list_bpi_head->get_trace_end_aabb_min_whace();
  phys_vec3 aabb2_max = m_list_bpi_head->get_trace_end_aabb_max_whace();
  m_env_collision_flags = m_list_bpi_head->get_env_collision_flags();
  for ( broad_phase_info *bpi = m_list_bpi_head->get_next_bpi(); bpi; bpi = bpi->get_next_bpi() )
  {
    bpi->collision_prolog();
    phys_aabb_add_aabb( bpi->get_trace_aabb_min_whace(), bpi->get_trace_aabb_max_whace(), &aabb1_min, &aabb1_max );
    phys_aabb_add_aabb( bpi->get_trace_end_aabb_min_whace(), bpi->get_trace_end_aabb_max_whace(), &aabb2_min, &aabb2_max );
    m_env_collision_flags |= bpi->get_env_collision_flags();
  }
  phys_vec3 p1, p2, half_dims;
  comp_trace_volume( aabb1_min, aabb1_max, aabb2_min, aabb2_max, &p1, &p2, &half_dims );
  m_trace_aabb_min_whace = p1 - half_dims;
  m_trace_aabb_max_whace = p1 + half_dims;
  m_trace_translation = p2 - p1;
}

inline void broad_phase_group::collision_epilog()
{
  if ( m_rbvm )
  {
    tlAssert( m_list_wci );

    const int wheel_count = m_rbvm->get_wheel_count();
    for ( int i = 0; i < wheel_count; ++i )
    {
      m_list_wci[i].collision_epilog( m_rbvm->get_wheel( i ) );
    }
  }
}

inline void broad_phase_base_list::add( broad_phase_base *bpb )
{
  broad_phase_base_list::node *cur = *m_list_cur;

  if ( !cur )
  {
    cur = (broad_phase_base_list::node *)G_BPM->g_collision_memory_buffer.mt_allocate( sizeof( broad_phase_base_list::node ),
                                                                                       PHYS_ALIGNOF( broad_phase_base_list::node ),
                                                                                       0,
                                                                                       "broad phase collision out of memory." );
    *m_list_cur = cur;
    cur->m_next = NULL;
  }

  cur->m_bpb = bpb;
  m_list_cur = &cur->m_next;
}

inline broad_phase_environment_info *get_bpei( const bpei_database_id database_id )
{
  return G_BPM->g_bpei_database.get_bpei_mt( database_id );
}

inline broad_phase_info_env *allocate_bpi_env()
{
  G_BPM->increment_bpi_env_count();

  broad_phase_info_env *bpi_env =
      reinterpret_cast<broad_phase_info_env *>( G_BPM->g_collision_memory_buffer.mt_allocate( sizeof( broad_phase_info_env ),
                                                                                              PHYS_ALIGNOF( broad_phase_info_env ),
                                                                                              0,
                                                                                              "broad phase collision out of memory." ) );
  if ( bpi_env )
  {
    new ( bpi_env ) broad_phase_info_env();
  }

  broad_phase_info_env *first;
  do
  {
    first = G_BPM->m_list_bpi_env;
    bpi_env->m_list_bpb_cluster_next = first;
  } while ( !tlAtomicCompareAndSwap( (volatile u32 *)&G_BPM->m_list_bpi_env, (u32)bpi_env, (u32)first ) );
  return bpi_env;
}

inline const phys_surface_type_info *surface_type_info_database_get( const int surface_type_1, const int surface_type_2 )
{
  tlAssert(G_BPM->g_surface_type_info_database)
  return &G_BPM->g_surface_type_info_database[surface_type_info_database_get_index( surface_type_1, surface_type_2 )];
}

inline int surface_type_info_database_get_index( const int surface_type_1, const int surface_type_2 )
{
  tlAssert( surface_type_1 >= 0 && surface_type_1 < G_BPM->m_max_num_surface_types )
  tlAssert( surface_type_2 >= 0 && surface_type_2 < G_BPM->m_max_num_surface_types )
  int index = surface_type_1 < surface_type_2 ? ( surface_type_1 + ( surface_type_2 + 1 ) * surface_type_2 / 2 ) :
                                                ( surface_type_2 + ( surface_type_1 + 1 ) * surface_type_1 / 2 );
  tlAssert( index >= 0 && index < G_BPM->m_max_num_surface_type_infos )
  return index;
}

inline void surface_type_info_database_set( const int surface_type_1, const int surface_type_2, const phys_surface_type_info &pst )
{
  tlAssert( G_BPM->g_surface_type_info_database )
  G_BPM->g_surface_type_info_database[surface_type_info_database_get_index( surface_type_1, surface_type_2 )] = pst;
}
