#include "collision/phys_broad_phase.h"
#include "collision/phys_collision.h"

#include "phys_task_manager.h"
#include "physics_system_internal.h"
#include "jobqueue.h"

gjk_unique_id_database_t g_gjk_unique_id_database;
broad_phase_memory *G_BPM;

broad_phase_base **g_bpb_ptr_list;
tlAtomicMutex g_prolog_task_mutex;
int g_bpb_list_index;
int g_bpb_list_max_index;
int g_bpb_cluster_sort_axis = -1;
broad_phase_base *g_bpb_list_cur;
int g_thread_id;

phys_assert_info pai_max_num_sap_active_pair = phys_assert_info( 0, 2, 1 );
phys_assert_info pai_create_sap_active_pair = phys_assert_info( 0, 1, 1 );

static void add_collision_pair( broad_phase_info *bpi1, broad_phase_info *bpi2, const float hit_time, phys_gjk_cache_info *gjk_ci )
{
  tlAssert( hit_time >= 0.0f && hit_time <= 1.0f )
  tlAssertMsg( !IS_NANF( hit_time ), "hit_time is NaN in add_collision_pair" )
  BPM_ALLOCATE( pcp, g_collision_memory_buffer, phys_collision_pair, 0, "phys_transient_allocator out of memory." );
  pcp->m_bpi1 = bpi1;
  pcp->m_bpi2 = bpi2;
  pcp->m_hit_time = hit_time;
  pcp->m_gjk_ci = gjk_ci;
  G_BPM->g_list_phys_collide_data.add( pcp );
}

static void add_collision_pair_mutex( broad_phase_info *bpi1, broad_phase_info *bpi2, const float hit_time, phys_gjk_cache_info *gjk_ci )
{
  tlAssert( hit_time >= 0.0f && hit_time <= 1.0f )
  tlAssertMsg( !IS_NANF( hit_time ), "hit_time is NaN in add_collision_pair_mutex" )
  BPM_ALLOCATE_MT( pcp, g_collision_memory_buffer, phys_collision_pair, 0, "phys_transient_allocator out of memory." );
  pcp->m_bpi1 = bpi1;
  pcp->m_bpi2 = bpi2;
  pcp->m_hit_time = hit_time;
  pcp->m_gjk_ci = gjk_ci;
  G_BPM->g_list_phys_collide_data.add_mt( pcp );
}

struct broad_phase_prolog_task_input
{
  phys_vec3 *m_aabb_min;
  phys_vec3 *m_aabb_max;
};

class axis_aligned_sweep_and_prune
{
public:
  enum
  {
    X_AXIS = 0,
    Y_AXIS = 1,
    Z_AXIS = 2,
    NUM_AXIS = 3,
    AE_MIN_ELEMENT = 0,
    AE_MAX_ELEMENT = 1
  };

  class sap_node;

  class axis_element
  {
  public:
    sap_node *m_node;
    int m_min_max;
    float m_val;
    int m_ae_list_index;
    axis_element *m_next;
    void init( sap_node *node, const int min_max )
    {
      m_node = node;
      m_min_max = min_max;
    }
    const float get_val() const { return m_val; }
    const int is_min() const { return m_min_max == AE_MIN_ELEMENT; }
    const int is_max() const { return m_min_max == AE_MAX_ELEMENT; }
  };
  class sap_node
  {
  public:
    broad_phase_base *m_bpb;
    axis_element m_ae1[3][2];
    int m_updated;
    void add_to_list( axis_element **list_first, axis_element *ae, axis_element *ae_next )
    {
      ae->init( this, AE_MIN_ELEMENT );
      ae_next->init( this, AE_MAX_ELEMENT );
      ae->m_next = ae_next;
      ae_next->m_next = *list_first;
      *list_first = ae;
    }
    void update_ae_val()
    {
      tlAssert( m_updated == 0 )
      m_updated = 1;

      phys_vec3 aabb[2];
      m_bpb->get_aabb( aabb );
      m_ae1[0][0].m_val = aabb[0][0];
      m_ae1[0][1].m_val = aabb[1][0];
      m_ae1[1][0].m_val = aabb[0][1];
      m_ae1[1][1].m_val = aabb[1][1];
      m_ae1[2][0].m_val = aabb[0][2];
      m_ae1[2][1].m_val = aabb[1][2];
    }
    void init( broad_phase_base *bpb, axis_element **xlist, axis_element **ylist, axis_element **zlist )
    {
      m_updated = 0;
      m_bpb = bpb;
      add_to_list( xlist, m_ae1[0], &m_ae1[0][1] );
      add_to_list( ylist, m_ae1[1], &m_ae1[1][1] );
      add_to_list( zlist, m_ae1[2], &m_ae1[2][1] );
    }
  };
  const int are_overlapping( sap_node *n1, sap_node *n2 )
  {
    return n1->m_ae1[0][0].m_ae_list_index < n2->m_ae1[0][1].m_ae_list_index &&
           n2->m_ae1[0][0].m_ae_list_index < n1->m_ae1[0][1].m_ae_list_index &&
           n1->m_ae1[1][0].m_ae_list_index < n2->m_ae1[1][1].m_ae_list_index &&
           n2->m_ae1[1][0].m_ae_list_index < n1->m_ae1[1][1].m_ae_list_index &&
           n1->m_ae1[2][0].m_ae_list_index < n2->m_ae1[2][1].m_ae_list_index &&
           n2->m_ae1[2][0].m_ae_list_index < n1->m_ae1[2][1].m_ae_list_index;
  }
  phys_simple_allocator<sap_node> m_sap_node_allocator;
  axis_element *m_x_list;
  axis_element *m_y_list;
  axis_element *m_z_list;

  phys_should_collide_callback_t m_should_collide_callback;

  const bool should_collide( broad_phase_base *bpb1, broad_phase_base *bpb2 )
  {
    if ( m_should_collide_callback )
    {
      return m_should_collide_callback( bpb1, bpb2 );
    }
    else
    {
      return true;
    }
  }
  class active_pair
  {
  public:
    sap_node *m_p1;
    sap_node *m_p2;
    active_pair *m_next;
    phys_gjk_cache_info *m_gjk_ci;
    void set( sap_node *p1, sap_node *p2 )
    {
      m_p1 = p1;
      m_p2 = p2;
      m_gjk_ci = NULL;
    }
  };
  phys_simple_allocator<active_pair> m_active_pair_allocator;
  int m_max_num_active_pairs;

  static axis_aligned_sweep_and_prune *allocate_buffer( const broad_phase_memory_info &bpmi )
  {
    PHYS_MEM_ALLOC_PERM( sizeof( axis_aligned_sweep_and_prune ), axis_aligned_sweep_and_prune, aasap )
    return aasap;
  }

  void remove_ap( active_pair *ap )
  {
    G_BPM->g_phys_gjk_cache_system.perm_free( ap->m_gjk_ci );
    m_active_pair_allocator.free( ap );
  }

  active_pair *m_list_bpi_bpi;
  active_pair *m_list_bpi_bpg;
  active_pair *m_list_bpg_bpg;
  void add_active_pair( sap_node *p1, sap_node *p2 )
  {
    PHYS_ASSERT( pai_max_num_sap_active_pair,
                 m_active_pair_allocator.get_count() < m_max_num_active_pairs,
                 "max num sap active pairs reached." );

    if ( m_active_pair_allocator.get_count() < m_max_num_active_pairs )
    {
      active_pair *ap = m_active_pair_allocator.allocate();
      PHYS_ASSERT( pai_create_sap_active_pair, ap, "list sap active pairs out of memory." );
      if ( ap )
      {
        ap->set( p1, p2 );
        if ( p1->m_bpb->is_bpi() )
        {
          if ( p2->m_bpb->is_bpi() )
          {
            ap->m_next = m_list_bpi_bpi;
            m_list_bpi_bpi = ap;
          }
          else
          {
            ap->m_next = m_list_bpi_bpg;
            m_list_bpi_bpg = ap;
          }
        }
        else if ( p2->m_bpb->is_bpi() )
        {
          ap->set( p2, p1 );
          ap->m_next = m_list_bpi_bpg;
          m_list_bpi_bpg = ap;
        }
        else
        {
          ap->m_next = m_list_bpg_bpg;
          m_list_bpg_bpg = ap;
        }
      }
    }
  }
  void init_system( const int max_num_active_pairs )
  {
    tlAssert( m_active_pair_allocator.get_count() == 0 )
    tlAssert( m_sap_node_allocator.get_count() == 0 )
    m_x_list = NULL;
    m_y_list = NULL;
    m_z_list = NULL;
    m_list_bpi_bpi = NULL;
    m_list_bpi_bpg = NULL;
    m_list_bpg_bpg = NULL;
    m_max_num_active_pairs = max_num_active_pairs;
  }
  void process_active_pair_list()
  {
    active_pair *ap;
    active_pair **ap_i;
    for ( ap = m_list_bpi_bpi, ap_i = &m_list_bpi_bpi; ap; ap = *ap_i )
    {
      if ( are_overlapping( ap->m_p1, ap->m_p2 ) )
      {
        ap_i = &ap->m_next;
        broad_phase_info *bpi1 = ap->m_p1->m_bpb->get_bpi();
        broad_phase_info *bpi2 = ap->m_p2->m_bpb->get_bpi();
        float hit_time;
        if ( phys_are_potentially_colliding_whace( bpi1, bpi2, &hit_time ) )
        {
          if ( !ap->m_gjk_ci )
          {
            ap->m_gjk_ci = G_BPM->g_phys_gjk_cache_system.perm_allocate();
          }
          add_collision_pair( bpi1, bpi2, hit_time, ap->m_gjk_ci );
        }
      }
      else
      {
        *ap_i = ap->m_next;
        remove_ap( ap );
      }
    }

    for ( ap = m_list_bpi_bpg, ap_i = &m_list_bpi_bpg; ap; ap = *ap_i )
    {
      if ( are_overlapping( ap->m_p1, ap->m_p2 ) )
      {
        ap_i = &ap->m_next;
        broad_phase_info *bpi1 = ap->m_p1->m_bpb->get_bpi();
        broad_phase_group *bpg1 = ap->m_p2->m_bpb->get_bpg();
        float hit_time;
        if ( phys_are_potentially_colliding_whace( bpi1, bpg1, &hit_time ) )
        {
          for ( broad_phase_info *bpi2 = bpg1->m_list_bpi_head; bpi2; bpi2 = bpi2->get_next_bpi() )
          {
            if ( phys_are_potentially_colliding_whace( bpi1, bpi2, &hit_time ) )
            {
              add_collision_pair( bpi1, bpi2, hit_time, NULL );
            }
          }
        }
      }
      else
      {
        *ap_i = ap->m_next;
        remove_ap( ap );
      }
    }

    for ( ap = m_list_bpg_bpg, ap_i = &m_list_bpg_bpg; ap; ap = *ap_i )
    {
      if ( are_overlapping( ap->m_p1, ap->m_p2 ) )
      {
        ap_i = &ap->m_next;
        broad_phase_group *bpg1 = ap->m_p1->m_bpb->get_bpg();
        broad_phase_group *bpg2 = ap->m_p2->m_bpb->get_bpg();
        for ( broad_phase_info *bpi1 = bpg1->m_list_bpi_head; bpi1; bpi1 = bpi1->get_next_bpi() )
        {
          float hit_time;
          if ( phys_are_potentially_colliding_whace( bpi1, bpg2, &hit_time ) )
          {
            for ( broad_phase_info *bpi2 = bpg2->m_list_bpi_head; bpi2; bpi2 = bpi2->get_next_bpi() )
            {
              if ( phys_are_potentially_colliding_whace( bpi1, bpi2, &hit_time ) )
              {
                add_collision_pair( bpi1, bpi2, hit_time, NULL );
              }
            }
          }
        }
      }
      else
      {
        *ap_i = ap->m_next;
        remove_ap( ap );
      }
    }
  }
  int compare( axis_element *a1_ptr, axis_element *a2_ptr ) { return a2_ptr->get_val() > a1_ptr->get_val(); }
  void swap( axis_element **a1_ptr, axis_element **a2_ptr )
  {
    axis_element *a1 = *a1_ptr;
    axis_element *a2 = *a2_ptr;
    tlAssert( a1->m_node != a2->m_node )
    tlAssert( a1->m_ae_list_index + 1 == a2->m_ae_list_index )
    ++a1->m_ae_list_index;
    --a2->m_ae_list_index;

    if ( a1->is_max() && a2->is_min() && are_overlapping( a1->m_node, a2->m_node ) &&
         should_collide( a1->m_node->m_bpb, a2->m_node->m_bpb ) )
    {
      add_active_pair( a1->m_node, a2->m_node );
    }
    *a1_ptr = a2;
    *a2_ptr = a1;
  }
  void merge_sort( axis_element **list, const int list_count )
  {
    if ( list_count >= 3 )
    {
      merge_sort( list, list_count / 2 );
      merge_sort( &list[list_count / 2], list_count - list_count / 2 );
      axis_element **middle = &list[list_count / 2];
      for ( axis_element **prev_middle = middle - 1;; prev_middle = middle++ )
      {
        if ( !( middle < &list[list_count] && compare( *middle, *prev_middle ) ) )
        {
          break;
        }
        swap( prev_middle, middle );
        axis_element **cur = prev_middle;
        for ( axis_element **prev_cur = prev_middle - 1;; --prev_cur )
        {
          if ( !( cur > list && compare( *cur, *prev_cur ) ) )
          {
            break;
          }
          swap( prev_cur, cur );
          cur = prev_cur;
        }
      }
    }
    else if ( list_count == 2 )
    {
      if ( compare( list[1], list[0] ) )
      {
        swap( &list[0], &list[1] );
      }
    }
  }
  void process()
  {
    int axis_element_count = 2 * m_sap_node_allocator.get_count();
    if ( axis_element_count > 0 )
    {
      phys_transient_allocator transient_buffer;

      TRANSIENT_ALLOCATE( xlist, transient_buffer, axis_element_count, axis_element * )
      TRANSIENT_ALLOCATE( ylist, transient_buffer, axis_element_count, axis_element * )
      TRANSIENT_ALLOCATE( zlist, transient_buffer, axis_element_count, axis_element * )

      axis_element *xcur = m_x_list;
      axis_element *ycur = m_y_list;
      axis_element *zcur = m_z_list;
      axis_element **xl_i = xlist;
      axis_element **yl_i = ylist;
      axis_element **zl_i = zlist;
      axis_element **xl_i_last = xlist + axis_element_count;
      int ae_list_index = 0;
      while ( xl_i != xl_i_last )
      {
        tlAssert( xcur->m_node->m_updated == 1 )
        tlAssert( ycur->m_node->m_updated == 1 )
        tlAssert( zcur->m_node->m_updated == 1 )
        *xl_i++ = xcur;
        xcur->m_ae_list_index = ae_list_index;
        xcur = xcur->m_next;
        *yl_i++ = ycur;
        ycur->m_ae_list_index = ae_list_index;
        ycur = ycur->m_next;
        *zl_i++ = zcur;
        zcur->m_ae_list_index = ae_list_index;
        zcur = zcur->m_next;
        ++ae_list_index;
      }
      tlAssert( xcur == NULL && ycur == NULL && zcur == NULL )
      m_x_list = *xlist;
      m_y_list = *ylist;
      m_z_list = *zlist;
      xl_i = xlist;
      yl_i = ylist;
      zl_i = zlist;
      while ( xl_i != xl_i_last )
      {
        ( *xl_i )->m_node->m_updated = 0;
        ( *yl_i )->m_node->m_updated = 0;
        ( *zl_i )->m_node->m_updated = 0;
        ( *xl_i )->m_next = xl_i[1];
        ++xl_i;
        ( *yl_i )->m_next = yl_i[1];
        ++yl_i;
        ( *zl_i )->m_next = zl_i[1];
        ++zl_i;
      }
      ( *( xl_i - 1 ) )->m_next = NULL;
      ( *( yl_i - 1 ) )->m_next = NULL;
      ( *( zl_i - 1 ) )->m_next = NULL;
      transient_buffer.reset();
    }
    process_active_pair_list();
  }

  void create_sap_node( broad_phase_base *bpb )
  {
    tlAssert( bpb->m_sap_node == NULL )
    sap_node *node = m_sap_node_allocator.allocate();
    node->init( bpb, &m_x_list, &m_y_list, &m_z_list );
    bpb->m_sap_node = node;
  }

  void remove( active_pair **list_ap, sap_node *node )
  {
    active_pair **ap_i;
    for ( active_pair *ap = *list_ap; ap; ap = *ap_i )
    {
      if ( ap->m_p1 == node || ap->m_p2 == node )
      {
        *ap_i = ap->m_next;
        remove_ap( ap );
      }
      else
      {
        ap_i = &ap->m_next;
      }
    }
  }
  void remove( axis_element **axis_list, sap_node *node )
  {
    int count = 0;
    axis_element **ae_i = axis_list;
    for ( axis_element *ae = *axis_list; ae && count < 2; ae = *ae_i )
    {
      if ( ae->m_node == node )
      {
        *ae_i = ae->m_next;
        ++count;
      }
      else
      {
        ae_i = &ae->m_next;
      }
    }
  }
  void destroy_sap_node( broad_phase_base *bpb )
  {
    if ( bpb->m_sap_node )
    {
      sap_node *node = (sap_node *)bpb->m_sap_node;
      remove( &m_list_bpi_bpi, node );
      remove( &m_list_bpi_bpg, node );
      remove( &m_list_bpg_bpg, node );
      remove( &m_x_list, node );
      remove( &m_y_list, node );
      remove( &m_z_list, node );
      m_sap_node_allocator.free( node );
      bpb->m_sap_node = NULL;
    }
  }
  axis_aligned_sweep_and_prune()
    : m_sap_node_allocator(),
      m_active_pair_allocator() {};
};

axis_aligned_sweep_and_prune *g_axis_aligned_sweep_and_prune;

class bpi_environment_collision_info
{
public:
  broad_phase_base *m_bpb_i_start;
  broad_phase_base *m_bpb_i_end;
  int m_bpb_count;
  int m_bpb_last_count;
  bpi_environment_collision_info()
    : m_bpb_i_start( G_BPM->g_list_bpb ),
      m_bpb_i_end( NULL ),
      m_bpb_count( G_BPM->g_list_bpb_count ),
      m_bpb_last_count( G_BPM->g_list_bpb_count )
  {
  }
  void get_next()
  {
    m_bpb_i_end = m_bpb_i_start;
    m_bpb_i_start = G_BPM->g_list_bpb;
    m_bpb_count = G_BPM->g_list_bpb_count - m_bpb_last_count;
    m_bpb_last_count = G_BPM->g_list_bpb_count;
  }
  bool not_done() { return m_bpb_count != 0; }
};

static inline u32 float_to_u32( const float v )
{
  union
  {
    float f;
    u32 u;
  } bits;

  bits.f = v;
  return bits.u;
}

void broad_phase_system_init( const broad_phase_memory_info &bpmi, phys_should_collide_callback_t should_collide_callback )
{
  G_BPM = broad_phase_memory::allocate_buffer( bpmi );
  g_axis_aligned_sweep_and_prune = axis_aligned_sweep_and_prune::allocate_buffer( bpmi );
  G_BPM->init_system();
  g_axis_aligned_sweep_and_prune->init_system( bpmi.m_max_num_sap_active_pairs );
  g_axis_aligned_sweep_and_prune->m_should_collide_callback = should_collide_callback;
}

broad_phase_group *create_broad_phase_group()
{
  return G_BPM->g_list_broad_phase_group.add();
}

broad_phase_info *create_broad_phase_info()
{
  return G_BPM->g_list_broad_phase_info.add();
}

broad_phase_collision_pair *create_broad_phase_collision_pair()
{
  return G_BPM->g_list_broad_phase_collision_pair.add();
}

void destroy_broad_phase_group( broad_phase_group *bpg )
{
  environment_collision_list_remove( bpg );
  aasap_list_remove( bpg );
  G_BPM->g_list_broad_phase_group.remove( bpg );
}

void destroy_broad_phase_info( broad_phase_info *bpi )
{
  environment_collision_list_remove( bpi );
  aasap_list_remove( bpi );
  G_BPM->g_list_broad_phase_info.remove( bpi );
}

void destroy_broad_phase_info_list( broad_phase_info *list_bpi )
{
  while ( list_bpi )
  {
    broad_phase_info *bpi = list_bpi;
    tlAssert( !bpi->get_flag( broad_phase_base::FLAG_ON_ENV_LIST ) )
    tlAssert( bpi->m_sap_node == NULL )
    broad_phase_info *next_bpi = bpi->get_next_bpi();
    G_BPM->g_list_broad_phase_info.remove( bpi );
    list_bpi = next_bpi;
  }
}

void destroy_broad_phase_collision_pair( broad_phase_collision_pair *bpcp )
{
  G_BPM->g_list_broad_phase_collision_pair.remove( bpcp );
}

void calc_largest_vel_sq( broad_phase_info *bpi )
{
  tlAssert( !bpi->is_bpi_env() );
  rigid_body *rb = bpi->get_rb();
  tlAssert( !rb->get_flag( rigid_body::FLAG_ENVIRONMENT_RIGID_BODY | rigid_body::FLAG_USER_RIGID_BODY ) );

  const phys_vec3 &a_vel = rb->get_a_vel();
  const phys_vec3 &t_vel = rb->get_t_vel();
  const float a_vel_sq = AbsSquared( a_vel );
  const float t_vel_sq = AbsSquared( t_vel );
  float largest_vel_sq = 0.0f;

  if ( phys_sqr( 0.001f ) <= a_vel_sq )
  {
    phys_vec3 dir = phys_cross( t_vel, a_vel );
    float ndir_sq = AbsSquared( dir );

    if ( ndir_sq < phys_sqr( 0.001f ) )
    {
      const phys_vec3 axis_x( 1.0f, 0.0f, 0.0f );
      phys_vec3 cross_axis = phys_cross( a_vel, axis_x );
      phys_vec3 v = t_vel + cross_axis;
      dir = phys_cross( v, a_vel );
      ndir_sq = AbsSquared( dir );

      if ( ndir_sq < phys_sqr( 0.001f ) )
      {
        const phys_vec3 axis_y( 0.0f, 2.0f, 0.0f );
        cross_axis = phys_cross( a_vel, axis_y );
        v = t_vel + cross_axis;
        dir = phys_cross( v, a_vel );
        ndir_sq = AbsSquared( dir );
      }
    }

    if ( ndir_sq < phys_sqr( 0.001f ) )
    {
      largest_vel_sq = t_vel_sq + a_vel_sq;
    }
    else
    {
      phys_vec3 dir_loc = phys_inv_multiply( *bpi->m_cg_to_world_xform, dir );
      const phys_vec3 support_pt_loc = bpi->m_gjk_geom->support_only( dir_loc );
      const phys_vec3 support_pt_world = phys_full_multiply( *bpi->m_cg_to_world_xform, support_pt_loc );
      const phys_vec3 &rb_pos = bpi->m_rb_to_world_xform->GetW();
      const phys_vec3 support_pt = support_pt_world - rb_pos;
      const phys_vec3 largest_vel = t_vel + phys_cross( a_vel, support_pt );
      largest_vel_sq = AbsSquared( largest_vel );
    }
  }
  else
  {
    largest_vel_sq = t_vel_sq;
  }

  float *lvs_ptr = rb->get_largest_vel_sq_ptr();
  float lvs = 0.0f;
  do
  {
    lvs = *lvs_ptr;
    if ( lvs > largest_vel_sq )
    {
      break;
    }
  } while ( !tlAtomicCompareAndSwap( (volatile unsigned int *)lvs_ptr, float_to_u32( largest_vel_sq ), float_to_u32( lvs ) ) );
}

void aasap_list_add( broad_phase_base *bpb )
{
  g_axis_aligned_sweep_and_prune->create_sap_node( bpb );
}

void aasap_list_remove( broad_phase_base *bpb )
{
  g_axis_aligned_sweep_and_prune->destroy_sap_node( bpb );
}

static void broad_phase_set_buffer()
{
  tlAssert( G_BPM->g_collision_memory_buffer.is_empty() )
  G_BPM->g_list_phys_collide_data.remove_all();
}

static void process_cluster_environment_collision_prolog( broad_phase_base *bpb, broad_phase_base *info )
{
  if ( bpb->is_bpi() )
  {
    bpb->get_bpi()->collision_prolog();
  }
  else
  {
    bpb->get_bpg()->collision_prolog();
  }
  bpb->set_bpb_cluster_next( NULL );
  bpb->set_flag( broad_phase_base::FLAG_IS_IN_CLUSTER, false );
  info->m_trace_aabb_min_whace = phys_min( info->m_trace_aabb_min_whace, bpb->get_trace_aabb_min_whace() );
  info->m_trace_aabb_max_whace = phys_max( info->m_trace_aabb_max_whace, bpb->get_trace_aabb_max_whace() );
}

static int bp_env_jq_batch_function1( jqBatch *batch )
{
  broad_phase_base bpb;
  bpb.m_trace_aabb_min_whace = phys_vec3( 1.0e30f, 1.0e30f, 1.0e30f );
  bpb.m_trace_aabb_max_whace = phys_vec3( -1.0e30f, -1.0e30f, -1.0e30f );
  for ( int i = tlAtomicIncrement( (volatile u32 *)&g_bpb_list_index ) - 1; i < g_bpb_list_max_index;
        i = tlAtomicIncrement( (volatile u32 *)&g_bpb_list_index ) - 1 )
  {
    process_cluster_environment_collision_prolog( g_bpb_ptr_list[i], &bpb );
  }
  broad_phase_prolog_task_input *info = (broad_phase_prolog_task_input *)batch->Input;
  g_prolog_task_mutex.Lock();
  *info->m_aabb_min = phys_min( *info->m_aabb_min, bpb.get_trace_aabb_min_whace() );
  *info->m_aabb_max = phys_max( *info->m_aabb_max, bpb.get_trace_aabb_max_whace() );
  g_prolog_task_mutex.Unlock();
  return 0;
}

static void swap_bpb( broad_phase_base **bpb1, broad_phase_base **bpb2 )
{
  broad_phase_base *temp = *bpb1;
  *bpb1 = *bpb2;
  *bpb2 = temp;
}

static int compare_bpb( broad_phase_base *bpb1, broad_phase_base *bpb2 )
{
  return bpb2->get_cluster_pos( g_bpb_cluster_sort_axis ) > bpb1->get_cluster_pos( g_bpb_cluster_sort_axis );
}

static void merge_sort_bpb( broad_phase_base **list, const int list_count )
{
  if ( list_count >= 3 )
  {
    merge_sort_bpb( list, list_count / 2 );
    merge_sort_bpb( &list[list_count / 2], list_count - list_count / 2 );
    broad_phase_base **middle = &list[list_count / 2];
    for ( broad_phase_base **prev_middle = middle - 1;; prev_middle = middle++ )
    {
      if ( !( middle < &list[list_count] && compare_bpb( *middle, *prev_middle ) ) )
      {
        break;
      }
      swap_bpb( prev_middle, middle );
      broad_phase_base **cur = prev_middle;
      for ( broad_phase_base **prev_cur = prev_middle - 1;; --prev_cur )
      {
        if ( !( cur > list && compare_bpb( *cur, *prev_cur ) ) )
        {
          break;
        }
        swap_bpb( prev_cur, cur );
        cur = prev_cur;
      }
    }
  }
  else if ( list_count == 2 )
  {
    if ( compare_bpb( list[1], list[0] ) )
    {
      swap_bpb( &list[0], &list[1] );
    }
  }
}

static void check_terrain_query_params( broad_phase_base *bpb )
{
  const float MAX_TRANSLATION_SQ = phys_sqr( 510.0f );
  const float ntranslation_sq = AbsSquared( bpb->get_trace_translation() );
  PHYS_ASSERT( pai_check_terrain_query_params, ntranslation_sq <= MAX_TRANSLATION_SQ, "terrain query translation too large." );
}

static phys_vec3 phys_aabb_min_add_hace( const phys_vec3 &aabb_min )
{
  return aabb_min - rigid_body_constraint_contact::get_half_std_active_limit_distance_eps_vec();
}

static phys_vec3 phys_aabb_max_add_hace( const phys_vec3 &aabb_max )
{
  return aabb_max + rigid_body_constraint_contact::get_half_std_active_limit_distance_eps_vec();
}

static void init_bpeqi( broad_phase_environment_query_input *bpeqi, broad_phase_base *bpb )
{
  bpeqi->trace_aabb_min_wace = phys_aabb_min_add_hace( bpb->get_trace_aabb_min_whace() );
  bpeqi->trace_aabb_max_wace = phys_aabb_max_add_hace( bpb->get_trace_aabb_max_whace() );
  bpeqi->trace_translation = bpb->get_trace_translation();
  bpeqi->env_collision_flags = bpb->get_env_collision_flags();
  tlAssertMsg( !IS_NANF( bpeqi->trace_aabb_min_wace.GetX() ) && !IS_NANF( bpeqi->trace_aabb_min_wace.GetY() ) &&
                   !IS_NANF( bpeqi->trace_aabb_min_wace.GetZ() ),
               "invalid vector" )
  tlAssertMsg( !IS_NANF( bpeqi->trace_aabb_max_wace.GetX() ) && !IS_NANF( bpeqi->trace_aabb_max_wace.GetY() ) &&
                   !IS_NANF( bpeqi->trace_aabb_max_wace.GetZ() ),
               "invalid vector" )
}

static bool bpi_do_gjk_intersect( broad_phase_info *p1, broad_phase_info *p2, const float hit_time )
{
  tlAssert( hit_time >= 0.0f && hit_time <= 1.0f )
  phys_gjk_geom_id id1 = p1->get_gjk_geom_id();
  phys_gjk_geom_id id2 = p2->get_gjk_geom_id();
  tlAssert( p1 && p2 && p1 != p2 )
  tlAssert( id1 != id2 )
  phys_gjk_cache_info *gjk_ci = G_BPM->g_phys_gjk_cache_system.get_gjk_cache_info_mutex( id1, id2, G_BPM->g_bp_gjk_cache_mutex, true );
  phys_collision_pair pcp;
  pcp.m_bpi1 = p1;
  pcp.m_bpi2 = p2;
  pcp.m_hit_time = hit_time;
  pcp.m_gjk_ci = gjk_ci;
  phys_gjk_input pgi;
  setup_gjk_input_from_pcp( &pgi, &pcp );
  phys_gjk_info g_phys_gjk_info;
  return phys_collide_do_gjk_intersect( &pgi, &g_phys_gjk_info );
}

static void collide_bpi_environment( broad_phase_info *bpi, const broad_phase_environement_query_results &bpeqr )
{
  if ( bpeqr.get_env_collision_flags() & bpi->get_env_collision_flags() )
  {
    broad_phase_base_list::node *bpi_env_iter = bpeqr.m_list_bpi_env.m_list;
    broad_phase_base_list::node *last_bpi_env_iter = *bpeqr.m_list_bpi_env.m_list_cur;
    while ( bpi_env_iter != last_bpi_env_iter )
    {
      broad_phase_info_env *bpi_env = static_cast<broad_phase_info_env *>( bpi_env_iter->m_bpb );
      float hit_time = 0.0f;
      if ( phys_are_potentially_colliding_whace( bpi, bpi_env, &hit_time ) )
      {
        if ( bpi_env->get_flag( broad_phase_base::FLAG_IS_AUTO_ACTIVATE ) )
        {
          phys_auto_activate_callback *aac = bpi_env->get_aac();
          if ( !aac->has_auto_activated() )
          {
            if ( bpi_do_gjk_intersect( bpi, bpi_env, hit_time ) )
            {
              G_BPM->g_bp_auto_activate_mutex.Lock();
              if ( !aac->has_auto_activated() )
              {
                aac->auto_activate( bpi );
              }
              G_BPM->g_bp_auto_activate_mutex.Unlock();
              if ( aac->has_auto_activated() )
              {
                add_collision_pair_mutex( bpi, bpi_env, hit_time, NULL );
              }
            }
          }
        }
      }
      else
      {
        add_collision_pair_mutex( bpi, bpi_env, hit_time, NULL );
      }
      bpi_env_iter = bpi_env_iter->m_next;
    }
  }
}

template <typename T, typename U>
static bool ENV_ARE_POTENTIALLY_COLLIDING( T *b1, U *b2, float *hit_time )
{
  return ( b2->get_env_collision_flags() & b1->get_env_collision_flags() ) && phys_are_potentially_colliding_whace( b1, b2, hit_time );
}

static void do_initial_tunnel_test( broad_phase_group *bpg, const broad_phase_environement_query_results &bpeqr )
{
  bpg->set_flag( broad_phase_group::FLAG_DO_INITIAL_TUNNEL_TEST, false );
  phys_gjk_info gjk_info;
  phys_gjk_input pgi;
  phys_collision_pair pcp;
  float half_min_sep_dist = rigid_body_constraint_contact::get_std_contact_min_sep_dist() * 0.5f;
  float hit_time = 0.0f;
  for ( broad_phase_info *bpi = bpg->m_list_bpi_head; bpi; bpi = bpi->get_next_bpi() )
  {
    float smallest_lambda = 1.0f;
    float reduced_radius = bpi->get_gjk_cg()->get_geom_radius() * 0.9f + half_min_sep_dist;
    broad_phase_base_list::node *bpi_env_iter = bpeqr.m_list_bpi_env.m_list;
    broad_phase_base_list::node *last_bpi_env_iter = *bpeqr.m_list_bpi_env.m_list_cur;
    while ( bpi_env_iter != last_bpi_env_iter )
    {
      if ( ENV_ARE_POTENTIALLY_COLLIDING( bpi, bpi_env_iter->m_bpb, &hit_time ) )
      {
        pcp.m_bpi1 = bpi;
        pcp.m_bpi2 = static_cast<broad_phase_info_env *>( bpi_env_iter->m_bpb );
        pcp.m_hit_time = hit_time;
        pcp.m_gjk_ci = NULL;
        setup_gjk_input_from_pcp( &pgi, &pcp );
        pgi.cg1_radius = reduced_radius;
        pgi.set_misc( rigid_body_constraint_contact::get_std_active_limit_distance_eps(), false, true );
        if ( gjk_info.phys_collide_do_gjk_collide( &pgi ) )
        {
          if ( phys_dot( gjk_info.cg1_cinfo_loc.m_n, gjk_info.m_cg1_relative_translation_loc ) < 0.0f &&
               smallest_lambda > gjk_info.m_continuous_collision_lambda )
          {
            smallest_lambda = gjk_info.m_continuous_collision_lambda;
          }
        }
      }
      bpi_env_iter = bpi_env_iter->m_next;
    }
    bpi->get_rb()->adjust_col_moved_vec( smallest_lambda );
    bpi->collision_prolog();
  }
}

static void collide_bpg_environment( broad_phase_group *bpg, const broad_phase_environement_query_results &bpeqr )
{
  float hit_time = 0.0f;
  if ( bpeqr.get_env_collision_flags() & bpg->get_env_collision_flags() )
  {
    if ( bpg->get_flag( broad_phase_group::FLAG_DO_INITIAL_TUNNEL_TEST ) )
    {
      do_initial_tunnel_test( bpg, bpeqr );
    }
    broad_phase_base_list::node *bpi_env_iter = bpeqr.m_list_bpi_env.m_list;
    broad_phase_base_list::node *last_bpi_env_iter = *bpeqr.m_list_bpi_env.m_list_cur;
    while ( bpi_env_iter != last_bpi_env_iter )
    {
      broad_phase_info_env *bpi_env = static_cast<broad_phase_info_env *>( bpi_env_iter->m_bpb );
      if ( ENV_ARE_POTENTIALLY_COLLIDING( bpg, bpi_env, &hit_time ) )
      {
        if ( bpi_env->get_flag( broad_phase_base::FLAG_IS_AUTO_ACTIVATE ) )
        {
          phys_auto_activate_callback *aac = bpi_env->get_aac();
          for ( broad_phase_info *bpi = bpg->m_list_bpi_head; bpi && !aac->has_auto_activated(); bpi = bpi->get_next_bpi() )
          {
            if ( ENV_ARE_POTENTIALLY_COLLIDING( bpi, bpi_env, &hit_time ) && bpi_do_gjk_intersect( bpi, bpi_env, hit_time ) )
            {
              G_BPM->g_bp_auto_activate_mutex.Lock();
              if ( !aac->has_auto_activated() )
              {
                aac->auto_activate( bpi );
              }
              G_BPM->g_bp_auto_activate_mutex.Unlock();
            }
          }
        }
        else
        {
          for ( broad_phase_info *bpi = bpg->m_list_bpi_head; bpi; bpi = bpi->get_next_bpi() )
          {
            if ( ENV_ARE_POTENTIALLY_COLLIDING( bpi, bpi_env, &hit_time ) )
            {
              add_collision_pair_mutex( bpi, bpi_env, hit_time, NULL );
            }
          }
        }
      }
      bpi_env_iter = bpi_env_iter->m_next;
    }
  }
}

static void process_cluster_environment_collision( broad_phase_base *bpb, const broad_phase_environement_query_results &bpeqr )
{
  tlAssert( bpb && bpb->get_bpb_cluster_next() )
  while ( true )
  {
    bpb = bpb->get_bpb_cluster_next();
    if ( !bpb )
    {
      break;
    }
    tlAssert( bpb->get_flag( broad_phase_base::FLAG_IS_IN_CLUSTER ) )
    if ( bpb->is_bpi() )
    {
      collide_bpi_environment( bpb->get_bpi(), bpeqr );
    }
    else
    {
      collide_bpg_environment( bpb->get_bpg(), bpeqr );
    }
  }
}

static int bp_env_jq_batch_function2( jqBatch *pBatch )
{
  const int thread_id = tlAtomicIncrement( (volatile u32 *)&g_bpb_list_index ) - 1;
  broad_phase_environement_query_results bpeqr( thread_id );
  while ( true )
  {
    broad_phase_base *bpb = g_bpb_list_cur;
    if ( !g_bpb_list_cur )
    {
      break;
    }
    if ( tlAtomicCompareAndSwap( (volatile u32 *)&g_bpb_list_cur, (u32)bpb->m_list_bpb_next, (u32)g_bpb_list_cur ) )
    {
      check_terrain_query_params( bpb );
      bpeqr.reset();
      broad_phase_environment_query_input bpeqi;
      init_bpeqi( &bpeqi, bpb );
      G_BPM->g_broad_phase_terrain_query_callback->query( bpeqi, &bpeqr );
      process_cluster_environment_collision( bpb, bpeqr );
    }
  }
  return 0;
}

jqModule bp_env_jq_module1Module( "bp_env_jq_module1", JQ_WORKER_GENERIC, &bp_env_jq_batch_function1, jqBatchGroup() );
jqModule bp_env_jq_module2Module( "bp_env_jq_module2", JQ_WORKER_GENERIC, &bp_env_jq_batch_function2, jqBatchGroup() );

static void broad_phase_process_object_environment_collision( bpi_environment_collision_info &eci )
{
  phys_transient_allocator transient_buffer;
  TRANSIENT_ALLOCATE( bpb_ptr_list, transient_buffer, eci.m_bpb_count, broad_phase_base * )
  broad_phase_base **bpb_ptr_cur = bpb_ptr_list;
  for ( broad_phase_base *bpb_i = eci.m_bpb_i_start; bpb_i != eci.m_bpb_i_end; bpb_i = bpb_i->m_list_bpb_next )
  {
    tlAssert( bpb_ptr_cur - bpb_ptr_list < eci.m_bpb_count )
    *bpb_ptr_cur++ = bpb_i;
  }
  tlAssert( bpb_ptr_cur - bpb_ptr_list == eci.m_bpb_count )
  g_bpb_ptr_list = bpb_ptr_list;
  g_bpb_list_index = 0;
  g_bpb_list_max_index = eci.m_bpb_count;
  phys_vec3 bp_aabb_min( 1.0e30f, 1.0e30f, 1.0e30f );
  phys_vec3 bp_aabb_max( -1.0e30f, -1.0e30f, -1.0e30f );
  broad_phase_prolog_task_input bppti;
  bppti.m_aabb_min = &bp_aabb_min;
  bppti.m_aabb_max = &bp_aabb_max;
  phys_task_manager_process( &bp_env_jq_module1Module, &bppti, eci.m_bpb_count );
  phys_task_manager_flush();
  phys_vec3 aabb_dim = bp_aabb_max - bp_aabb_min;
  if ( aabb_dim[0] < aabb_dim[1] )
  {
    if ( aabb_dim[1] < aabb_dim[2] )
    {
      g_bpb_cluster_sort_axis = 2;
    }
    else
    {
      g_bpb_cluster_sort_axis = 1;
    }
  }
  else
  {
    if ( aabb_dim[0] < aabb_dim[2] )
    {
      g_bpb_cluster_sort_axis = 2;
    }
    else
    {
      g_bpb_cluster_sort_axis = 0;
    }
  }

  merge_sort_bpb( bpb_ptr_list, eci.m_bpb_count );
  G_BPM->g_list_bpb = *bpb_ptr_list;
  eci.m_bpb_i_start = *bpb_ptr_list;
  bpb_ptr_cur = bpb_ptr_list;
  broad_phase_base **bpb_ptr_list_last = &bpb_ptr_list[eci.m_bpb_count];
  while ( bpb_ptr_cur < bpb_ptr_list_last )
  {
    ( *bpb_ptr_cur )->m_list_bpb_next = bpb_ptr_cur[1];
    bpb_ptr_cur++;
  }
  ( *( bpb_ptr_cur - 1 ) )->m_list_bpb_next = eci.m_bpb_i_end;
  broad_phase_base *bpb_cluster_list = NULL;
  int bpb_cluster_list_count = 0;
  for ( broad_phase_base *bpb_i = eci.m_bpb_i_start; bpb_i != eci.m_bpb_i_end; bpb_i = bpb_i->m_list_bpb_next )
  {
    if ( bpb_i->get_flag( broad_phase_base::FLAG_IS_IN_CLUSTER ) )
    {
      continue;
    }

    tlAssert( bpb_i->get_bpb_cluster_next() == NULL )
    bpb_i->set_flag( broad_phase_base::FLAG_IS_IN_CLUSTER, true );
    phys_vec3 aabb1_min = bpb_i->get_trace_aabb_min_whace();
    phys_vec3 aabb1_max = bpb_i->get_trace_aabb_max_whace();
    phys_vec3 aabb2_min = bpb_i->get_trace_end_aabb_min_whace();
    phys_vec3 aabb2_max = bpb_i->get_trace_end_aabb_max_whace();
    uint env_collision_flags = bpb_i->get_env_collision_flags();
    broad_phase_base *bpb_cluster_head = bpb_i;
    float bpb_i_aabb_min = bpb_i->get_cluster_pos( g_bpb_cluster_sort_axis );
    for ( broad_phase_base *bpb_j = bpb_i->m_list_bpb_next; bpb_j != eci.m_bpb_i_end; bpb_j = bpb_j->m_list_bpb_next )
    {
      if ( bpb_j->get_cluster_pos( g_bpb_cluster_sort_axis ) - bpb_i_aabb_min > 136.0f )
      {
        break;
      }
      if ( bpb_j->get_flag( broad_phase_base::FLAG_IS_IN_CLUSTER ) )
      {
        continue;
      }

      tlAssert( bpb_j->get_bpb_cluster_next() == NULL )
      phys_vec3 aabb1_min_new = phys_min( aabb1_min, bpb_j->get_trace_aabb_min_whace() );
      phys_vec3 aabb1_max_new = phys_max( aabb1_max, bpb_j->get_trace_aabb_max_whace() );
      phys_vec3 dims1_new = aabb1_max_new - aabb1_min_new;
      if ( dims1_new.GetX() > 136.0f || dims1_new.GetY() > 136.0f || dims1_new.GetZ() > 136.0f )
      {
        continue;
      }

      phys_vec3 aabb2_min_new = phys_min( aabb2_min, bpb_j->get_trace_end_aabb_min_whace() );
      phys_vec3 aabb2_max_new = phys_max( aabb2_max, bpb_j->get_trace_end_aabb_max_whace() );
      phys_vec3 dims2_new = aabb2_max_new - aabb2_min_new;
      if ( dims2_new.GetX() > 136.0f || dims2_new.GetY() > 136.0f || dims2_new.GetZ() > 136.0f )
      {
        continue;
      }
      bpb_j->set_flag( broad_phase_base::FLAG_IS_IN_CLUSTER, true );
      aabb1_min = aabb1_min_new;
      aabb1_max = aabb1_max_new;
      aabb2_min = aabb2_min_new;
      aabb2_max = aabb2_max_new;
      env_collision_flags |= bpb_j->get_env_collision_flags();
      bpb_j->set_bpb_cluster_next( bpb_cluster_head );
      bpb_cluster_head = bpb_j;
    }

    TRANSIENT_ALLOCATE_CONSTRUCT( bpb_cluster, transient_buffer, 1, broad_phase_base );
    bpb_cluster->m_list_bpb_next = bpb_cluster_list;
    bpb_cluster_list = bpb_cluster;
    ++bpb_cluster_list_count;
    phys_vec3 p1, p2, half_dims;
    comp_trace_volume( aabb1_min, aabb1_max, aabb2_min, aabb2_max, &p1, &p2, &half_dims );
    bpb_cluster->m_trace_aabb_min_whace = p1 - half_dims;
    bpb_cluster->m_trace_aabb_max_whace = p1 + half_dims;
    bpb_cluster->m_trace_translation = p2 - p1;
    bpb_cluster->set_bpb_cluster_next( bpb_cluster_head );
    bpb_cluster->set_env_collision_flags( env_collision_flags );
  }

  g_bpb_list_cur = bpb_cluster_list;
  g_bpb_list_index = 0;
  g_bpb_list_max_index = bpb_cluster_list_count;
  g_thread_id = 0;
  phys_task_manager_process( &bp_env_jq_module2Module, NULL, bpb_cluster_list_count );
  phys_task_manager_flush();
  transient_buffer.reset();
}

static void broad_phase_process_object_environment_collision()
{
  bpi_environment_collision_info eci;
  while ( eci.not_done() )
  {
    broad_phase_process_object_environment_collision( eci );
  }
}

static void broad_phase_process_collision_pairs()
{
  float hit_time = 0.0f;
  list_broad_phase_collision_pair::iterator bpcp_i = G_BPM->g_list_broad_phase_collision_pair.begin();
  list_broad_phase_collision_pair::iterator bpcp_i_end = G_BPM->g_list_broad_phase_collision_pair.end();
  while ( bpcp_i != bpcp_i_end )
  {
    broad_phase_collision_pair &bpcp = *bpcp_i;
    if ( phys_are_potentially_colliding_whace( bpcp.m_bpi1, bpcp.m_bpi2, &hit_time ) )
    {
      add_collision_pair( bpcp.m_bpi1, bpcp.m_bpi2, hit_time, NULL );
    }
    ++bpcp_i;
  }
}

void broad_phase_nullify_buffer()
{
  G_BPM->g_list_phys_collide_data.remove_all();
}

void broad_phase_process()
{
  G_BPM->reset_statistics();
  tlAssert( !tlScratchpadLocked )
  tlScratchpadLocked = true;
  broad_phase_set_buffer();
  broad_phase_process_object_environment_collision();
  broad_phase_process_collision_pairs();
  for ( broad_phase_base *bpb = G_BPM->g_list_bpb; bpb; bpb = bpb->m_list_bpb_next )
  {
    if ( bpb->m_sap_node )
    {
      ( (axis_aligned_sweep_and_prune::sap_node *)bpb->m_sap_node )->update_ae_val();
    }
  }
  g_axis_aligned_sweep_and_prune->process();
  phys_link_list<phys_collision_pair>::iterator pcp_i = G_BPM->g_list_phys_collide_data.begin();
  phys_link_list<phys_collision_pair>::iterator pcp_i_end = G_BPM->g_list_phys_collide_data.end();
  while ( pcp_i != pcp_i_end )
  {
    phys_collision_pair &pcp = *pcp_i;
    if ( !pcp.m_gjk_ci )
    {
      pcp.m_gjk_ci =
          G_BPM->g_phys_gjk_cache_system.get_gjk_cache_info( pcp.m_bpi1->get_gjk_geom_id(), pcp.m_bpi2->get_gjk_geom_id(), true );
    }
    ++pcp_i;
  }
  G_BPM->update_memory_high_water();
  process_list_do_gjk_collide_and_contact_manifold( &G_BPM->g_list_phys_collide_data );
  broad_phase_nullify_buffer();
  G_BPM->g_phys_gjk_cache_system.update_cache();
  G_BPM->g_bpei_database.update_database();
}

void broad_phase_reset_buffer()
{
  G_BPM->g_collision_memory_buffer.reset();
}
