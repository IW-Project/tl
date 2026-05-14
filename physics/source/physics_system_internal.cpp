#include "physics_system_internal.h"

#include "rbc_defs/rbc_def_internal.h"
#include "phys_task_manager.h"
#include "jobqueue.h"

physics_system *g_physics_system;
int g_list_island_cur;

void verify_is_in_physics_system( rigid_body_constraint_contact *rbc, rigid_body *b1_, rigid_body *b2_ )
{
  g_physics_system->validate_member( b1_ );
  g_physics_system->validate_member( b2_ );
  g_physics_system->validate_member( rbc );
}

void physics_system::validate_member( const rigid_body_constraint_contact *rbc )
{
  m_list_rbc_contact.validate_member( rbc );
}

void physics_system::validate_member( const rigid_body *rb )
{
  if ( !rb )
  {
    return;
  }

  if ( rb->is_environment_rigid_body() )
  {
    tlAssert( rb == &m_environment_rigid_body );
  }
  else if ( rb->is_user_rigid_body() )
  {
    m_list_user_rigid_body.validate_member( static_cast<const user_rigid_body *>( rb ) );
  }
  else
  {
    m_list_rigid_body.validate_member( rb );
  }
}

#define GIPN( rb ) ( &( rb )->m_partition_node )

void IPN_init( rigid_body *rb_partition_head )
{
  GIPN( rb_partition_head )->m_partition_head = rb_partition_head;
  GIPN( rb_partition_head )->m_partition_tail = rb_partition_head;
  GIPN( rb_partition_head )->m_next_node = NULL;
  GIPN( rb_partition_head )->m_partition_size = 1;
}

void IPN_reset_rbc_lists( rigid_body *rb_partition_head )
{
  tlAssert( GIPN( rb_partition_head )->m_partition_head == rb_partition_head );
  GIPN( rb_partition_head )->m_rbc_point_first = NULL;
  GIPN( rb_partition_head )->m_rbc_hinge_first = NULL;
  GIPN( rb_partition_head )->m_rbc_dist_first = NULL;
  GIPN( rb_partition_head )->m_rbc_ragdoll_first = NULL;
  GIPN( rb_partition_head )->m_rbc_wheel_first = NULL;
  GIPN( rb_partition_head )->m_rbc_angular_actuator_first = NULL;
  GIPN( rb_partition_head )->m_rbc_upright_first = NULL;
  GIPN( rb_partition_head )->m_rbc_custom_orientation_first = NULL;
  GIPN( rb_partition_head )->m_rbc_custom_path_first = NULL;
  GIPN( rb_partition_head )->m_rbc_contact_first = NULL;
}

void IPN_merge( rigid_body *dest, rigid_body *source )
{
  tlAssert( GIPN( dest )->m_partition_head == dest );
  tlAssert( GIPN( source )->m_partition_head == source );
  tlAssert( dest != source );
  GIPN( dest )->m_partition_tail->m_partition_node.m_next_node = source;
  GIPN( dest )->m_partition_tail = GIPN( source )->m_partition_tail;
  GIPN( dest )->m_partition_size += GIPN( source )->m_partition_size;
  for ( rigid_body *rb = source; rb; rb = rb->m_partition_node.m_next_node )
  {
    GIPN( rb )->m_partition_head = dest->m_partition_node.m_partition_head;
  }
  GIPN( source )->m_partition_tail = NULL;
  GIPN( source )->m_partition_size = 0;
}

rigid_body *IPN_get_partition( const rigid_body_constraint &rbc )
{
  if ( rbc.get_b1() && GIPN( rbc.get_b1() )->m_partition_head )
  {
    return GIPN( rbc.get_b1() )->m_partition_head;
  }
  tlAssert( rbc.get_b2() && GIPN( rbc.get_b2() )->m_partition_head );
  return GIPN( rbc.get_b2() )->m_partition_head;
}

void IPN_partition_process( const rigid_body_constraint &rbc, int *island_count )
{
  rigid_body *b1 = rbc.get_b1();
  rigid_body *b2 = rbc.get_b2();

  if ( b1 && b2 )
  {
    rigid_body *b1_partition_head = GIPN( b1 )->m_partition_head;
    rigid_body *b2_partition_head = GIPN( b2 )->m_partition_head;

    if ( ( b1_partition_head && b2_partition_head ) && ( b1_partition_head != b2_partition_head ) )
    {
      --*island_count;
      tlAssert( *island_count > 0 );
      if ( GIPN( b1_partition_head )->m_partition_size < GIPN( b2_partition_head )->m_partition_size )
      {
        IPN_merge( b2_partition_head, b1_partition_head );
      }
      else
      {
        IPN_merge( b1_partition_head, b2_partition_head );
      }
    }
  }
}

void IPN_verify_rigid_bodies( rigid_body *rb_partition_head )
{
  tlAssert( GIPN( rb_partition_head )->m_partition_head == rb_partition_head );

  for ( rigid_body_constraint_point *rbc = GIPN( rb_partition_head )->m_rbc_point_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_hinge *rbc = GIPN( rb_partition_head )->m_rbc_hinge_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_distance *rbc = GIPN( rb_partition_head )->m_rbc_dist_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_ragdoll *rbc = GIPN( rb_partition_head )->m_rbc_ragdoll_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_wheel *rbc = GIPN( rb_partition_head )->m_rbc_wheel_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_angular_actuator *rbc = GIPN( rb_partition_head )->m_rbc_angular_actuator_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_upright *rbc = GIPN( rb_partition_head )->m_rbc_upright_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_custom_orientation *rbc = GIPN( rb_partition_head )->m_rbc_custom_orientation_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_custom_path *rbc = GIPN( rb_partition_head )->m_rbc_custom_path_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }

  for ( rigid_body_constraint_contact *rbc = GIPN( rb_partition_head )->m_rbc_contact_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    g_physics_system->validate_member( rbc->get_b1() );
    g_physics_system->validate_member( rbc->get_b2() );
  }
}

template <typename T>
void IPN_add( rigid_body *rb_partition_head, T **rbc_first, T *rbc )
{
  tlAssert( GIPN( rb_partition_head )->m_partition_head == rb_partition_head );
  rbcint::set_next( rbc, *rbc_first );
  *rbc_first = rbc;
}

bool rigid_body_island_compare( rigid_body *elem, rigid_body *pivot )
{
  return GIPN( elem )->m_partition_size > GIPN( pivot )->m_partition_size;
}

void island_sort_swap( rigid_body **a, rigid_body **b )
{
  rigid_body *temp = *a;
  *a = *b;
  *b = temp;
}

void rigid_body_island_qsort( rigid_body **list, const int list_count )
{
  if ( list_count >= 3 )
  {
    rigid_body_island_qsort( list, list_count / 2 );
    rigid_body_island_qsort( &list[list_count / 2], list_count - ( list_count / 2 ) );
    rigid_body **middle = &list[list_count / 2];
    for ( rigid_body **prev_middle = middle - 1;; prev_middle = middle++ )
    {
      if ( !( middle < &list[list_count] && rigid_body_island_compare( *middle, *prev_middle ) ) )
      {
        break;
      }
      island_sort_swap( prev_middle, middle );
      rigid_body **cur = prev_middle;
      for ( rigid_body **prev_cur = prev_middle - 1;; --prev_cur )
      {
        if ( !( cur > list && rigid_body_island_compare( *cur, *prev_cur ) ) )
        {
          break;
        }
        island_sort_swap( prev_cur, cur );
        cur = prev_cur;
      }
    }
  }
  else if ( list_count == 2 && rigid_body_island_compare( list[1], list[0] ) )
  {
    island_sort_swap( &list[0], &list[1] );
  }
}

#define PHYS_RBC_GENERATE_PARTITIONS( rbc_list )          \
  for ( auto &rbc_i : rbc_list )                          \
  {                                                       \
    rbcint::process_constraint_info( &rbc_i );            \
    IPN_partition_process( rbc_i, &m_list_island_count ); \
  }

#define PHYS_RBC_ADD_TO_PARTITION( rbc_list, rbc_name )                                         \
  for ( auto &rbc_i : rbc_list )                                                                \
  {                                                                                             \
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );                                 \
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_##rbc_name##_first, &rbc_i ); \
  }

void physics_system::generate_partitions_and_stuff( phys_transient_allocator *transient_buffer )
{
  rbint::constraint_info_reset( &m_environment_rigid_body );
  m_environment_rigid_body.m_partition_node.m_partition_head = NULL;
  for ( phys_free_list<user_rigid_body>::iterator it = m_list_user_rigid_body.begin(); it != m_list_user_rigid_body.end(); ++it )
  {
    user_rigid_body &rb_i = *it;
    rbint::constraint_info_reset( &rb_i );
    GIPN( &rb_i )->m_partition_head = NULL;
  }

  for ( phys_free_list<rigid_body>::iterator it = m_list_rigid_body.begin(); it != m_list_rigid_body.end(); ++it )
  {
    rigid_body &rb_i = *it;
    rbint::constraint_info_reset( &rb_i );
    IPN_init( &rb_i );
  }
  m_list_island_count = m_list_rigid_body.get_count();

  for ( phys_free_list<rigid_body_constraint_point>::iterator it = m_list_rbc_point.begin(); it != m_list_rbc_point.end(); ++it )
  {
    rigid_body_constraint_point &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_hinge>::iterator it = m_list_rbc_hinge.begin(); it != m_list_rbc_hinge.end(); ++it )
  {
    rigid_body_constraint_hinge &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_distance>::iterator it = m_list_rbc_dist.begin(); it != m_list_rbc_dist.end(); ++it )
  {
    rigid_body_constraint_distance &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_ragdoll>::iterator it = m_list_rbc_ragdoll.begin(); it != m_list_rbc_ragdoll.end(); ++it )
  {
    rigid_body_constraint_ragdoll &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_wheel>::iterator it = m_list_rbc_wheel.begin(); it != m_list_rbc_wheel.end(); ++it )
  {
    rigid_body_constraint_wheel &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_angular_actuator>::iterator it = m_list_rbc_angular_actuator.begin(); it != m_list_rbc_angular_actuator.end(); ++it )
  {
    rigid_body_constraint_angular_actuator &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_upright>::iterator it = m_list_rbc_upright.begin(); it != m_list_rbc_upright.end(); ++it )
  {
    rigid_body_constraint_upright &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_custom_orientation>::iterator it = m_list_rbc_custom_orientation.begin(); it != m_list_rbc_custom_orientation.end(); ++it )
  {
    rigid_body_constraint_custom_orientation &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_custom_path>::iterator it = m_list_rbc_custom_path.begin(); it != m_list_rbc_custom_path.end(); ++it )
  {
    rigid_body_constraint_custom_path &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body_constraint_contact>::iterator it = m_list_rbc_contact.begin(); it != m_list_rbc_contact.end(); ++it )
  {
    rigid_body_constraint_contact &rbc_i = *it;
    rbcint::process_constraint_info( &rbc_i );
    IPN_partition_process( rbc_i, &m_list_island_count );
  }

  for ( phys_free_list<rigid_body>::iterator it = m_list_rigid_body.begin(); it != m_list_rigid_body.end(); ++it )
  {
    rigid_body &rb_i = *it;
    if ( GIPN( &rb_i )->m_partition_size > 0 )
    {
      IPN_reset_rbc_lists( &rb_i );
    }
  }

  for ( phys_free_list<rigid_body_constraint_point>::iterator it = m_list_rbc_point.begin(); it != m_list_rbc_point.end(); ++it )
  {
    rigid_body_constraint_point &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_point_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_hinge>::iterator it = m_list_rbc_hinge.begin(); it != m_list_rbc_hinge.end(); ++it )
  {
    rigid_body_constraint_hinge &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_hinge_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_distance>::iterator it = m_list_rbc_dist.begin(); it != m_list_rbc_dist.end(); ++it )
  {
    rigid_body_constraint_distance &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_dist_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_ragdoll>::iterator it = m_list_rbc_ragdoll.begin(); it != m_list_rbc_ragdoll.end(); ++it )
  {
    rigid_body_constraint_ragdoll &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_ragdoll_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_wheel>::iterator it = m_list_rbc_wheel.begin(); it != m_list_rbc_wheel.end(); ++it )
  {
    rigid_body_constraint_wheel &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_wheel_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_angular_actuator>::iterator it = m_list_rbc_angular_actuator.begin(); it != m_list_rbc_angular_actuator.end(); ++it )
  {
    rigid_body_constraint_angular_actuator &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_angular_actuator_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_upright>::iterator it = m_list_rbc_upright.begin(); it != m_list_rbc_upright.end(); ++it )
  {
    rigid_body_constraint_upright &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_upright_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_custom_orientation>::iterator it = m_list_rbc_custom_orientation.begin(); it != m_list_rbc_custom_orientation.end(); ++it )
  {
    rigid_body_constraint_custom_orientation &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_custom_orientation_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_custom_path>::iterator it = m_list_rbc_custom_path.begin(); it != m_list_rbc_custom_path.end(); ++it )
  {
    rigid_body_constraint_custom_path &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_custom_path_first, &rbc_i );
  }

  for ( phys_free_list<rigid_body_constraint_contact>::iterator it = m_list_rbc_contact.begin(); it != m_list_rbc_contact.end(); ++it )
  {
    rigid_body_constraint_contact &rbc_i = *it;
    rigid_body *rb_partition_head = IPN_get_partition( rbc_i );
    IPN_add( rb_partition_head, &GIPN( rb_partition_head )->m_rbc_contact_first, &rbc_i );
  }

  tlAssert( transient_buffer->is_empty() );

  if ( m_list_island_count <= 0 )
  {
    m_list_island = NULL;
  }
  else
  {
    m_list_island = (rigid_body **)transient_buffer->allocate( sizeof( rigid_body * ) * m_list_island_count,
                                                               PHYS_ALIGNOF( rigid_body * ),
                                                               0,
                                                               "phys_transient_allocator out of memory." );

    int list_island_cur = 0;
    for ( phys_free_list<rigid_body>::iterator it = m_list_rigid_body.begin(); it != m_list_rigid_body.end(); ++it )
    {
      rigid_body &rb_i = *it;
      if ( GIPN( &rb_i )->m_partition_size > 0 )
      {
        IPN_verify_rigid_bodies( &rb_i );
        tlAssert( list_island_cur < m_list_island_count );
        m_list_island[list_island_cur++] = &rb_i;
      }
    }

    tlAssert( list_island_cur == m_list_island_count );
    rigid_body_island_qsort( m_list_island, m_list_island_count );
  }
}

int frame_coherent_sort_compare( rigid_body_constraint_contact *i, rigid_body_constraint_contact *i_next )
{
  return i->get_solver_priority() < i_next->get_solver_priority();
}

void frame_coherent_sort_swap( rigid_body_constraint_contact **i, rigid_body_constraint_contact **i_next )
{
  rigid_body_constraint_contact *temp = *i;
  *i = *i_next;
  *i_next = temp;
}

void merge_sort( rigid_body_constraint_contact **list, const int list_count )
{
  if ( list_count >= 3 )
  {
    merge_sort( list, list_count / 2 );
    merge_sort( &list[list_count / 2], list_count - ( list_count / 2 ) );
    rigid_body_constraint_contact **middle = &list[list_count / 2];
    for ( rigid_body_constraint_contact **prev_middle = middle - 1;; prev_middle = middle++ )
    {
      if ( !( middle < &list[list_count] && frame_coherent_sort_compare( *middle, *prev_middle ) ) )
      {
        break;
      }
      frame_coherent_sort_swap( prev_middle, middle );
      rigid_body_constraint_contact **cur = prev_middle;
      for ( rigid_body_constraint_contact **prev_cur = prev_middle - 1;; --prev_cur )
      {
        if ( !( cur > list && frame_coherent_sort_compare( *cur, *prev_cur ) ) )
        {
          break;
        }
        frame_coherent_sort_swap( prev_cur, cur );
        cur = prev_cur;
      }
    }
  }
  else if ( list_count == 2 )
  {
    if ( frame_coherent_sort_compare( list[1], list[0] ) )
    {
      frame_coherent_sort_swap( &list[0], &list[1] );
    }
  }
}

void physics_system::solver_priority_sort( phys_transient_allocator *transient_buffer )
{
  if ( m_list_rbc_contact.get_count() >= 4 )
  {
    tlAssert( transient_buffer->is_empty() );
    rigid_body_constraint_contact **ptr_array = (rigid_body_constraint_contact **)transient_buffer->allocate(
        sizeof( rigid_body_constraint_contact * ) * m_list_rbc_contact.get_count(),
        PHYS_ALIGNOF( rigid_body_constraint_contact * ),
        0,
        "phys_transient_allocator out of memory." );
    m_list_rbc_contact.ptr_array_read( ptr_array, m_list_rbc_contact.get_count() );
    merge_sort( ptr_array, m_list_rbc_contact.get_count() );
    m_list_rbc_contact.ptr_array_write( ptr_array, m_list_rbc_contact.get_count() );
    transient_buffer->reset();
  }
}

void phys_assert_info_frame_advance_all()
{
  for ( phys_assert_info *pai = g_list_phys_assert_info; pai; pai = pai->m_next )
  {
    pai->frame_advance();
  }
}

void physics_system::frame_advance( const float delta_t )
{
  phys_assert_info_frame_advance_all();

  int sub_count = (int)( delta_t / m_max_delta_t );
  if ( delta_t > ( m_max_delta_t * sub_count ) )
  {
    sub_count++;
  }
  tlAssert( delta_t <= m_max_delta_t * sub_count );

  // Prolog phase - initialize constraints
  for ( phys_free_list<user_rigid_body>::iterator it = m_list_user_rigid_body.begin(); it != m_list_user_rigid_body.end(); ++it )
  {
    user_rigid_body &urb = *it;
    rbint::prolog_frame_advance( &urb, delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_point>::iterator it = m_list_rbc_point.begin(); it != m_list_rbc_point.end(); ++it )
  {
    rigid_body_constraint_point &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_hinge>::iterator it = m_list_rbc_hinge.begin(); it != m_list_rbc_hinge.end(); ++it )
  {
    rigid_body_constraint_hinge &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_distance>::iterator it = m_list_rbc_dist.begin(); it != m_list_rbc_dist.end(); ++it )
  {
    rigid_body_constraint_distance &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_ragdoll>::iterator it = m_list_rbc_ragdoll.begin(); it != m_list_rbc_ragdoll.end(); ++it )
  {
    rigid_body_constraint_ragdoll &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_wheel>::iterator it = m_list_rbc_wheel.begin(); it != m_list_rbc_wheel.end(); ++it )
  {
    rigid_body_constraint_wheel &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_angular_actuator>::iterator it = m_list_rbc_angular_actuator.begin(); it != m_list_rbc_angular_actuator.end(); ++it )
  {
    rigid_body_constraint_angular_actuator &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_upright>::iterator it = m_list_rbc_upright.begin(); it != m_list_rbc_upright.end(); ++it )
  {
    rigid_body_constraint_upright &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_custom_orientation>::iterator it = m_list_rbc_custom_orientation.begin(); it != m_list_rbc_custom_orientation.end(); ++it )
  {
    rigid_body_constraint_custom_orientation &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_custom_path>::iterator it = m_list_rbc_custom_path.begin(); it != m_list_rbc_custom_path.end(); ++it )
  {
    rigid_body_constraint_custom_path &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_contact>::iterator it = m_list_rbc_contact.begin(); it != m_list_rbc_contact.end(); ++it )
  {
    rigid_body_constraint_contact &rbc = *it;
    rbc.outer_prolog_update( delta_t );
  }

  // Time stepping loop
  for ( int step = 0; step < sub_count; ++step )
  {
    const float step_delta_t = delta_t / sub_count;
    time_step( step_delta_t, step + 1 == sub_count );
  }

  // Epilog phase - finalize constraints
  for ( phys_free_list<rigid_body_constraint_point>::iterator it = m_list_rbc_point.begin(); it != m_list_rbc_point.end(); ++it )
  {
    rigid_body_constraint_point &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_hinge>::iterator it = m_list_rbc_hinge.begin(); it != m_list_rbc_hinge.end(); ++it )
  {
    rigid_body_constraint_hinge &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_distance>::iterator it = m_list_rbc_dist.begin(); it != m_list_rbc_dist.end(); ++it )
  {
    rigid_body_constraint_distance &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_ragdoll>::iterator it = m_list_rbc_ragdoll.begin(); it != m_list_rbc_ragdoll.end(); ++it )
  {
    rigid_body_constraint_ragdoll &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_wheel>::iterator it = m_list_rbc_wheel.begin(); it != m_list_rbc_wheel.end(); ++it )
  {
    rigid_body_constraint_wheel &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_angular_actuator>::iterator it = m_list_rbc_angular_actuator.begin(); it != m_list_rbc_angular_actuator.end(); ++it )
  {
    rigid_body_constraint_angular_actuator &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_upright>::iterator it = m_list_rbc_upright.begin(); it != m_list_rbc_upright.end(); ++it )
  {
    rigid_body_constraint_upright &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_custom_orientation>::iterator it = m_list_rbc_custom_orientation.begin(); it != m_list_rbc_custom_orientation.end(); ++it )
  {
    rigid_body_constraint_custom_orientation &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_custom_path>::iterator it = m_list_rbc_custom_path.begin(); it != m_list_rbc_custom_path.end(); ++it )
  {
    rigid_body_constraint_custom_path &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }

  for ( phys_free_list<rigid_body_constraint_contact>::iterator it = m_list_rbc_contact.begin(); it != m_list_rbc_contact.end(); ++it )
  {
    rigid_body_constraint_contact &rbc = *it;
    rbc.outer_epilog_update( delta_t );
  }
}

void physics_system::time_step( const float outside_delta_t, const bool last_step )
{
  set_outside_sub_delta_t( outside_delta_t );
  set_flag( 1u, 1 );

  // Collision prolog for all rigid bodies
  for ( phys_free_list<rigid_body>::iterator it = m_list_rigid_body.begin(); it != m_list_rigid_body.end(); ++it )
  {
    rigid_body &rb = *it;
    rbint::collision_prolog( &rb, outside_delta_t );
  }

  for ( phys_free_list<user_rigid_body>::iterator it = m_list_user_rigid_body.begin(); it != m_list_user_rigid_body.end(); ++it )
  {
    user_rigid_body &rb = *it;
    rbint::collision_prolog( &rb, outside_delta_t );
  }

  // Call collision callback
  if ( m_collision_callback )
  {
    m_collision_callback();
  }

  set_flag( 1u, 0 );

  // Remove contact constraints with no contact points
  for ( phys_free_list<rigid_body_constraint_contact>::iterator i = m_list_rbc_contact.begin(); i != m_list_rbc_contact.end(); )
  {
    if ( ( *i ).get_point_count() )
    {
      ++i;
    }
    else
    {
      phys_free_list<rigid_body_constraint_contact>::iterator next = i.next_after_remove();
      m_list_rbc_contact.remove( &*i );
      i = next;
    }
  }

  // Setup transient buffers and solve
  phys_transient_allocator transient_buffer;
  solver_priority_sort( &transient_buffer );
  generate_partitions_and_stuff( &transient_buffer );
  constraint_solver_process( &transient_buffer, this, outside_delta_t );
  transient_buffer.reset();

  // Clear island list
  m_list_island = NULL;
  m_list_island_count = 0;

  // Take next step for user rigid bodies
  if ( last_step )
  {
    for ( phys_free_list<user_rigid_body>::iterator it = m_list_user_rigid_body.begin(); it != m_list_user_rigid_body.end(); ++it )
    {
      user_rigid_body &urb = *it;
      rbint::take_last_step( &urb, outside_delta_t );
    }
  }
  else
  {
    for ( phys_free_list<user_rigid_body>::iterator it = m_list_user_rigid_body.begin(); it != m_list_user_rigid_body.end(); ++it )
    {
      user_rigid_body &urb = *it;
      rbint::take_next_step( &urb, outside_delta_t );
    }
  }

  // Update contact point buffer high water mark
  m_contact_point_buffer_high_water = tl_max( m_contact_point_buffer_high_water, (int)m_contact_point_buffer_1.get_current_alloc_size() );

  // Swap contact point buffers
  m_contact_point_buffer_2.reset();
  phys_transient_allocator temp_buffer = m_contact_point_buffer_2;
  m_contact_point_buffer_2 = m_contact_point_buffer_1;
  m_contact_point_buffer_1 = temp_buffer;
}

physics_system::physics_system()
{
  m_flags = 0;
  m_outside_sub_delta_t = 0.0f;
  m_collision_callback = NULL;
  m_max_delta_t = 0.051282052f;
  m_max_vel_iters = 8;
  m_max_vel_pos_iters = 4;
  m_environment_rigid_body.set();
  m_solver_memory_high_water = 0;
  m_contact_point_buffer_high_water = 0;
}

physics_system *physics_system::create_physics_system()
{
  PHYS_MEM_ALLOC_PERM( sizeof( physics_system ), physics_system, psys );
  return psys;
}

void physics_system::destroy_physics_system( physics_system *psys )
{
  psys->~physics_system();
}

void physics_system::initialize()
{
  g_physics_system = create_physics_system();
}

void physics_system::shutdown()
{
  destroy_physics_system( g_physics_system );
  g_physics_system = NULL;
}
