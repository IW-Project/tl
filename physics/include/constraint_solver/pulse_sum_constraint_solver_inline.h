#pragma once

#include "pulse_sum_constraint_solver.h"
#include <rbc_defs/rbc_def_internal.h>

static char *SOLVER_MEMORY_ALLOCATOR_ERROR_MSG = 0;

inline void pulse_sum_constraint_solver::add_urb( turb_search_tree_t *turb_search_tree,
                                                  turb_list_t *list_turb,
                                                  urbri_list_t *list_urbri,
                                                  rigid_body_constraint *rbc )
{
  user_rigid_body **rbc_urb = rbcint::get_urb( rbc );
  if ( rbc_urb )
  {
    temp_user_rigid_body *turb = turb_search_tree->find( *rbc_urb );
    if ( !turb )
    {
      int a = __alignof( temp_user_rigid_body );
      int b = PHYS_MEM_MIN_ALIGNMENT;
      turb = (temp_user_rigid_body *)
                 m_solver_memory_allocator.allocate( sizeof( temp_user_rigid_body ), tl_max( a, b ), 0, SOLVER_MEMORY_ALLOCATOR_ERROR_MSG );
      if ( turb )
      {
        temp_user_rigid_body( turb );
      }
      turb->set( *rbc_urb );
      turb_search_tree->add( *rbc_urb, turb );
      list_turb->add( turb );
    }

    int a = __alignof( user_rigid_body_restore_info );
    int b = PHYS_MEM_MIN_ALIGNMENT;
    user_rigid_body_restore_info *urbri = (user_rigid_body_restore_info *)m_solver_memory_allocator.allocate(
        sizeof( user_rigid_body_restore_info ),
        tl_max( a, b ),
        0,
        SOLVER_MEMORY_ALLOCATOR_ERROR_MSG );

    urbri->set( rbc_urb, turb );
    list_urbri->add( urbri );
  }
}

inline void pulse_sum_constraint_solver::set_solver_params( const float outside_delta_t,
                                                            const int psys_max_vel_iters,
                                                            const int psys_max_vel_pos_iters )
{
  m_outside_delta_t = outside_delta_t;
  m_psys_max_vel_iters = psys_max_vel_iters;
  m_psys_max_vel_pos_iters = psys_max_vel_pos_iters;
  m_memory_high_water = 0;
}

inline void pulse_sum_constraint_solver::solve_iterative( const int max_iters, const float max_error_sq )
{
  float error_sq = max_error_sq * 2.0f;
  int iters = 0;
  while ( iters <= max_iters && max_error_sq < error_sq )
  {
    ++iters;

    for ( phys_link_list<pulse_sum_normal>::iterator i = m_list_pulse_sum_normal.begin(); i != m_list_pulse_sum_normal.end(); ++i )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq, true );
    }

    for ( phys_link_list<pulse_sum_point>::iterator i = m_list_pulse_sum_point.begin(); i != m_list_pulse_sum_point.end(); ++i )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq );
    }

    for ( phys_link_list<pulse_sum_angular>::iterator i = m_list_pulse_sum_angular.begin(); i != m_list_pulse_sum_angular.end(); ++i )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq );
    }

    for ( phys_link_list<pulse_sum_wheel>::iterator i = m_list_pulse_sum_wheel.begin(); i != m_list_pulse_sum_wheel.end(); ++i )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq );
    }

    for ( phys_link_list<pulse_sum_contact>::iterator i = m_list_pulse_sum_contact.begin(); i != m_list_pulse_sum_contact.end(); ++i )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq );
    }
  }
}

inline void pulse_sum_constraint_solver::solve_constraints( rigid_body *const head )
{
  for ( list_pulse_sum_node::iterator i = m_list_pulse_sum_node.begin(); i != m_list_pulse_sum_node.end(); ++i )
  {
    pulse_sum_node &psn = *i;
    rbint::euler_integrate_velocity( psn.m_rb, m_si.m_delta_t );
    psn.t_vel = PHYS_ZERO_VEC;
    psn.a_vel = PHYS_ZERO_VEC;
  }

  for ( list_pulse_sum_normal::iterator i = m_list_pulse_sum_normal.begin(); i != m_list_pulse_sum_normal.end(); ++i )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  for ( list_pulse_sum_point::iterator i = m_list_pulse_sum_point.begin(); i != m_list_pulse_sum_point.end(); ++i )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  for ( list_pulse_sum_angular::iterator i = m_list_pulse_sum_angular.begin(); i != m_list_pulse_sum_angular.end(); ++i )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  for ( list_pulse_sum_wheel::iterator i = m_list_pulse_sum_wheel.begin(); i != m_list_pulse_sum_wheel.end(); ++i )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  for ( list_pulse_sum_contact::iterator i = m_list_pulse_sum_contact.begin(); i != m_list_pulse_sum_contact.end(); ++i )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  solve_iterative( m_si.m_max_vel_iters, m_si.m_max_vel_error_sq );
  for ( rigid_body_constraint_point *rbc = head->m_partition_node.m_rbc_point_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->zero_pulse_sums();
  }

  for ( rigid_body_constraint_hinge *rbc = head->m_partition_node.m_rbc_hinge_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->zero_pulse_sums();
  }
}

inline void pulse_sum_constraint_solver::execute_constraint_solver( rigid_body * head )
{
  tlAssert( head )
  float max_delta_t = head->get_max_delta_t();
  for ( rigid_body *n = head->m_partition_node.m_next_node; n; n = n->m_partition_node.m_next_node )
  {
    if ( max_delta_t > n->get_max_delta_t() )
    {
      max_delta_t = n->get_max_delta_t();
    }
  }
  int sub_steps = m_outside_delta_t / max_delta_t;
  if ( sub_steps * max_delta_t < m_outside_delta_t )
  {
    ++sub_steps;
  }
  tlAssert( m_outside_delta_t <= max_delta_t * sub_steps );
  const float delta_t = m_outside_delta_t;
  turb_search_tree_t turb_search_tree;
  turb_list_t list_turb;
  urbri_list_t list_urbri;
  tlAssert( m_solver_memory_allocator.is_empty() )
  for ( rigid_body *n = head; n; n = n->m_partition_node.m_next_node )
  {
    rbint::solver_prolog( n, delta_t );
    n->swap_last_position();
  }
  for ( rigid_body_constraint_point *rbc = head->m_partition_node.m_rbc_point_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_hinge *rbc = head->m_partition_node.m_rbc_hinge_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_distance *rbc = head->m_partition_node.m_rbc_dist_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_ragdoll *rbc = head->m_partition_node.m_rbc_ragdoll_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_wheel *rbc = head->m_partition_node.m_rbc_wheel_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_angular_actuator *rbc = head->m_partition_node.m_rbc_angular_actuator_first; rbc;
        rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_upright *rbc = head->m_partition_node.m_rbc_upright_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_custom_orientation *rbc = head->m_partition_node.m_rbc_custom_orientation_first; rbc;
        rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_custom_path *rbc = head->m_partition_node.m_rbc_custom_path_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  for ( rigid_body_constraint_contact *rbc = head->m_partition_node.m_rbc_contact_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    add_urb( &turb_search_tree, &list_turb, &list_urbri, rbc );
  }
  phys_transient_allocator::allocator_state saved_allocator_state = m_solver_memory_allocator.capture_state();
  m_si.m_max_vel_iters = tl_max( m_psys_max_vel_iters / sub_steps, 1 );
  m_si.m_max_vel_pos_iters = tl_max( m_psys_max_vel_pos_iters / sub_steps, 1 );
  m_si.m_max_vel_error_sq = phys_sqr( 0.34f );
  m_si.m_max_vel_pos_error_sq = phys_sqr( 1.7f );
  m_si.m_delta_t = delta_t / sub_steps;
  for ( int step = 0; step < sub_steps; ++step )
  {
    m_solver_memory_allocator.reset_to_state( saved_allocator_state );
    m_list_pulse_sum_node.remove_all();
    m_list_pulse_sum_normal.remove_all();
    m_list_pulse_sum_point.remove_all();
    m_list_pulse_sum_angular.remove_all();
    m_list_pulse_sum_wheel.remove_all();
    m_list_pulse_sum_contact.remove_all();
    for ( rigid_body *n = head; n; n = n->m_partition_node.m_next_node )
    {
      rbint::setup_constraint( n, create_pulse_sum_node() );
    }
    for ( rigid_body_constraint_point *rbc = head->m_partition_node.m_rbc_point_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_hinge *rbc = head->m_partition_node.m_rbc_hinge_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_distance *rbc = head->m_partition_node.m_rbc_dist_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_ragdoll *rbc = head->m_partition_node.m_rbc_ragdoll_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_wheel *rbc = head->m_partition_node.m_rbc_wheel_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_angular_actuator *rbc = head->m_partition_node.m_rbc_angular_actuator_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_upright *rbc = head->m_partition_node.m_rbc_upright_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_custom_orientation *rbc = head->m_partition_node.m_rbc_custom_orientation_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_custom_path *rbc = head->m_partition_node.m_rbc_custom_path_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    for ( rigid_body_constraint_contact *rbc = head->m_partition_node.m_rbc_contact_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->setup_constraint( this, m_si.m_delta_t );
    }
    solve_constraints( head );
    turb_list_t::iterator turb = list_turb.begin();
    turb_list_t::iterator turb_end = list_turb.end();
    while ( turb != turb_end )
    {
      rbint::substep( &(*turb), m_si.m_delta_t );
      ++turb;
    }
    for ( rigid_body_constraint_point *rbc = head->m_partition_node.m_rbc_point_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_hinge *rbc = head->m_partition_node.m_rbc_hinge_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_distance *rbc = head->m_partition_node.m_rbc_dist_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_ragdoll *rbc = head->m_partition_node.m_rbc_ragdoll_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_wheel *rbc = head->m_partition_node.m_rbc_wheel_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_angular_actuator *rbc = head->m_partition_node.m_rbc_angular_actuator_first; rbc;
          rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_upright *rbc = head->m_partition_node.m_rbc_upright_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_custom_orientation *rbc = head->m_partition_node.m_rbc_custom_orientation_first; rbc;
          rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_custom_path *rbc = head->m_partition_node.m_rbc_custom_path_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    for ( rigid_body_constraint_contact *rbc = head->m_partition_node.m_rbc_contact_first; rbc; rbc = rbcint::get_next( rbc ) )
    {
      rbc->inner_update( m_si.m_delta_t );
    }
    m_memory_high_water = tl_max(m_memory_high_water, m_solver_memory_allocator.get_current_alloc_size());
  }
  urbri_list_t::iterator urbri = list_urbri.begin();
  urbri_list_t::iterator urbri_end = list_urbri.end();
  while ( urbri != urbri_end )
  {
    ( *urbri ).restore();
    ++urbri;
  }
  for ( rigid_body_constraint_point *rbc = head->m_partition_node.m_rbc_point_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_hinge *rbc = head->m_partition_node.m_rbc_hinge_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_distance *rbc = head->m_partition_node.m_rbc_dist_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_ragdoll *rbc = head->m_partition_node.m_rbc_ragdoll_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_wheel *rbc = head->m_partition_node.m_rbc_wheel_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_angular_actuator *rbc = head->m_partition_node.m_rbc_angular_actuator_first; rbc;
        rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_upright *rbc = head->m_partition_node.m_rbc_upright_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_custom_orientation *rbc = head->m_partition_node.m_rbc_custom_orientation_first; rbc;
        rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_custom_path *rbc = head->m_partition_node.m_rbc_custom_path_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body_constraint_contact *rbc = head->m_partition_node.m_rbc_contact_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->epilog_vel_constraint( m_si.m_delta_t );
  }
  for ( rigid_body *n = head; n; n = n->m_partition_node.m_next_node )
  {
    rbint::solver_epilog( n, delta_t );
    n->update_last_position();
  }
  m_solver_memory_allocator.reset();
}

inline void pulse_sum_constraint_solver::set_pulse_sum( pulse_sum_cache * const psc, const float ps)
{
  psc->set_pulse_sum( ps );
}

inline const float pulse_sum_constraint_solver::get_pulse_sum( const pulse_sum_cache * psc ) const
{
  return psc->get_pulse_sum();
}