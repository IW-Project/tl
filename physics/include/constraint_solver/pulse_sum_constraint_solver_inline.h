#pragma once

#include "pulse_sum_constraint_solver.h"
#include <rbc_defs/rbc_def_internal.h>

char *SOLVER_MEMORY_ALLOCATOR_ERROR_MSG = 0;

void pulse_sum_constraint_solver::add_urb( turb_search_tree_t *turb_search_tree,
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

void pulse_sum_constraint_solver::set_solver_params( const float outside_delta_t,
                                                     const int psys_max_vel_iters,
                                                     const int psys_max_vel_pos_iters )
{
  m_outside_delta_t = outside_delta_t;
  m_psys_max_vel_iters = psys_max_vel_iters;
  m_psys_max_vel_pos_iters = psys_max_vel_pos_iters;
  m_memory_high_water = 0;
}

void pulse_sum_constraint_solver::solve_iterative( const int max_iters, const float max_error_sq )
{
  float error_sq = max_error_sq * 2.0f;
  int iters = 0;
  while ( iters <= max_iters && max_error_sq < error_sq )
  {
    ++iters;

    for ( phys_link_list<pulse_sum_normal>::iterator i = m_list_pulse_sum_normal.begin(); i != m_list_pulse_sum_normal.end(); i++ )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq, true );
    }

    for ( phys_link_list<pulse_sum_point>::iterator i = m_list_pulse_sum_point.begin(); i != m_list_pulse_sum_point.end(); i++ )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq );
    }

    for ( phys_link_list<pulse_sum_angular>::iterator i = m_list_pulse_sum_angular.begin(); i != m_list_pulse_sum_angular.end(); i++ )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq );
    }

    for ( phys_link_list<pulse_sum_wheel>::iterator i = m_list_pulse_sum_wheel.begin(); i != m_list_pulse_sum_wheel.end(); i++ )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq );
    }

    for ( phys_link_list<pulse_sum_contact>::iterator i = m_list_pulse_sum_contact.begin(); i != m_list_pulse_sum_contact.end(); i++ )
    {
      ( *i ).SOLVER_apply_relaxation( error_sq );
    }
  }
}

void pulse_sum_constraint_solver::solve_constraints( rigid_body *const head )
{
  for ( list_pulse_sum_node::iterator i = m_list_pulse_sum_node.begin(); i != m_list_pulse_sum_node.end(); i++ )
  {
    pulse_sum_node& psn = *i;
    rbint::euler_integrate_velocity( psn.m_rb, m_si.m_delta_t );
    psn.t_vel = PHYS_ZERO_VEC;
    psn.a_vel = PHYS_ZERO_VEC;
  }

  for ( list_pulse_sum_normal::iterator i = m_list_pulse_sum_normal.begin(); i != m_list_pulse_sum_normal.end(); i++ )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  for ( list_pulse_sum_point::iterator i = m_list_pulse_sum_point.begin(); i != m_list_pulse_sum_point.end(); i++ )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  for ( list_pulse_sum_angular::iterator i = m_list_pulse_sum_angular.begin(); i != m_list_pulse_sum_angular.end(); i++ )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  for ( list_pulse_sum_wheel::iterator i = m_list_pulse_sum_wheel.begin(); i != m_list_pulse_sum_wheel.end(); i++ )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  for ( list_pulse_sum_contact::iterator i = m_list_pulse_sum_contact.begin(); i != m_list_pulse_sum_contact.end(); i++ )
  {
    ( *i ).SOLVER_solver_prolog( m_si.m_delta_t );
  }

  solve_iterative( m_si.m_max_vel_iters, m_si.m_max_vel_error_sq );
  for ( rigid_body_constraint_point* rbc = head->m_partition_node.m_rbc_point_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->zero_pulse_sums();
  }

  for ( rigid_body_constraint_hinge* rbc = head->m_partition_node.m_rbc_hinge_first; rbc; rbc = rbcint::get_next( rbc ) )
  {
    rbc->zero_pulse_sums();
  }
}