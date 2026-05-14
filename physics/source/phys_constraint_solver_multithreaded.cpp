#include "constraint_solver/pulse_sum_normal.h"

#include "rigid_body.h"
#include "physics_system_internal.h"
#include "constraint_solver/pulse_sum_constraint_solver.h"
#include <jobqueue.h>
#include <phys_task_manager.h>

struct constraint_solver_task_input
{
  rigid_body **m_list_island;
  int m_list_island_count;
  int *m_list_island_cur;
  int m_psys_psc_visit_counter;
  int m_psys_next_psc_visit_counter;
  int m_psys_max_vel_iters;
  int m_psys_max_vel_pos_iters;
  float m_outside_delta_t;
};

int phys_jq_constraint_solver_batch_function( jqBatch *pBatch )
{
  constraint_solver_task_input *csti = (constraint_solver_task_input *)pBatch->Input;
  pulse_sum_constraint_solver cs;
  cs.set_solver_params( csti->m_outside_delta_t, csti->m_psys_max_vel_iters, csti->m_psys_max_vel_pos_iters );
  for ( int rb_i = tlAtomicIncrement( (volatile u32 *)&csti->m_list_island_cur ) - 1; rb_i < csti->m_list_island_count;
        rb_i = tlAtomicIncrement( (volatile u32 *)&csti->m_list_island_cur ) - 1 )
  {
    cs.execute_constraint_solver( csti->m_list_island[rb_i] );
  }
  return 0;
}

jqModule phys_jq_constraint_solverModule( "phys_jq_constraint_solver",
                                          JQ_WORKER_GENERIC,
                                          &phys_jq_constraint_solver_batch_function,
                                          jqBatchGroup() );

void constraint_solver_process( phys_transient_allocator *transient_buffer, physics_system *psys, const float outside_delta_t )
{
  // Check if there are islands to process
  if ( psys->m_list_island_count > 0 )
  {
    extern jqModule phys_jq_constraint_solverModule;

    int align = tl_max( (int)sizeof( unsigned long ), 4 );
    constraint_solver_task_input *params = (constraint_solver_task_input *)transient_buffer->allocate(
        sizeof( constraint_solver_task_input ),
        PHYS_ALIGNOF( constraint_solver_task_input ),
        0,
        "phys_transient_allocator out of memory." );

    if ( params )
    {
      params->m_list_island = psys->m_list_island;
      params->m_list_island_count = psys->m_list_island_count;
      params->m_list_island_cur = &g_list_island_cur;
      params->m_psys_max_vel_iters = psys->m_max_vel_iters;
      params->m_outside_delta_t = outside_delta_t;
      params->m_psys_max_vel_pos_iters = psys->m_max_vel_pos_iters;

      phys_task_manager_process( &phys_jq_constraint_solverModule, params, psys->m_list_island_count );
    }
  }

  // Flush task manager if needed
  if ( phys_task_manager_needs_flush() )
  {
    phys_task_manager_flush();
  }
}

const phys_vec3 pulse_sum_normal::get_relative_velocity_change_dir()
{
  phys_vec3 vc_dir = ( m_b1->m_inv_mass * m_ud ) + phys_cross( m_b1_ap, m_ud );
  if ( m_b2 )
  {
    vc_dir += ( m_b2->m_inv_mass * m_ud ) + phys_cross( m_b2_ap, m_ud );
  }

  return vc_dir;
}

const phys_vec3 pulse_sum_normal::get_relative_velocity()
{
  phys_vec3 v = m_b1->m_rb->get_t_vel() + phys_cross( m_b1->m_rb->get_a_vel(), m_b1_r );
  if ( m_b2 )
  {
    v -= ( m_b2->m_rb->get_t_vel() + phys_cross( m_b2->m_rb->get_a_vel(), m_b2_r ) );
  }
  else
  {
    v -= object_vel_();
  }
  return v;
}