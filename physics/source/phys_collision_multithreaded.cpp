#include "collision/phys_collision.h"

#include "collision/phys_broad_phase.h"
#include "physics_system.h"
#include "phys_task_manager.h"
#include "jobqueue.h"

phys_collision_pair *g_list_pcp_iterator;
contact_point_info *g_list_output_cpi;

struct narrow_phase_collision_task_input
{
  contact_point_info **m_list_output_cpi;
  rigid_body_constraint_contact *m_rbc_contact_search_tree_root;
  phys_collision_pair **m_list_pcp_iterator;
};

static int phys_gjk_collide_jq_batch_function( jqBatch *pBatch )
{
  narrow_phase_collision_task_input *input = (narrow_phase_collision_task_input *)pBatch->Input;
  phys_contact_manifold_process cman_process;
  phys_gjk_info gjk_info;
  phys_transient_allocator *cpi_allocator = contact_point_info::get_cpi_allocator();
  cman_process.setup_misc_params( cpi_allocator, input->m_rbc_contact_search_tree_root );
  for ( phys_collision_pair *pcp = *input->m_list_pcp_iterator; pcp; pcp = *input->m_list_pcp_iterator )
  {
    if ( tlAtomicCompareAndSwap( (volatile u32 *)input->m_list_pcp_iterator, (u32)pcp->m_next_link, (u32)pcp ) )
    {
      phys_collide_do_gjk_collide_and_contact_manifold( pcp, &gjk_info, &cman_process );
    }
  }

  contact_point_info *list_cpi_first = cman_process.m_list_cpi.get_first();
  if ( list_cpi_first )
    cman_process.m_list_cpi.atomic_prepend_to( input->m_list_output_cpi );
  return 0;
}

static void process_cpi( contact_point_info *cpi )
{
  while ( cpi )
  {
    contact_point_info *next_cpi = cpi->get_next_link();
    tlAssert( cpi->m_pcp );
    rigid_body *rb1 = cpi->m_pcp->m_bpi1->get_rb();
    rigid_body *rb2 = cpi->m_pcp->m_bpi2->get_rb();
    rigid_body_constraint_contact *rbc =
        cpi->m_rbc_contact ? cpi->m_rbc_contact :
                             phys_sys::create_rbc_contact( rb1, rb2, cpi->get_flag( contact_point_info::FLAG_NO_OVERFLOW_ERROR ) );

    if ( rbc )
    {
      rbc->set_solver_priority( cpi->get_solver_priority() );
      rbc->add_cpi_simple( cpi, rb1, rb2 );
    }
    cpi = next_cpi;
  }
}

jqModule phys_gjk_collide_jqModule( "phys_gjk_collide_jq", JQ_WORKER_GENERIC, &phys_gjk_collide_jq_batch_function, jqBatchGroup() );

void process_list_do_gjk_collide_and_contact_manifold( phys_link_list<phys_collision_pair> *list_pcd )
{
  if ( list_pcd->get_count() > 0 )
  {
    g_list_pcp_iterator = list_pcd->get_first();
    g_list_output_cpi = 0;
    phys_transient_allocator *cpi_allocator = contact_point_info::get_cpi_allocator();
    TRANSIENT_ALLOCATE( input, ( *cpi_allocator ), 1, narrow_phase_collision_task_input );
    input->m_list_pcp_iterator = &g_list_pcp_iterator;
    input->m_rbc_contact_search_tree_root = g_physics_system->m_search_tree_rbc_contact.get_root();
    input->m_list_output_cpi = &g_list_output_cpi;
    phys_task_manager_process( &phys_gjk_collide_jqModule, input, list_pcd->get_count() );
    phys_task_manager_flush();
    process_cpi( g_list_output_cpi );
  }
}