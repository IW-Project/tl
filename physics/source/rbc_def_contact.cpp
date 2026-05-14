#include "rbc_defs/rbc_def_contact.h"

#include "constraint_solver/pulse_sum_constraint_solver.h"
#include "physics_system_internal.h"

void rigid_body_constraint_contact::verify_constraint( rigid_body *const b1_, rigid_body *const b2_ )
{
  tlAssert( ( b1 == b1_ && b2 == b2_ ) || ( b1 == b2_ && b2 == b1_ ) );
  verify_is_in_physics_system( this, b1_, b2_ );
}

rigid_body_constraint_contact::rigid_body_constraint_contact()
  : m_solver_priority( 0 )
{
}

rigid_body_constraint_contact::~rigid_body_constraint_contact() {}

void rigid_body_constraint_contact::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  m_list_contact_point_info_buffer_2.remove_all();
  phys_simple_link_list<contact_point_info>::iterator cpi_i = m_list_contact_point_info_buffer_1.begin();
  phys_simple_link_list<contact_point_info>::iterator cpi_end = m_list_contact_point_info_buffer_1.end();
  while ( cpi_i != cpi_end )
  {
    contact_point_info &cpi = *cpi_i;
    cpi.check_surface_properties();
    psys->create_pulse_sum_contact( b1, b2, &cpi, delta_t );
    ++cpi_i;
  }
}

phys_transient_allocator *contact_point_info::get_cpi_allocator()
{
  return &g_physics_system->m_contact_point_buffer_1;
}

pulse_sum_contact *
pulse_sum_constraint_solver::create_pulse_sum_contact( rigid_body *b1, rigid_body *b2, contact_point_info *cpi, const float delta_t )
{
  // manually did this instead of the macro
  size_t offset = PHYS_ALIGN( sizeof( pulse_sum_contact ), tl_max( 16, PHYS_MEM_MIN_ALIGNMENT ) );
  size_t size = sizeof( pulse_sum_contact ) * cpi->m_point_pair_count + offset;
  pulse_sum_contact *psc = (pulse_sum_contact *)m_solver_memory_allocator.allocate( size, tl_max( 16, PHYS_MEM_MIN_ALIGNMENT ), 1 );
  if ( psc )
  {
    new ( psc ) pulse_sum_contact();
    m_list_pulse_sum_contact.add( psc );
    psc->m_list_pscp = (pulse_sum_contact_point *)( (char *)psc + offset );
    psc->m_list_pscp_count = cpi->m_point_pair_count;
    psc->set( b1, b2, cpi, delta_t );
  }
  PHYS_ASSERT( pai_create_pulse_sum_contact,
               psc || cpi->get_flag( contact_point_info::FLAG_NO_OVERFLOW_ERROR ),
               "psc || cpi->get_flag(contact_point_info::FLAG_NO_OVERFLOW_ERROR)" );
  return psc;
}
