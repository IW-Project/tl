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
    contact_point_info& cpi = *cpi_i;
    cpi.check_surface_properties();
    psys->create_pulse_sum_contact( b1, b2, &cpi, delta_t );
    ++cpi_i;
  }
}

phys_transient_allocator *contact_point_info::get_cpi_allocator()
{
  return &g_physics_system->m_contact_point_buffer_1;
}