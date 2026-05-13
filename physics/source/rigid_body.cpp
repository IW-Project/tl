#include "rigid_body.h"

#include "rigid_body_internal.h"
#include "physics_system_internal.h"

void rigid_body::set( const float mass,
                      const phys_vec3 &inertia,
                      const phys_mat44 &mat,
                      const phys_vec3 &t_vel,
                      const phys_vec3 &a_vel,
                      const int stable_min_contact_count )
{
  tlAssertMsg( !IS_NANF( mass ), "invalid float number" );
  tlAssertMsg( !IS_NANF( inertia.GetX() ) && !IS_NANF( inertia.GetY() ) && !IS_NANF( inertia.GetZ() ), "invalid vector" );
  tlAssertMsg( !IS_NANF( t_vel.GetX() ) && !IS_NANF( t_vel.GetY() ) && !IS_NANF( t_vel.GetZ() ), "invalid vector" );
  tlAssertMsg( !IS_NANF( a_vel.GetX() ) && !IS_NANF( a_vel.GetY() ) && !IS_NANF( a_vel.GetZ() ), "invalid vector" );

  set_mass( mass );
  set_inertia( inertia );

  m_mat = mat;
  VALIDATE_POSITION_VECTOR( m_mat.GetW(), this );

  m_last_position = m_mat.GetW();
  VALIDATE_POSITION_VECTOR( m_last_position, this );

  m_moved_vec = PHYS_ZERO_VEC;
  m_smallest_lambda = 0.0f;

  m_t_vel = t_vel;
  m_a_vel = a_vel;
  VALIDATE_POSITION_VECTOR( m_t_vel, this );
  VALIDATE_POSITION_VECTOR( m_a_vel, this );

  m_stable_min_contact_count = stable_min_contact_count;
  m_force_sum = PHYS_ZERO_VEC;
  m_torque_sum = PHYS_ZERO_VEC;
  m_last_t_vel = m_t_vel;
  m_last_a_vel = m_a_vel;
  VALIDATE_POSITION_VECTOR( m_last_t_vel, this );
  VALIDATE_POSITION_VECTOR( m_last_a_vel, this );
  m_gravity_acc_vec = phys_vec3( 0.0f, -9.8000002f, 0.0f );
  m_flags = 0;
  m_tick = 0;
  m_max_delta_t = 1.f / 29.f;
  m_max_avel = 1000.0f;
  m_stable_energy_time = 0.0f;
  m_largest_vel_sq = 1000.0f;
  m_t_drag_coef = 0.0f;
  m_a_drag_coef = 0.0f;
  m_userdata = NULL;

  if ( get_physics_system_flag( physics_system::FLAG_IN_COLLISION_CALLBACK ) )
  {
    rbint::collision_prolog( this, get_physics_system_outside_sub_delta_t() );
  }
  PHYS_ASSERT_ORTHONORMAL( m_mat );
}

void rigid_body::set_inertia( const phys_vec3 &inertia )
{
  tlAssert( !IS_NANF( inertia.GetX() ) && !IS_NANF( inertia.GetY() ) && !IS_NANF( inertia.GetZ() ) );
  tlAssert( inertia.GetX() > 0.000001f );
  tlAssert( inertia.GetY() > 0.000001f );
  tlAssert( inertia.GetZ() > 0.000001f );
  m_inv_inertia = phys_Inv( inertia );
}

void rigid_body::set_mass( const float mass )
{
  tlAssert( !IS_NANF( mass ) );
  tlAssert( mass > 0.000001f );
  m_inv_mass = 1.0f / mass;
}

void rigid_body::add_force( const phys_vec3 &force, const phys_vec3 &point, const float torque_mult )
{
  tlAssertMsg( !IS_NANF( force.GetX() ) && !IS_NANF( force.GetY() ) && !IS_NANF( force.GetZ() ), "invalid vector" );
  tlAssertMsg( !IS_NANF( point.GetX() ) && !IS_NANF( point.GetY() ) && !IS_NANF( point.GetZ() ), "invalid vector" );
  tlAssertMsg( !IS_NANF( torque_mult ), "invalid float number" );

  m_force_sum += force;
  m_torque_sum += phys_cross( point - m_mat.GetW(), force ) * torque_mult;

  VALIDATE_POSITION_VECTOR( m_force_sum, this );
  VALIDATE_POSITION_VECTOR( m_torque_sum, this );
}

void user_rigid_body::setPosition( const phys_mat44 *const dictator )
{
  m_dictator_mat = *dictator;
  m_dictator = &m_dictator_mat;
}

const phys_mat44 *user_rigid_body::get_dictator() const
{
  return m_dictator;
}

void user_rigid_body::set( const phys_mat44 *const dictator )
{
  if ( dictator )
  {
    m_mat = *dictator;
    m_dictator = dictator;
  }
  else
  {
    m_dictator = &m_mat;
  }

  VALIDATE_POSITION_VECTOR( m_mat.GetW(), this );
  VALIDATE_POSITION_VECTOR( m_last_position, this );

  m_moved_vec = PHYS_ZERO_VEC;
  m_smallest_lambda = 0.0f;
  m_flags = 0;
  m_inv_mass = 0.0f;
  m_inv_inertia = PHYS_ZERO_VEC;
  m_t_vel = PHYS_ZERO_VEC;
  m_a_vel = PHYS_ZERO_VEC;
  m_last_t_vel = PHYS_ZERO_VEC;
  m_last_a_vel = PHYS_ZERO_VEC;
  m_gravity_acc_vec = PHYS_ZERO_VEC;
  m_node = NULL;
  m_max_delta_t = 1.f / 29.f;
  m_userdata = NULL;
  set_flag( FLAG_USER_RIGID_BODY, 1 );
  if ( get_physics_system_flag( physics_system::FLAG_IN_COLLISION_CALLBACK ) )
  {
    rbint::collision_prolog( this, get_physics_system_outside_sub_delta_t() );
  }
}

void environment_rigid_body::set()
{
  m_flags = 0;
  m_inv_mass = 0.0f;
  m_inv_inertia = PHYS_ZERO_VEC;
  SetIdentity( m_mat );

  m_last_position = m_mat.GetW();
  VALIDATE_POSITION_VECTOR( m_last_position, this );

  m_moved_vec = PHYS_ZERO_VEC;
  m_smallest_lambda = 0.0f;
  m_t_vel = PHYS_ZERO_VEC;
  m_a_vel = PHYS_ZERO_VEC;
  m_last_t_vel = PHYS_ZERO_VEC;
  m_last_a_vel = PHYS_ZERO_VEC;
  m_gravity_acc_vec = PHYS_ZERO_VEC;
  m_node = NULL;
  m_max_delta_t = 1.f / 29.f;
  m_userdata = NULL;
  set_flag( FLAG_ENVIRONMENT_RIGID_BODY, 1 );
}
