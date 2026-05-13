#pragma once

#include "pulse_sum_point.h"
#include "pulse_sum_base.h"
#include "rigid_body.h"
#include "rigid_body_internal.h"

inline void pulse_sum_point::calc_abs_and_setup( const float delta_t, const bool is_spring, const float spring_k, const float damp_k )
{
  phys_vec3 b1_inv_mass( m_b1->m_inv_mass );
  phys_mat44 &b1_inv_inertia = m_b1->m_world_inv_inertia;

  m_b1_apx = phys_multiply( b1_inv_inertia, phys_cross( m_b1_r, PHYS_X_VEC ) );
  phys_vec3 zx = phys_cross( m_b1_apx, m_b1_r );
  zx[0] += b1_inv_mass.GetX();

  m_b1_apy = phys_multiply( b1_inv_inertia, phys_cross( m_b1_r, PHYS_Y_VEC ) );
  phys_vec3 zy = phys_cross( m_b1_apy, m_b1_r );
  zy[1] += b1_inv_mass.GetY();

  m_b1_apz = phys_multiply( b1_inv_inertia, phys_cross( m_b1_r, PHYS_Z_VEC ) );
  phys_vec3 zz = phys_cross( m_b1_apz, m_b1_r );
  zz[2] += b1_inv_mass.GetZ();

  if ( m_b2 )
  {
    phys_vec3 b2_inv_mass( m_b2->m_inv_mass );
    phys_mat44 &b2_inv_inertia = m_b2->m_world_inv_inertia;

    m_b2_apx = phys_multiply( b2_inv_inertia, phys_cross( m_b2_r, PHYS_X_VEC ) );
    phys_vec3 v = phys_cross( m_b2_apx, m_b2_r );
    zx += v;
    zx[0] += b2_inv_mass.GetX();

    m_b2_apy = phys_multiply( b2_inv_inertia, phys_cross( m_b2_r, PHYS_Y_VEC ) );
    v = phys_cross( m_b2_apy, m_b2_r );
    zy += v;
    zy[1] += b2_inv_mass.GetY();

    m_b2_apz = phys_multiply( b2_inv_inertia, phys_cross( m_b2_r, PHYS_Z_VEC ) );
    v = phys_cross( m_b2_apz, m_b2_r );
    zz += v;
    zz[2] += b2_inv_mass.GetZ();
  }
  if ( is_spring )
  {
    phys_vec3 pos = get_pos();
    float erp;
    pulse_sum_calc_spring_params( spring_k, damp_k, delta_t, &erp, &m_cfm );
    zx[0] += m_cfm;
    zy[1] += m_cfm;
    zz[2] += m_cfm;
    m_right_side = ( -erp / delta_t ) * pos;
    m_big_dirt = PHYS_ZERO_VEC;
  }
  else
  {
    m_big_dirt = ( -0.5 / tl_max( delta_t, MIN_NERP_DELTA_T ) ) * get_pos();
    m_right_side = PHYS_ZERO_VEC;
    m_cfm = 0;
  }

  m_cr23 = phys_cross( zy, zz );
  m_cr31 = phys_cross( zz, zx );
  m_cr12 = phys_cross( zx, zy );
  m_denom = phys_vec3( zx.GetX(), zy.GetY(), zz.GetZ() );
  float oo_delt = 1.0f / phys_dot( m_cr12, zz );
  m_cr23 *= oo_delt;
  m_cr31 *= oo_delt;
  m_cr12 *= oo_delt;
}

inline const phys_vec3 pulse_sum_point::get_vel()
{
  const phys_vec3 &t_vel = m_b1->m_rb->get_t_vel();
  const phys_vec3 &a_vel = m_b1->m_rb->get_a_vel();

  phys_vec3 _a = a_vel;
  phys_vec3 _b = phys_cross( _a, m_b1_r );
  phys_vec3 _c = t_vel + _b;

  phys_vec3 _a2;
  if ( m_b2 )
  {
    _a2 = m_b2->m_rb->get_t_vel() + phys_cross( m_b2->m_rb->get_a_vel(), m_b2_r );
  }
  else
  {
    _a2 = object_vel_();
  }

  return _c - _a2;
}

inline const phys_vec3 pulse_sum_point::get_pos()
{
  // Get position of body 1 in world space
  const phys_mat44 &body1_mat = m_b1->m_rb->get_mat();
  phys_vec3 body1_world_pos = body1_mat.GetW();

  // Add local offset to get the contact point on body 1
  phys_vec3 point1 = body1_world_pos + m_b1_r;

  // Determine the second point (either on body 2 or from object_col_pt)
  phys_vec3 point2;
  if ( m_b2 )
  {
    // Get position of body 2 in world space
    const phys_mat44 &body2_mat = m_b2->m_rb->get_mat();
    phys_vec3 body2_world_pos = body2_mat.GetW();

    // Add local offset to get the contact point on body 2
    point2 = body2_world_pos + m_b2_r;
  }
  else
  {
    // Use the provided collision point
    point2 = object_col_pt_();
  }

  // Return the offset vector between the points
  return point1 - point2;
}

inline const phys_vec3 pulse_sum_point::get_objective()
{
  phys_vec3 body1_ang_vel = m_b1->a_vel;
  phys_vec3 body1_r = m_b1_r;
  phys_vec3 angular_component = phys_cross( body1_ang_vel, body1_r );

  // Total velocity at contact point = linear + angular contribution
  phys_vec3 contact_point_vel = m_b1->t_vel + angular_component;

  // If there's a second body, calculate relative velocity
  if ( m_b2 )
  {
    // Calculate body 2's velocity at its contact point
    phys_vec3 body2_ang_vel = m_b2->a_vel;
    phys_vec3 body2_r = m_b2_r;
    phys_vec3 body2_angular_component = phys_cross( body2_ang_vel, body2_r );
    phys_vec3 body2_contact_vel = m_b2->t_vel + body2_angular_component;

    // Relative velocity = body1_vel - body2_vel
    contact_point_vel -= body2_contact_vel;
  }

  return contact_point_vel;
}

inline void pulse_sum_point::apply( phys_vec3 &s_ )
{
  phys_vec3 _v = m_b1->m_inv_mass * s_;
  m_b1->t_vel += _v;

  phys_vec3 _x = s_.GetX() * m_b1_apx;
  phys_vec3 _y = s_.GetY() * m_b1_apy;
  phys_vec3 _xy = _x + _y;
  phys_vec3 _z = s_.GetZ() * m_b1_apz;
  phys_vec3 _xyz = _xy + _z;
  m_b1->a_vel += _xyz;

  if ( m_b2 )
  {
    _v = m_b2->m_inv_mass * s_;
    m_b2->t_vel += _v;

    _x = s_.GetX() * m_b2_apx;
    _y = s_.GetY() * m_b2_apy;
    _xy = _x + _y;
    _z = s_.GetZ() * m_b2_apz;
    _xyz = _xy + _z;
    m_b2->a_vel -= _xyz;
  }
}

inline void pulse_sum_point::project()
{
  apply( m_pulse_sum );
}

inline phys_vec3 &pulse_sum_point::object_vel_()
{
  return m_b2_apx;
}

inline phys_vec3 &pulse_sum_point::object_col_pt_()
{
  return m_b2_r;
}

inline void pulse_sum_point::set_object_vel( phys_vec3 &object_vel )
{
  tlAssert( m_b2 == NULL );
  m_b2_apx = object_vel;
}

inline void pulse_sum_point::set_object_col_pt( phys_vec3 &object_col_p )
{
  tlAssert( m_b2 == NULL );
  m_b2_r = object_col_p;
}

inline const phys_vec3 pulse_sum_point::phys_diag_multiply_and_square( phys_vec3 &v1, phys_vec3 &v2 )
{
  phys_vec3 temp( v1.GetX() * v2.GetX(), v1.GetY() * v2.GetY(), v1.GetZ() * v2.GetZ() );
  return phys_vec3( temp.GetX() * temp.GetX(), temp.GetY() * temp.GetY(), temp.GetZ() * temp.GetZ() );
}

inline void pulse_sum_point::SOLVER_apply_relaxation( float &error_sq )
{
  phys_vec3 _a = get_objective();
  phys_vec3 _b = m_cfm * m_pulse_sum;
  phys_vec3 m_objective = ( _a + _b ) - m_right_side;
  phys_vec3 m_last_pulse_sum = m_pulse_sum;

  phys_vec3 _v = ( ( m_objective.GetX() * m_cr23 ) + ( m_objective.GetY() * m_cr31 ) + ( m_objective.GetZ() * m_cr12 ) );
  m_pulse_sum -= _v;
  phys_vec3 delta_pulse_sum = m_pulse_sum - m_last_pulse_sum;
  apply( delta_pulse_sum );
  phys_vec3 error_ = phys_diag_multiply_and_square( delta_pulse_sum, m_denom );
  if ( error_.GetX() > error_sq )
  {
    error_sq = error_.GetX();
  }
  if ( error_.GetY() > error_sq )
  {
    error_sq = error_.GetY();
  }
  if ( error_.GetZ() > error_sq )
  {
    error_sq = error_.GetZ();
  }
}

inline void pulse_sum_point::SOLVER_solver_prolog( const float delta_t )
{
  phys_vec3 vel = get_vel();
  m_right_side -= vel;
  m_pulse_sum[0] = delta_t * m_pulse_sum_cache[0].get_pulse_sum();
  m_pulse_sum[1] = delta_t * m_pulse_sum_cache[1].get_pulse_sum();
  m_pulse_sum[2] = delta_t * m_pulse_sum_cache[2].get_pulse_sum();
  project();
}

inline void pulse_sum_point::SOLVER_solver_intermediate( const float delta_t )
{
  pulse_sum_cache *psc = m_pulse_sum_cache;
  psc[0].set_pulse_sum( m_pulse_sum.GetX() / delta_t );
  psc[1].set_pulse_sum( m_pulse_sum.GetY() / delta_t );
  psc[2].set_pulse_sum( m_pulse_sum.GetZ() / delta_t );
  m_right_side += m_big_dirt;
}

inline void pulse_sum_point::set( rigid_body *const b1,
                           phys_vec3 &b1_r,
                           rigid_body *const b2,
                           phys_vec3 &b2_r,
                           pulse_sum_cache *const ps_cache,
                           const float delta_t,
                           const bool is_spring,
                           const float spring_k,
                           const float damp_k )
{
  tlAssert( ps_cache );
  tlAssert( b1 );
  tlAssert( b2 );
  tlAssert( b1 != b2 );
  tlAssert( rbint::get_pulse_sum_node( b1 ) || rbint::get_pulse_sum_node( b2 ) );
  tlAssert( rbint::verify_pulse_sum_node( b1 ) && rbint::verify_pulse_sum_node( b2 ) );
  tlAssert( rbint::get_pulse_sum_node( b1 ) != NULL );

  m_b1 = rbint::get_pulse_sum_node( b1 );
  m_b1_r = b1_r;

  if ( rbint::get_pulse_sum_node( b2 ) )
  {
    m_b2 = rbint::get_pulse_sum_node( b2 );
    m_b2_r = b2_r;
  }
  else
  {
    m_b2 = NULL;
    phys_vec3 object_vel = rbint::gtv( b2, b2_r );
    set_object_vel( object_vel );
    phys_vec3 object_col_pt = rbint::add_pos( b2, b2_r );
    set_object_col_pt( object_col_pt );
  }
  m_pulse_sum_cache = ps_cache;
  calc_abs_and_setup( delta_t, is_spring, spring_k, damp_k );
}

inline pulse_sum_point::pulse_sum_point() {}
