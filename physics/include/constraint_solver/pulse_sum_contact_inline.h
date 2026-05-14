#pragma once

#include "pulse_sum_contact.h"
#include "rigid_body_internal.h"
#include "pulse_sum_point.h"

inline void pulse_sum_contact_point::calc_abs( pulse_sum_contact *psc )
{
  phys_vec3 b1_t_n = phys_cross( m_b1_r, psc->m_ud_n );
  m_b1_ap_n = phys_multiply( psc->m_b1->m_world_inv_inertia, b1_t_n );
  m_denom_xx = psc->m_b1->m_inv_mass + phys_dot( m_b1_ap_n, b1_t_n );

  phys_vec3 b1_t_f1 = phys_cross( m_b1_r, psc->m_ud_f1 );
  m_b1_ap_f1 = phys_multiply( psc->m_b1->m_world_inv_inertia, b1_t_f1 );
  m_denom_yy = psc->m_b1->m_inv_mass + phys_dot( m_b1_ap_f1, b1_t_f1 );

  phys_vec3 b1_t_f2 = phys_cross( m_b1_r, psc->m_ud_f2 );
  m_b1_ap_f2 = phys_multiply( psc->m_b1->m_world_inv_inertia, b1_t_f2 );
  m_denom_zz = psc->m_b1->m_inv_mass + phys_dot( m_b1_ap_f2, b1_t_f2 );

  float denom_yz = phys_dot( m_b1_ap_f2, b1_t_n );
  m_denom_xy = phys_dot( m_b1_ap_f1, b1_t_n );
  m_denom_xz = phys_dot( m_b1_ap_f2, b1_t_n );

  if ( psc->m_b2 )
  {
    phys_vec3 b2_t_n = phys_cross( m_b2_r, psc->m_ud_n );
    m_b2_ap_n = phys_multiply( psc->m_b2->m_world_inv_inertia, b2_t_n );
    m_denom_xx += psc->m_b2->m_inv_mass + phys_dot( m_b2_ap_n, b2_t_n );

    phys_vec3 b2_t_f1 = phys_cross( m_b2_r, psc->m_ud_f1 );
    m_b2_ap_f1 = phys_multiply( psc->m_b2->m_world_inv_inertia, b2_t_f1 );
    m_denom_yy += psc->m_b2->m_inv_mass + phys_dot( m_b2_ap_f1, b2_t_f1 );

    phys_vec3 b2_t_f2 = phys_cross( m_b2_r, psc->m_ud_f2 );
    m_b2_ap_f2 = phys_multiply( psc->m_b2->m_world_inv_inertia, b2_t_f2 );
    m_denom_zz += psc->m_b2->m_inv_mass + phys_dot( m_b2_ap_f2, b2_t_f2 );

    denom_yz += phys_dot( m_b2_ap_f1, b1_t_n );
    m_denom_xy += phys_dot( m_b1_ap_f1, b1_t_n );
    m_denom_xz += phys_dot( m_b1_ap_f1, b1_t_n );
  }
  float det = ( m_denom_yy * m_denom_zz ) - ( denom_yz * denom_yz );
  tlAssert( det > 0.0f );
  m_inv_yy = m_denom_zz * ( 1.0f / det );
  m_inv_yz = -denom_yz * ( 1.0f / det );
  m_inv_zz = m_denom_yy * ( 1.0f / det );
}

inline const float pulse_sum_contact_point::get_impact_vel( pulse_sum_contact *psc )
{
  phys_vec3 last_t_vel = rbint::get_last_t_vel( psc->m_b1->m_rb );
  phys_vec3 last_a_vel = rbint::get_last_a_vel( psc->m_b1->m_rb );
  phys_vec3 _a = last_a_vel;
  phys_vec3 _b = phys_cross( _a, m_b1_r );
  phys_vec3 lv = last_t_vel + _b;
  phys_vec3 lv2;
  if ( psc->m_b2 )
  {
    last_t_vel = rbint::get_last_t_vel( psc->m_b2->m_rb );
    last_a_vel = rbint::get_last_a_vel( psc->m_b2->m_rb );
    _a = last_a_vel;
    _b = phys_cross( _a, m_b2_r );
    lv2 = last_t_vel + _b;
  }
  else
  {
    lv2 = object_vel_();
  }
  lv = lv - lv2;
  phys_vec3 t_vel = psc->m_b1->m_rb->get_t_vel();
  phys_vec3 a_vel = psc->m_b1->m_rb->get_a_vel();
  phys_vec3 v = t_vel + phys_cross( a_vel, m_b1_r );
  phys_vec3 v2;
  if ( psc->m_b2 )
  {
    t_vel = psc->m_b2->m_rb->get_t_vel();
    a_vel = psc->m_b2->m_rb->get_a_vel();
    v2 = t_vel + phys_cross( a_vel, m_b2_r );
  }
  else
  {
    v2 = object_vel_();
  }
  v = v - v2;

  float a = phys_dot( lv, psc->m_ud_n );
  float b = phys_dot( v, psc->m_ud_n );

  return tl_max( a, b );
}

inline const phys_vec3 pulse_sum_contact_point::get_vel( pulse_sum_contact *psc )
{
  phys_vec3 last_t_vel = rbint::get_last_t_vel( psc->m_b1->m_rb );
  phys_vec3 last_a_vel = rbint::get_last_a_vel( psc->m_b1->m_rb );
  phys_vec3 _a = last_a_vel;
  phys_vec3 _b = phys_cross( _a, m_b1_r );
  phys_vec3 lv = last_t_vel + _b;
  phys_vec3 lv2;
  if ( psc->m_b2 )
  {
    last_t_vel = rbint::get_last_t_vel( psc->m_b2->m_rb );
    last_a_vel = rbint::get_last_a_vel( psc->m_b2->m_rb );
    _a = last_a_vel;
    _b = phys_cross( _a, m_b2_r );
    lv2 = last_t_vel + _b;
  }
  else
  {
    lv2 = object_vel_();
  }
  phys_vec3 retv = lv - lv2;
  return phys_vec3( phys_dot( retv, psc->m_ud_n ), phys_dot( retv, psc->m_ud_f1 ), phys_dot( retv, psc->m_ud_f2 ) );
}

inline const float pulse_sum_contact_point::get_pos( pulse_sum_contact *psc )
{
  return phys_dot( ( ( psc->m_b1->m_rb->get_mat().GetW() ) + m_b1_r ) -
                       ( psc->m_b2 ? ( psc->m_b2->m_rb->get_mat().GetW() ) + m_b2_r : object_col_pt_() ),
                   psc->m_ud_n );
}

inline const phys_vec3 pulse_sum_contact_point::get_objective( pulse_sum_contact *psc )
{
  phys_vec3 _a = psc->m_b1->a_vel;
  phys_vec3 _b = phys_cross( _a, m_b1_r );
  phys_vec3 retv = psc->m_b1->t_vel + _b;
  if ( psc->m_b2 )
  {
    _a = psc->m_b2->a_vel;
    _b = phys_cross( _a, m_b2_r );
    retv -= ( psc->m_b2->t_vel + _b );
  }
  return phys_vec3( phys_dot( retv, psc->m_ud_n ), phys_dot( retv, psc->m_ud_f1 ), phys_dot( retv, psc->m_ud_f2 ) );
}

inline void pulse_sum_contact_point::apply( pulse_sum_contact *psc, phys_vec3 &s_ )
{
  phys_vec3 f = s_.GetX() * psc->m_ud_n + s_.GetY() * psc->m_ud_f1 + s_.GetZ() * psc->m_ud_f2;
  psc->m_b1->t_vel += psc->m_b1->m_inv_mass * f;
  psc->m_b1->a_vel += s_.GetX() * m_b1_ap_n + s_.GetY() * m_b1_ap_f1 + s_.GetZ() * m_b1_ap_f2;
  if ( psc->m_b2 )
  {
    psc->m_b2->t_vel -= psc->m_b2->m_inv_mass * f;
    psc->m_b2->a_vel -= s_.GetX() * m_b2_ap_n + s_.GetY() * m_b2_ap_f1 + s_.GetZ() * m_b2_ap_f2;
  }
}

inline void pulse_sum_contact_point::clamp_n( pulse_sum_contact *psc )
{
  if ( m_pulse_sum.GetX() > 0.0f )
  {
    m_pulse_sum.SetX( 0.0f );
  }
}

inline void pulse_sum_contact_point::clamp_f( pulse_sum_contact *psc )
{
  const float m_pulse_sum_fric_limit = -psc->m_fric_coef * m_pulse_sum.GetX();
  const float pulse_sum_fric_sq = phys_sqr( m_pulse_sum.GetY() ) + phys_sqr( m_pulse_sum.GetZ() );
  if ( pulse_sum_fric_sq > phys_sqr( m_pulse_sum_fric_limit ) )
  {
    const float pulse_sum_fric = sqrtf( pulse_sum_fric_sq );
    tlAssert( pulse_sum_fric > 0.0f )
    tlAssert( m_pulse_sum_fric_limit >= 0.0f )
    m_pulse_sum[1] = m_pulse_sum[1] * ( m_pulse_sum_fric_limit / pulse_sum_fric );
    m_pulse_sum[2] = m_pulse_sum[2] * ( m_pulse_sum_fric_limit / pulse_sum_fric );
  }
}

inline void pulse_sum_contact_point::project( pulse_sum_contact *psc )
{
  clamp_n( psc );
  clamp_f( psc );
  apply( psc, m_pulse_sum );
}

inline phys_vec3 &pulse_sum_contact_point::object_vel_()
{
  return m_b1_ap_n;
}

inline phys_vec3 &pulse_sum_contact_point::object_col_pt_()
{
  return m_b1_r;
}

inline void pulse_sum_contact_point::set_object_vel( pulse_sum_contact *psc, phys_vec3 &object_vel )
{
  tlAssert( psc->m_b2 == NULL );
  m_b2_ap_n = object_vel;
}

inline void pulse_sum_contact_point::set_object_col_pt( pulse_sum_contact *psc, phys_vec3 &object_vel )
{
  tlAssert( psc->m_b2 == NULL );
  m_b2_r = object_vel;
}

inline void pulse_sum_contact_point::SOLVER_apply_relaxation( pulse_sum_contact *psc, float &error_sq )
{
  phys_vec3 m_last_pulse_sum = m_pulse_sum;
  phys_vec3 _a = get_objective( psc );
  phys_vec3 m_objective = _a - m_right_side;
  m_pulse_sum[0] -= m_objective.GetX() / m_denom_xx;
  clamp_n( psc );

  float delta_ps_n = m_pulse_sum.GetX() - m_last_pulse_sum.GetX();
  m_objective[1] += delta_ps_n * m_denom_xy;
  m_objective[2] += delta_ps_n * m_denom_xz;
  m_pulse_sum[1] -= ( m_inv_yy * m_objective.GetY() ) + ( m_inv_yz * m_objective.GetZ() );
  m_pulse_sum[2] -= ( m_inv_yz * m_objective.GetY() ) + ( m_inv_zz * m_objective.GetZ() );
  clamp_f( psc );

  phys_vec3 delta_pulse_sum = m_pulse_sum - m_last_pulse_sum;
  apply( psc, delta_pulse_sum );
  phys_vec3 denom( m_denom_xx, m_denom_yy, m_denom_zz );
  phys_vec3 error_ = phys_diag_multiply_and_square( delta_pulse_sum, denom );
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

inline void pulse_sum_contact_point::SOLVER_solver_prolog( pulse_sum_contact *psc, pulse_sum_cache *m_pulse_sum_cache, const float delta_t )
{
  m_right_side -= get_vel( psc );
  m_pulse_sum[0] = delta_t * m_pulse_sum_cache[0].get_pulse_sum();
  m_pulse_sum[1] = delta_t * m_pulse_sum_cache[1].get_pulse_sum();
  m_pulse_sum[2] = delta_t * m_pulse_sum_cache[2].get_pulse_sum();
  project( psc );
}

inline void pulse_sum_contact_point::SOLVER_solver_intermediate( pulse_sum_contact *psc, pulse_sum_cache *m_pulse_sum_cache, const float delta_t )
{
  m_pulse_sum_cache[0].set_pulse_sum( m_pulse_sum.GetX() / delta_t );
  m_pulse_sum_cache[1].set_pulse_sum( m_pulse_sum.GetY() / delta_t );
  m_pulse_sum_cache[2].set_pulse_sum( m_pulse_sum.GetZ() / delta_t );
  m_right_side[0] += m_big_dirt;
}

inline void pulse_sum_contact_point::setup_vel_uni_restitution( pulse_sum_contact *psc,
                                                         const float restitution_k,
                                                         const float max_restitution_v,
                                                         const float delta_t,
                                                         const float max_penalty_restitution_vel,
                                                         const float min_restitution_impact_vel_sq )
{
  const float dist = get_pos( psc );
  m_big_dirt = -dist / tl_max( delta_t, MIN_NERP_DELTA_T );
  if ( m_big_dirt < 0.0f )
  {
    m_big_dirt = m_big_dirt * 0.3;
  }
  if ( m_big_dirt < -max_penalty_restitution_vel )
  {
    m_big_dirt = -max_penalty_restitution_vel;
  }

  if ( m_big_dirt < 0.0f )
  {
    m_right_side = PHYS_ZERO_VEC;
  }
  else
  {
    m_right_side = phys_vec3( m_big_dirt, 0.0f, 0.0f );
    m_big_dirt = 0.0f;
  }

  if ( restitution_k > 0.0000099999997f && max_restitution_v > 0.0000099999997f && dist >= 0.0f )
  {
    float rv = get_impact_vel( psc ) * restitution_k;
    if ( max_restitution_v < rv )
    {
      rv = max_restitution_v;
    }
    if ( -1e-05f < m_big_dirt )
    {
      m_right_side[0] -= rv;
    }
    else if ( -rv < m_big_dirt )
    {
      m_right_side[0] = -rv;
      m_big_dirt = 0.0f;
    }
  }
}

inline pulse_sum_contact_point::pulse_sum_contact_point() {}

inline void pulse_sum_contact::SOLVER_apply_relaxation( float &error_sq )
{
  pulse_sum_contact_point *last_pscp_i = &m_list_pscp[m_list_pscp_count];
  for ( pulse_sum_contact_point *pscp_i = m_list_pscp; pscp_i != last_pscp_i; ++pscp_i )
  {
    pscp_i->SOLVER_apply_relaxation( this, error_sq );
  }
}

inline void pulse_sum_contact::SOLVER_solver_prolog( const float delta_t )
{
  contact_point_info::pulse_sum_cache_info *ps_cache_info =
      reinterpret_cast<contact_point_info::pulse_sum_cache_info *>( m_pulse_sum_cache_list );
  pulse_sum_contact_point *last_pscp_i = &m_list_pscp[m_list_pscp_count];
  for ( pulse_sum_contact_point *pscp_i = m_list_pscp; pscp_i != last_pscp_i; ++pscp_i )
  {
    pscp_i->SOLVER_solver_prolog( this, ps_cache_info->m_ps_cache_list, delta_t );
    ++ps_cache_info;
  }
}

inline void pulse_sum_contact::SOLVER_solver_intermediate( const float delta_t )
{
  contact_point_info::pulse_sum_cache_info *ps_cache_info =
      reinterpret_cast<contact_point_info::pulse_sum_cache_info *>( m_pulse_sum_cache_list );
  pulse_sum_contact_point *last_pscp_i = &m_list_pscp[m_list_pscp_count];
  for ( pulse_sum_contact_point *pscp_i = m_list_pscp; pscp_i != last_pscp_i; ++pscp_i )
  {
    pscp_i->SOLVER_solver_intermediate( this, ps_cache_info->m_ps_cache_list, delta_t );
    ++ps_cache_info;
  }
}

inline void pulse_sum_contact::set( rigid_body *const b1, rigid_body *const b2, contact_point_info *cpi, const float delta_t )
{
  tlAssert( b1 );
  tlAssert( b2 );
  tlAssert( b1 != b2 );
  tlAssert( rbint::get_pulse_sum_node( b1 ) || rbint::get_pulse_sum_node( b2 ) );
  tlAssert( rbint::verify_pulse_sum_node( b1 ) && rbint::verify_pulse_sum_node( b2 ) );
  tlAssertMsg( rbint::get_pulse_sum_node( b1 ) != NULL, "b1 in contact constraint cannot be environment or user rigid body." );

  m_b1 = rbint::get_pulse_sum_node( b1 );
  m_b2 = rbint::get_pulse_sum_node( b2 );
  m_ud_n = cpi->m_normal;
  m_ud_f1 = construct_orth_ud( m_ud_n );
  m_ud_f2 = phys_cross( m_ud_n, m_ud_f1 );
  m_fric_coef = cpi->m_fric_coef;
  m_pulse_sum_cache_list = cpi->m_list_pulse_sum_cache_info;
  tlAssert( m_list_pscp_count == cpi->m_point_pair_count );
  phys_vec3 gravity_acc_vec = m_b1->m_rb->get_gravity_acc_vec();
  float min_restitution_impact_vel_sq = phys_sqr( 0.30000001f ) * AbsSquared( gravity_acc_vec );

  const phys_vec3 *b1_r_loc = cpi->m_list_b1_r_loc;
  const phys_vec3 *b2_r_loc = cpi->m_list_b2_r_loc;

  for ( int pp_i = 0; pp_i < cpi->m_point_pair_count; ++pp_i )
  {
    pulse_sum_contact_point *pscp_i = &m_list_pscp[pp_i];
    if ( pscp_i )
    {
      *pscp_i = pulse_sum_contact_point();
    }
    pscp_i->m_b1_r = rbint::multiply( b1, *b1_r_loc );
    phys_vec3 b2_r = rbint::multiply( b2, *b2_r_loc );
    if ( m_b2 )
    {
      pscp_i->m_b2_r = b2_r;
    }
    else
    {
      phys_vec3 object_vel = rbint::gtv( b2, b2_r );
      pscp_i->set_object_vel( this, object_vel );
      phys_vec3 pos = rbint::add_pos( b2, b2_r );
      pscp_i->set_object_col_pt( this, pos );
    }
    pscp_i->calc_abs( this );
    pscp_i->setup_vel_uni_restitution( this,
                                       cpi->m_bounce_coef,
                                       cpi->m_max_restitution_vel,
                                       delta_t,
                                       get_std_max_penalty_restitution_vel(),
                                       min_restitution_impact_vel_sq );
    ++b1_r_loc;
    ++b2_r_loc;
  }
}

inline const float pulse_sum_contact::get_std_max_penalty_restitution_vel()
{
  return 50.0f;
}

inline pulse_sum_contact::pulse_sum_contact() {}
