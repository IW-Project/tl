#pragma once

#include "pulse_sum_normal.h"
#include "rigid_body_internal.h"

using namespace math;

void pulse_sum_normal::set_flag( const u32 f, const bool b )
{
  if ( b )
  {
    m_flags |= f;
  }
  else
  {
    m_flags &= ~f;
  }
}

const u32 pulse_sum_normal::get_flag( const u32 f )
{
  return f & m_flags;
}

void pulse_sum_normal::calc_abs( const phys_vec3 &b1_r_displace )
{
  phys_vec3 v = m_b1_r + b1_r_displace;
  m_b1_ap = phys_multiply( m_b1->m_world_inv_inertia, phys_cross( v, m_ud ) );
  m_denom = m_b1->m_inv_mass + phys_dot( phys_cross( m_b1_ap, m_b1_r ), m_ud );
  if ( m_b2 )
  {
    m_b2_ap = phys_multiply( m_b2->m_world_inv_inertia, phys_cross( m_b2_r, m_ud ) );
    m_denom += m_b2->m_inv_mass + phys_dot( phys_cross( m_b2_ap, m_b2_r ), m_ud );
  }
}

const float pulse_sum_normal::get_vel()
{
  phys_vec3 t_vel = m_b1->m_rb->get_t_vel();
  phys_vec3 a_vel = m_b1->m_rb->get_a_vel();

  phys_vec3 _a = a_vel;
  phys_vec3 _b = phys_cross( _a, m_b1_r );
  phys_vec3 v = t_vel + _b;
  if ( m_b2 )
  {
    t_vel = m_b2->m_rb->get_t_vel();
    a_vel = m_b2->m_rb->get_a_vel();
    _a = a_vel;
    _b = phys_cross( _a, m_b2_r );
    v -= ( t_vel + _b );
  }
  else
  {
    v -= object_vel_();
  }
  return phys_dot( v, m_ud );
}

const float pulse_sum_normal::get_last_vel()
{
  phys_vec3 v = LAST_T_VEL( m_b1->m_rb ) + phys_cross( LAST_A_VEL( m_b1->m_rb ), m_b1_r );
  if ( m_b2 )
  {
    v -= ( LAST_T_VEL( m_b2->m_rb ) + phys_cross( LAST_A_VEL( m_b2->m_rb ), m_b2_r ) );
  }
  else
  {
    v -= object_vel_();
  }

  return phys_dot( v, m_ud );
}

const float pulse_sum_normal::get_pos()
{
  phys_vec3 v = ( m_b1->m_rb->get_mat().GetW() + m_b1_r ) - ( m_b2 ? m_b2->m_rb->get_mat().GetW() + m_b2_r : object_col_pt_() );
  return phys_dot( v, m_ud );
}

phys_vec3 &pulse_sum_normal::object_vel_()
{
  return m_b2_ap;
}

phys_vec3 &pulse_sum_normal::object_col_pt_()
{
  return m_b2_r;
}

const float pulse_sum_normal::clamp_pulse_sum( const float ps )
{
  if ( get_flag( 1u ) )
  {
    tlAssert( m_pulse_parent );
    tlAssert( m_pulse_limit_ratio >= 0.0f );
    m_pulse_sum_max = m_pulse_limit_ratio * Abs( m_pulse_parent->get_pulse_sum() );
    m_pulse_sum_min = -m_pulse_sum_max;
  }
}

const float pulse_sum_normal::get_objective()
{
  phys_vec3 _a = m_b1->a_vel;
  phys_vec3 _b = phys_cross( _a, m_b1_r );
  phys_vec3 relv = m_b1->t_vel + _b;
  if ( m_b2 )
  {
    _a = m_b2->a_vel;
    _b = phys_cross( _a, m_b2_r );
    relv -= ( m_b2->t_vel + _b );
  }
  return phys_dot( relv, m_ud );
}

void pulse_sum_normal::apply( float &s_ )
{
  phys_vec3 _v = ( s_ * m_b1->m_inv_mass ) * m_ud;
  m_b1->t_vel += _v;
  m_b1->a_vel += ( s_ * m_b1_ap );
  if ( m_b2 )
  {
    _v = ( s_ * m_b2->m_inv_mass ) * m_ud;
    m_b2->t_vel -= _v;
    m_b2->a_vel -= ( s_ * m_b2_ap );
  }
}

void pulse_sum_normal::project()
{
  m_pulse_sum = clamp_pulse_sum( m_pulse_sum );
  apply( m_pulse_sum );
}

void pulse_sum_normal::SOLVER_apply_relaxation( float &error_sq, const bool add_error )
{
  float m_last_pulse_sum = m_pulse_sum;
  m_pulse_sum = clamp_pulse_sum( m_last_pulse_sum - ( ( ( get_objective() + ( m_cfm * m_last_pulse_sum ) ) - m_right_side ) / m_denom ) );
  tlAssert( m_pulse_sum >= ( m_pulse_sum_min - .0001f ) && m_pulse_sum <= ( m_pulse_sum_max + .0001f ) );
  float s_ = m_pulse_sum - m_last_pulse_sum;
  apply( s_ );
  if ( add_error )
  {
    float error_sq___ = phys_sqr( ( m_pulse_sum - m_last_pulse_sum ) * m_denom );
    if ( error_sq___ > error_sq )
    {
      error_sq = error_sq___;
    }
  }
}

void pulse_sum_normal::SOLVER_solver_prolog( const float delta_t )
{
  m_right_side = m_right_side - get_vel();
  m_pulse_sum = delta_t * m_pulse_sum_cache->get_pulse_sum();
  project();
}

void pulse_sum_normal::SOLVER_solver_intermediate( const float delta_t )
{
  m_pulse_sum_cache->set_pulse_sum( m_pulse_sum / delta_t );
  m_right_side = m_right_side + m_big_dirt;
}

void pulse_sum_normal::set( rigid_body *const b1,
                            const phys_vec3 &b1_r,
                            rigid_body *const b2,
                            const phys_vec3 &b2_r,
                            const phys_vec3 &ud,
                            pulse_sum_cache *const ps_cache,
                            const phys_vec3 &b1_r_displace )
{
  tlAssert( ps_cache );
  tlAssert( b1 );
  tlAssert( b2 );
  tlAssert( b1 != b2 );
  PHYS_ASSERT_UNIT( ud );
  tlAssert( rbint::get_pulse_sum_node( b1 ) || rbint::get_pulse_sum_node( b2 ) );
  tlAssert( rbint::verify_pulse_sum_node( b1 ) && rbint::verify_pulse_sum_node( b2 ) );

  if ( rbint::get_pulse_sum_node( b1 ) )
  {
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
      phys_vec3 pos = rbint::add_pos( b2, b2_r );
      set_object_col_pt( pos );
    }
    m_ud = ud;
  }
  else
  {
    m_b1 = rbint::get_pulse_sum_node( b2 );
    m_b1_r = b2_r;
    m_b2 = NULL;
    phys_vec3 object_vel = rbint::gtv( b1, b1_r );
    set_object_vel( object_vel );
    phys_vec3 pos = rbint::add_pos( b1, b1_r );
    set_object_col_pt( pos );
    m_ud = -ud;
  }
  m_pulse_sum_cache = ps_cache;
  m_flags = 0;
  calc_abs( b1_r_displace );
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
}

const float pulse_sum_normal::get_unclamped_pulse_sum()
{
  float v_ = get_objective();
  return clamp_pulse_sum( m_pulse_sum + ( ( ( m_right_side - v_ ) - ( m_cfm * m_pulse_sum ) ) / m_denom ) );
}
