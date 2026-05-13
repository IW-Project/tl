#pragma once

#include "pulse_sum_angular.h"
#include "pulse_sum_base.h"
#include "rigid_body.h"
#include "rigid_body_internal.h"

void pulse_sum_angular::set_flag( const unsigned int f, const bool b )
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

const unsigned int pulse_sum_angular::get_flag( const unsigned int f )
{
  return ( m_flags & f );
}

void pulse_sum_angular::calc_abs()
{
  m_b1_ap = phys_multiply( m_b1->m_world_inv_inertia, m_ud );
  m_denom = phys_dot( m_b1_ap, m_ud );
  if ( m_b2 )
  {
    m_b2_ap = phys_multiply( m_b2->m_world_inv_inertia, m_ud );
    m_denom += phys_dot( m_b2_ap, m_ud );
  }

  tlAssert( m_denom > 0.00001f );
}

const float pulse_sum_angular::get_vel()
{
  phys_vec3 _a = m_b1->m_rb->get_a_vel();
  phys_vec3 _b = m_b2 ? m_b2->m_rb->get_a_vel() : object_vel_();
  return phys_dot( ( _a - _b ), m_ud );
}

const float pulse_sum_angular::get_pos()
{
  phys_vec3 _a = m_b1_r;
  phys_vec3 _b = m_b2 ? m_b2_r : object_col_pt_();
  return -phys_dot( phys_cross( _a, _b ), m_ud );
}

phys_vec3 &pulse_sum_angular::object_vel_()
{
  return m_b2_ap;
}

phys_vec3 &pulse_sum_angular::object_col_pt_()
{
  return m_b2_r;
}

void pulse_sum_angular::set_object_vel( const phys_vec3 &object_vel )
{
  m_b2_ap = object_vel;
}

void pulse_sum_angular::set_object_col_pt( const phys_vec3 &object_col_p )
{
  m_b2_r = object_col_p;
}

const float pulse_sum_angular::clamp_pulse_sum( const float ps )
{
  tlAssert( m_pulse_sum_min <= m_pulse_sum_max );

  if ( m_pulse_sum_min > ps )
  {
    return m_pulse_sum_min;
  }
  if ( ps > m_pulse_sum_max )
  {
    return m_pulse_sum_max;
  }
  return ps;
}

const float pulse_sum_angular::get_objective()
{
  phys_vec3 relv = m_b1->a_vel;
  if ( m_b2 )
  {
    relv -= m_b2->a_vel;
  }
  phys_vec3 _a = relv;
  return phys_dot( _a, m_ud );
}

void pulse_sum_angular::apply( float &s_ )
{
  m_b1->a_vel += s_ * m_b1_ap;
  if ( m_b2 )
  {
    m_b2->a_vel -= s_ * m_b2_ap;
  }
}

void pulse_sum_angular::project()
{
  m_pulse_sum = clamp_pulse_sum( m_pulse_sum );
  apply( m_pulse_sum );
}

void pulse_sum_angular::SOLVER_apply_relaxation( float &error_sq )
{
  float objective = get_objective();
  float m_last_pulse_sum = m_pulse_sum;
  m_pulse_sum = clamp_pulse_sum( m_last_pulse_sum - ( ( ( objective + ( m_cfm * m_last_pulse_sum ) ) - m_right_side ) / m_denom ) );
  tlAssert( m_pulse_sum >= ( m_pulse_sum_min - .0001f ) && m_pulse_sum <= ( m_pulse_sum_max + .0001f ) );
  float s_ = m_pulse_sum - m_last_pulse_sum;
  apply( s_ );
  float error_sq___ = phys_sqr( ( m_pulse_sum - m_last_pulse_sum ) * m_denom );
  if ( error_sq___ > error_sq )
  {
    error_sq = error_sq___;
  }
}

void pulse_sum_angular::SOLVER_solver_prolog( const float delta_t )
{
  m_right_side = m_right_side - get_vel();
  m_pulse_sum = delta_t * m_pulse_sum_cache->get_pulse_sum();
  project();
}

void pulse_sum_angular::SOLVER_solver_intermediate( const float delta_t )
{
  m_pulse_sum_cache->set_pulse_sum( m_pulse_sum / delta_t );
  m_right_side += m_big_dirt;
}

void pulse_sum_angular::set( rigid_body *const b1,
                             phys_vec3 &b1_r,
                             rigid_body *const b2,
                             phys_vec3 &b2_r,
                             phys_vec3 &ud,
                             pulse_sum_cache *const ps_cache )
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
    ;
    if ( rbint::get_pulse_sum_node( b2 ) )
    {
      m_b2 = rbint::get_pulse_sum_node( b2 );
      m_b2_r = b2_r;
    }
    else
    {
      m_b2 = NULL;
      set_object_vel( b2->get_a_vel() );
      set_object_col_pt( b2_r );
    }
    m_ud = ud;
  }
  else
  {
    m_b1 = rbint::get_pulse_sum_node( b2 );
    m_b1_r = b2_r;
    m_b2 = NULL;
    set_object_vel( b1->get_a_vel() );
    set_object_col_pt( b1_r );
    m_ud = -ud;
  }
  m_pulse_sum_cache = ps_cache;
  m_flags = 0;
  calc_abs();
}

void pulse_sum_angular::set_pulse_sum_limits_unbounded()
{
  m_pulse_sum_min = -10000000.f;
  m_pulse_sum_max = 10000000.f;
}

void pulse_sum_angular::set_pulse_sum_limits_cone( const float pulse_sum_max )
{
  m_pulse_sum_min = -pulse_sum_max;
  m_pulse_sum_max = pulse_sum_max;
}

void pulse_sum_angular::set_pulse_sum_limits_negative()
{
  m_pulse_sum_min = -10000000.f;
  m_pulse_sum_max = 0.0f;
}

void pulse_sum_angular::set_pulse_sum_limits( float min, float max )
{
  m_pulse_sum_min = min;
  m_pulse_sum_max = max;
}

void pulse_sum_angular::setup_vel_bi_standard( const float delta_t )
{
  m_big_dirt = ( -0.5f / tl_max( delta_t, MIN_NERP_DELTA_T ) ) * get_pos();
  m_right_side = 0;
  m_cfm = 0;
}

void pulse_sum_angular::setup_vel_uni_standard( const float delta_t, const float max_penalty_restitution_vel )
{
  m_big_dirt = -get_pos() / tl_max( delta_t, MIN_NERP_DELTA_T );
  if ( m_big_dirt < 0.0f )
  {
    m_big_dirt = m_big_dirt * 0.3f;
  }
  if ( -max_penalty_restitution_vel > m_big_dirt )
  {
    m_big_dirt = -max_penalty_restitution_vel;
  }
  if ( m_big_dirt < 0.0f )
  {
    m_right_side = 0;
  }
  else
  {
    m_right_side = m_big_dirt;
    m_big_dirt = 0;
  }
  m_cfm = 0;
}

void pulse_sum_angular::setup_vel_uni_standard_1( const float erp, const float cfm, const float delta_t ) {}

void pulse_sum_angular::setup_vel_simple()
{
  m_right_side = 0;
  m_big_dirt = 0;
  m_cfm = 0;
}

void pulse_sum_angular::setup_vel_custom_pos( const float erp, const float cfm, const float delta_t, const float right_side ) {}

void pulse_sum_angular::setup_vel_custom( const float erp, const float cfm, const float delta_t )
{
  m_right_side = ( -erp / delta_t ) * get_pos();
  m_big_dirt = 0;
  m_cfm = cfm;
  m_denom = m_denom + cfm;
}

void pulse_sum_angular::setup_vel_custom_big_dirt( const float erp, const float cfm, const float delta_t )
{
  m_big_dirt = ( -erp / delta_t ) * get_pos();
  m_right_side = 0;
  m_cfm = cfm;
  m_denom = m_denom + cfm;
}

void pulse_sum_angular::setup_vel_custom_right_side( const float right_side, const float cfm )
{
  m_right_side = right_side;
  m_big_dirt = 0;
  m_cfm = cfm;
  m_denom = m_denom + cfm;
}

void pulse_sum_angular::add_right_side( const float r )
{
  m_right_side += r;
}

const float pulse_sum_angular::get_std_active_limit_angle_eps()
{
  return 0.043633226f;
}

const float pulse_sum_angular::get_std_active_limit_sin_angle_eps()
{
  return 0.043618999f;
}

const float pulse_sum_angular::get_std_max_penalty_restitution_vel()
{
  return 5.0f;
}

inline pulse_sum_angular::pulse_sum_angular() {}
