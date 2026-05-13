#pragma once

#include "pulse_sum_wheel.h"

inline const bool pulse_sum_wheel::pulse_chain_within_limits()
{
  pulse_sum_normal *ps1 = m_side;
  pulse_sum_normal *ps2 = m_fwd;

  float a_ = -ps1->get_limit_ratio() * m_suspension.get_pulse_sum();
  a_ = tl_min( a_, m_side_fric_max );

  float b_ = -ps2->get_limit_ratio() * m_suspension.get_pulse_sum();
  b_ = tl_min( b_, m_side_fric_max );

  float a2_ = phys_sqr( a_ );
  float b2_ = phys_sqr( b_ );
  float p1_ = ps1->get_pulse_sum();
  float p2_ = ps2->get_pulse_sum();
  return rel_cmp_ge( a2_ * b2_, ( a2_ * phys_sqr( p2_ ) ) + ( b2_ * phys_sqr( p1_ ) ), 0.0000099999997f );
}

inline const bool pulse_sum_wheel::clamp_pulse_sum_pulse_chain( float *ps1_, float *ps2_ )
{
  pulse_sum_normal *ps1 = m_side;
  pulse_sum_normal *ps2 = m_fwd;

  float a_ = -ps1->get_limit_ratio() * m_suspension.get_pulse_sum();
  a_ = tl_min( a_, m_side_fric_max );

  float b_ = -ps2->get_limit_ratio() * m_suspension.get_pulse_sum();
  b_ = tl_min( b_, m_side_fric_max );

  float a2_ = phys_sqr( a_ );
  float b2_ = phys_sqr( b_ );

  float p1_ = phys_sqr( ps1->get_pulse_sum() );
  float p2_ = phys_sqr( ps2->get_pulse_sum() );

  float numer_sq = ( a2_ * p2_ ) + ( b2_ * p1_ );
  float denom_sq = a2_ * b2_;

  if ( ( denom_sq <= numer_sq ) || ( denom_sq <= 0.0000099999997f ) )
  {
    ps2->set_flag( 4, true );
    float _t = numer_sq <= 0.0000099999997f ? 0.0f : sqrtf( numer_sq / denom_sq );
    *ps1_ = _t * p1_;
    *ps2_ = _t * p2_;
    return true;
  }
  else
  {
    ps2->set_flag( 4, false );
    *ps1_ = ps1->get_pulse_sum();
    *ps2_ = ps2->get_pulse_sum();
    return false;
  }
}

inline void pulse_sum_wheel::addp_pulse_chain()
{
  pulse_sum_normal *ps1 = m_side;
  pulse_sum_normal *ps2 = m_fwd;

  float ps1_, ps2_;
  if ( clamp_pulse_sum_pulse_chain( &ps1_, &ps2_ ) )
  {
    float s_ = ps1_ - ps1->get_pulse_sum();
    ps1->apply( s_ );
    ps1->m_pulse_sum = ps1_;

    s_ = ps2_ - ps2->get_pulse_sum();
    ps2->apply( s_ );
    ps2->m_pulse_sum = ps2_;
  }
  tlAssert( pulse_chain_within_limits() );
}

inline void pulse_sum_wheel::SOLVER_apply_relaxation( float &error_sq )
{
  m_suspension.SOLVER_apply_relaxation( error_sq, true );
  if ( m_side )
  {
    float m_side_last_pulse_sum = m_side->get_pulse_sum();
    m_side->SOLVER_apply_relaxation( error_sq, m_fwd == 0 );
    if ( m_fwd )
    {
      float m_fwd_last_pulse_sum = m_fwd->get_pulse_sum();
      m_fwd->SOLVER_apply_relaxation( error_sq, false );
      addp_pulse_chain();
      float error_sq___ = phys_sqr( ( m_side->get_pulse_sum() - m_side_last_pulse_sum ) * m_side->get_denom() );
      if ( error_sq___ > error_sq )
      {
        error_sq = error_sq___;
      }
      error_sq___ = phys_sqr( ( m_fwd->get_pulse_sum() - m_fwd_last_pulse_sum ) * m_fwd->get_denom() );
      if ( error_sq___ > error_sq )
      {
        error_sq = error_sq___;
      }
    }
  }
}

inline void pulse_sum_wheel::SOLVER_solver_prolog( const float delta_t )
{
  m_suspension.SOLVER_solver_prolog( delta_t );
  if ( m_side )
  {
    m_side->SOLVER_solver_prolog( delta_t );
    if ( m_fwd )
    {
      m_fwd->SOLVER_solver_prolog( delta_t );
      addp_pulse_chain();
    }
  }
}

inline void pulse_sum_wheel::SOLVER_solver_intermediate( const float delta_t )
{
  m_suspension.SOLVER_solver_intermediate( delta_t );
  if ( m_side )
  {
    m_side->SOLVER_solver_intermediate( delta_t );
    if ( m_fwd )
    {
      m_fwd->SOLVER_solver_intermediate( delta_t );
    }
  }
}

inline void pulse_sum_wheel::set()
{
  m_side = NULL;
  m_fwd = NULL;
}

inline void pulse_sum_wheel::set_side_fwd_ratios( const float side_ratio, const float fwd_ratio, const float side_fric_max )
{
  m_side->m_pulse_limit_ratio = side_ratio;
  m_fwd->m_pulse_limit_ratio = fwd_ratio;
  m_side_fric_max = side_fric_max;
}

inline pulse_sum_normal *pulse_sum_wheel::get_suspension()
{
  return &m_suspension;
}

inline pulse_sum_wheel::pulse_sum_wheel() {}