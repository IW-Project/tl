#pragma once

#include "phys_base.h"
#include "phys_mem.h"

#include "pulse_sum_base.h"
#include "pulse_sum_cache.h"

#pragma pack( push, 16 )
class pulse_sum_normal : public phys_link_list_base<pulse_sum_normal>
{
  enum flags_e
  {
    PULSE_LIMIT_PARENT_RATIO = 1,
    PULSE_LIMIT_REACHED = 2,
    PULSE_CHAIN_LIMIT_REACHED = 4,
    APPLY_MIN_FACTOR = 8,
  };

private:
  void set_flag( const u32 f, const bool b );
  const u32 get_flag( const u32 f );
  phys_vec3 m_ud;
  phys_vec3 m_b1_r;
  phys_vec3 m_b2_r;
  phys_vec3 m_b1_ap;
  phys_vec3 m_b2_ap;
  float m_pulse_sum_min;
  float m_pulse_sum_max;
  float m_pulse_sum;
  float m_right_side;
  float m_big_dirt;
  float m_cfm;
  float m_denom;
  float m_pulse_limit_ratio;
  u32 m_flags;
  pulse_sum_normal *m_pulse_parent;
  pulse_sum_node *m_b1;
  pulse_sum_node *m_b2;
  pulse_sum_cache *m_pulse_sum_cache;
  void calc_abs( const phys_vec3 &b1_r_displace );
  const float get_vel();
  const float get_last_vel();
  const float get_pos();
  phys_vec3 &object_vel_();
  phys_vec3 &object_col_pt_();
  const float clamp_pulse_sum( const float ps );
  const float get_objective();
  void apply( float &s_ );
  void project();
  void SOLVER_apply_relaxation( float &error_sq, const bool add_error );
  void SOLVER_solver_prolog( const float delta_t );
  void SOLVER_solver_intermediate( const float delta_t );

public:
  void set( rigid_body *const b1,
            const phys_vec3 &b1_r,
            rigid_body *const b2,
            const phys_vec3 &b2_r,
            const phys_vec3 &ud,
            pulse_sum_cache *const ps_cache,
            const phys_vec3 &b1_r_displace );

  inline void set_object_vel( phys_vec3 &object_vel )
  {
    tlAssert( m_b2 == NULL );
    m_b2_ap = object_vel;
  }

  inline void set_object_col_pt( phys_vec3 &object_col_pt )
  {
    tlAssert( m_b2 == NULL );
    m_b2_r = object_col_pt;
  }

  inline void setup_vel_bi_standard( const float delta_t )
  {
    m_big_dirt = -0.5f / tl_max( delta_t, MIN_NERP_DELTA_T );
    m_right_side = 0.0f;
    m_cfm = 0.0f;
  }

  inline void setup_vel_uni_standard( const float delta_t, const float max_penalty_restitution_vel )
  {
    m_big_dirt = -get_pos() / tl_max( delta_t, MIN_NERP_DELTA_T );
    if ( m_big_dirt < 0.0f )
    {
      m_big_dirt = m_big_dirt * 0.30000001f;
    }
    if ( -max_penalty_restitution_vel > m_big_dirt )
    {
      m_big_dirt = -max_penalty_restitution_vel;
    }
    if ( m_big_dirt < 0.0f )
    {
      m_right_side = 0.0f;
    }
    else
    {
      m_right_side = -m_big_dirt;
      m_big_dirt = 0.0f;
    }

    m_cfm = 0.0f;
  }

  inline void setup_vel_uni_standard_pos_adjust( const float delta_t, const float pos_adjust, const float max_penalty_restitution_vel )
  {
    m_big_dirt = -( get_pos() + pos_adjust ) / tl_max( delta_t, MIN_NERP_DELTA_T );
    if ( m_big_dirt < 0.0f )
    {
      m_big_dirt = m_big_dirt * 0.30000001f;
    }
    if ( -max_penalty_restitution_vel > m_big_dirt )
    {
      m_big_dirt = -max_penalty_restitution_vel;
    }
    if ( m_big_dirt < 0.0f )
    {
      m_right_side = 0.0f;
    }
    else
    {
      m_right_side = -m_big_dirt;
      m_big_dirt = 0.0f;
    }
    m_cfm = 0.0f;
  }

  void setup_vel_uni_restitution( const float, const float, const float, const float );

  inline void setup_vel_simple()
  {
    m_big_dirt = 0.0f;
    m_right_side = 0.0f;
    m_cfm = 0.0f;
  }

  void setup_vel_custom_pos( const float, const float, const float, const float );

  inline void setup_vel_custom( const float erp, const float cfm, const float delta_t )
  {
    m_right_side = ( -erp / delta_t ) * get_pos();
    m_big_dirt = 0.0f;
    m_cfm = cfm;
    m_denom = m_denom + cfm;
  }

  inline void setup_vel_custom_right_side( const float right_side, const float cfm )
  {
    m_right_side = right_side;
    m_big_dirt = 0.0f;
    m_cfm = cfm;
    m_denom = m_denom + cfm;
  }

  inline void add_right_side( const float r ) { m_right_side += r; }

  inline void set_pulse_sum_limits( const float min, const float max )
  {
    m_pulse_sum_min = min;
    m_pulse_sum_max = max;
  }

  inline void set_pulse_sum_limits_unbounded()
  {
    m_pulse_sum_min = -10000000.f;
    m_pulse_sum_max = 10000000.f;
  }

  inline void set_pulse_sum_limits_cone( const float pulse_sum_max )
  {
    m_pulse_sum_min = -pulse_sum_max;
    m_pulse_sum_max = pulse_sum_max;
  }

  inline void set_pulse_sum_limits_positive()
  {
    m_pulse_sum_min = 0.0f;
    m_pulse_sum_max = 10000000.f;
  }

  inline void set_pulse_sum_limits_negative()
  {
    m_pulse_sum_min = -10000000.f;
    m_pulse_sum_max = 0.0f;
  }

  inline void set_pulse_sum_limits_parent_ratio( const float ratio, const pulse_sum_normal *const parent )
  {
    m_pulse_parent = (pulse_sum_normal *)parent;
    m_pulse_limit_ratio = ratio;
    set_flag( PULSE_LIMIT_PARENT_RATIO, true );
    set_flag( PULSE_LIMIT_REACHED, false );
    set_flag( PULSE_CHAIN_LIMIT_REACHED, false );
  }

  inline const float get_pulse_sum_min() { return m_pulse_sum_min; }

  inline const float get_pulse_sum_max() { return m_pulse_sum_max; }

  inline const float get_pulse_sum() { return m_pulse_sum; }

  inline const float get_denom() { return m_denom; }

  inline const float get_cfm() { return m_cfm; }

  inline const float get_right_side() { return m_right_side; }

  inline const float get_limit_ratio() { return m_pulse_limit_ratio; }

  phys_vec3 &get_ud() { return m_ud; }

  const phys_vec3 get_relative_velocity_change_dir();
  const phys_vec3 get_relative_velocity();
  const float get_unclamped_pulse_sum();

  inline u32 is_pulse_limit_reached() { return get_flag( PULSE_LIMIT_REACHED ); }

  inline u32 is_pulse_chain_limit_reached() { return get_flag( PULSE_CHAIN_LIMIT_REACHED ); }

  inline const float get_std_active_limit_distance_eps() { return 3.4000001f; }

  inline const float get_std_max_penalty_restitution_vel() { return 170.0f; }

  inline pulse_sum_normal() {}

  friend class pulse_sum_wheel;
  friend class pulse_sum_constraint_solver;
  friend class rigid_body_constraint;
  friend class rigid_body_constraint_wheel;
};
#pragma pack( pop )

#include "pulse_sum_normal_inline.h"
