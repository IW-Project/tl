#pragma once

#include "phys_math.h"
#include "phys_mem.h"
#include "phys_base.h"
#include "pulse_sum_base.h"
#include "pulse_sum_cache.h"

class pulse_sum_angular : public phys_link_list_base<pulse_sum_angular>
{
  enum flags_e
  {
    APPLY_MIN_FACTOR = 1
  };

private:
  void set_flag( const unsigned int f, const bool b );
  const unsigned int get_flag( const unsigned int f );
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
  unsigned int m_flags;
  pulse_sum_node *m_b1;
  pulse_sum_node *m_b2;
  pulse_sum_cache *m_pulse_sum_cache;
  void calc_abs();
  const float get_vel();
  const float get_pos();
  phys_vec3 &object_vel_();
  phys_vec3 &object_col_pt_();
  void set_object_vel( const phys_vec3 &object_vel );
  void set_object_col_pt( const phys_vec3 &object_col_p );
  const float clamp_pulse_sum( const float ps );
  const float get_objective();
  void apply( float &s_ );
  void project();
  void SOLVER_apply_relaxation( float &error_sq );
  void SOLVER_solver_prolog( const float delta_t );
  void SOLVER_solver_intermediate( const float delta_t );

public:
  void set( rigid_body *const b1, phys_vec3 &b1_r, rigid_body *const b2, phys_vec3 &b2_r, phys_vec3 &ud, pulse_sum_cache *const ps_cache );
  void set_pulse_sum_limits_unbounded();
  void set_pulse_sum_limits_cone( const float pulse_sum_max );
  void set_pulse_sum_limits_negative();
  void set_pulse_sum_limits( float min, float max );
  void setup_vel_bi_standard( const float delta_t );
  void setup_vel_uni_standard( const float delta_t, const float max_penalty_restitution_vel );
  void setup_vel_uni_standard_1( const float, const float, const float );
  void setup_vel_simple();
  void setup_vel_custom_pos( const float, const float, const float, const float );
  void setup_vel_custom( const float erp, const float cfm, const float delta_t );
  void setup_vel_custom_big_dirt( const float erp, const float cfm, const float delta_t );
  void setup_vel_custom_right_side( const float right_side, const float cfm );
  void add_right_side( const float r );
  static const float get_std_active_limit_angle_eps();
  static const float get_std_active_limit_sin_angle_eps();
  static const float get_std_max_penalty_restitution_vel();
  pulse_sum_angular();

  friend class pulse_sum_constraint_solver;
};