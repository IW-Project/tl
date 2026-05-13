#pragma once

#include "constraint_solver/pulse_sum_cache.h"
#include "rbc_def_base.h"
#include "phys_base.h"
#include "phys_math.h"

class rigid_body_constraint_point : public rigid_body_constraint
{
private:
  __declspec( align( 8 ) ) phys_vec3 m_b1_r_loc;
  phys_vec3 m_b2_r_loc;

  enum ps_cache_e
  {
    PSC_X = 0,
    PSC_Y = 1,
    PSC_Z = 2,
    NUM_PSC = 3
  };

  pulse_sum_cache m_ps_cache[NUM_PSC];
  float m_stress;
  float m_spring_k;
  float m_damp_k;
  bool m_spring_enabled;

public:
  rigid_body_constraint_point()
  {
    m_stress = 0.0f;
    m_spring_k = 0.0f;
    m_damp_k = 0.0f;
    m_spring_enabled = false;
  }

  void set( phys_vec3 &b1_r_loc, phys_vec3 &b2_r_loc );

  void set_spring_params( const bool spring_enabled, const float spring_k, const float damp_k )
  {
    m_spring_enabled = spring_enabled;
    m_spring_k = spring_k;
    m_damp_k = damp_k;
  }

  const phys_vec3 get_b1_r_loc() const { return m_b1_r_loc; }

  const phys_vec3 get_b2_r_loc() const { return m_b2_r_loc; }

  void setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t );

  float get_stress() const { return m_stress; }

  void zero_pulse_sums()
  {
    for ( int i = 0; i < NUM_PSC; ++i )
    {
      m_ps_cache[i].zero_pulse_sum();
    }
  }

  void epilog_vel_constraint( const float delta_t );
};

#pragma pack( push, 16 )

class ALIGN(16) rigid_body_constraint_hinge : public rigid_body_constraint
{
private:
  __declspec( align( 8 ) ) phys_vec3 m_b1_r_loc;
  phys_vec3 m_b2_r_loc;
  phys_vec3 m_b1_axis_loc;
  phys_vec3 m_b2_axis_loc;
  phys_vec3 m_b1_a1_loc;
  phys_vec3 m_b1_a2_loc;
  phys_vec3 m_b1_ref_loc;
  phys_vec3 m_b2_ref_min_loc;
  phys_vec3 m_b2_ref_max_loc;
  float m_damp_k;
  unsigned int m_flags;

  enum ps_cache_e
  {
    PSC_X = 0,
    PSC_Y = 1,
    PSC_Z = 2,
    PSC_DAMP = 3,
    PSC_A1 = 4,
    PSC_A2 = 5,
    PSC_MIN_LIMIT = 6,
    PSC_MAX_LIMIT = 7,
    NUM_PSC = 8
  };

  pulse_sum_cache m_ps_cache[NUM_PSC];

  enum flags_e
  {
    HINGE_MIN_LIMIT_ACTIVE = 1,
    HINGE_MAX_LIMIT_ACTIVE = 2,
    PRIVATE_FLAGS_END = 2
  };

  enum public_flags_e
  {
    NO_HINGE_LIMITS = 4,
    SNIDER_STYLE_HINGE = 8,
    NO_POINT_TO_POINT = 16
  };

public:
  void set_flag( const int f, const u32 b )
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

  const u32 get_flag( const int f ) { return f & m_flags; }

  void set( phys_vec3 &b1_r_loc,
            phys_vec3 &b2_r_loc,
            phys_vec3 &b1_axis_loc,
            phys_vec3 &b2_axis_loc,
            phys_vec3 &b1_ref_loc,
            phys_vec3 &b2_ref_loc,
            const float theta_min,
            const float theta_max,
            const float damp_k );

  void set_damp_k( const float damp_k ) { m_damp_k = damp_k; }

  void zero_pulse_sums() override
  {
    for ( int i = 0; i < NUM_PSC; ++i )
    {
      m_ps_cache[i].zero_pulse_sum();
    }
  }

  void setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t ) override;
};

#pragma pack( pop )

#pragma pack( push, 16 )

class ALIGN(16) rigid_body_constraint_distance : public rigid_body_constraint
{
private:
  phys_vec3 m_b1_r_loc;
  phys_vec3 m_b2_r_loc;
  float m_min_distance;
  float m_max_distance;
  float m_next_max_distance;
  float m_max_distance_vel;
  float m_damp_coef;
  unsigned int m_flags;
  void set_flag( const unsigned int f, const int b );
  const int get_flag( const unsigned int );

  enum flags_e
  {
    FLAG_ENABLE = 1,
    FLAG_B2_R_IS_DAMP_POINT = 2
  };

  enum ps_cache_e
  {
    PSC_MAX_DIST = 0,
    PSC_MIN_DIST = 1,
    PSC_DAMP = 2,
    NUM_PSC = 3
  };

  pulse_sum_cache m_ps_cache_list[NUM_PSC];

public:
  void set( phys_vec3 &b1_r_loc, phys_vec3 &b2_r_loc, const float min_distance, const float max_distance );

  void set_next_max_distance( const float next_max_distance ) { m_next_max_distance = next_max_distance; }

  void set_damp_coef( const float damp_coef ) { m_damp_coef = damp_coef; }

  const float get_damp_coef() { return m_damp_coef; }

  void set_b1_r_is_damp_point();
  void set_b2_r_is_damp_point();

  void set_enable( const bool enable )
  {
    if ( enable )
    {
      set_flag( FLAG_ENABLE, 1 );
    }
    else
    {
      set_flag( FLAG_ENABLE, 0 );
    }
  }

  const int get_enable() { return get_flag( FLAG_ENABLE ); }

  const phys_vec3 get_b1_r_loc() { return m_b1_r_loc; }

  const phys_vec3 get_b2_r_loc() { return m_b2_r_loc; }

  const float get_min_distance() { return m_min_distance; }

  const float get_max_distance() { return m_max_distance; }

  void zero_pulse_sums() override
  {
    for ( int i = 0; i < NUM_PSC; ++i )
    {
      m_ps_cache_list[i].zero_pulse_sum();
    }
  }

  void setup_constraint( pulse_sum_constraint_solver *, const float ) override;
  void outer_prolog_update( const float delta_t ) override;
  void inner_update( const float delta_t ) override;
  void outer_epilog_update( const float delta_t ) override;
};

class ALIGN(16) rigid_body_constraint_angular_actuator : public rigid_body_constraint
{
  phys_mat44 m_target_mat;
  phys_mat44 m_next_target_mat;
  phys_vec3 m_a_vel;
  float m_power;
  float m_power_scale;
  bool m_enabled;
  enum ps_cache_e
  {
    PSC_X = 0,
    PSC_Y = 1,
    PSC_Z = 2,
    NUM_PSC = 3
  };
  pulse_sum_cache m_ps_cache_list[NUM_PSC];

public:
  void set( float power, phys_mat44 &target_mat );
  void set_next_target( phys_mat44 &next_target_mat ) { m_next_target_mat = next_target_mat; }
  void set_power( const float p ) { m_power = p; }
  void set_power_scale( const float s ) { m_power_scale = s; }
  void set_enabled( bool enabled ) { m_enabled = enabled; }
  bool is_enabled() const { return m_enabled; }

  void zero_pulse_sums()
  {
    for ( int i = 0; i < NUM_PSC; ++i )
    {
      m_ps_cache_list[i].zero_pulse_sum();
    }
  }

  void setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t );
  void outer_prolog_update( const float delta_t );
  void inner_update( const float delta_t );
  void outer_epilog_update( const float delta_t );
};

#pragma pack( pop )

#pragma pack( push, 16 )

class ALIGN(16) rigid_body_constraint_upright : public rigid_body_constraint
{
private:
  __declspec( align( 8 ) ) phys_vec3 m_b1_forward_axis_loc;
  phys_vec3 m_b1_right_axis_loc;
  phys_vec3 m_b1_up_axis_loc;
  phys_vec3 m_b1_lean_axis_loc;
  phys_vec3 m_b2_up_axis_loc;
  phys_vec3 m_last_t_vel;
  phys_vec3 m_last_a_vel;
  float m_avg_side_force;
  float m_avg_normal_force;
  float m_lean_angle_calc_delta_t;
  float m_lean_angle;
  float m_lean_angle_multiplier;
  float m_max_lean_angle;
  float m_moving_average_total_time;
  bool m_enabled;

  enum ps_cache_e
  {
    PSC_FORWARD_AXIS = 0,
    NUM_PSC = 1
  };

  pulse_sum_cache m_ps_cache_list[NUM_PSC];

public:
  void
  set( phys_vec3 &, phys_vec3 &, phys_vec3 &, phys_vec3 &, const float, const float, const float, const float, const bool, const bool );

  inline void set_last_t_vel( phys_vec3 &last_t_vel ) { m_last_t_vel = last_t_vel; }

  inline const phys_vec3 &get_last_t_vel() const { return m_last_t_vel; }

  inline void set_last_a_vel( phys_vec3 &last_a_vel ) { m_last_a_vel = last_a_vel; }

  inline const phys_vec3 &get_last_a_vel() const { return m_last_a_vel; }

  inline void set_avg_side_force( const float avg_side_force ) { m_avg_side_force = avg_side_force; }

  inline const float get_avg_side_force() const { return m_avg_side_force; }

  inline void set_avg_normal_force( const float avg_normal_force ) { m_avg_normal_force = avg_normal_force; }

  inline const float get_avg_normal_force() const { return m_avg_normal_force; }

  inline void set_lean_angle_calc_delta_t( const float lean_angle_calc_delta_t ) { m_lean_angle_calc_delta_t = lean_angle_calc_delta_t; }

  inline const float get_lean_angle_calc_delta_t() const { return m_lean_angle_calc_delta_t; }

  inline void set_b1_lean_axis_loc( phys_vec3 &b1_lean_axis_loc ) { m_b1_lean_axis_loc = b1_lean_axis_loc; }

  inline void set_b2_up_axis_loc( phys_vec3 &b2_up_axis_loc ) { m_b2_up_axis_loc = b2_up_axis_loc; }

  inline void set_lean_angle_multiplier( const float lean_angle_multiplier ) { m_lean_angle_multiplier = lean_angle_multiplier; }

  inline float get_lean_angle_multiplier() const { return m_lean_angle_multiplier; }

  inline void set_moving_average_total_time( const float moving_average_total_time )
  {
    m_moving_average_total_time = moving_average_total_time;
  }

  inline float get_moving_average_total_time() const { return m_moving_average_total_time; }

  inline void set_max_lean_angle( const float max_lean_angle ) { m_max_lean_angle = max_lean_angle; }

  inline const float get_max_lean_angle() const { return m_max_lean_angle; }

  inline const float get_lean_angle() const { return m_lean_angle; }

  inline const phys_vec3 &get_b1_forward_axis_loc() const { return m_b1_forward_axis_loc; }

  inline const phys_vec3 &get_b1_right_axis_loc() const { return m_b1_right_axis_loc; }

  inline const phys_vec3 &get_b1_up_axis_loc() const { return m_b1_up_axis_loc; }

  inline const phys_vec3 &get_b1_lean_axis_loc() const { return m_b1_lean_axis_loc; }

  inline const phys_vec3 &get_b2_up_axis_loc() const { return m_b2_up_axis_loc; }

  const float calc_current_lean_angle() const;
  const phys_vec3 calc_b1_lean_axis_loc( const float lean_angle ) const;
  void update_lean_axis( phys_vec3 &b1_lean_center, phys_vec3 &b1_lean_axis_loc );

  inline void set_enabled( const bool enabled ) { m_enabled = enabled; }

  inline bool is_enabled() { return m_enabled; }

  inline void zero_pulse_sums()
  {
    for ( int i = 0; i < NUM_PSC; ++i )
    {
      m_ps_cache_list[i].zero_pulse_sum();
    }
  }

  void setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t );
  void epilog_vel_constraint( const float delta_t );
};

#pragma pack( pop )
