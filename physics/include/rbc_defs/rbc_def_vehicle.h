#pragma once

#include "phys_math.h"
#include "rbc_def_base.h"
#include "rbc_def_generic.h"

#include "constraint_solver/pulse_sum_normal.h"

class rigid_body_constraint_wheel : public rigid_body_constraint
{
private:
  friend class rbcint;

  enum internal_wheel_flags_e
  {
    WHEEL_FLAG_IS_COLLIDING = ( 1 << 0 ),
    WHEEL_FLAG_HARD_LIMIT_ACTIVE = ( 1 << 1 ),
    WHEEL_FLAG_IS_SLIDING = ( 1 << 2 )
  };

  unsigned char __align0[16];
  phys_vec3 m_b2_hitp_loc;
  phys_vec3 m_b2_hitn_loc;
  phys_vec3 m_b1_wheel_center_loc;
  phys_vec3 m_b1_suspension_dir_loc;
  phys_vec3 m_b1_wheel_axis_loc;
  float m_wheel_radius;
  float m_fwd_fric_k;
  float m_side_fric_k;
  float m_side_fric_max;
  float m_suspension_stiffness_k;
  float m_suspension_damp_k;
  float m_hard_limit_dist;
  float m_roll_stability_factor;
  float m_pitch_stability_factor;
  float m_turning_radius_ratio_max_speed;
  float m_turning_radius_ratio_accel;
  float m_desired_speed_k;
  float m_acceleration_factor_k;
  float m_braking_factor_k;
  float m_wheel_vel;
  float m_wheel_fwd;
  float m_wheel_pos;
  float m_wheel_displaced_center_dist;
  float m_wheel_normal_force;
  unsigned int m_wheel_state;
  unsigned int m_wheel_flags;

  inline void set_wheel_flag( internal_wheel_flags_e f, const unsigned int b ) { b ? m_wheel_flags |= f : m_wheel_flags &= ~f; }

  const unsigned int get_wheel_flag( internal_wheel_flags_e f ) const { return m_wheel_flags & f; }

  enum ps_cache_e
  {
    PSC_HARD_LIMIT = 0,
    PSC_SUSPENSION = 1,
    PSC_SIDE_FRIC = 2,
    PSC_FWD_FRIC = 3,
    NUM_PSC = 4
  };

  pulse_sum_cache m_ps_cache_list[NUM_PSC];
  pulse_sum_normal *m_ps_suspension;
  pulse_sum_normal *m_ps_side_fric;
  pulse_sum_normal *m_ps_fwd_fric;

public:
  enum wheel_state_e
  {
    WHEEL_STATE_ACCELERATING = 0,
    WHEEL_STATE_BRAKING = 1
  };

  enum wheel_flags_e
  {
    WHEEL_FLAG_HAS_TURNING = ( 1 << 3 ),
    WHEEL_FLAG_HAS_POWER = ( 1 << 4 ),
    WHEEL_FLAG_HAS_POWER_BRAKING = ( 1 << 5 ),
    WHEEL_FLAG_HAS_BRAKING = ( 1 << 6 ),
    WHEEL_FLAG_DISABLE_TURN_ON_BREAK = ( 1 << 7 )
  };

  inline void set_wheel_flag( wheel_flags_e f, const unsigned int b ) { b ? m_wheel_flags |= f : m_wheel_flags &= ~f; }

  const unsigned int get_wheel_flag( wheel_flags_e f ) const { return m_wheel_flags & f; }

  void set( const phys_vec3 &wheel_center_loc,
            const phys_vec3 &suspension_dir_loc,
            const phys_vec3 &wheel_axis_loc,
            const float wheel_radius,
            const float fwd_fric_k,
            const float side_fric_k,
            const float suspension_stiffness_k,
            const float suspension_damp_k,
            const float hard_limit_dist,
            const float roll_stability_factor,
            const float pitch_stability_factor,
            const float side_fric_max );

  inline void set_wheel_radius( const float wheel_radius ) { m_wheel_radius = wheel_radius; }

  inline void set_side_fric_k( const float side_fric_k ) { m_side_fric_k = side_fric_k; }

  inline void set_fwd_fric_k( const float fwd_fric_k ) { m_fwd_fric_k = fwd_fric_k; }

  inline void set_suspension_stiffness_k( float suspension_stiffness_k ) { m_suspension_stiffness_k = suspension_stiffness_k; }

  inline void set_suspension_damp_k( float suspension_damp_k ) { m_suspension_damp_k = suspension_damp_k; }

  inline void set_hard_limit_dist( const float hard_limit_dist ) { m_hard_limit_dist = hard_limit_dist; }

  inline const float get_wheel_radius() const { return m_wheel_radius; }

  inline float get_wheel_radius() { return m_wheel_radius; }

  inline const float get_side_fric_k() const { return m_side_fric_k; }

  inline const float get_fwd_fric_k() const { return m_fwd_fric_k; }

  inline const float get_suspension_stiffness_k() const { return m_suspension_stiffness_k; }

  inline const float get_suspension_damp_k() const { return m_suspension_damp_k; }

  void get_wheel_collide_segment( const phys_mat44 &b1_mat, phys_vec3 *const p0, phys_vec3 *const p1 ) const;
  void set_no_collision();
  void set_collision( rigid_body *const rb, const phys_vec3 &hitp_loc, const phys_vec3 &hitn_loc );

  inline const unsigned int get_wheel_is_colliding() const { return get_wheel_flag( WHEEL_FLAG_IS_COLLIDING ); }

  inline const unsigned int get_wheel_hard_limit_active() const { return get_wheel_flag( WHEEL_FLAG_HARD_LIMIT_ACTIVE ); }

  inline void set_wheel_axis_loc( const phys_vec3 &wheel_axis_loc, int ) { m_b1_wheel_axis_loc = wheel_axis_loc; }

  inline void set_wheel_axis_loc( const phys_vec3 &wheel_axis_loc ) { m_b1_wheel_axis_loc = wheel_axis_loc; }

  inline void set_turning_radius_ratio( const float turning_radius_ratio )
  {
    tlAssertMsg( false, "Turning radius ratio is no longer supported" );
  }

  inline void set_turning_radius_ratio_max_speed( const float turning_radius_ratio_max_speed )
  {
    m_turning_radius_ratio_max_speed = turning_radius_ratio_max_speed;
  }

  inline void set_turning_radius_ratio_accel( const float turning_radius_ratio_accel )
  {
    m_turning_radius_ratio_accel = turning_radius_ratio_accel;
  }

  inline void set_roll_stability_factor( const float roll_stability_factor ) { m_roll_stability_factor = roll_stability_factor; }

  const float get_roll_stability_factor() const { return m_roll_stability_factor; }

  inline void set_pitch_stability_factor( const float pitch_stability_factor ) { m_pitch_stability_factor = pitch_stability_factor; }

  const float get_pitch_stability_factor() const { return m_pitch_stability_factor; }

  void set_wheel_state_accelerating( const float desired_speed_k, const float acceleration_factor_k );
  void get_wheel_state_accelerating( float &desired_speed_k, float &acceleration_factor_k );
  void set_wheel_state_braking( const float wheel_state_braking );

  inline unsigned int get_wheel_state() { return m_wheel_state; }

  inline rigid_body *get_chassis() { return b1; }

  inline const rigid_body *get_chassis() const { return b1; }

  inline const phys_vec3 get_wheel_center_loc( int ) const { return m_b1_wheel_center_loc; }

  inline const phys_vec3 get_wheel_center_loc() const { return m_b1_wheel_center_loc; }

  inline const phys_vec3 get_suspension_dir_loc( int ) const { return m_b1_suspension_dir_loc; }

  inline const phys_vec3 get_suspension_dir_loc() const { return m_b1_suspension_dir_loc; }

  inline const phys_vec3 get_wheel_axis_loc( int ) const { return m_b1_wheel_axis_loc; }

  inline const phys_vec3 get_wheel_axis_loc() const { return m_b1_wheel_axis_loc; }

  inline const float get_hard_limit_dist() const { return m_hard_limit_dist; }

  inline const float get_displaced_center_dist() const { return m_wheel_displaced_center_dist; }

  inline const phys_vec3 get_hitp_loc() const { return m_b2_hitp_loc; }

  inline const phys_vec3 get_hitn_loc() const { return m_b2_hitn_loc; }

  inline const float get_wheel_vel() const { return m_wheel_vel; }

  inline const float get_wheel_pos() const { return m_wheel_pos; }

  inline void set_wheel_pos( const float wheel_pos ) { m_wheel_pos = wheel_pos; }

  inline const phys_vec3 get_wheel_displaced_center_loc() const
  {
    return m_b1_wheel_center_loc - m_wheel_displaced_center_dist * m_b1_suspension_dir_loc;
  }

  inline const float get_wheel_normal_force() const { return m_wheel_normal_force; }

  inline const unsigned int get_wheel_is_sliding() const { return get_wheel_flag( WHEEL_FLAG_IS_SLIDING ); }

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

class rb_vehicle_model
{
  enum rb_vehicle_model_flags_e
  {
    FLAG_IS_POWER_BRAKING = ( 1 << 0 ),
    FLAG_IS_BRAKING = ( 1 << 1 ),
    FLAG_IS_FORWARD_ACCELERATION = ( 1 << 2 ),
    FLAG_IS_REVERSE_ACCELERATION = ( 1 << 3 ),
    FLAG_IS_COASTING = ( 1 << 4 )
  };

  phys_vec3 m_right_dir_loc;
  phys_vec3 m_forward_dir_loc;

  phys_static_array<rigid_body_constraint_wheel *, 4> m_wheels;

  float m_desired_speed_factor;
  float m_acceleration_factor;
  float m_power_braking_factor;
  float m_braking_factor;
  float m_coasting_factor;

  float m_reference_wheel_radius;

  float m_steer_factor;
  float m_steer_factor_offset;
  float m_steer_current_angle;
  float m_steer_max_angle;
  float m_steer_speed;
  phys_vec3 m_steer_front_pt_loc;
  float m_steer_front_back_length;

  unsigned int m_state_flags;

  rigid_body_constraint_upright *m_rbc_upright;

  void set_flag( rb_vehicle_model_flags_e f, const unsigned int b ) { b ? m_state_flags |= f : m_state_flags &= ~f; }

  const unsigned int get_flag( rb_vehicle_model_flags_e f ) const { return m_state_flags & f; }

public:
  void add_wheel( rigid_body_constraint_wheel *rbc_wheel ) { *m_wheels.add() = rbc_wheel; }

  rigid_body_constraint_wheel *get_wheel( const int i ) { return m_wheels[i]; }

  const rigid_body_constraint_wheel *get_wheel( const int i ) const { return m_wheels[i]; }

  void remove_wheels() { m_wheels.remove_all(); }

  const int get_wheel_count() const { return m_wheels.get_count(); }

  void set( const float power_braking_factor,
            const float braking_factor,
            const float desired_speed_factor,
            const float acceleration_factor,
            const float coasting_factor,
            const float reference_wheel_radius,
            const float steer_max_angle,
            const float steer_speed,
            const phys_vec3 &steer_front_pt_loc,
            const float steer_front_back_length,
            const phys_vec3 &right_dir_loc,
            const phys_vec3 &forward_dir_loc )
  {
    tlAssertMsg( false, "unimplemented" );
  }

  void set_rbc_upright( rigid_body_constraint_upright *rbc_upright ) { m_rbc_upright = rbc_upright; }

  rigid_body_constraint_upright *get_rbc_upright() { return m_rbc_upright; }

  const float calc_theoretical_max_lean_angle() const;

  void set_power_braking_factor( const float power_braking_factor ) { m_power_braking_factor = power_braking_factor; }

  void set_braking_factor( const float braking_factor ) { m_braking_factor = braking_factor; }

  void set_acceleration_factor( const float acceleration_factor ) { m_acceleration_factor = acceleration_factor; }

  void set_desired_speed_factor( const float desired_speed_factor ) { m_desired_speed_factor = desired_speed_factor; }

  void set_coasting_factor( const float coasting_factor ) { m_coasting_factor = coasting_factor; }

  const float get_power_braking_factor() const { return m_power_braking_factor; }

  const float get_braking_factor() const { return m_braking_factor; }

  const float get_acceleration_factor() const { return m_acceleration_factor; }

  const float get_desired_speed_factor() const { return m_desired_speed_factor; }

  const float get_coasting_factor() const { return m_coasting_factor; }

  const phys_vec3 get_steer_front_pt_loc() const { return m_steer_front_pt_loc; }

  const phys_vec3 get_right_dir_loc() const { return m_right_dir_loc; }

  const phys_vec3 get_forward_dir_loc() const { return m_forward_dir_loc; }

  void set_power_braking( const unsigned int b ) { set_flag( FLAG_IS_POWER_BRAKING, b ); }

  void set_braking( const unsigned int b ) { set_flag( FLAG_IS_BRAKING, b ); }

  void set_forward_acceleration( const unsigned int b ) { set_flag( FLAG_IS_FORWARD_ACCELERATION, b ); }

  void set_reverse_acceleration( const unsigned int b ) { set_flag( FLAG_IS_REVERSE_ACCELERATION, b ); }

  void set_coasting( const unsigned int b ) { set_flag( FLAG_IS_COASTING, b ); }

  const unsigned int get_power_braking() const { return get_flag( FLAG_IS_POWER_BRAKING ); }

  const unsigned int get_braking() const { return get_flag( FLAG_IS_BRAKING ); }

  const unsigned int get_forward_acceleration() const { return get_flag( FLAG_IS_FORWARD_ACCELERATION ); }

  const unsigned int get_reverse_acceleration() const { return get_flag( FLAG_IS_REVERSE_ACCELERATION ); }

  const unsigned int get_coasting() const { return get_flag( FLAG_IS_COASTING ); }

  void set_steer_factor( const float steer_factor ) { m_steer_factor = steer_factor; }

  void set_steer_factor_offset( const float steer_factor_offset ) { m_steer_factor_offset = steer_factor_offset; }

  void set_steer_max_angle( const float steer_max_angle ) { m_steer_max_angle = steer_max_angle; }

  void set_steer_speed( const float steer_speed ) { m_steer_speed = steer_speed; }

  const float get_steer_factor() const { return m_steer_factor; }

  const float get_steer_factor_offset() const { return m_steer_factor_offset; }

  const float get_steer_max_angle() const { return m_steer_max_angle; }

  const float get_steer_speed() const { return m_steer_speed; }

  void update_braking_and_acceleration( const float delta_t ) { tlAssertMsg( false, "unimplemented" ); }

  void update_steering( const float delta_t ) { tlAssertMsg( false, "unimplemented" ); }

  void update_upright_constraint( const float delta_t ) { tlAssertMsg( false, "unimplemented" ); }

  void get_wheel_matrix( const int i, phys_mat44 *mat ) const { tlAssertMsg( false, "unimplemented" ); }
};
