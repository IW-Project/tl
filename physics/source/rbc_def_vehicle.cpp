#include "rbc_defs/rbc_def_vehicle.h"

#include "constraint_solver/pulse_sum_constraint_solver.h"
#include "physics_system_internal.h"

namespace
{
  float lerp_float( const float tgt, const float cur, const float rate, const float delta_t )
  {
    const float delta = tgt - cur;
    const float step = ( delta <= 0.0f ? -rate : rate ) * delta_t;

    if ( math::Abs( delta ) > 0.05f )
    {
      if ( math::Abs( step ) <= math::Abs( delta ) )
      {
        return cur + step;
      }
    }

    return tgt;
  }
} // namespace

void rigid_body_constraint_wheel::set( const phys_vec3 &wheel_center_loc,
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
                                       const float side_fric_max )
{
  m_b1_wheel_center_loc = wheel_center_loc;
  m_b1_suspension_dir_loc = suspension_dir_loc;
  m_b1_wheel_axis_loc = wheel_axis_loc;
  m_wheel_state = 1;
  m_wheel_flags = 0;
  m_wheel_radius = wheel_radius;
  m_fwd_fric_k = fwd_fric_k;
  m_side_fric_k = side_fric_k;
  m_side_fric_max = side_fric_max;
  m_suspension_stiffness_k = suspension_stiffness_k;
  m_suspension_damp_k = suspension_damp_k;
  m_hard_limit_dist = hard_limit_dist;
  m_roll_stability_factor = roll_stability_factor;
  m_pitch_stability_factor = pitch_stability_factor;
  m_braking_factor_k = 0.0f;
  m_turning_radius_ratio_max_speed = 1.0f;
  m_turning_radius_ratio_accel = 1.0f;
  m_wheel_vel = 0.0f;
  m_wheel_pos = 0.0f;
  m_wheel_fwd = 0.0f;
  m_wheel_displaced_center_dist = 0.0f;
  PHYS_ASSERT_UNIT( m_b1_suspension_dir_loc );
  PHYS_ASSERT_UNIT( m_b1_wheel_axis_loc );
  PHYS_ASSERT_ORTHOGONAL( m_b1_suspension_dir_loc, m_b1_wheel_axis_loc );
}

void rigid_body_constraint_wheel::get_wheel_collide_segment( const phys_mat44 &b1_mat, phys_vec3 *const p0, phys_vec3 *const p1 ) const
{
  PHYS_ASSERT_ALIGNED( *p0 );
  PHYS_ASSERT_ALIGNED( *p1 );

  // Transform wheel center to world space (rotation + translation)
  phys_vec3 wheel_center = phys_multiply( b1_mat, get_wheel_center_loc() ) + b1_mat.GetW();

  // Transform suspension direction to world space (rotation only)
  phys_vec3 susp_dir = phys_multiply( b1_mat, get_suspension_dir_loc() );

  // Scale suspension direction by wheel radius
  phys_vec3 scaled_susp = m_wheel_radius * susp_dir;

  // p0 is the top of the wheel (center - scaled suspension direction)
  *p0 = wheel_center - scaled_susp;

  // p1 is the bottom of the wheel (center + scaled suspension direction)
  *p1 = wheel_center + scaled_susp;
}

void rigid_body_constraint_wheel::set_no_collision()
{
  set_wheel_flag( WHEEL_FLAG_IS_COLLIDING, 0 );
  b2 = NULL;
}

void rigid_body_constraint_wheel::set_collision( rigid_body *const rb, const phys_vec3 &hitp_loc, const phys_vec3 &hitn_loc )
{
  set_wheel_flag( WHEEL_FLAG_IS_COLLIDING, 1 );
  b2 = rb;
  m_b2_hitp_loc = hitp_loc;
  m_b2_hitn_loc = hitn_loc;
}

void rigid_body_constraint_wheel::set_wheel_state_accelerating( const float desired_speed_k, const float acceleration_factor_k )
{
  m_wheel_state = WHEEL_STATE_ACCELERATING;
  m_desired_speed_k = desired_speed_k;
  m_acceleration_factor_k = acceleration_factor_k;
}

void rigid_body_constraint_wheel::get_wheel_state_accelerating( float &desired_speed_k, float &acceleration_factor_k )
{
  desired_speed_k = m_desired_speed_k;
  acceleration_factor_k = m_acceleration_factor_k;
}

void rigid_body_constraint_wheel::set_wheel_state_braking( const float wheel_state_braking )
{
  m_wheel_state = WHEEL_STATE_BRAKING;
  m_braking_factor_k = wheel_state_braking;
}

void rigid_body_constraint_wheel::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  // Clear sliding flag and reset state
  m_wheel_flags &= ~WHEEL_FLAG_IS_SLIDING;
  m_wheel_fwd = 0.0f;
  m_ps_suspension = NULL;
  m_ps_side_fric = NULL;
  m_ps_fwd_fric = NULL;

  // Only setup constraints if wheel is colliding
  if ( !( m_wheel_flags & WHEEL_FLAG_IS_COLLIDING ) )
    return;

  // Transform suspension direction to world space
  phys_vec3 suspension_dir_world = phys_multiply( b1->get_mat(), get_suspension_dir_loc() );

  // Transform wheel center to world space (local offset from body center)
  phys_vec3 b1_wheel_center_loc = phys_multiply( b1->get_mat(), get_wheel_center_loc() );

  // Calculate wheel displaced position (center + suspension_dir * wheel_radius)
  phys_vec3 b1_wheel_displaced_loc = b1_wheel_center_loc + m_wheel_radius * suspension_dir_world;

  // Transform hit point from b2 local to world
  phys_vec3 b2_hitp_world = phys_multiply( b2->get_mat(), m_b2_hitp_loc );

  // Calculate b1 and b2 world positions
  phys_vec3 b1_r_world = b1->get_mat().GetW() + b1_wheel_displaced_loc;
  phys_vec3 b2_r_world = b2->get_mat().GetW() + b2_hitp_world;

  // Calculate displacement vector from b2 to b1
  phys_vec3 displacement = b1_r_world - b2_r_world;

  // Calculate penetration depth along suspension direction
  float penetration_depth = phys_dot( displacement, suspension_dir_world );

  // Check if hard limit is active
  float hard_limit_threshold = m_hard_limit_dist - 3.4f;
  if ( hard_limit_threshold <= penetration_depth )
  {
    m_wheel_flags |= WHEEL_FLAG_HARD_LIMIT_ACTIVE;
  }
  else
  {
    m_wheel_flags &= ~WHEEL_FLAG_HARD_LIMIT_ACTIVE;
  }

  // Recalculate positions for constraint setup
  b1_r_world = b1->get_mat().GetW() + b1_wheel_displaced_loc;
  b2_r_world = b2->get_mat().GetW() + b2_hitp_world;
  displacement = b1_r_world - b2_r_world;

  // Setup hard limit constraint if active
  if ( m_wheel_flags & WHEEL_FLAG_HARD_LIMIT_ACTIVE )
  {
    // Transform hit normal to world space
    phys_vec3 hitn_world = phys_multiply( b2->get_mat(), m_b2_hitn_loc );
    phys_vec3 normal_dir = -hitn_world;

    // Create pulse sum for hard limit
    pulse_sum_normal *ps_hard_limit = psys->create_pulse_sum_normal();

    // Calculate hard limit position
    phys_vec3 b1_hard_limit_loc = b1_wheel_displaced_loc - m_hard_limit_dist * suspension_dir_world;

    // Setup hard limit constraint
    ps_hard_limit->set( b1, b1_hard_limit_loc, b2, b2_hitp_world, normal_dir, &m_ps_cache_list[PSC_HARD_LIMIT], PHYS_ZERO_VEC );

    ps_hard_limit->set_pulse_sum_limits_negative();
    ps_hard_limit->setup_vel_uni_standard( delta_t, ps_hard_limit->get_std_max_penalty_restitution_vel() );
  }

  // Transform hit normal to world and negate
  phys_vec3 hitn_world = phys_multiply( b2->get_mat(), m_b2_hitn_loc );
  phys_vec3 normal_dir = -hitn_world;

  // Project out component of displacement along normal
  float disp_along_normal = phys_dot( displacement, normal_dir );
  phys_vec3 tangential_displacement = displacement - disp_along_normal * normal_dir;

  b1_wheel_displaced_loc -= disp_along_normal * normal_dir;
  b2_hitp_world -= disp_along_normal * normal_dir;

  // Calculate roll stability correction using the getter result
  phys_vec3 roll_stability_correction = ( -m_roll_stability_factor ) * suspension_dir_world;

  // Create pulse sum wheel (contains suspension + side/fwd friction)
  pulse_sum_wheel *ps_wheel = psys->create_pulse_sum_wheel();
  ps_wheel->m_side = NULL;
  ps_wheel->m_fwd = NULL;

  // Setup suspension constraint
  m_ps_suspension = ps_wheel->get_suspension();
  m_ps_suspension
      ->set( b1, b1_wheel_displaced_loc, b2, b2_hitp_world, normal_dir, &m_ps_cache_list[PSC_SUSPENSION], roll_stability_correction );

  // Setup suspension constraints limits
  m_ps_suspension->set_pulse_sum_limits_negative();

  // Setup suspension spring/damper
  float erp, cfm;
  pulse_sum_calc_spring_params( m_suspension_stiffness_k, m_suspension_damp_k, delta_t, &erp, &cfm );
  m_ps_suspension->setup_vel_custom( erp, cfm, delta_t );

  // Transform wheel axis to world space
  phys_vec3 wheel_axis_world = phys_multiply( b1->get_mat(), get_wheel_axis_loc() );

  // Create rotation matrix from normal to wheel axis
  phys_mat44 rot_mat;
  make_rotate( &rot_mat, suspension_dir_world, normal_dir );

  // Transform wheel axis through rotation
  phys_vec3 side_dir = phys_multiply( rot_mat, wheel_axis_world );

  // Create and setup side friction constraint
  m_ps_side_fric = psys->create_pulse_sum_normal();
  m_ps_side_fric
      ->set( b1, b1_wheel_displaced_loc, b2, b2_hitp_world, side_dir, &m_ps_cache_list[PSC_SIDE_FRIC], roll_stability_correction );

  // Inline setup_vel_simple()
  m_ps_side_fric->setup_vel_simple();

  // Calculate forward direction (perpendicular to side and normal)
  phys_vec3 fwd_dir = phys_cross( side_dir, normal_dir );
  fwd_dir = -fwd_dir;

  // Get relative velocity at contact
  phys_vec3 rel_vel = m_ps_suspension->get_relative_velocity();

  // Calculate wheel velocity
  m_wheel_vel = phys_dot( rel_vel, fwd_dir ) / m_wheel_radius;

  // Check if wheel needs power constraint (accelerating or braking with significant force)
  bool needs_power_constraint = false;
  if ( m_wheel_state == WHEEL_STATE_ACCELERATING && m_acceleration_factor_k >= 0.0001f )
  {
    needs_power_constraint = true;
  }
  else if ( m_wheel_state == WHEEL_STATE_BRAKING && m_braking_factor_k >= 0.0001f )
  {
    needs_power_constraint = true;
  }

  if ( !needs_power_constraint )
  {
    // No power - just friction coupling between side and suspension
    m_ps_side_fric->set_pulse_sum_limits_parent_ratio( m_side_fric_k, m_ps_suspension );
    return;
  }

  // Set side friction to independent limits for powered wheel
  m_ps_side_fric->set_pulse_sum_limits_unbounded();

  // Create forward friction constraint
  ps_wheel->m_side = m_ps_side_fric;
  m_ps_fwd_fric = psys->create_pulse_sum_wheel_fwd( ps_wheel );

  // Calculate pitch stability correction
  phys_vec3 pitch_stability_correction = ( -m_pitch_stability_factor ) * suspension_dir_world;

  // Setup forward friction constraint
  m_ps_fwd_fric->set( b1, b1_wheel_displaced_loc, b2, b2_hitp_world, fwd_dir, &m_ps_cache_list[PSC_FWD_FRIC], pitch_stability_correction );

  if ( m_wheel_state == WHEEL_STATE_BRAKING )
  {
    // Braking mode - simple friction with braking bias
    m_ps_fwd_fric->setup_vel_simple();

    // Inline set_pulse_sum_limits_cone
    float braking_limit = m_braking_factor_k * delta_t;
    m_ps_fwd_fric->set_pulse_sum_limits_cone( braking_limit );
  }
  else // WHEEL_STATE_ACCELERATING
  {
    // Acceleration mode - driven wheel with speed target
    float desired_speed = m_desired_speed_k * m_turning_radius_ratio_max_speed;
    float accel_force = m_acceleration_factor_k * m_turning_radius_ratio_accel;

    // Calculate constraint CFM from acceleration force
    float wheel_radius_sq = m_wheel_radius * m_wheel_radius;
    float cfm_fwd = ( accel_force * delta_t ) / wheel_radius_sq;

    if ( cfm_fwd <= 0.000001f )
      cfm_fwd = 0.00001f;

    cfm_fwd = 1.0f / cfm_fwd;

    m_ps_fwd_fric->setup_vel_custom_right_side( m_wheel_radius * desired_speed, cfm_fwd );

    // Set pulse sum limits based on desired speed direction
    if ( desired_speed > 0.0001f )
    {
      m_ps_fwd_fric->set_pulse_sum_limits_positive();
    }
    else if ( desired_speed < -0.0001f )
    {
      m_ps_fwd_fric->set_pulse_sum_limits_negative();
    }
    else
    {
      m_ps_fwd_fric->set_pulse_sum_limits_unbounded();
    }
  }

  // Setup friction pyramid constraint between side and forward friction
  ps_wheel->set_side_fwd_ratios( m_side_fric_k, m_fwd_fric_k, m_side_fric_max );

  // Calculate wheel forward speed for output
  phys_vec3 fwd_vel_change = m_ps_fwd_fric->get_relative_velocity_change_dir();
  m_wheel_fwd = phys_dot( fwd_vel_change, fwd_dir );
}

void rigid_body_constraint_wheel::epilog_vel_constraint( const float delta_t )
{
  static float velocity_clamp = 0.5f;
  static float lr = 1000.0f;
  static float lr_0 = 1.0f;

  const float prev_wheel_displaced_center_dist = m_wheel_displaced_center_dist;
  const unsigned int prev_wheel_flags = m_wheel_flags;

  if ( m_wheel_flags & WHEEL_FLAG_IS_COLLIDING )
  {
    const phys_vec3 b1_suspension_dir = phys_multiply( b1->get_mat(), get_suspension_dir_loc() );
    const phys_vec3 b1_r = m_wheel_radius * b1_suspension_dir;
    const phys_vec3 b1_wheel_center = phys_multiply( b1->get_mat(), get_wheel_center_loc() );
    const phys_vec3 b2_r = phys_multiply( b2->get_mat(), m_b2_hitp_loc );
    const phys_vec3 b1_r_world = b1->get_mat().GetW() + ( b1_wheel_center + b1_r );
    const phys_vec3 b2_r_world = b2->get_mat().GetW() + b2_r;
    const phys_vec3 displacement = b1_r_world - b2_r_world;

    float ndist = phys_dot( displacement, b1_suspension_dir );
    if ( m_hard_limit_dist < ndist )
    {
      ndist = m_hard_limit_dist;
    }

    m_wheel_displaced_center_dist = ndist;
  }
  else
  {
    m_wheel_displaced_center_dist = 0.0f;
  }

  if ( m_wheel_displaced_center_dist < prev_wheel_displaced_center_dist )
  {
    float ndist = prev_wheel_displaced_center_dist - ( 51.0f * delta_t );
    if ( ndist < m_wheel_displaced_center_dist )
    {
      ndist = m_wheel_displaced_center_dist;
    }
    m_wheel_displaced_center_dist = ndist;
  }

  if ( m_ps_suspension )
  {
    m_wheel_normal_force = m_ps_cache_list[PSC_SUSPENSION].get_pulse_sum();
  }
  else
  {
    m_wheel_normal_force = 0.0f;
  }

  const float wheel_linear_speed = math::Abs( m_wheel_vel * m_wheel_radius );
  const bool apply_wheel_velocity = ( velocity_clamp < wheel_linear_speed );

  if ( m_ps_side_fric )
  {
    if ( m_ps_fwd_fric )
    {
      if ( m_ps_fwd_fric->is_pulse_chain_limit_reached() )
      {
        m_wheel_flags = prev_wheel_flags | WHEEL_FLAG_IS_SLIDING;
      }

      m_wheel_vel = ( m_ps_fwd_fric->get_unclamped_pulse_sum() * m_wheel_fwd / m_wheel_radius ) + m_wheel_vel;
    }
    else if ( m_ps_side_fric->is_pulse_limit_reached() )
    {
      m_wheel_flags = prev_wheel_flags | WHEEL_FLAG_IS_SLIDING;
    }
  }

  if ( apply_wheel_velocity )
  {
    m_wheel_pos = ( m_wheel_vel * delta_t ) + m_wheel_pos;
  }

  if ( ( m_wheel_flags & WHEEL_FLAG_IS_COLLIDING ) == 0 )
  {
    const float lerp_rate = ( m_braking_factor_k >= 50.0f ) ? lr : lr_0;
    m_wheel_vel = lerp_float( 0.0f, m_wheel_vel, lerp_rate, delta_t );
  }
}
