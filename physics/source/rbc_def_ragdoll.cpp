#include "rbc_defs/rbc_def_ragdoll.h"

#include "constraint_solver/pulse_sum_constraint_solver.h"
#include "rigid_body_internal.h"

void ragdoll_joint_limit_info::set( phys_vec3 &b1_ud_loc, const float theta_limit )
{
  tlAssert( theta_limit > 0.0f );
  m_b1_ud_loc = phys_Unitize( b1_ud_loc );
  m_b1_ud_limit_co_ = cosf( theta_limit );
  m_b1_ud_limit_si_ = sinf( theta_limit );

  float a = 0.0f;
  float b = theta_limit - pulse_sum_angular::get_std_active_limit_angle_eps();
  m_b1_ud_active_limit_co_ = cosf( tl_min( a, b ) );
  PHYS_ASSERT_UNIT( m_b1_ud_loc );
}

void ragdoll_joint_limit_info::set_b1_ud_loc( phys_vec3 &b1_ud_loc )
{
  m_b1_ud_loc = phys_Unitize( b1_ud_loc );
  PHYS_ASSERT_UNIT( m_b1_ud_loc );
}

void ragdoll_joint_limit_info::set_theta_limit( const float theta_limit )
{
  tlAssert( theta_limit > 0.0f );
  m_b1_ud_limit_co_ = cosf( theta_limit );
  m_b1_ud_limit_si_ = sinf( theta_limit );

  float a = 0.0f;
  float b = theta_limit - pulse_sum_angular::get_std_active_limit_angle_eps();
  m_b1_ud_active_limit_co_ = cosf( tl_min( a, b ) );
}

void rigid_body_constraint_ragdoll::set_flag( const unsigned int f, const bool b )
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

const unsigned int rigid_body_constraint_ragdoll::get_flag( const unsigned int f )
{
  return ( m_flags & f );
}

void rigid_body_constraint_ragdoll::setup_hinge( pulse_sum_constraint_solver *psys,
                                                 phys_vec3 &b1_ref,
                                                 phys_vec3 &b2_axis,
                                                 const float delta_t )
{
  const float active_limit_sin_angle_eps = pulse_sum_angular::get_std_active_limit_sin_angle_eps();
  const float std_max_penalty_restitution_vel = pulse_sum_angular::get_std_max_penalty_restitution_vel();

  // Transform m_b2_ref_min_loc from body2's local space to world space
  phys_vec3 b2_ref_min_world = rbint::multiply( b2, m_b2_ref_min_loc );

  // Compute cross product: b1_ref x b2_ref_min_world
  phys_vec3 cross_result = phys_cross( b1_ref, b2_ref_min_world );

  // Compute dot product: b2_axis . cross_result
  float dot_product = phys_dot( b2_axis, cross_result );

  // Create minimum angle limit constraint if above threshold
  if ( dot_product >= -active_limit_sin_angle_eps )
  {
    phys_vec3 neg_b2_axis = -b2_axis;

    pulse_sum_angular *ps_min =
        psys->create_pulse_sum_angular( b1, b1_ref, b2, b2_ref_min_world, neg_b2_axis, &m_ps_cache_list[PSC_MIN_LIMIT] );

    ps_min->set_pulse_sum_limits_negative();
    ps_min->setup_vel_uni_standard( delta_t, std_max_penalty_restitution_vel );
  }

  // Transform m_b2_ref_max_loc from body2's local space to world space
  phys_vec3 b2_ref_max_world = rbint::multiply( b2, m_b2_ref_max_loc );

  // Compute cross product: b1_ref x b2_ref_max_world
  cross_result = phys_cross( b1_ref, b2_ref_max_world );

  // Compute dot product: b2_axis . cross_result
  dot_product = phys_dot( b2_axis, cross_result );

  // Create maximum angle limit constraint if below threshold
  if ( dot_product <= active_limit_sin_angle_eps )
  {
    pulse_sum_angular *ps_max =
        psys->create_pulse_sum_angular( b1, b1_ref, b2, b2_ref_max_world, b2_axis, &m_ps_cache_list[PSC_MAX_LIMIT] );

    ps_max->set_pulse_sum_limits_negative();
    ps_max->setup_vel_uni_standard( delta_t, std_max_penalty_restitution_vel );
  }
}

void rigid_body_constraint_ragdoll::set( phys_vec3 &b1_r_loc, phys_vec3 &b2_r_loc )
{
  m_b1_r_loc = b1_r_loc;
  m_b2_r_loc = b2_r_loc;
}

void rigid_body_constraint_ragdoll::set_snider_style( phys_vec3 &b1_axis_loc, phys_vec3 &b1_ref_loc )
{
  m_b1_axis_loc = phys_Unitize( b1_axis_loc );
  m_b1_ref_loc = phys_Unitize( b1_ref_loc );
  if ( get_flag( FLAG_HAS_HINGE ) )
  {
    m_b1_a1_loc = construct_orth_ud( m_b1_axis_loc );
    m_b1_a2_loc = phys_cross( m_b1_axis_loc, m_b1_a1_loc );
  }
  PHYS_ASSERT_UNIT( m_b1_axis_loc );
  PHYS_ASSERT_UNIT( m_b1_ref_loc );
}

void rigid_body_constraint_ragdoll::set_theta_min_max( phys_vec3 &b2_ref_loc, const float theta_min, const float theta_max )
{
  phys_mat44 rv;
  make_rotate( rv, m_b2_axis_loc, theta_min, 1000.0f );
  m_b2_ref_min_loc = phys_Unitize( phys_multiply( rv, b2_ref_loc ) );

  make_rotate( rv, m_b2_axis_loc, theta_max, 1000.0f );
  m_b2_ref_max_loc = phys_Unitize( phys_multiply( rv, b2_ref_loc ) );
}

void rigid_body_constraint_ragdoll::set_hinge( phys_vec3 &b1_axis_loc,
                                               phys_vec3 &b2_axis_loc,
                                               phys_vec3 &b1_ref_loc,
                                               phys_vec3 &b2_ref_loc,
                                               const float theta_min,
                                               const float theta_max )
{
  m_b1_axis_loc = phys_Unitize( b1_axis_loc );
  m_b2_axis_loc = phys_Unitize( b2_axis_loc );
  m_b1_ref_loc = phys_Unitize( b1_ref_loc );

  set_flag( FLAG_HAS_HINGE, true );

  m_b1_a1_loc = construct_orth_ud( m_b1_axis_loc );
  m_b1_a2_loc = phys_cross( m_b1_axis_loc, m_b1_a1_loc );

  phys_mat44 rv;
  make_rotate( rv, m_b2_axis_loc, theta_min, 1000.0f );
  m_b2_ref_min_loc = phys_Unitize( phys_multiply( rv, b2_ref_loc ) );

  make_rotate( rv, m_b2_axis_loc, theta_max, 1000.0f );
  m_b2_ref_max_loc = phys_Unitize( phys_multiply( rv, b2_ref_loc ) );
}

void rigid_body_constraint_ragdoll::set_swivel( phys_vec3 &b1_axis_loc,
                                                phys_vec3 &b2_axis_loc,
                                                phys_vec3 &b1_ref_loc,
                                                phys_vec3 &b2_ref_loc,
                                                const float theta_min,
                                                const float theta_max )
{
  m_b1_axis_loc = phys_Unitize( b1_axis_loc );
  m_b2_axis_loc = phys_Unitize( b2_axis_loc );
  m_b1_ref_loc = phys_Unitize( b1_ref_loc );

  set_flag( FLAG_HAS_SWIVEL, true );

  phys_mat44 rv;
  make_rotate( rv, m_b2_axis_loc, theta_min, 1000.0f );
  m_b2_ref_min_loc = phys_Unitize( phys_multiply( rv, b2_ref_loc ) );

  make_rotate( rv, m_b2_axis_loc, theta_max, 1000.0f );
  m_b2_ref_max_loc = phys_Unitize( phys_multiply( rv, b2_ref_loc ) );
}

void rigid_body_constraint_ragdoll::add_joint_limit( phys_vec3 &b1_ud_loc, const float theta_limit )
{
  tlAssert( m_joint_limits_count < MAX_JOINT_LIMITS );
  m_joint_limits[m_joint_limits_count].set( b1_ud_loc, theta_limit );
  ++m_joint_limits_count;
}

ragdoll_joint_limit_info *rigid_body_constraint_ragdoll::get_joint_limit( const int index )
{
  if ( index >= 0 && index < m_joint_limits_count )
  {
    return &m_joint_limits[index];
  }
  return NULL;
}

const int rigid_body_constraint_ragdoll::get_joint_limit_count()
{
  return m_joint_limits_count;
}

void rigid_body_constraint_ragdoll::disable_joint_limits( const bool disable )
{
  set_flag( FLAG_DISABLE_JOINT_LIMITS, disable );
}

void rigid_body_constraint_ragdoll::set_damp_k( const float damp_k )
{
  m_damp_k = damp_k;
  if ( damp_k <= 0.0f )
  {
    set_flag( FLAG_HAS_DAMP, false );
  }
  else
  {
    set_flag( FLAG_HAS_DAMP, true );
  }
}

void rigid_body_constraint_ragdoll::set_damp_type_motor()
{
  // No implementation or usage in either binary
}

void rigid_body_constraint_ragdoll::set_damp_type_implicit()
{
  // No implementation or usage in either binary
}

void rigid_body_constraint_ragdoll::set_force_limits_active( const bool active )
{
  set_flag( FLAG_FORCE_LIMITS_ACTIVE, active );
}

const unsigned int rigid_body_constraint_ragdoll::get_force_limits_active()
{
  return get_flag( FLAG_FORCE_LIMITS_ACTIVE );
}

phys_vec3 &rigid_body_constraint_ragdoll::get_b1_r_loc()
{
  return m_b1_r_loc;
}

phys_vec3 &rigid_body_constraint_ragdoll::get_b2_r_loc()
{
  return m_b2_r_loc;
}

phys_vec3 &rigid_body_constraint_ragdoll::get_b1_axis_loc()
{
  return m_b1_axis_loc;
}

phys_vec3 &rigid_body_constraint_ragdoll::get_b2_axis_loc()
{
  return m_b2_axis_loc;
}

phys_vec3 &rigid_body_constraint_ragdoll::get_b1_ref_loc()
{
  return m_b1_ref_loc;
}

const float rigid_body_constraint_ragdoll::pull_together()
{
  phys_vec3 b1_r_world = rbint::multiply( b1, m_b1_r_loc );
  phys_vec3 b2_r_world = rbint::multiply( b2, m_b2_r_loc );
  phys_vec3 delta = b1_r_world - b2_r_world;

  b2->dangerous_get_mat().GetW() += delta;

  // Check for NaN/Inf conditions for each component
  float abs_x = fabsf( b2->dangerous_get_mat().GetW()[0] );
  float abs_y = fabsf( b2->dangerous_get_mat().GetW()[1] );
  float abs_z = fabsf( b2->dangerous_get_mat().GetW()[2] );

  if ( abs_x > 100000.0f || abs_y > 100000.0f || abs_z > 100000.0f )
  {
    phys_exec_debug_callback( b2 );
  }

  return AbsSquared( delta );
}

void rigid_body_constraint_ragdoll::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  // Transform reference points to world space
  phys_vec3 b1_r_world = rbint::multiply( b1, m_b1_r_loc );
  phys_vec3 b2_r_world = rbint::multiply( b2, m_b2_r_loc );

  // Transform axis vectors to world space
  phys_vec3 b1_axis_world = rbint::multiply( b1, m_b1_axis_loc );
  phys_vec3 b2_axis_world = rbint::multiply( b2, m_b2_axis_loc );

  // Create point constraint
  psys->create_point( b1, b1_r_world, b2, b2_r_world, &m_ps_cache_list[PSC_X], delta_t, false, 0.0f, 0.0f );

  // Handle damp constraint if enabled
  if ( get_flag( FLAG_HAS_DAMP ) )
  {
    phys_vec3 rel_av = b2->get_a_vel() - b1->get_a_vel();
    float av_mag = Abs( rel_av );
    phys_vec3 damp_axis;

    if ( av_mag <= 0.0001f )
    {
      damp_axis = b2_axis_world;
    }
    else
    {
      damp_axis = rel_av / av_mag;
    }

    pulse_sum_angular *ps_damp =
        psys->create_pulse_sum_angular( b1, PHYS_ZERO_VEC, b2, PHYS_ZERO_VEC, damp_axis, &m_ps_cache_list[PSC_DAMP] );

    if ( get_flag( FLAG_DAMP_TYPE_IMPLICIT ) )
    {
      float cfm = 0.0f;
      pulse_sum_calc_damp_params( m_damp_k, delta_t, &cfm );
      ps_damp->setup_vel_custom_right_side( 0.0f, cfm );
      ps_damp->set_pulse_sum_limits_unbounded();
    }
    else
    {
      ps_damp->setup_vel_simple();
      ps_damp->set_pulse_sum_limits_cone( delta_t * m_damp_k );
    }
  }

  // Handle hinge constraint if enabled
  if ( get_flag( FLAG_HAS_HINGE ) )
  {
    phys_vec3 b1_a1_world = rbint::multiply( b1, m_b1_a1_loc );
    phys_vec3 b1_a2_world = rbint::multiply( b1, m_b1_a2_loc );

    psys->create_hinge( b1, b1_axis_world, b2, b2_axis_world, b1_a1_world, b1_a2_world, &m_ps_cache_list[PSC_A1], delta_t );

    phys_vec3 b1_ref_world = rbint::multiply( b1, m_b1_ref_loc );
    setup_hinge( psys, b1_ref_world, b2_axis_world, delta_t );
  }

  // Handle swivel constraint if enabled
  if ( get_flag( FLAG_HAS_SWIVEL ) )
  {
    if ( !get_flag( FLAG_DISABLE_JOINT_LIMITS ) )
    {
      // Process joint limits
      for ( int i = 0; i < m_joint_limits_count; ++i )
      {
        phys_vec3 b1_ud_world = rbint::multiply( b1, m_joint_limits[i].m_b1_ud_loc );
        float co_ = phys_dot( b2_axis_world, b1_ud_world );

        if ( m_joint_limits[i].m_b1_ud_active_limit_co_ >= co_ )
        {
          phys_vec3 proj = co_ * b2_axis_world;
          phys_vec3 perp = b2_axis_world - proj;
          float perp_mag = Abs( perp );

          if ( perp_mag >= 0.0001f )
          {
            phys_vec3 scaled_perp = ( m_joint_limits[i].m_b1_ud_limit_si_ / perp_mag ) * perp;
            phys_vec3 combined = scaled_perp + ( m_joint_limits[i].m_b1_ud_limit_co_ * b1_ud_world );
            float scale = 1.0f / m_joint_limits[i].m_b1_ud_limit_si_;
            phys_vec3 torque_axis = scale * phys_cross( combined, b1_ud_world );

            PHYS_ASSERT_UNIT( torque_axis );

            pulse_sum_angular *ps_limit =
                psys->create_pulse_sum_angular( b1, combined, b2, b2_axis_world, torque_axis, &m_ps_cache_list[PSC_J_LIMIT0 + i] );

            ps_limit->set_pulse_sum_limits_negative();
            ps_limit->setup_vel_uni_standard( delta_t, pulse_sum_angular::get_std_max_penalty_restitution_vel() );
          }
        }
      }
    }

    // Create swivel hinge constraint
    phys_mat44 rot_matrix;
    make_rotate( &rot_matrix, b1_axis_world, b2_axis_world );
    phys_vec3 b1_ref_world = rbint::multiply( b1, m_b1_ref_loc );
    phys_vec3 rotated_ref = phys_multiply( rot_matrix, b1_ref_world );
    setup_hinge( psys, rotated_ref, b2_axis_world, delta_t );
  }
}
