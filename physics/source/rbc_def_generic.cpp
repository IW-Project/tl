#include "rbc_defs/rbc_def_generic.h"

#include "constraint_solver/pulse_sum_constraint_solver.h"

using namespace math;

void rigid_body_constraint_point::set( phys_vec3 &b1_r_loc, phys_vec3 &b2_r_loc )
{
  m_b1_r_loc = b1_r_loc;
  m_b2_r_loc = b2_r_loc;
}

void rigid_body_constraint_point::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  phys_vec3 b1_r = rbint::multiply( b1, m_b1_r_loc );
  phys_vec3 b2_r = rbint::multiply( b2, m_b2_r_loc );

  psys->create_point( b1, b1_r, b2, b2_r, m_ps_cache, delta_t, m_spring_enabled, m_spring_k, m_damp_k );
}

void rigid_body_constraint_point::epilog_vel_constraint( const float delta_t )
{
  float x = m_ps_cache[PSC_X].get_pulse_sum();
  float y = m_ps_cache[PSC_Y].get_pulse_sum();
  float z = m_ps_cache[PSC_Z].get_pulse_sum();
  m_stress = ( x * x ) + ( y * y ) + ( z * z );
}

void rigid_body_constraint_hinge::set( phys_vec3 &b1_r_loc,
                                       phys_vec3 &b2_r_loc,
                                       phys_vec3 &b1_axis_loc,
                                       phys_vec3 &b2_axis_loc,
                                       phys_vec3 &b1_ref_loc,
                                       phys_vec3 &b2_ref_loc,
                                       const float theta_min,
                                       const float theta_max,
                                       const float damp_k )
{
  m_flags = 0;
  m_b1_r_loc = b1_r_loc;
  m_b2_r_loc = b2_r_loc;
  m_b1_axis_loc = phys_Unitize( b1_axis_loc );
  m_b2_axis_loc = phys_Unitize( b2_axis_loc );
  m_b1_ref_loc = phys_Unitize( b1_ref_loc );
  m_b1_a1_loc = construct_orth_ud( m_b1_axis_loc );
  m_b1_a2_loc = phys_cross( m_b1_axis_loc, m_b1_a1_loc );
  phys_mat44 rv;
  make_rotate( rv, m_b2_axis_loc, theta_min, 1000.0f );
  m_b2_ref_min_loc = phys_Unitize( phys_multiply( rv, b2_ref_loc ) );
  make_rotate( rv, m_b2_axis_loc, theta_max, 1000.0f );
  m_b2_ref_max_loc = phys_Unitize( phys_multiply( rv, b2_ref_loc ) );
  m_damp_k = damp_k;
  PHYS_ASSERT_UNIT( m_b1_axis_loc );
  PHYS_ASSERT_UNIT( m_b2_axis_loc );
  PHYS_ASSERT_UNIT( m_b1_ref_loc );
  PHYS_ASSERT_UNIT( m_b1_a1_loc );
  PHYS_ASSERT_UNIT( m_b1_a2_loc );
  PHYS_ASSERT_UNIT( m_b2_ref_min_loc );
  PHYS_ASSERT_UNIT( m_b2_ref_max_loc );
  PHYS_ASSERT_ORTHOGONAL( m_b1_axis_loc, m_b1_ref_loc );
  PHYS_ASSERT_ORTHOGONAL( m_b1_a1_loc, m_b1_a2_loc );
}

void rigid_body_constraint_hinge::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  if ( !get_flag( NO_POINT_TO_POINT ) )
  {
    psys->create_point( b1,
                        rbint::multiply( b1, m_b1_r_loc ),
                        b2,
                        rbint::multiply( b2, m_b2_r_loc ),
                        &m_ps_cache[PSC_X],
                        delta_t,
                        false,
                        0.0f,
                        0.0f );
  }
  phys_vec3 b1_axis = rbint::multiply( b1, m_b1_axis_loc );
  phys_vec3 b2_axis = rbint::multiply( b2, m_b2_axis_loc );
  psys->create_hinge( b1,
                      b1_axis,
                      b2,
                      b2_axis,
                      rbint::multiply( b1, m_b1_a1_loc ),
                      rbint::multiply( b2, m_b1_a2_loc ),
                      &m_ps_cache[PSC_A1],
                      delta_t );

  if ( m_damp_k > 0.0001f )
  {
    phys_vec3 zero1 = PHYS_ZERO_VEC;
    phys_vec3 zero2 = PHYS_ZERO_VEC;
    pulse_sum_angular *ps1 = psys->create_pulse_sum_angular( b1, zero1, b2, zero2, b2_axis, &m_ps_cache[PSC_DAMP] );
    ps1->setup_vel_simple();
    ps1->set_pulse_sum_limits_cone( delta_t * m_damp_k );
  }

  phys_vec3 b1_ref = rbint::multiply( b1, m_b1_ref_loc );
  phys_vec3 b2_ref_min = rbint::multiply( b2, m_b2_ref_min_loc );
  phys_vec3 b2_ref_max = rbint::multiply( b2, m_b2_ref_max_loc );
  if ( get_flag( SNIDER_STYLE_HINGE ) )
  {
    bool min_active = phys_dot( b1_ref, b2_ref_min ) >= phys_dot( b1_ref, b2_ref_max );
    set_flag( HINGE_MIN_LIMIT_ACTIVE, min_active );
    set_flag( HINGE_MAX_LIMIT_ACTIVE, !min_active );
  }
  else
  {
    set_flag( HINGE_MIN_LIMIT_ACTIVE,
              phys_dot( phys_cross( b1_ref, b2_ref_min ), b2_axis ) >= -pulse_sum_angular::get_std_active_limit_sin_angle_eps() );

    set_flag( HINGE_MAX_LIMIT_ACTIVE,
              pulse_sum_angular::get_std_active_limit_sin_angle_eps() >= phys_dot( phys_cross( b1_ref, b2_ref_max ), b2_axis ) );
  }

  if ( get_flag( HINGE_MIN_LIMIT_ACTIVE ) )
  {
    phys_vec3 n_b2_axis = -b2_axis;
    pulse_sum_angular *ps1 = psys->create_pulse_sum_angular( b1, b1_ref, b2, b2_ref_min, n_b2_axis, &m_ps_cache[PSC_MIN_LIMIT] );
    ps1->set_pulse_sum_limits_negative();
    ps1->setup_vel_uni_standard( delta_t, pulse_sum_angular::get_std_max_penalty_restitution_vel() );
  }
  if ( get_flag( HINGE_MAX_LIMIT_ACTIVE ) )
  {
    pulse_sum_angular *ps1 = psys->create_pulse_sum_angular( b1, b1_ref, b2, b2_ref_max, b2_axis, &m_ps_cache[PSC_MAX_LIMIT] );
    ps1->set_pulse_sum_limits_negative();
    ps1->setup_vel_uni_standard( delta_t, pulse_sum_angular::get_std_max_penalty_restitution_vel() );
  }
}

void rigid_body_constraint_distance::set( phys_vec3 &b1_r_loc, phys_vec3 &b2_r_loc, const float min_distance, const float max_distance )
{
  m_b1_r_loc = b1_r_loc;
  m_b2_r_loc = b2_r_loc;
  m_min_distance = min_distance;
  m_max_distance = max_distance;
  m_next_max_distance = max_distance;
  m_max_distance_vel = 0;
  m_damp_coef = 0;
  m_flags = 0;
  set_flag( FLAG_ENABLE, 1 );
}

void rigid_body_constraint_distance::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  if ( get_enable() )
  {
    tlAssert( m_min_distance >= 0.0f );
    tlAssert( m_min_distance <= m_max_distance );

    phys_vec3 b1_r = phys_multiply( b1->get_mat(), m_b1_r_loc );
    phys_vec3 b2_r = phys_multiply( b2->get_mat(), m_b2_r_loc );

    phys_vec3 b1_pt = rbint::add_pos( b1, b1_r );
    phys_vec3 b2_pt = rbint::add_pos( b2, b2_r );

    phys_vec3 vdist = b2_pt - b1_pt;
    float nvdist_sq = AbsSquared( vdist );
    if ( phys_sqr( 0.001f ) <= nvdist_sq )
    {
      phys_vec3 ud = ( 1.0f / sqrtf( nvdist_sq ) ) * vdist;
      pulse_sum_normal *ps_max = psys->create_pulse_sum_normal();
      phys_vec3 neg_ud = -ud;
      phys_vec3 b1_r_displace = PHYS_ZERO_VEC;
      ps_max->set( b1, b1_r, b2, b2_r, neg_ud, &m_ps_cache_list[PSC_MAX_DIST], b1_r_displace );
      ps_max->set_pulse_sum_limits_negative();
      ps_max->setup_vel_uni_standard_pos_adjust( delta_t, -m_max_distance, 1700.0f );
      ps_max->add_right_side( m_max_distance_vel );
      if ( 0.01f < m_min_distance )
      {
        pulse_sum_normal *ps_min = psys->create_pulse_sum_normal();
        ps_min->set( b1, b1_r, b2, b2_r, ud, &m_ps_cache_list[PSC_MIN_DIST], b1_r_displace );
        ps_min->set_pulse_sum_limits_positive();
        ps_min->setup_vel_uni_standard_pos_adjust( delta_t, m_min_distance, 1700.0f );
      }

      if ( m_damp_coef > 0.0001f && phys_sqr( m_max_distance - 0.1f ) <= nvdist_sq && nvdist_sq <= phys_sqr( m_max_distance + 0.1f ) )
      {
        tlAssert( ps_max );
        phys_vec3 damp_dir = ps_max->get_relative_velocity();
        float ndamp_dir = Abs( damp_dir -= ( phys_dot( damp_dir, ud ) * ud ) );
        if ( ndamp_dir < 0.0001f )
        {
          damp_dir = ps_max->get_relative_velocity_change_dir();
          ndamp_dir = Abs( damp_dir -= ( phys_dot( damp_dir, ud ) * ud ) );
        }

        if ( 0.0001f < ndamp_dir )
        {
          damp_dir *= ( 1.0f / ndamp_dir );
          pulse_sum_normal *ps_damp = psys->create_pulse_sum_normal();
          phys_vec3 r1, r2;

          if ( get_flag( FLAG_B2_R_IS_DAMP_POINT ) == 0 )
          {
            r1 = b1_r;
            r2 = rbint::sub_pos( b2, r1 );
          }
          else
          {
            r1 = rbint::sub_pos( b1, b2_pt );
            r2 = b2_r;
          }

          ps_damp->set( b1, r1, b2, r2, damp_dir, &m_ps_cache_list[PSC_DAMP], b1_r_displace );

          float cfm;
          pulse_sum_calc_damp_params( m_damp_coef, delta_t, &cfm );
          ps_damp->setup_vel_custom_right_side( 0.0f, cfm );
          ps_damp->set_pulse_sum_limits_unbounded();
        }
      }
    }
  }
}

void rigid_body_constraint_distance::outer_prolog_update( const float delta_t )
{
  m_max_distance_vel = ( m_next_max_distance - m_max_distance ) / delta_t;
}

void rigid_body_constraint_distance::inner_update( const float delta_t )
{
  m_max_distance = m_max_distance + ( m_max_distance_vel * delta_t );
}

void rigid_body_constraint_distance::outer_epilog_update( const float delta_t )
{
  m_max_distance = m_next_max_distance;
}

const float rigid_body_constraint_upright::calc_current_lean_angle() const
{
  phys_vec3 b2_up_axis = phys_inv_multiply( b1->get_mat(), get_b2_up_axis_loc() );
  float x = phys_dot( b2_up_axis, m_b1_up_axis_loc );
  float y = -phys_dot( b2_up_axis, m_b1_right_axis_loc );

  if ( ( x * x ) + ( y * y ) <= phys_sqr( 0.0001f ) )
  {
    return 0.0f;
  }
  else
  {
    return atan2f( y, x );
  }
}

const phys_vec3 rigid_body_constraint_upright::calc_b1_lean_axis_loc( const float lean_angle ) const
{
  return ( cosf( lean_angle ) * m_b1_up_axis_loc ) - ( sinf( lean_angle ) * m_b1_right_axis_loc );
}

void rigid_body_constraint_upright::update_lean_axis( phys_vec3 &b1_lean_center, phys_vec3 &b1_lean_axis_loc )
{
  m_b1_lean_axis_loc = b1_lean_axis_loc;
  if ( is_enabled() )
  {
    const phys_vec3 b1_forward_axis = rbint::multiply( b1, m_b1_forward_axis_loc );
    phys_vec3 b2_up_axis = m_b2_up_axis_loc;

    const float nb2_up_axis = Abs( ( phys_dot( m_b2_up_axis_loc, b2_up_axis ) * b2_up_axis ) -= b1_forward_axis );
    if ( nb2_up_axis >= 0.0001f )
    {
      b2_up_axis *= ( 1.0f / nb2_up_axis );
      phys_mat44 lean_mat;
      make_rotate( &lean_mat, phys_multiply( b1->get_mat(), m_b1_lean_axis_loc ), b2_up_axis );
      lean_mat.SetW( b1_lean_center - ( phys_multiply( lean_mat, b1_lean_center ) ) );
      phys_mat44 dest = b1->dangerous_get_mat();
      phys_full_multiply_mat( b1->dangerous_get_mat(), lean_mat, b1->get_mat() );
      VALIDATE_POSITION_VECTOR( b1->dangerous_get_mat().GetW(), b1 );
    }
  }
}

void rigid_body_constraint_upright::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  if ( is_enabled() )
  {
    phys_vec3 b1_forward_axis = phys_multiply( b1->get_mat(), m_b1_forward_axis_loc );
    phys_vec3 b1_lean_axis = phys_multiply( b1->get_mat(), m_b1_lean_axis_loc );
    pulse_sum_angular *ps1 = psys->create_pulse_sum_angular( b1, b1_lean_axis, b2, m_b2_up_axis_loc, b1_forward_axis, m_ps_cache_list );
    ps1->set_pulse_sum_limits_unbounded();
    ps1->setup_vel_bi_standard( delta_t );
  }
}

void rigid_body_constraint_upright::epilog_vel_constraint( const float delta_t )
{
  if ( is_enabled() )
  {
    m_lean_angle = calc_current_lean_angle();
    m_lean_angle_calc_delta_t = m_lean_angle_calc_delta_t + delta_t;
  }
}

void rigid_body_constraint_angular_actuator::set( float power, phys_mat44 &target_mat )
{
  set_power( power );
  set_power_scale( 1.0f );
  m_target_mat = target_mat;
  set_next_target( target_mat );
  m_a_vel = PHYS_ZERO_VEC;
  set_enabled( true );
  PHYS_ASSERT_ORTHONORMAL( m_target_mat );
}

void rigid_body_constraint_angular_actuator::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  if ( is_enabled() )
  {
    const float ps_limit = ( delta_t * m_power ) * m_power_scale;
    const phys_mat44 &b1_mat = b1->get_mat();
    const phys_mat44 &b2_mat = b2->get_mat();

    phys_mat44 mat;
    phys_full_multiply_mat( mat, m_target_mat, b1_mat );

    const phys_vec3 a_vel = phys_multiply( b1_mat, m_a_vel );

    pulse_sum_angular *psa = psys->create_pulse_sum_angular( b1, mat.GetX(), b2, b2_mat.GetX(), mat.GetY(), &m_ps_cache_list[PSC_X] );
    psa->set_pulse_sum_limits_cone( ps_limit );
    psa->setup_vel_custom_big_dirt( 1.0f, 0.0f, delta_t );
    psa->add_right_side( -phys_dot( a_vel, mat.GetY() ) );

    psa = psys->create_pulse_sum_angular( b1, mat.GetY(), b2, b2_mat.GetY(), mat.GetZ(), &m_ps_cache_list[PSC_Y] );
    psa->set_pulse_sum_limits_cone( ps_limit );
    psa->setup_vel_custom_big_dirt( 1.0f, 0.0f, delta_t );
    psa->add_right_side( -phys_dot( a_vel, mat.GetZ() ) );

    psa = psys->create_pulse_sum_angular( b1, mat.GetZ(), b2, b2_mat.GetZ(), mat.GetX(), &m_ps_cache_list[PSC_Z] );
    psa->set_pulse_sum_limits_cone( ps_limit );
    psa->setup_vel_custom_big_dirt( 1.0f, 0.0f, delta_t );
    psa->add_right_side( -phys_dot( a_vel, mat.GetX() ) );
  }
}

void rigid_body_constraint_angular_actuator::outer_prolog_update( const float delta_t )
{
  m_target_mat.SetW( PHYS_ZERO_VEC );
  m_next_target_mat.SetW( PHYS_ZERO_VEC );

  phys_vec3 t_vel;
  nuge::calc_velocities( m_target_mat, m_next_target_mat, delta_t, &t_vel, &m_a_vel );
}

void rigid_body_constraint_angular_actuator::inner_update( const float delta_t )
{
  phys_mat44 rv;
  make_rotate( rv, m_a_vel, delta_t, 1000.0f );
  phys_multiply_mat( m_target_mat, m_target_mat, rv );
}

void rigid_body_constraint_angular_actuator::outer_epilog_update( const float delta_t )
{
  m_target_mat = m_next_target_mat;
}
