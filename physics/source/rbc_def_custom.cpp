#include "rbc_defs/rbc_def_custom.h"

#include "constraint_solver/pulse_sum_constraint_solver.h"
#include "physics_system_internal.h"

void rigid_body_constraint_custom_orientation::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  const phys_mat44 b1_mat = b1->get_mat();
  const phys_mat44 b2_mat = b2->get_mat();

  // Yaw constraint setup
  if ( m_torque_resistance_pitch_roll > 0.0f )
  {
    float current_power = m_torque_resistance_pitch_roll * 10.0f;

    pulse_sum_angular *psa = psys->create_pulse_sum_angular( b1, b1_mat.GetZ(), b2, b2_mat.GetZ(), b1_mat.GetY(), &m_ps_cache_list[0] );
    psa->set_pulse_sum_limits_unbounded();

    float cfm;
    pulse_sum_calc_damp_params( current_power, delta_t, &cfm );
    psa->setup_vel_custom_right_side( 0.0f, cfm );

    // Second constraint for pitch
    psa = psys->create_pulse_sum_angular( b1, b1_mat.GetZ(), b2, b2_mat.GetZ(), b1_mat.GetX(), &m_ps_cache_list[1] );
    psa->set_pulse_sum_limits_unbounded();

    pulse_sum_calc_damp_params( current_power, delta_t, &cfm );
    psa->setup_vel_custom_right_side( 0.0f, cfm );
  }

  if ( m_torque_resistance_yaw > 0.0f )
  {
    float current_power = m_torque_resistance_yaw * 10.0f;

    pulse_sum_angular *psa = psys->create_pulse_sum_angular( b1, b1_mat.GetY(), b2, b2_mat.GetY(), b1_mat.GetX(), &m_ps_cache_list[2] );
    psa->set_pulse_sum_limits_unbounded();

    float cfm;
    pulse_sum_calc_damp_params( current_power, delta_t, &cfm );
    psa->setup_vel_custom_right_side( 0.0f, cfm );
  }

  if ( m_active && m_upright_strength > 0.0f )
  {
    float roll = math::Abs( b1->get_mat().GetY().GetZ() );
    float pitch = math::Abs( b1->get_mat().GetX().GetZ() );
    if ( b1->get_mat().GetZ().GetZ() > 0.0f )
    {
      roll = 1.0;
      pitch = 1.0f;
    }

    float pitch_power = delta_t * ( m_upright_strength * 30.0f );
    float current_power = pitch_power * pitch;

    pulse_sum_angular *psa = psys->create_pulse_sum_angular( b1, b1_mat.GetZ(), b2, b2_mat.GetZ(), b1_mat.GetY(), &m_ps_cache_list[3] );
    if ( m_no_orientation_correction )
    {
      if ( b1_mat.GetX().GetZ() >= 0.0f )
      {
        psa->set_pulse_sum_limits( 0.0f, current_power );
      }
      else
      {
        psa->set_pulse_sum_limits( -current_power, 0.0f );
      }
      psa->setup_vel_custom( 0.0f, 0.0f, delta_t );
    }
    else
    {
      psa->set_pulse_sum_limits_cone( current_power );
      psa->setup_vel_custom( 1.0f, 0.0f, delta_t );
    }

    float roll_power = delta_t * ( m_upright_strength * 100.0f );
    current_power = roll_power * roll;
    psa = psys->create_pulse_sum_angular( b1, b1_mat.GetZ(), b2, b2_mat.GetZ(), b1_mat.GetX(), &m_ps_cache_list[4] );
    if ( m_no_orientation_correction )
    {
      if ( b1_mat.GetY().GetZ() >= 0.0f )
      {
        psa->set_pulse_sum_limits( 0.0f, current_power );
      }
      else
      {
        psa->set_pulse_sum_limits( -current_power, 0.0f );
      }
      psa->setup_vel_custom( 0.0f, 0.0f, delta_t );
    }
    else
    {
      psa->set_pulse_sum_limits_cone( current_power );
      psa->setup_vel_custom( 1.0f, 0.0f, delta_t );
    }
  }
}

/*
Data           :   enregistered ecx, Object Ptr, Type: class rigid_body_constraint_custom_path * const, this
Data           :   ebx Relative, [00000008], Param, Type: class pulse_sum_constraint_solver *, psys
Data           :   ebx Relative, [0000000C], Param, Type: const float, delta_t
Data           :   ebp Relative, [FFFFFFB0], Local, Type: class phys_vec3, b2_r_loc
Data           :   ebp Relative, [FFFFFF80], Local, Type: class phys_vec3, b1_r
Data           :   ebp Relative, [FFFFFF90], Local, Type: class phys_vec3, b2_r
Data           :   static, [00A1A068][0003:0001D068], Static Local, Type: float, psn_spring_d
Data           :   ebp Relative, [FFFFFFEC], Local, Type: float, psn_erp
Data           :   ebp Relative, [FFFFFFFC], Local, Type: float, psn_cfm
Data           :   ebp Relative, [FFFFFFFC], Local, Type: const float, mass_scale
Data           :   static, [00A1A06C][0003:0001D06C], Static Local, Type: float, psn_spring_k
Data           :   ebp Relative, [FFFFFFF0], Local, Type: class phys_vec3, b2_y
Data           :   ebp Relative, [FFFFFFEC], Local, Type: const float, nb2_r
Data           :   ebp Relative, [FFFFFFC0], Local, Type: class phys_vec3, b1_r
Data           :   ebp Relative, [FFFFFFD0], Local, Type: class phys_vec3, b2_r
Data           :   ebp Relative, [FFFFFFEC], Local, Type: const float, inertia_scale
Data           :   ebp Relative, [FFFFFFB0], Local, Type: class phys_vec3, ud
Data           :   static, [00A1A060][0003:0001D060], Static Local, Type: float, psa_spring_d
Data           :   static, [00A1A064][0003:0001D064], Static Local, Type: float, psa_spring_k
Data           :   ebp Relative, [FFFFFFEC], Local, Type: float, psa_cfm
Data           :   ebp Relative, [FFFFFFD0], Local, Type: class phys_vec3, b1_ud
Data           :   ebp Relative, [FFFFFFEC], Local, Type: const float, nb1_r_length
Data           :   ebp Relative, [FFFFFFA0], Local, Type: class phys_vec3, b2_pt
Data           :   ebp Relative, [FFFFFFC0], Local, Type: class phys_vec3, b1_r_length
Data           :   ebp Relative, [FFFFFFEC], Local, Type: const float, inertia_scale
Data           :   ebp Relative, [FFFFFFB0], Local, Type: class phys_vec3, b1_r_
Data           :   static, [00A1A058][0003:0001D058], Static Local, Type: float, psa_spring_d
Data           :   static, [00A1A05C][0003:0001D05C], Static Local, Type: float, psa_spring_k
Data           :   ebp Relative, [FFFFFFEC], Local, Type: float, psa_cfm
Data           :   ebp Relative, [FFFFFFC0], Local, Type: class phys_vec3, axis
*/
void rigid_body_constraint_custom_path::setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t )
{
  phys_vec3 b2_r_loc( 0.0f, 0.0f, 0.0f );
  phys_vec3 b1_r = rbint::multiply( b1, b1_r_loc );
  phys_vec3 b2_r = rbint::multiply( b2, b2_r_loc );
  tlAssert( m_urb );
  const float mass_scale = 1.0f / b1->get_inv_mass();
  static float psn_spring_k = 50.0f;
  static float psn_spring_d = 12.0f;
  float psn_erp, psn_cfm;
  pulse_sum_calc_spring_params( psn_spring_k * mass_scale, psn_spring_d * mass_scale, delta_t, &psn_erp, &psn_cfm );
  pulse_sum_normal *psn = psys->create_pulse_sum_normal();
  psn->set( b1, b1_r, b2, b2_r, PHYS_X_VEC, &m_list_psc[PSC_X], PHYS_ZERO_VEC );
  psn->setup_vel_custom( psn_erp, psn_cfm, delta_t );
  psn->set_pulse_sum_limits_unbounded();

  psn = psys->create_pulse_sum_normal();
  psn->set( b1, b1_r, b2, b2_r, PHYS_Y_VEC, &m_list_psc[PSC_Y], PHYS_ZERO_VEC );
  psn->setup_vel_custom( psn_erp, psn_cfm, delta_t );
  psn->set_pulse_sum_limits_unbounded();

  phys_vec3 b1_forward = b1->get_mat().GetX();
  phys_vec3 b2_right = b2->get_mat().GetY();

  phys_vec3 b2_r2 = b1_forward - ( phys_dot( b1_forward, b2_right ) * b2_right );
  float nb2_r2 = Abs( b2_r2 );
  bool backwards = phys_dot( b1->get_mat().GetX(), b2->get_mat().GetX() ) < 0.f;
  if ( nb2_r2 > 0.0001f && !backwards )
  {
    b2_r2 /= nb2_r2;
    phys_vec3 ud = phys_cross( b2_right, b2_r2 );

    const float inertia_scale = phys_dot( rbint::mul_L( b1, ud ), ud );
    static float psa_spring_k = 100.0f;
    static float psa_spring_d = 5.0f;
    float psa_erp, psa_cfm;
    pulse_sum_calc_spring_params( psa_spring_k * inertia_scale, psa_spring_d * inertia_scale, delta_t, &psa_erp, &psa_cfm );

    pulse_sum_angular *psa = psys->create_pulse_sum_angular( b1, b1_forward, b2, b2_r2, ud, &m_list_psc[PSC_YAW] );
    psa->setup_vel_custom( psa_erp, psa_cfm, delta_t );
    psa->set_pulse_sum_limits_unbounded();
  }

  phys_vec3 b1_ud = b1->get_mat().GetZ();
  phys_vec3 b2_pt( 0.0f, 0.0f, 1.0f );
  phys_vec3 b1_r_length = b2_pt - ( phys_dot( b2_pt, b1_ud ) * b1_ud );
  float nb1_r_length = Abs( b1_r_length );
  if ( nb1_r_length >= 0.0001f )
  {
    phys_vec3 b1_r_ = ( ( 0.86602539f / nb1_r_length ) * b1_r_length ) + ( 0.5f * b1_ud );
    phys_vec3 axis = 1.1547005f * phys_cross( b1_r_, b1_ud );
    PHYS_ASSERT_UNIT( axis );
    const float inertia_scale = phys_dot( rbint::mul_L( b1, axis ), axis );
    static float psa_spring_k = 100.0f;
    static float psa_spring_d = 5.0f;
    float psa_erp, psa_cfm;
    pulse_sum_calc_spring_params( psa_spring_k * inertia_scale, psa_spring_d * inertia_scale, delta_t, &psa_erp, &psa_cfm );

    pulse_sum_angular *psa = psys->create_pulse_sum_angular( b1, b1_r_, b2, b2_pt, axis, &m_list_psc[PSC_UPRIGHT] );
    psa->set_pulse_sum_limits_negative();
    psa->setup_vel_custom( psa_erp, psa_cfm, delta_t );
  }
}