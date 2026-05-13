#pragma once

#include "constraint_solver/pulse_sum_cache.h"
#include "rbc_def_base.h"
#include "phys_math.h"

class user_rigid_body;

class rigid_body_constraint_custom_orientation : public rigid_body_constraint
{
public:
  inline rigid_body_constraint_custom_orientation()
    : m_active( false ),
      m_no_orientation_correction( false ),
      m_torque_resistance_pitch_roll( 50.f ),
      m_torque_resistance_yaw( 10.f ),
      m_upright_strength( 50.f )
  {
  }

  inline void set_active( const bool b ) { m_active = b; }

  inline void set_no_orientation_correction( const bool b ) { m_no_orientation_correction = b; }

  inline void set_resistance_pitch_roll( float p ) { m_torque_resistance_pitch_roll = p; }

  inline void set_resistance_yaw( float p ) { m_torque_resistance_yaw = p; }

  inline void set_upright_strength( float p ) { m_upright_strength = p; }

  inline void zero_pulse_sums()
  {
    for ( int i = 0; i < NUM_PSC; ++i )
    {
      m_ps_cache_list[i].zero_pulse_sum();
    }
  }

  void setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t ) override;

  enum ps_cache_e
  {
    PSC_RESISTANCE_PITCH = 0,
    PSC_RESISTANCE_ROLL = 1,
    PSC_RESISTANCE_YAW = 2,
    PSC_UPRIGHT_PITCH = 3,
    PSC_UPRIGHT_ROLL = 4,
    NUM_PSC = 5
  };

private:
  pulse_sum_cache m_ps_cache_list[NUM_PSC];
  bool m_active;
  bool m_no_orientation_correction;
  float m_torque_resistance_pitch_roll;
  float m_torque_resistance_yaw;
  float m_upright_strength;
};

class ALIGN(16) rigid_body_constraint_custom_path : public rigid_body_constraint
{
public:
  void zero_pulse_sums() override
  {
    for ( int i = 0; i < NUM_PSC; ++i )
    {
      m_list_psc[i].zero_pulse_sum();
    }
  }

  void setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t ) override;
  phys_mat44 m_path_mat;
  phys_vec3 b1_r_loc;
  user_rigid_body *m_urb;

  enum ps_cache_e
  {
    PSC_X = 0,
    PSC_Y = 1,
    PSC_YAW = 2,
    PSC_UPRIGHT = 3,
    NUM_PSC = 4
  };

private:
  pulse_sum_cache m_list_psc[NUM_PSC];
};
