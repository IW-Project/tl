#pragma once

#include "constraint_solver/pulse_sum_cache.h"
#include "rbc_def_base.h"
#include "phys_base.h"
#include "phys_math.h"

#define MAX_JOINT_LIMITS 2

class ragdoll_joint_limit_info
{
public:
  phys_vec3 m_b1_ud_loc;
  float m_b1_ud_limit_co_;
  float m_b1_ud_limit_si_;
  float m_b1_ud_active_limit_co_;
  void set( phys_vec3 &b1_ud_loc, const float theta_limit );
  void set_b1_ud_loc( phys_vec3 &b1_ud_loc );
  void set_theta_limit( const float theta_limit );
};

class ALIGN(16) rigid_body_constraint_ragdoll : public rigid_body_constraint
{
private:
  unsigned char __align0[16];

public:
  phys_vec3 m_b1_r_loc;
  phys_vec3 m_b2_r_loc;

  enum flags_e
  {
    FLAG_HAS_HINGE = ( ( 1 << 2 ) << 0 ),
    FLAG_HAS_SWIVEL = ( ( 1 << 2 ) << 1 ),
    FLAG_MIN_LIMIT_ACTIVE = ( ( 1 << 2 ) << 2 ),
    FLAG_MAX_LIMIT_ACTIVE = ( ( 1 << 2 ) << 3 ),
    FLAG_HAS_DAMP = ( ( 1 << 2 ) << 4 ),
    FLAG_FORCE_LIMITS_ACTIVE = ( ( 1 << 2 ) << 5 ),
    FLAG_DISABLE_JOINT_LIMITS = ( ( 1 << 2 ) << 6 ),
    FLAG_DAMP_TYPE_IMPLICIT = ( ( 1 << 2 ) << 7 ),
  };

  unsigned int m_flags;
  void set_flag( const unsigned int f, const bool b );
  const unsigned int get_flag( const unsigned int f );

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
    PSC_J_LIMIT0 = 8,
    PSC_J_LIMIT1 = 9,
    NUM_PSC = 10
  };
  class pulse_sum_cache m_ps_cache_list[NUM_PSC];

private:
  unsigned char __align1[4];

public:
  phys_vec3 m_b1_axis_loc;
  phys_vec3 m_b2_axis_loc;
  phys_vec3 m_b1_a1_loc;
  phys_vec3 m_b1_a2_loc;
  phys_vec3 m_b1_ref_loc;
  phys_vec3 m_b2_ref_min_loc;
  phys_vec3 m_b2_ref_max_loc;
  ragdoll_joint_limit_info m_joint_limits[MAX_JOINT_LIMITS];
  int m_joint_limits_count;
  float m_damp_k;

  void setup_hinge( pulse_sum_constraint_solver *psys, phys_vec3 &b1_ref, phys_vec3 &b2_axis, const float delta_t );

  inline rigid_body_constraint_ragdoll()
  {
    m_flags = 0;
    m_joint_limits_count = 0;
  }

  void set( phys_vec3 &b1_r_loc, phys_vec3 &b2_r_loc );
  void set_snider_style( phys_vec3 &b1_axis_loc, phys_vec3 &b1_ref_loc );
  void set_theta_min_max( phys_vec3 &b2_ref_loc, const float theta_min, const float theta_max );
  void set_hinge( phys_vec3 &b1_axis_loc,
                  phys_vec3 &b2_axis_loc,
                  phys_vec3 &b1_ref_loc,
                  phys_vec3 &b2_ref_loc,
                  const float theta_min,
                  const float theta_max );
  void set_swivel( phys_vec3 &b1_axis_loc,
                   phys_vec3 &b2_axis_loc,
                   phys_vec3 &b1_ref_loc,
                   phys_vec3 &b2_ref_loc,
                   const float theta_min,
                   const float theta_max );
  void add_joint_limit( phys_vec3 &b1_ud_loc, const float theta_limit );
  ragdoll_joint_limit_info *get_joint_limit( const int index );
  const int get_joint_limit_count();
  void disable_joint_limits( const bool disable );
  void set_damp_k( const float damp_k );
  void set_damp_type_motor();
  void set_damp_type_implicit();
  void set_force_limits_active( const bool active );
  const unsigned int get_force_limits_active();
  phys_vec3 &get_b1_r_loc();
  phys_vec3 &get_b2_r_loc();
  phys_vec3 &get_b1_axis_loc();
  phys_vec3 &get_b2_axis_loc();
  phys_vec3 &get_b1_ref_loc();
  const float pull_together();

  inline void zero_pulse_sums()
  {
    for ( int i = 0; i < NUM_PSC; ++i )
    {
      m_ps_cache_list[i].zero_pulse_sum();
    }
  }

  void setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t );
};