#pragma once

#include "phys_math.h"
#include "phys_mem.h"
#include "phys_base.h"
#include "pulse_sum_base.h"
#include "pulse_sum_cache.h"

#include <rbc_defs/rbc_def_contact.h>

inline const phys_vec3 phys_diag_multiply_and_square( const phys_vec3 &v1, const phys_vec3 &v2 )
{
  phys_vec3 temp( v1.GetX() * v2.GetX(), v1.GetY() * v2.GetY(), v1.GetZ() * v2.GetZ() );
  return phys_vec3( temp.GetX() * temp.GetX(), temp.GetY() * temp.GetY(), temp.GetZ() * temp.GetZ() );
}

class pulse_sum_contact : public phys_link_list_base<pulse_sum_contact>
{
  friend class pulse_sum_contact_point;

private:
  phys_vec3 m_ud_n;
  phys_vec3 m_ud_f1;
  phys_vec3 m_ud_f2;
  float m_fric_coef;
  pulse_sum_node *m_b1;
  pulse_sum_node *m_b2;
  pulse_sum_contact_point *m_list_pscp;
  void *m_pulse_sum_cache_list;
  int m_list_pscp_count;
  void SOLVER_apply_relaxation( float &error_sq );
  void SOLVER_solver_prolog( const float delta_t );
  void SOLVER_solver_intermediate( const float delta_t );

public:
  void set( rigid_body *const b1, rigid_body *const b2, contact_point_info *cpi, const float delta_t );
  const float get_std_max_penalty_restitution_vel();
  pulse_sum_contact();

  friend class pulse_sum_constraint_solver;
};

class pulse_sum_contact_point
{
public:
  phys_vec3 m_b1_r;
  phys_vec3 m_b2_r;
  phys_vec3 m_b1_ap_n;
  phys_vec3 m_b2_ap_n;
  phys_vec3 m_b1_ap_f1;
  phys_vec3 m_b2_ap_f1;
  phys_vec3 m_b1_ap_f2;
  phys_vec3 m_b2_ap_f2;
  phys_vec3 m_pulse_sum;
  phys_vec3 m_right_side;
  float m_big_dirt;
  float m_denom_xx;
  float m_denom_yy;
  float m_denom_zz;
  float m_denom_xy;
  float m_denom_xz;
  float m_inv_yy;
  float m_inv_yz;
  float m_inv_zz;
  void calc_abs( pulse_sum_contact *psc );
  const float get_impact_vel( pulse_sum_contact *psc );
  const phys_vec3 get_vel( pulse_sum_contact *psc );
  const float get_pos( pulse_sum_contact *psc );
  const phys_vec3 get_objective( pulse_sum_contact *psc );
  void apply( pulse_sum_contact *psc, phys_vec3 &s_ );
  void clamp_n( pulse_sum_contact *psc );
  void clamp_f( pulse_sum_contact *psc );
  void project( pulse_sum_contact *psc );
  phys_vec3 &object_vel_();
  phys_vec3 &object_col_pt_();
  void set_object_vel( pulse_sum_contact *psc, phys_vec3 &object_vel );
  void set_object_col_pt( pulse_sum_contact *psc, phys_vec3 &object_vel );
  void SOLVER_apply_relaxation( pulse_sum_contact *psc, float &error_sq );
  void SOLVER_solver_prolog( pulse_sum_contact *psc, pulse_sum_cache *m_pulse_sum_cache, const float delta_t );
  void SOLVER_solver_intermediate( pulse_sum_contact *psc, pulse_sum_cache *m_pulse_sum_cache, const float delta_t );
  void setup_vel_uni_restitution( pulse_sum_contact *psc,
                                  const float restitution_k,
                                  const float max_restitution_v,
                                  const float delta_t,
                                  const float max_penalty_restitution_vel,
                                  const float min_restitution_impact_vel_sq );
  pulse_sum_contact_point();
};

#include "pulse_sum_contact_inline.h"