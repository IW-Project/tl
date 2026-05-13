#pragma once

#include "phys_math.h"
#include "phys_mem.h"
#include "phys_base.h"
#include "pulse_sum_base.h"
#include "pulse_sum_cache.h"

class pulse_sum_point : public phys_link_list_base<pulse_sum_point>
{
private:
  phys_vec3 m_b1_r;
  phys_vec3 m_b2_r;
  phys_vec3 m_b1_apx;
  phys_vec3 m_b2_apx;
  phys_vec3 m_b1_apy;
  phys_vec3 m_b2_apy;
  phys_vec3 m_b1_apz;
  phys_vec3 m_b2_apz;
  phys_vec3 m_pulse_sum;
  phys_vec3 m_right_side;
  phys_vec3 m_big_dirt;
  phys_vec3 m_cr23;
  phys_vec3 m_cr31;
  phys_vec3 m_cr12;
  phys_vec3 m_denom;
  float m_cfm;
  pulse_sum_node *m_b1;
  pulse_sum_node *m_b2;
  pulse_sum_cache *m_pulse_sum_cache;
  void calc_abs_and_setup( const float delta_t, const bool is_spring, const float spring_k, const float damp_k );
  const phys_vec3 get_vel();
  const phys_vec3 get_pos();
  const phys_vec3 get_objective();
  void apply( phys_vec3 &s_ );
  void project();
  phys_vec3 &object_vel_();
  phys_vec3 &object_col_pt_();
  void set_object_vel( phys_vec3 &object_vel );
  void set_object_col_pt( phys_vec3 &object_col_p );
  const phys_vec3 phys_diag_multiply_and_square( phys_vec3 &v1, phys_vec3 &v2 );
  void SOLVER_apply_relaxation( float &error_sq );
  void SOLVER_solver_prolog( const float delta_t );
  void SOLVER_solver_intermediate( const float delta_t );

public:
  void set( rigid_body *const b1,
            phys_vec3 &b1_r,
            rigid_body *const b2,
            phys_vec3 &b2_r,
            pulse_sum_cache *const ps_cache,
            const float delta_t,
            const bool is_spring,
            const float spring_k,
            const float damp_k );
  pulse_sum_point();

  friend class pulse_sum_constraint_solver;
};

#include "pulse_sum_point_inline.h"
