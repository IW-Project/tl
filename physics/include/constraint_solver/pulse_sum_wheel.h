#pragma once

#include "phys_base.h"
#include "phys_mem.h"

#include "pulse_sum_normal.h"

#pragma pack( push, 8 )
class pulse_sum_wheel : public phys_link_list_base<pulse_sum_wheel>
{
private:
  pulse_sum_normal m_suspension;
  pulse_sum_normal *m_side;
  pulse_sum_normal *m_fwd;
  float m_side_fric_max;
  const bool pulse_chain_within_limits();
  const bool clamp_pulse_sum_pulse_chain( float *ps1_, float *ps2_ );
  void addp_pulse_chain();
  void SOLVER_apply_relaxation( float &error_sq );
  void SOLVER_solver_prolog( const float delta_t );
  void SOLVER_solver_intermediate( const float delta_t );

public:
  void set();
  void set_side_fwd_ratios( const float side_ratio, const float fwd_ratio, const float side_fric_max );
  pulse_sum_normal *get_suspension();
  pulse_sum_wheel();

  friend class pulse_sum_constraint_solver;
  friend class rigid_body_constraint_wheel;
};
#pragma pack( pop )

#include "pulse_sum_wheel_inline.h"
