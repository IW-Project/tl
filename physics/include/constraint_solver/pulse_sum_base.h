#pragma once

#include "phys_base.h"
#include "phys_mem.h"
#include "phys_math.h"

class rigid_body;

const float MIN_NERP_DELTA_T = 0.016666668f;

class pulse_sum_node : public phys_link_list_base<pulse_sum_node>
{
private:
  char m_padding[0x10];

public:
  phys_mat44 m_world_inv_inertia;
  phys_vec3 t_vel;
  phys_vec3 a_vel;
  float m_inv_mass;
  rigid_body *m_rb;
};

inline void pulse_sum_calc_spring_params( const float spring_k, const float damp_k, const float delta_t, float *erp, float *cfm )
{
  *cfm = 1.0f / ( ( delta_t * ( delta_t * spring_k ) ) + ( delta_t * damp_k ) );
  *erp = ( delta_t * ( delta_t * spring_k ) ) * *cfm;
}

inline void pulse_sum_calc_damp_params( const float damp_k, const float delta_t, float *cfm )
{
  *cfm = 1.0f / ( delta_t * damp_k );
}
