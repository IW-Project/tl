#ifndef PHYS_COLLISION_H
#define PHYS_COLLISION_H

#include "phys_gjk.h"
#include "collision/phys_broad_phase_base.h"

extern phys_collision_pair *g_list_pcp_iterator;
extern contact_point_info *g_list_output_cpi;

inline void setup_gjk_input_from_pcp( phys_gjk_input *pgi, const phys_collision_pair *pcp )
{
  tlAssert( pcp->m_hit_time >= 0.0f && pcp->m_hit_time <= 1.0f )
  broad_phase_info *p1 = pcp->m_bpi1;
  broad_phase_info *p2 = pcp->m_bpi2;
  pgi->set_continuous_collision_params( p1->m_trace_translation, p2->m_trace_translation, pcp->m_hit_time, 1.0f );
  float half_min_sep_dist = 0.5f * rigid_body_constraint_contact::get_std_contact_min_sep_dist();
  pgi->set( p1->get_gjk_cg(),
            p2->get_gjk_cg(),
            p1->get_cg_to_world_xform(),
            p2->get_cg_to_world_xform(),
            p1->get_gjk_cg()->get_geom_radius() + half_min_sep_dist,
            p2->get_gjk_cg()->get_geom_radius() + half_min_sep_dist,
            pcp->m_gjk_ci );
}

inline bool phys_collide_do_gjk_intersect( phys_gjk_input *pgi, phys_gjk_info *gjk_info )
{
  pgi->set_misc( rigid_body_constraint_contact::get_std_active_limit_distance_eps(), true, true );
  return gjk_info->phys_collide_do_gjk_collide( pgi );
}

void process_list_do_gjk_collide_and_contact_manifold( phys_link_list<phys_collision_pair> *list_pcd );

inline void phys_collide_do_gjk_collide_and_contact_manifold( phys_collision_pair *pcp,
                                                              phys_gjk_info *gjk_info,
                                                              phys_contact_manifold_process *cman_process )
{
  tlAssert( pcp->m_hit_time >= 0.0f && pcp->m_hit_time <= 1.0f )
  phys_gjk_input pgi;
  setup_gjk_input_from_pcp( &pgi, pcp );
  pgi.set_misc(rigid_body_constraint_contact::get_std_active_limit_distance_eps(), false, true);
  if (gjk_info->phys_collide_do_gjk_collide(&pgi))
  {
    cman_process->process( pcp, gjk_info );
  }
}

#endif // PHYS_COLLISION_H
