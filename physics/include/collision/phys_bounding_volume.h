#pragma once

#include <phys_math.h>

#include <rbc_defs/rbc_def_contact.h>
#include <collision/phys_gjk.h>

class phys_gjk_geom;

inline const phys_vec3 phys_AbsValue( const phys_vec3 &v )
{
  return phys_vec3( fabsf( v.GetX() ), fabsf( v.GetY() ), fabsf( v.GetZ() ) );
}

inline const float POS_DOT_PROD( const phys_vec3 &v1, const phys_vec3 &v2 )
{
  return phys_dot( phys_AbsValue( v1 ), v2 );
}

inline void phys_calc_world_aabb( const phys_vec3 &local_center,
                                  const phys_vec3 &local_half_aabb_dims,
                                  const phys_mat44 &local_to_world_xform,
                                  phys_vec3 *aabb_min,
                                  phys_vec3 *aabb_max )
{
  phys_vec3 world_center = phys_full_multiply( local_to_world_xform, local_center );
  phys_vec3 world_half_aabb_dim = phys_vec3( POS_DOT_PROD( local_to_world_xform.GetX(), local_half_aabb_dims ),
                                             POS_DOT_PROD( local_to_world_xform.GetY(), local_half_aabb_dims ),
                                             POS_DOT_PROD( local_to_world_xform.GetZ(), local_half_aabb_dims ) );

  *aabb_min = world_center - world_half_aabb_dim;
  *aabb_max = world_center + world_half_aabb_dim;
}

inline void phys_calc_local_aabb( const phys_vec3 &aabb_min,
                                  const phys_vec3 &aabb_max,
                                  const phys_mat44 &local_to_world_xform,
                                  phys_vec3 *local_aabb_min,
                                  phys_vec3 *local_aabb_max )
{
  phys_vec3 half_aabb_dim = 0.5f * ( aabb_max - aabb_min );
  phys_vec3 half_local_aabb_dim = phys_vec3( POS_DOT_PROD( local_to_world_xform.GetX(), half_aabb_dim ),
                                             POS_DOT_PROD( local_to_world_xform.GetY(), half_aabb_dim ),
                                             POS_DOT_PROD( local_to_world_xform.GetZ(), half_aabb_dim ) );

  phys_vec3 local_center = phys_full_inv_multiply( local_to_world_xform, 0.5f * ( aabb_min + aabb_max ) );
  *local_aabb_min = local_center - half_local_aabb_dim;
  *local_aabb_max = local_center + half_local_aabb_dim;
}

inline void phys_aabb_add_point( const phys_mat44 &xform, const phys_vec3 &point, phys_vec3 *aabb_min, phys_vec3 *aabb_max )
{
  phys_vec3 point_abs = phys_multiply( xform, point );
  *aabb_min = phys_min( *aabb_min, point_abs );
  *aabb_max = phys_max( *aabb_max, point_abs );
}

inline void phys_aabb_init_point( const phys_mat44 &xform, const phys_vec3 &point, phys_vec3 *aabb_min, phys_vec3 *aabb_max )
{
  *aabb_min = phys_multiply( xform, point );
  *aabb_max = *aabb_min;
}

inline void phys_aabb_add_hace( phys_vec3 *aabb_min, phys_vec3 *aabb_max )
{
  phys_vec3 hace = rigid_body_constraint_contact::get_half_std_active_limit_distance_eps_vec();
  *aabb_min -= hace;
  *aabb_max += hace;
}

inline void phys_calc_aabb_whace( const phys_gjk_geom *geom, const phys_mat44 &xform, phys_vec3 *aabb_min, phys_vec3 *aabb_max )
{
  geom->calc_aabb( xform, aabb_min, aabb_max );
  phys_aabb_add_hace( aabb_min, aabb_max );
}

inline void phys_aabb_add_sphere( const phys_mat44 &xform, const phys_vec3 &center, const float radius, phys_vec3 *aabb_min, phys_vec3 *aabb_max )
{
  phys_vec3 center_abs = phys_multiply( xform, center );
  phys_vec3 r_vec( radius, radius, radius );
  *aabb_min = phys_min( *aabb_min, center_abs - r_vec );
  *aabb_max = phys_max( *aabb_max, center_abs + r_vec );
}

inline void phys_aabb_init_sphere( const phys_mat44 &xform, const phys_vec3 &center, const float radius, phys_vec3 *aabb_min, phys_vec3 *aabb_max )
{
  phys_vec3 center_abs = phys_multiply( xform, center );
  phys_vec3 r_vec( radius, radius, radius );
  *aabb_min = center_abs - r_vec;
  *aabb_max = center_abs + r_vec;
}

inline void phys_aabb_add_aabb( const phys_vec3 &add_aabb_min, const phys_vec3 &add_aabb_max, phys_vec3 *aabb_min, phys_vec3 *aabb_max )
{
  *aabb_min = phys_min( *aabb_min, add_aabb_min );
  *aabb_max = phys_max( *aabb_max, add_aabb_max );
}

inline const bool phys_are_potentially_colliding( const phys_vec3 &aabb_min0,
                                                  const phys_vec3 &aabb_max0,
                                                  const phys_vec3 &aabb0_translation,
                                                  const phys_vec3 &aabb_min1,
                                                  const phys_vec3 &aabb_max1,
                                                  float *hit_time )
{
  const float THRESH = 0.00001f;
  float tmin = 0.0f;
  float tmax = 1.0f;

  for ( int i = 0; i < 3; ++i )
  {
    const float dif_min1_max0 = aabb_min1[i] - aabb_max0[i];
    const float dif_max1_min0 = aabb_max1[i] - aabb_min0[i];
    if ( aabb0_translation[i] <= THRESH )
    {
      if ( aabb0_translation[i] >= -THRESH )
      {
        if ( dif_min1_max0 > 0.0f || dif_max1_min0 < 0.0f )
        {
          return false;
        }
      }
      else
      {
        tmax = tl_min( tmax, dif_min1_max0 / aabb0_translation[i] );
        tmin = tl_max( tmin, dif_max1_min0 / aabb0_translation[i] );
      }
    }
    else
    {
      tmin = tl_max( tmin, dif_min1_max0 / aabb0_translation[i] );
      tmax = tl_min( tmax, dif_max1_min0 / aabb0_translation[i] );
    }

    if ( tmin > tmax )
    {
      return false;
    }
  }

  tlAssert( tmin >= 0.0f && tmin <= 1.0f );
  tlAssertMsg( !IS_NANF( tmin ), "invalid float number" );
  *hit_time = tmin;
  return true;
}

inline bool phys_are_aabb_overlapping( const phys_vec3 &aabb1_min,
                                       const phys_vec3 &aabb1_max,
                                       const phys_vec3 &aabb2_min,
                                       const phys_vec3 &aabb2_max )
{
  return aabb2_max.GetX() >= aabb1_min.GetX() && aabb2_max.GetY() >= aabb1_min.GetY() && aabb2_max.GetZ() >= aabb1_min.GetZ() &&
         aabb1_max.GetX() >= aabb2_min.GetX() && aabb1_max.GetY() >= aabb2_min.GetY() && aabb1_max.GetZ() >= aabb2_min.GetZ();
}

template <typename T1, typename T2>
bool phys_are_potentially_colliding_whace( T1 *p1, T2 *p2, float *hit_time )
{
  phys_vec3 trans = p1->get_trace_translation() - p2->get_trace_translation();
  return phys_are_potentially_colliding( p1->get_trace_aabb_min_whace(),
                                         p1->get_trace_aabb_max_whace(),
                                         trans,
                                         p2->get_trace_aabb_min_whace(),
                                         p2->get_trace_aabb_max_whace(),
                                         hit_time );
}

inline void comp_trace_volume( const phys_vec3 &aabb1_min,
                               const phys_vec3 &aabb1_max,
                               const phys_vec3 &aabb2_min,
                               const phys_vec3 &aabb2_max,
                               phys_vec3 *p1,
                               phys_vec3 *p2,
                               phys_vec3 *half_dims )
{
  phys_vec3 dims1 = aabb1_max - aabb1_min;
  phys_vec3 dims2 = aabb2_max - aabb2_min;
  *half_dims = 0.5f * phys_max( dims1, dims2 );
  phys_vec3 total_aabb_min = phys_min( aabb1_min, aabb2_min );
  phys_vec3 total_aabb_max = phys_max( aabb1_max, aabb2_max );
  phys_vec3 cmin = total_aabb_min + *half_dims;
  phys_vec3 cmax = total_aabb_max - *half_dims;

  *p1 = 0.5f * ( aabb1_min + aabb1_max );
  *p1 = phys_min( *p1, cmax );
  *p1 = phys_max( *p1, cmin );

  *p2 = 0.5f * ( aabb2_min + aabb2_max );
  *p2 = phys_min( *p2, cmax );
  *p2 = phys_max( *p2, cmin );
}
