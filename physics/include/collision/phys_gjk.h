#pragma once

#include "phys_math.h"
#include "phys_gjk_cache_system.h"
#include "phys_contact_manifold.h"
#include "phys_convex_hull.h"

extern const int BIT_COUNT[16];

class phys_gjk_collision_info
{
public:
  phys_vec3 m_p1;
  phys_vec3 m_p2;
  phys_vec3 m_n;
};

class phys_gjk_geom
{
public:
  const phys_vec3 support_only( const phys_vec3 &v ) const
  {
    phys_vec3 support_vert, support_ind;
    support( v, &support_vert, &support_ind );
    return support_vert;
  }
  const phys_vec3 support_only( const phys_mat44 &xform, const phys_vec3 &v ) const
  {
    return phys_multiply( xform, support_only( phys_inv_multiply( xform, v ) ) );
  }
  virtual void support( const phys_vec3 &, phys_vec3 *, phys_vec3 * ) const = 0;
  static void set_ind( phys_vec3 *v, const int ind ) { *(int *)v = ind; }
  static const int get_ind( const phys_vec3 *v ) { return *(const int *)v; }
  virtual void
  get_simplex( const cached_simplex_info &cache_info, const int index_count, phys_vec3 *simplex_verts, phys_vec3 *simplex_inds ) const = 0;
  virtual void set_simplex( const phys_vec3 *simplex_inds, const int w_set, const phys_vec3 &normal, cached_simplex_info *cache_info ) const
  {
    phys_vec3 *inds = cache_info->m_indices;
    for ( int i = 0; i < 4; ++i )
    {
      if ( ( ( 1 << i ) & w_set ) != 0 )
      {
        *inds++ = simplex_inds[i];
      }
    }
  }
  virtual const phys_vec3 get_center( const phys_mat44 &xform ) const { return phys_multiply( xform, get_center() ); }
  virtual const phys_vec3 get_center() const = 0;
  virtual void get_feature( phys_contact_manifold *cman ) const = 0;
  virtual const float get_geom_radius() const { return 0.0f; }
  virtual void calc_aabb( const phys_mat44 &, phys_vec3 *, phys_vec3 * ) const = 0;
  virtual const bool
  ray_cast( const phys_vec3 &ray_pos, const phys_vec3 &ray_dir, const float t_input, float *t_output, phys_vec3 *hitn ) const
  {
    return false;
  }
  virtual bool is_polyhedron() const = 0;
  virtual ~phys_gjk_geom() {}
};

class phys_gjk_input
{
public:
  phys_vec3 m_cg1_translation;
  phys_vec3 m_cg2_translation;
  float m_start_time;
  float m_end_time;
  inline void
  set_continuous_collision_params( phys_vec3 &cg1_translation, phys_vec3 &cg2_translation, const float start_time, const float end_time )
  {
    m_cg1_translation = cg1_translation;
    m_cg2_translation = cg2_translation;
    m_start_time = start_time;
    m_end_time = end_time;
  }
  const phys_gjk_geom *gjk_cg1;
  const phys_gjk_geom *gjk_cg2;
  const phys_mat44 *cg1_to_world_xform;
  const phys_mat44 *cg2_to_world_xform;
  phys_gjk_cache_info *gjk_ci;
  float cg1_radius;
  float cg2_radius;
  float m_sep_thresh;
  bool m_intersection_test_only;
  bool m_continuous_collision;
  inline void set_misc( const float sep_thresh, const bool intersection_test_only, const bool continuous_collision )
  {
    m_sep_thresh = sep_thresh;
    m_intersection_test_only = intersection_test_only;
    m_continuous_collision = continuous_collision;
  }
  inline void set( const phys_gjk_geom *gjk_cg1_,
                   const phys_gjk_geom *gjk_cg2_,
                   const phys_mat44 *cg1_to_world_xform_,
                   const phys_mat44 *cg2_to_world_xform_,
                   const float cg1_radius_,
                   const float cg2_radius_,
                   phys_gjk_cache_info *gjk_ci_ )
  {
    gjk_cg1 = gjk_cg1_;
    gjk_cg2 = gjk_cg2_;
    cg1_to_world_xform = cg1_to_world_xform_;
    cg2_to_world_xform = cg2_to_world_xform_;
    cg1_radius = cg1_radius_;
    cg2_radius = cg2_radius_;
    gjk_ci = gjk_ci_;
  }
};

class phys_gjk_info
{
public:
  struct phys_gjk_set_info
  {
    float m_lamda[4];
    int m_candidate;
  };
  enum gjk_retval_e
  {
    GJK_SEPARATED = 0,
    GJK_VALID = 1,
    GJK_PENETRATING = 2,
    GJK_INVALID = 3
  };
  enum gjk_flags_e
  {
    FLAG_EXIT_ON_SEP_THRESH = ( 1 << 0 ),
    INTERSECTION_TEST_ONLY = ( 1 << 1 ),
    FLAG_TEST_CONVERGENCE = ( 1 << 2 ),
    CONTINUOUS_COLLISION = ( 1 << 3 ),
    FLAG_TEST_UD_LT_SP = ( 1 << 4 ),
    FLAG_IS_SEPARATED = ( 1 << 5 )
  };

public:
  phys_mat44 cg2_to_cg1_xform;
  phys_vec3 m_cg1_relative_translation_loc;
  float m_continuous_collision_lambda;
  phys_gjk_collision_info cg1_cinfo_loc;
  phys_vec3 m_gjk_origin;
  phys_vec3 m_w_verts[4];
  phys_vec3 m_a_verts[4];
  phys_vec3 m_b_verts[4];
  phys_vec3 m_a_inds[4];
  phys_vec3 m_b_inds[4];
  phys_vec3 m_support_dir;
  float m_geom_radii_sum;
  int m_cc_reset_iter;
  int m_flags;
  int m_w_set;
  int m_last_w_set;
  int m_gjk_iter;
  float m_gjk_sep_thresh;
  float m_gjk_pen_thresh_sq;
  float m_upper_dist_sq;
  float m_lower_dist_sq;
  float m_dot_ij[4][4];
  phys_gjk_set_info m_set_list[16];
  void set_flag( const int f, const int b );
  int get_flag( const int f );
  phys_gjk_set_info *get_set_info( const int w_set );
  const int compress_verts( const int w_set );
  void comp_v( const int w_set, phys_vec3 *v );
  void comp_closest_points( const int w_set, phys_vec3 *a, phys_vec3 *b );
  void comp_lambda_1( const int w_set );
  void comp_lambda_2( const int w_set, const int i );
  void comp_lambda_3( const int w_set, const int i, const int j );
  void comp_lambda_4();
  const int gjk_subalgorithm( const int w_set, const int new_index );
  const int seed_simplex( const int w_set );
  const int init_gjk( const phys_gjk_input *d, phys_vec3 &initial_support_dir, const bool in_separation_loop );
  gjk_retval_e gjk( const phys_gjk_input *d, phys_vec3 &initial_support_dir, const bool in_separation_loop );
  gjk_retval_e collide( const phys_gjk_input *d );
  gjk_retval_e gjk_ray_cast( const phys_gjk_input *d, phys_vec3 &initial_support_dir, const bool in_separation_loop );
  const phys_vec3 get_initial_support_dir( const phys_gjk_input *d );
  void gjk_cache_update_invalid( const phys_gjk_input *d );
  void gjk_cache_update_separated( const phys_gjk_input *d );
  void gjk_cache_update_colliding( const phys_gjk_input *d );
  void gjk_cache_update_test_only_valid( const phys_gjk_input *d );
  void gjk_cache_update_test_only_penetrating( const phys_gjk_input *d );
  const bool phys_collide_do_gjk_collide( const phys_gjk_input *d );

  static const float get_std_pen_thresh() { return 0.034000002f; }
  static const float get_std_cc_reset_moveback() { return 0.050999999f; }
  static const float get_std_sin_ang_conv_thresh_sq() { return 0.000099999997f; }
  static const float get_std_conv_thresh() { return 0.001f; }
  static const float get_std_sep_conv_thresh() { return 0.001f; }
  static const float get_std_extra_separation() { return 17.0f; }
};

inline void get_simplex( const phys_gjk_geom *cg1,
                         const phys_gjk_geom *cg2,
                         phys_gjk_cache_info *gjk_ci,
                         phys_vec3 *a_verts,
                         phys_vec3 *a_inds,
                         phys_vec3 *b_verts,
                         phys_vec3 *b_inds,
                         int *vert_count )
{
  tlAssert( gjk_ci->is_simplex_valid() )
  cg1->get_simplex( gjk_ci->m_support_a, gjk_ci->m_support_count, a_verts, a_inds );
  cg2->get_simplex( gjk_ci->m_support_b, gjk_ci->m_support_count, b_verts, b_inds );
  *vert_count = gjk_ci->m_support_count;
}

inline void set_simplex( const phys_gjk_geom *cg1,
                         const phys_gjk_geom *cg2,
                         phys_gjk_cache_info *gjk_ci,
                         const phys_vec3 *a_normal,
                         const phys_vec3 *b_normal,
                         const phys_vec3 *a_inds,
                         const phys_vec3 *b_inds,
                         const int w_set )
{
  tlAssert( gjk_ci->is_simplex_valid() );
  gjk_ci->set_flag( phys_gjk_cache_info::FLAG_IS_SIMPLEX_VALID, true );
  cg1->set_simplex( a_inds, w_set, *a_normal, &gjk_ci->m_support_a );
  cg2->set_simplex( b_inds, w_set, *b_normal, &gjk_ci->m_support_b );
  gjk_ci->m_support_count = BIT_COUNT[w_set];
}
