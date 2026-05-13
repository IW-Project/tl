#pragma once

#include "phys_math.h"
#include "phys_mem.h"

#include "rbc_defs/rbc_def_contact.h"

class phys_collision_pair;
class phys_gjk_info;

extern const char *g_contact_manifold_error_msg;

class phys_vec2
{
  float x, y;

public:
  phys_vec2() {}
  phys_vec2( const float x_, const float y_, const float )
    : x( x_ ),
      y( y_ )
  {
  }

  const float GetX() const { return x; }
  const float GetY() const { return y; }
  inline void SetX( const float v ) { x = v; }
  inline void SetY( const float v ) { y = v; }

  inline void operator+=( const phys_vec2 &v )
  {
    x += v.x;
    y += v.y;
  }
  inline void operator-=( const phys_vec2 &v )
  {
    x -= v.x;
    y -= v.y;
  }
  inline void operator*=( const float d )
  {
    x *= d;
    y *= d;
  }
  inline void operator/=( const float d )
  {
    x /= d;
    y /= d;
  }

  inline const phys_vec2 operator-() const { return phys_vec2( -x, -y, 0 ); }

  friend inline const float Abs( const phys_vec2 &v ) { return sqrtf( v.x * v.x + v.y * v.y ); }

  friend inline const float AbsSquared( const phys_vec2 &v ) { return v.x * v.x + v.y * v.y; }

  friend inline const phys_vec2 operator*( const float c, const phys_vec2 &v ) { return phys_vec2( c * v.x, c * v.y, 0 ); }

  friend inline const phys_vec2 operator*( const phys_vec2 &v, const float c ) { return phys_vec2( c * v.x, c * v.y, 0 ); }

  friend inline const phys_vec2 operator/( const phys_vec2 &v, const float c ) { return phys_vec2( v.x / c, v.y / c, 0 ); }

  friend inline const phys_vec2 operator+( const phys_vec2 &v1, const phys_vec2 &v2 ) { return phys_vec2( v1.x + v2.x, v1.y + v2.y, 0 ); }

  friend inline const phys_vec2 operator-( const phys_vec2 &v1, const phys_vec2 &v2 ) { return phys_vec2( v1.x - v2.x, v1.y - v2.y, 0 ); }

  friend inline const float phys_v2_dot( const phys_vec2 &v1, const phys_vec2 &v2 ) { return v1.x * v2.x + v1.y * v2.y; }

  friend inline const float phys_v2_cross( const phys_vec2 &v1, const phys_vec2 &v2 ) { return v1.x * v2.y - v2.x * v1.y; }

  friend inline const bool phys_v2_le( const phys_vec2 &v1, const phys_vec2 &v2 )
  {
    return ( v1.x < v2.x ) || ( v1.x == v2.x && v1.y < v2.y );
  }

  friend inline const phys_vec2 phys_v2_rotr( const phys_vec2 &v ) { return phys_vec2( -v.y, v.x, 0 ); }

  friend inline const phys_vec3 phys_v2_to_v3_multiply( const phys_mat44 &m, const phys_vec2 &v )
  {
    return v.GetX() * m.GetX() + v.GetY() * m.GetY();
  }

  friend inline const phys_vec2 phys_v3_to_v2_inv_multiply( const phys_mat44 &m, const phys_vec3 &v )
  {
    return phys_vec2( phys_dot( v, m.GetX() ), phys_dot( v, m.GetY() ), 0 );
  }
};

struct contact_manifold_mesh_point
{
  phys_vec3 m_p;
  phys_vec2 m_contact_p;
  void set( const phys_vec3 &p ) { m_p = p; }
  contact_manifold_mesh_point() {}
};

typedef contact_manifold_mesh_point **poly_vert_id;

class phys_contact_manifold
{
public:
  const float GET_COS_SQ( contact_manifold_mesh_point *mp ) { return mp->m_contact_p.GetX(); }
  const float GET_NP_SQ( contact_manifold_mesh_point *mp ) { return mp->m_contact_p.GetY(); }
  void SET_COS_SQ( contact_manifold_mesh_point *mp, const float cos_sq ) { mp->m_contact_p.SetX( cos_sq ); }
  void SET_NP_SQ( contact_manifold_mesh_point *mp, const float np_sq ) { mp->m_contact_p.SetY( np_sq ); }
  const float get_LENGTH_TOLERANCE_SQ() { return phys_sqr( 0.0033999998f ); }
  const float get_CLOSE_MESH_POINT_COS_SQ() { return 2.0f; }
  const float get_convex_poly_min_length_sq() { return phys_sqr( 0.34f ); }
  const float get_convex_poly_min_sin_sq() { return 0.00030458649f; }
  phys_vec3 m_feature_normal;
  phys_vec3 m_feature_hitp;
  phys_vec3 m_feature_hitn;
  float m_feature_distance_eps;
  float m_sin_feautre_angular_eps_sq;
  int m_close_mesh_point_count;
  void set_get_feature_params( const phys_vec3 &hitp,
                               const phys_vec3 &hitn,
                               const float feature_distance_eps,
                               const float sin_feautre_angular_eps_sq )
  {
    m_feature_hitp = hitp;
    m_feature_hitn = hitn;
    m_feature_distance_eps = feature_distance_eps;
    m_sin_feautre_angular_eps_sq = sin_feautre_angular_eps_sq;
    m_list_mesh_point =
        reinterpret_cast<contact_manifold_mesh_point *>( m_allocator->fast_allocate( tl_max( 16, 4 ), g_contact_manifold_error_msg ) );
    m_list_mesh_point_count = 0;
    m_close_mesh_point_count = 0;
    m_list_sorted_mesh_point = 0;
    m_list_contact_point = NULL;
  }
  void set_comp_feature_normal_eps( const float feature_distance_eps, const float sin_feautre_angular_eps_sq )
  {
    m_feature_distance_eps = feature_distance_eps;
    m_sin_feautre_angular_eps_sq = sin_feautre_angular_eps_sq;
  }
  phys_memory_heap *m_allocator;
  contact_manifold_mesh_point *m_list_mesh_point;
  int m_list_mesh_point_count;
  contact_manifold_mesh_point **m_list_sorted_mesh_point;
  contact_manifold_mesh_point **m_list_contact_point;
  int m_list_contact_point_count;
  void nullify_pointers()
  {
    m_list_mesh_point = NULL;
    m_list_sorted_mesh_point = NULL;
    m_list_contact_point = NULL;
  }
  bool rht( const phys_vec2 &e1, const phys_vec2 &e2, const float min_length2, const float min_sin_sq )
  {
    const float cr = phys_v2_cross( e1, e2 );
    if ( cr > 0.0f )
    {
      const float ne1_sq = AbsSquared( e1 );
      if ( min_length2 < ne1_sq )
      {
        const float ne2_sq = AbsSquared( e2 );
        return min_length2 < ne2_sq && min_sin_sq < ( ( cr * cr ) / ( ne1_sq * ne2_sq ) );
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  void generate_convex_poly_internal();
  void set_allocator( phys_memory_heap *allocator ) { m_allocator = allocator; }
  void add_feature_point( const phys_vec3 &p )
  {
    const phys_vec3 dist = p - m_feature_hitp;
    const float orth_dist = phys_dot( dist, m_feature_hitn );
    const float orth_dist_sq = phys_sqr( orth_dist );
    const float ndist_sq = AbsSquared( dist );

    if ( m_feature_distance_eps < orth_dist && ( ndist_sq * m_sin_feautre_angular_eps_sq ) < orth_dist_sq )
    {
      return;
    }

    contact_manifold_mesh_point *mp = reinterpret_cast<contact_manifold_mesh_point *>(
        m_allocator->fast_allocate( sizeof( contact_manifold_mesh_point ), g_contact_manifold_error_msg ) );
    tlAssert( (uintptr_t)( mp ) % PHYS_ALIGNOF( contact_manifold_mesh_point ) == 0 );
    tlAssert( mp == m_list_mesh_point + m_list_mesh_point_count );
    ++m_list_mesh_point_count;
    mp->set( dist );
    SET_NP_SQ( mp, ndist_sq );
    if ( ndist_sq <= get_LENGTH_TOLERANCE_SQ() )
    {
      SET_COS_SQ( mp, get_CLOSE_MESH_POINT_COS_SQ() );
      ++m_close_mesh_point_count;
    }
    else
    {
      SET_COS_SQ( mp, 1.0f - ( orth_dist_sq / ndist_sq ) );
    }
  }
  int get_mesh_point_count() const { return m_list_mesh_point_count; }
  void xform_mesh_points( phys_mat44 &xform )
  {
    for ( int i = 0; i < m_list_mesh_point_count; ++i )
    {
      contact_manifold_mesh_point &mp = m_list_mesh_point[i];
      mp.m_p = phys_full_multiply( xform, mp.m_p );
    }
  }
  void xform_and_translate_mesh_points( const phys_mat44 &xform, const phys_vec3 &translation )
  {
    contact_manifold_mesh_point **last_mp_i = &m_list_sorted_mesh_point[m_list_mesh_point_count];
    for ( contact_manifold_mesh_point **mp_i = m_list_sorted_mesh_point; mp_i != last_mp_i; ++mp_i )
    {
      contact_manifold_mesh_point &mp = **mp_i;
      mp.m_p = phys_full_multiply( xform, mp.m_p ) + translation;
    }
    m_feature_normal = phys_multiply( xform, m_feature_normal );
  }
  void generate_convex_poly( const phys_mat44 &contact_mat );
  phys_vec2 &get_poly_vert( const int i ) const
  {
    tlAssert( i >= 0 && i < m_list_contact_point_count );
    return m_list_contact_point[i]->m_contact_p;
  }
  const int get_poly_vert_count() const { return m_list_contact_point_count; }
  const float compute_convex_poly_area();
  const float compute_convex_poly_perimeter();
  void comp_feature_normal();

  static const float get_STD_COMP_FEATURE_NORMAL_DISTANCE_EPS( const float penetration_t )
  {
    tlAssert( penetration_t >= 0.0f && penetration_t <= 1.0f );
    return ( penetration_t * 1.7f ) + 1.7f;
  }

  static const float get_STD_COMP_FEATURE_NORMAL_SIN_ANGULAR_EPS_SQ( const float penetration_t )
  {
    tlAssert( penetration_t >= 0.0f && penetration_t <= 1.0f );
    return ( penetration_t * 0.0075892694f ) + 0.0000068538761f;
  }

  static const float get_STD_GET_FEATURE_DISTANCE_EPS( const float penetration_t )
  {
    tlAssert( penetration_t >= 0.0f && penetration_t <= 1.0f );
    return ( penetration_t * 5.1000004f ) + 1.7f;
  }

  static const float get_STD_GET_FEATURE_SIN_ANGULAR_EPS_SQ( const float penetration_t )
  {
    tlAssert( penetration_t >= 0.0f && penetration_t <= 1.0f );
    return ( penetration_t * 0.46984631f ) + 0.03015369f;
  }

  static const float get_STD_PENETRATION_T( const float penetration_depth )
  {
    return tl_clamp( penetration_depth / -10.200001, 0.0f, 1.0f );
  }
};

inline const phys_vec2 &get_contact_p( const poly_vert_id mp )
{
  return ( *mp )->m_contact_p;
}

class phys_contact_manifold_process
{
public:
  struct bridge
  {
    phys_vec2 m_intersection_p;
    poly_vert_id m_left_i;
    poly_vert_id m_right_i;
  };
  struct isect_info
  {
    phys_contact_manifold *m_cman;
    poly_vert_id m_i;
    poly_vert_id m_next_i;
    poly_vert_id m_last_i;
    phys_vec2 m_edge;
    poly_vert_id first_pv() { return m_i; }
    poly_vert_id last_pv() { return m_last_i; }
    poly_vert_id next_pv( poly_vert_id i ) { return i == m_last_i ? m_cman->m_list_contact_point : i + 1; }
    poly_vert_id prev_pv( poly_vert_id i ) { return i == m_cman->m_list_contact_point ? m_last_i : i - 1; }
    void init( phys_contact_manifold *cman );
    void update();
  };
  const float get_poly_vert_displacement_factor() { return 0.34f; }
  phys_mat44 contact_mat;
  phys_mat44 cg1_to_rb2_xform;
  phys_transient_allocator *m_cpi_allocator;
  phys_link_list<contact_point_info> m_list_cpi;
  contact_point_info *m_cpi;
  contact_manifold_mesh_point **m_list_isect_point;
  rigid_body_constraint_contact *m_rbc_contact_search_tree_root;
  phys_contact_manifold cman1;
  phys_contact_manifold cman2;
  int m_contact_point_count;
  phys_memory_heap m_allocator;
  enum
  {
    ALLOCATOR_MEMORY_SIZE = 16384
  };
  char m_allocator_memory[ALLOCATOR_MEMORY_SIZE];
  phys_contact_manifold_process()
    : contact_mat(),
      cg1_to_rb2_xform(),
      cman1(),
      cman2(),
      m_allocator(),
      m_cpi_allocator( NULL ),
      m_cpi( NULL )
  {
    m_list_cpi.remove_all();
    m_allocator.set_buffer( m_allocator_memory, ALLOCATOR_MEMORY_SIZE, 1 );
    cman1.set_allocator( &m_allocator );
    cman2.set_allocator( &m_allocator );
  }
  void setup_misc_params( phys_transient_allocator *cpi_allocator, rigid_body_constraint_contact *rbc_contact_search_tree_root )
  {
    m_cpi_allocator = cpi_allocator;
    m_rbc_contact_search_tree_root = rbc_contact_search_tree_root;
  }
  void nullify_pointers()
  {
    cman1.nullify_pointers();
    cman2.nullify_pointers();
    m_list_isect_point = NULL;
  }
  void comp_contact_mat( const phys_vec3 &contact_normal )
  {
    const phys_vec3 yrow = ( fabsf( contact_normal.GetX() ) < .8f ) ?
                               phys_vec3( 1.0f, 0.0f, 0.0f ) - contact_normal.GetX() * contact_normal :
                               phys_vec3( 0.0f, 1.0f, 0.0f ) - contact_normal.GetY() * contact_normal;
    const float nyrow = Abs( yrow );
    tlAssert( nyrow > 0.1f );
    contact_mat.SetY( yrow / nyrow );
    contact_mat.SetX( phys_cross( contact_mat.GetY(), contact_normal ) );
    contact_mat.SetZ( contact_normal );
  }
  const float GET_VERTEX_AREA( contact_manifold_mesh_point *mp ) { return mp->m_p.GetX(); }
  void SET_VERTEX_AREA( contact_manifold_mesh_point *mp, const float area ) { mp->m_p.SetX( area ); }
  const float CALC_AREA( contact_manifold_mesh_point *mp_0, contact_manifold_mesh_point *mp_1, contact_manifold_mesh_point *mp_2 )
  {
    return phys_v2_cross( mp_1->m_contact_p - mp_0->m_contact_p, mp_2->m_contact_p - mp_0->m_contact_p );
  }
  poly_vert_id GET_PREV_MP( poly_vert_id mp, poly_vert_id first, poly_vert_id last )
  {
    if ( mp > first )
    {
      return mp - 1;
    }
    return last;
  }
  poly_vert_id GET_NEXT_MP( poly_vert_id mp, poly_vert_id first, poly_vert_id last )
  {
    if ( mp < last )
    {
      return mp + 1;
    }
    return first;
  }
  static void displace_contact_p( poly_vert_id mp, const phys_vec2 &d, const phys_mat44 &contact_mat )
  {
    ( *mp )->m_contact_p += d;
    ( *mp )->m_p += phys_v2_to_v3_multiply( contact_mat, d );
  }
  void intersect_poly_segment( phys_contact_manifold * cman, phys_vec2 & p0, phys_vec2 & p1);
  void copy_poly( phys_contact_manifold * cman)
  {
    m_list_isect_point = cman->m_list_contact_point;
    m_contact_point_count = cman->m_list_contact_point_count;
  }
  bool find_bottom( phys_contact_manifold_process::bridge * b,
                    phys_contact_manifold_process::isect_info * left_cman,
                    phys_contact_manifold_process::isect_info * right_cman );
  void intersect_poly_poly();
  void process( phys_collision_pair * pcp, phys_gjk_info * gjk_info );
};