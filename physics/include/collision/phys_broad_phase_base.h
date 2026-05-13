#pragma once

#include "phys_mem.h"
#include "phys_math.h"
#include "collision/phys_bounding_volume.h"
#include "rigid_body.h"

class broad_phase_group;
class phys_auto_activate_callback;
class phys_gjk_geom;
class phys_gjk_cache_info;

void calc_largest_vel_sq( class broad_phase_info *bpi );

class broad_phase_base
{
public:
  enum
  {
    FLAG_IS_BPI = 1,
    FLAG_IS_BPG = 2,
    FLAG_IS_BPI_ENV = 4,
    FLAG_IS_IN_CLUSTER = 16,
    FLAG_ON_ENV_LIST = 32,
    FLAG_ON_BPG_LIST = 64,
    FLAG_IS_AUTO_ACTIVATE = 128,
    BPB_FIRST_UNUSED_FLAG = 512
  };
  phys_vec3 m_trace_aabb_min_whace;
  phys_vec3 m_trace_aabb_max_whace;
  phys_vec3 m_trace_translation;
  uint m_flags;
  broad_phase_base *m_list_bpb_next;
  broad_phase_base *m_list_bpb_cluster_next;
  void *m_sap_node;
  void *m_user_data;
  uint m_env_collision_flags;
  uint m_my_collision_type_flags;
  broad_phase_base *get_bpb_cluster_next() { return m_list_bpb_cluster_next; }
  inline void set_bpb_cluster_next( broad_phase_base *bpb ) { m_list_bpb_cluster_next = bpb; }
  inline const float get_cluster_pos( const int axis ) { return m_trace_aabb_min_whace[axis]; }
  inline void set_env_collision_flags( const uint env_collision_flags ) { m_env_collision_flags = env_collision_flags; }
  inline const uint get_env_collision_flags() { return m_env_collision_flags; }
  inline void set_my_collision_type_flags( const uint my_collision_type_flags ) { m_my_collision_type_flags = my_collision_type_flags; }
  inline const uint get_my_collision_type_flags() { return m_my_collision_type_flags; }
  void set_flag( const uint f, const int b )
  {
    if ( b )
      m_flags |= f;
    else
      m_flags &= ~f;
  }
  inline const uint get_flag( const uint f ) { return m_flags & f; }
  void get_aabb( phys_vec3 *aabb )
  {
    aabb[0] = phys_min( m_trace_aabb_min_whace, get_trace_end_aabb_min_whace() );
    aabb[1] = phys_max( m_trace_aabb_max_whace, get_trace_end_aabb_max_whace() );
  }
  phys_vec3 &get_trace_aabb_min_whace() { return m_trace_aabb_min_whace; }
  phys_vec3 &get_trace_aabb_max_whace() { return m_trace_aabb_max_whace; }
  phys_vec3 &get_trace_translation() { return m_trace_translation; }
  inline const phys_vec3 get_trace_end_aabb_min_whace() { return m_trace_aabb_min_whace + m_trace_translation; }
  inline const phys_vec3 get_trace_end_aabb_max_whace() { return m_trace_aabb_max_whace + m_trace_translation; }
  inline const uint is_bpi() { return get_flag( FLAG_IS_BPI ); }
  inline const uint is_bpg() { return get_flag( FLAG_IS_BPG ); }
  inline const uint is_bpi_env() { return get_flag( FLAG_IS_BPI_ENV ); }
  broad_phase_info *get_bpi()
  {
    tlAssert( is_bpi() );
    return (broad_phase_info *) this;
  }
  broad_phase_group *get_bpg()
  {
    tlAssert( is_bpg() );
    return (broad_phase_group *) this;
  }
  broad_phase_info *get_bpi_env()
  {
    tlAssert( is_bpi_env() );
    return (broad_phase_info *) this;
  }
  phys_auto_activate_callback *get_aac()
  {
    tlAssert( get_flag( FLAG_IS_AUTO_ACTIVATE ) );
    return reinterpret_cast<phys_auto_activate_callback *>( m_sap_node );
  }
  broad_phase_base() {}
};

class broad_phase_info : public broad_phase_base
{
public:
  enum
  {
    FLAG_CALC_CG_TO_WORLD_XFORM = 512,
    BPI_FIRST_UNUSED_FLAG = 1024
  };

  rigid_body *m_rb;
  const phys_mat44 *m_rb_to_world_xform;
  const phys_mat44 *m_cg_to_world_xform;
  const phys_mat44 *m_cg_to_rb_xform;
  phys_gjk_geom *m_gjk_geom;
  uint m_gjk_geom_id;
  int m_surface_type;
  void set_client_flag( const uint f, const int b )
  {
    if ( b )
      m_flags |= f;
    else
      m_flags &= ~f;
  }
  inline const uint get_client_flag( const uint f ) const { return m_flags & f; }
  broad_phase_info *get_next_bpi() const { return reinterpret_cast<broad_phase_info *>( m_list_bpb_next ); }
  void set( rigid_body *rb,
            const phys_mat44 *rb_to_world_xform,
            const phys_mat44 *cg_to_world_xform,
            const phys_mat44 *cg_to_rb_xform,
            phys_gjk_geom *gjk_geom,
            const uint gjk_geom_id,
            const bool calc_cg_to_world_xform,
            const int surface_type,
            void *user_data,
            const uint env_collision_flags )
  {
    m_flags = 0;
    m_list_bpb_next = NULL;
    m_rb = rb;
    set_flag( FLAG_IS_BPI, 1 );
    m_rb = rb;
    m_rb_to_world_xform = rb_to_world_xform;
    m_cg_to_world_xform = cg_to_world_xform;
    m_cg_to_rb_xform = cg_to_rb_xform;
    m_gjk_geom = gjk_geom;
    m_gjk_geom_id = gjk_geom_id;
    set_flag( FLAG_CALC_CG_TO_WORLD_XFORM, calc_cg_to_world_xform );
    m_surface_type = surface_type;
    m_user_data = user_data;
    m_env_collision_flags = env_collision_flags;
  }
  void set_bpi_env( phys_auto_activate_callback *auto_activate_callback )
  {
    tlAssert( is_bpi() );
    tlAssert( m_sap_node == NULL );
    m_flags = 0;
    m_sap_node = reinterpret_cast<void *>( auto_activate_callback );
    set_flag( FLAG_IS_BPI_ENV, 1 );
    set_flag( FLAG_IS_AUTO_ACTIVATE, auto_activate_callback != NULL );
  }
  rigid_body *get_rb() const { return m_rb; }
  const phys_mat44 *get_rb_to_world_xform() const { return m_rb_to_world_xform; }
  const phys_mat44 *get_cg_to_world_xform() const { return m_cg_to_world_xform; }
  const phys_mat44 *get_cg_to_rb_xform() const { return m_cg_to_rb_xform; }
  phys_gjk_geom *get_gjk_cg() const { return m_gjk_geom; }
  inline const uint get_gjk_geom_id() const { return m_gjk_geom_id; }
  void collision_prolog();
  broad_phase_info() {}
};

typedef broad_phase_info broad_phase_info_env;

class phys_auto_activate_callback
{
public:
  virtual bool has_auto_activated() = 0;
  virtual void auto_activate( class broad_phase_info * ) = 0;
  virtual ~phys_auto_activate_callback() {}
};

class phys_collision_pair : public phys_link_list_base<phys_collision_pair>
{
public:
  phys_collision_pair() { m_hit_time = -1.0f; }
  broad_phase_info *m_bpi1;
  broad_phase_info *m_bpi2;
  float m_hit_time;
  phys_gjk_cache_info *m_gjk_ci;
};

class phys_surface_type_info
{
public:
  float m_friction_coef;
  float m_bounce_coef;
  unsigned int m_solver_priority;
  bool m_no_overflow_error;

  phys_surface_type_info( const float friction_coef,
                          const float bounce_coef,
                          const unsigned int solver_priority,
                          const bool no_overflow_error )
    : m_friction_coef( friction_coef ),
      m_bounce_coef( bounce_coef ),
      m_solver_priority( solver_priority ),
      m_no_overflow_error( no_overflow_error )
  {
  }
  inline void set( const float friction_coef, const float bounce_coef, const unsigned int solver_priority, const bool no_overflow_error )
  {
    m_friction_coef = friction_coef;
    m_bounce_coef = bounce_coef;
    m_solver_priority = solver_priority;
    m_no_overflow_error = no_overflow_error;
  }
};
