#pragma once

#include "pulse_sum_base.h"
#include "pulse_sum_cache.h"
#include "phys_avl_tree.h"
#include "phys_transient_allocator.h"
#include "rigid_body.h"
#include "pulse_sum_wheel.h"
#include "pulse_sum_point.h"
#include "pulse_sum_contact.h"
#include "pulse_sum_angular.h"

#include "rbc_defs/rbc_def_base.h"

typedef phys_link_list<pulse_sum_angular> list_pulse_sum_angular;
typedef phys_link_list<pulse_sum_contact> list_pulse_sum_contact;
typedef phys_link_list<pulse_sum_node> list_pulse_sum_node;
typedef phys_link_list<pulse_sum_normal> list_pulse_sum_normal;
typedef phys_link_list<pulse_sum_point> list_pulse_sum_point;
typedef phys_link_list<pulse_sum_wheel> list_pulse_sum_wheel;

class pulse_sum_constraint_solver
{
public:
  class __declspec( align( 16 ) ) temp_user_rigid_body : public user_rigid_body,
                                                         public phys_link_list_base<temp_user_rigid_body>
  {
  public:
    user_rigid_body *m_avl_key;
    phys_inplace_avl_tree_node<temp_user_rigid_body> m_avl_tree_node;

    struct avl_tree_accessor
    {
      static phys_inplace_avl_tree_node<temp_user_rigid_body> *get_avl_node( temp_user_rigid_body *turb ) { return &turb->m_avl_tree_node; }

      static user_rigid_body *get_avl_key( const temp_user_rigid_body *turb ) { return turb->m_avl_key; }

      static void set_avl_key( temp_user_rigid_body *turb, user_rigid_body *avl_key ) { turb->m_avl_key = avl_key; }
    };

    inline void set( user_rigid_body *original_urb ) { m_avl_key = original_urb; }

    temp_user_rigid_body() {}
  };

  class user_rigid_body_restore_info : public phys_link_list_base<user_rigid_body_restore_info>
  {
  public:
    user_rigid_body **m_rbc_urb;
    user_rigid_body *m_original_urb;

    inline void set( user_rigid_body **rbc_urb, temp_user_rigid_body *turb )
    {
      m_rbc_urb = rbc_urb;
      m_original_urb = *rbc_urb;
      *m_rbc_urb = turb;
    }

    inline void restore() { *m_rbc_urb = m_original_urb; }
  };

  typedef phys_inplace_avl_tree<user_rigid_body *, temp_user_rigid_body, temp_user_rigid_body::avl_tree_accessor> turb_search_tree_t;
  typedef phys_simple_link_list<temp_user_rigid_body> turb_list_t;
  typedef phys_simple_link_list<user_rigid_body_restore_info> urbri_list_t;

  void add_urb( turb_search_tree_t *turb_search_tree, turb_list_t *list_turb, urbri_list_t *list_urbri, rigid_body_constraint *rbc );

  struct solver_info
  {
    int m_max_vel_iters;
    int m_max_vel_pos_iters;
    float m_max_vel_error_sq;
    float m_max_vel_pos_error_sq;
    float m_delta_t;
  };

  float m_outside_delta_t;
  int m_psys_max_vel_iters;
  int m_psys_max_vel_pos_iters;
  solver_info m_si;
  phys_transient_allocator m_solver_memory_allocator;
  list_pulse_sum_node m_list_pulse_sum_node;
  int m_memory_high_water;
  void set_solver_params( const float, const int, const int );
  list_pulse_sum_normal m_list_pulse_sum_normal;
  list_pulse_sum_point m_list_pulse_sum_point;
  list_pulse_sum_angular m_list_pulse_sum_angular;
  list_pulse_sum_wheel m_list_pulse_sum_wheel;
  list_pulse_sum_contact m_list_pulse_sum_contact;
  void solve_iterative( const int max_iters, const float max_error_sq );
  void solve_constraints( rigid_body *const head );
  void execute_constraint_solver( rigid_body *head );
  void set_pulse_sum( pulse_sum_cache *const psc, const float ps );
  const float get_pulse_sum( const pulse_sum_cache * ) const;
  pulse_sum_node *create_pulse_sum_node()
  {
    TRANSIENT_ALLOCATE_CONSTRUCT( psn, m_solver_memory_allocator, 1, pulse_sum_node );
    m_list_pulse_sum_node.add( psn );
    return psn;
  }
  pulse_sum_normal *create_pulse_sum_normal()
  {
    TRANSIENT_ALLOCATE_CONSTRUCT( psn, m_solver_memory_allocator, 1, pulse_sum_normal );
    m_list_pulse_sum_normal.add( psn );
    return psn;
  }
  pulse_sum_normal *create_pulse_sum_normal_();
  inline void create_point( rigid_body *const b1,
                            const phys_vec3 &b1_r,
                            rigid_body *const b2,
                            const phys_vec3 &b2_r,
                            pulse_sum_cache *const ps_cache,
                            const float delta_t,
                            const bool is_spring,
                            const float spring_k,
                            const float damp_k )
  {
    TRANSIENT_ALLOCATE_CONSTRUCT( psp, m_solver_memory_allocator, 1, pulse_sum_point );
    psp->set( b1, b1_r, b2, b2_r, ps_cache, delta_t, is_spring, spring_k, damp_k );
  }
  pulse_sum_angular *create_pulse_sum_angular( rigid_body *const b1,
                                               const phys_vec3 &b1_r,
                                               rigid_body *const b2,
                                               const phys_vec3 &b2_r,
                                               const phys_vec3 &ud,
                                               pulse_sum_cache *const ps_cache )
  {
    TRANSIENT_ALLOCATE_CONSTRUCT( psa, m_solver_memory_allocator, 1, pulse_sum_angular );
    m_list_pulse_sum_angular.add( psa );
    psa->set( b1, b1_r, b2, b2_r, ud, ps_cache );
    return psa;
  }
  pulse_sum_wheel *create_pulse_sum_wheel()
  {
    TRANSIENT_ALLOCATE_CONSTRUCT( psw, m_solver_memory_allocator, 1, pulse_sum_wheel );
    m_list_pulse_sum_wheel.add( psw );
    return psw;
  }
  pulse_sum_normal *create_pulse_sum_wheel_side( pulse_sum_wheel *psw )
  {
    pulse_sum_normal *psn = create_pulse_sum_normal();
    psw->m_side = psn;
    return psn;
  }
  pulse_sum_normal *create_pulse_sum_wheel_fwd( pulse_sum_wheel *psw )
  {
    pulse_sum_normal *psn = create_pulse_sum_normal();
    psw->m_fwd = psn;
    return psn;
  }
  pulse_sum_contact *create_pulse_sum_contact( rigid_body *b1, rigid_body *b2, contact_point_info *cpi, const float delta_t );
  void create_hinge( rigid_body *const b1,
                     const phys_vec3 &b1_axis,
                     rigid_body *const b2,
                     const phys_vec3 &b2_axis,
                     const phys_vec3 &a1,
                     const phys_vec3 &a2,
                     pulse_sum_cache *const ps_cache,
                     const float delta_t )
  {
    pulse_sum_angular* psa = create_pulse_sum_angular(b1, b1_axis, b2, b2_axis, a1, ps_cache);
    psa->set_pulse_sum_limits_unbounded();
    psa->setup_vel_bi_standard( delta_t );
    psa = create_pulse_sum_angular(b1, b1_axis, b2, b2_axis, a2, ps_cache);
    psa->set_pulse_sum_limits_unbounded();
    psa->setup_vel_bi_standard( delta_t );
  }
};

#include "pulse_sum_constraint_solver_inline.h"
