#pragma once

#include "rigid_body.h"
#include "phys_mem.h"
#include "phys_transient_allocator.h"
#include "phys_avl_tree.h"
#include "physics_types.h"
#include "physics_system_internal.h"

template <typename list_type, typename T>
T *create( list_type *list_name, rigid_body *const b1, rigid_body *const b2, const int no_error, const char *error_msg );

class phys_sys
{
public:
  static rigid_body *create_rigid_body( const int no_error )
  {
    return g_physics_system->m_list_rigid_body.add( no_error, "POOL OUT OF MEMORY, rigid_body, INCREASE phys_mem_info::m_num_rigid_body." );
  }

  static user_rigid_body *create_user_rigid_body( const int no_error )
  {
    return g_physics_system->m_list_user_rigid_body.add(
        no_error,
        "POOL OUT OF MEMORY, user_rigid_body, INCREASE phys_mem_info::m_num_user_rigid_body." );
  }

  static rigid_body_constraint_point *create_rbc_point( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_point>, rigid_body_constraint_point>(
        &g_physics_system->m_list_rbc_point, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_point, INCREASE phys_mem_info::m_num_rbc_point." );
  }

  static rigid_body_constraint_hinge *create_rbc_hinge( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_hinge>, rigid_body_constraint_hinge>(
        &g_physics_system->m_list_rbc_hinge, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_hinge, INCREASE phys_mem_info::m_num_rbc_hinge." );
  }

  static rigid_body_constraint_distance *create_rbc_dist( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_distance>, rigid_body_constraint_distance>(
        &g_physics_system->m_list_rbc_dist, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_distance, INCREASE phys_mem_info::m_num_rbc_dist." );
  }
  static rigid_body_constraint_ragdoll *create_rbc_ragdoll( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_ragdoll>, rigid_body_constraint_ragdoll>(
        &g_physics_system->m_list_rbc_ragdoll, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_ragdoll, INCREASE phys_mem_info::m_num_rbc_ragdoll." );
  }
  static rigid_body_constraint_wheel *create_rbc_wheel( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_wheel>, rigid_body_constraint_wheel>(
        &g_physics_system->m_list_rbc_wheel, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_wheel, INCREASE phys_mem_info::m_num_rbc_wheel." );
  }
  static rigid_body_constraint_angular_actuator *
  create_rbc_angular_actuator( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_angular_actuator>, rigid_body_constraint_angular_actuator>(
        &g_physics_system->m_list_rbc_angular_actuator, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_angular_actuator, INCREASE phys_mem_info::m_num_rbc_angular_actuator." );
  }
  static rigid_body_constraint_upright *create_rbc_upright( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_upright>, rigid_body_constraint_upright>(
        &g_physics_system->m_list_rbc_upright, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_upright, INCREASE phys_mem_info::m_num_rbc_upright." );
  }
  static rigid_body_constraint_custom_orientation *
  create_rbc_custom_orientation( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_custom_orientation>, rigid_body_constraint_custom_orientation>(
        &g_physics_system->m_list_rbc_custom_orientation, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_custom_orientation, INCREASE phys_mem_info::m_num_rbc_custom_orientation." );
  }
  static rigid_body_constraint_custom_path *create_rbc_custom_path( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_custom_path>, rigid_body_constraint_custom_path>(
        &g_physics_system->m_list_rbc_custom_path, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_custom_path, INCREASE phys_mem_info::m_num_rbc_custom_path." );
  }
  static rigid_body_constraint_contact *create_rbc_contact( rigid_body *const b1, rigid_body *const b2, const int no_error )
  {
    return create<phys_free_list<rigid_body_constraint_contact>, rigid_body_constraint_contact>(
        &g_physics_system->m_list_rbc_contact, b1, b2, no_error,
        "POOL OUT OF MEMORY, rigid_body_constraint_contact, INCREASE phys_mem_info::m_num_rbc_contact." );
  }
  static user_rigid_body *get_user_rigid_body( const phys_mat44 *const dictactor );
  static environment_rigid_body *get_environment_rigid_body();
  static rigid_body_constraint_contact *get_rbc_contact( rigid_body *const b1, rigid_body *const b2 );
  static void destroy( rigid_body_constraint_contact *const rbc ) { g_physics_system->m_list_rbc_contact.remove( rbc ); }
  static void destroy( rigid_body_constraint_custom_path *const rbc ) { g_physics_system->m_list_rbc_custom_path.remove( rbc ); }
  static void destroy( rigid_body_constraint_custom_orientation *const rbc )
  {
    g_physics_system->m_list_rbc_custom_orientation.remove( rbc );
  }
  static void destroy( rigid_body_constraint_upright *const rbc ) { g_physics_system->m_list_rbc_upright.remove( rbc ); }
  static void destroy( rigid_body_constraint_angular_actuator *const rbc ) { g_physics_system->m_list_rbc_angular_actuator.remove( rbc ); }
  static void destroy( rigid_body_constraint_wheel *const rbc ) { g_physics_system->m_list_rbc_wheel.remove( rbc ); }
  static void destroy( rigid_body_constraint_ragdoll *const rbc ) { g_physics_system->m_list_rbc_ragdoll.remove( rbc ); }
  static void destroy( rigid_body_constraint_distance *const rbc ) { g_physics_system->m_list_rbc_dist.remove( rbc ); }
  static void destroy( rigid_body_constraint_hinge *const rbc ) { g_physics_system->m_list_rbc_hinge.remove( rbc ); }
  static void destroy( rigid_body_constraint_point *const rbc ) { g_physics_system->m_list_rbc_point.remove( rbc ); }
  static void destroy( user_rigid_body *const rb )
  {
    destroy_all_constraint( rb );
    g_physics_system->m_list_user_rigid_body.remove( rb );
  }
  static void destroy( rigid_body *const rb )
  {
    destroy_all_constraint( rb );
    g_physics_system->m_list_rigid_body.remove( rb );
  }
  static void destroy_all_rigid_body()
  {
    g_physics_system->m_list_rbc_point.remove_all();
    g_physics_system->m_list_rbc_hinge.remove_all();
    g_physics_system->m_list_rbc_dist.remove_all();
    g_physics_system->m_list_rbc_ragdoll.remove_all();
    g_physics_system->m_list_rbc_wheel.remove_all();
    g_physics_system->m_list_rbc_angular_actuator.remove_all();
    g_physics_system->m_list_rbc_upright.remove_all();
    g_physics_system->m_list_rbc_custom_orientation.remove_all();
    g_physics_system->m_list_rbc_custom_path.remove_all();
    g_physics_system->m_list_rbc_contact.remove_all();
    g_physics_system->m_list_rigid_body.remove_all();
  }
  static void destroy_all_user_rigid_body();
  static void destroy_all_rbc_point() { g_physics_system->m_list_rbc_point.remove_all(); }
  static void destroy_all_rbc_hinge() { g_physics_system->m_list_rbc_hinge.remove_all(); }
  static void destroy_all_rbc_dist() { g_physics_system->m_list_rbc_dist.remove_all(); }
  static void destroy_all_rbc_ragdoll() { g_physics_system->m_list_rbc_ragdoll.remove_all(); }
  static void destroy_all_rbc_wheel() { g_physics_system->m_list_rbc_wheel.remove_all(); }
  static void destroy_all_rbc_angular_actuator() { g_physics_system->m_list_rbc_angular_actuator.remove_all(); }
  static void destroy_all_rbc_upright() { g_physics_system->m_list_rbc_upright.remove_all(); }
  static void destroy_all_rbc_custom_orientation() { g_physics_system->m_list_rbc_custom_orientation.remove_all(); }
  static void destroy_all_rbc_custom_path() { g_physics_system->m_list_rbc_custom_path.remove_all(); }
  static void destroy_all_rbc_contact() { g_physics_system->m_list_rbc_contact.remove_all(); }
  static void destroy_all_constraint( rigid_body *const rb );
  static void destroy_all_constraint_with_user_rigid_body( rigid_body *const rb );
  static void destroy_all_unused_user_rigid_body();
  static void fixup_wheel_constraints( rigid_body *const rb );
  static void update_constraint_infos();
  static void set_max_delta_t( const float max_delta_t );
  static float get_max_delta_t();
  static void set_v_tol( const int max_v_iters );
  static void get_v_tol( int *max_v_iters );
  static void set_vp_tol( const int max_vp_iters );
  static void get_vp_tol( int *max_vp_iters );
  static void set_collision_callback( phys_collision_callback_t collision_callback );
  static int get_rigid_body_alloc_count();
  static int get_user_rigid_body_alloc_count();
  static int get_rbc_point_alloc_count() { return g_physics_system->m_list_rbc_point.get_count(); }
  static int get_rbc_hinge_alloc_count() { return g_physics_system->m_list_rbc_hinge.get_count(); }
  static int get_rbc_dist_alloc_count() { return g_physics_system->m_list_rbc_dist.get_count(); }
  static int get_rbc_ragdoll_alloc_count() { return g_physics_system->m_list_rbc_ragdoll.get_count(); }
  static int get_rbc_wheel_alloc_count() { return g_physics_system->m_list_rbc_wheel.get_count(); }
  static int get_rbc_angular_actuator_alloc_count() { return g_physics_system->m_list_rbc_angular_actuator.get_count(); }
  static int get_rbc_upright_alloc_count() { return g_physics_system->m_list_rbc_upright.get_count(); }
  static int get_rbc_custom_orientation_alloc_count() { return g_physics_system->m_list_rbc_custom_orientation.get_count(); }
  static int get_rbc_custom_path_alloc_count() { return g_physics_system->m_list_rbc_custom_path.get_count(); }
  static int get_rbc_contact_alloc_count() { return g_physics_system->m_list_rbc_contact.get_count(); }
  static void phys_frame_advance( float delta_t );
  static void phys_init();
  static void phys_shutdown();
};

