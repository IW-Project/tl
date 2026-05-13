#pragma once

#include "rigid_body.h"
#include "phys_mem.h"
#include "phys_transient_allocator.h"
#include "phys_avl_tree.h"
#include "physics_types.h"
#include "rbc_defs/rbc_def_contact.h"
#include "rbc_defs/rbc_def_generic.h"
#include "rbc_defs/rbc_def_ragdoll.h"
#include "rbc_defs/rbc_def_vehicle.h"
#include "rbc_defs/rbc_def_custom.h"
#include "collision/phys_gjk.h"
#include "collision/phys_broad_phase.h"

typedef phys_free_list<rigid_body> list_rigid_body;
typedef phys_free_list<rigid_body_constraint_angular_actuator> list_rigid_body_constraint_angular_actuator;
typedef phys_free_list<rigid_body_constraint_contact> list_rigid_body_constraint_contact;
typedef phys_free_list<rigid_body_constraint_custom_orientation> list_rigid_body_constraint_custom_orientation;
typedef phys_free_list<rigid_body_constraint_custom_path> list_rigid_body_constraint_custom_path;
typedef phys_free_list<rigid_body_constraint_distance> list_rigid_body_constraint_distance;
typedef phys_free_list<rigid_body_constraint_hinge> list_rigid_body_constraint_hinge;
typedef phys_free_list<rigid_body_constraint_point> list_rigid_body_constraint_point;
typedef phys_free_list<rigid_body_constraint_ragdoll> list_rigid_body_constraint_ragdoll;
typedef phys_free_list<rigid_body_constraint_upright> list_rigid_body_constraint_upright;
typedef phys_free_list<rigid_body_constraint_wheel> list_rigid_body_constraint_wheel;
typedef phys_free_list<user_rigid_body> list_user_rigid_body;

class physics_system
{
public:
  enum physics_system_flags_e
  {
    FLAG_IN_COLLISION_CALLBACK = 1
  };

  const uint get_flag( const uint f ) { return f & m_flags; }
  void set_flag( const uint f, uint b );
  void set_outside_sub_delta_t( const float );
  int m_flags;
  float m_outside_sub_delta_t;
  phys_collision_callback_t m_collision_callback;
  float m_max_delta_t;
  int m_max_vel_iters;
  int m_max_vel_pos_iters;
  environment_rigid_body m_environment_rigid_body;
  phys_inplace_avl_tree<rigid_body_pair_key, rigid_body_constraint_contact, rigid_body_constraint_contact::avl_tree_accessor>
      m_search_tree_rbc_contact;
  list_user_rigid_body m_list_user_rigid_body;
  list_rigid_body m_list_rigid_body;
  list_rigid_body_constraint_contact m_list_rbc_contact;
  list_rigid_body_constraint_point m_list_rbc_point;
  list_rigid_body_constraint_hinge m_list_rbc_hinge;
  list_rigid_body_constraint_distance m_list_rbc_dist;
  list_rigid_body_constraint_ragdoll m_list_rbc_ragdoll;
  list_rigid_body_constraint_wheel m_list_rbc_wheel;
  list_rigid_body_constraint_custom_orientation m_list_rbc_custom_orientation;
  list_rigid_body_constraint_custom_path m_list_rbc_custom_path;
  list_rigid_body_constraint_angular_actuator m_list_rbc_angular_actuator;
  list_rigid_body_constraint_upright m_list_rbc_upright;
  phys_transient_allocator m_contact_point_buffer_1;
  phys_transient_allocator m_contact_point_buffer_2;
  rigid_body **m_list_island;
  int m_list_island_count;
  int m_solver_memory_high_water;
  int m_contact_point_buffer_high_water;
  void validate_member( const rigid_body_constraint_contact *rbc );
  void validate_member( const rigid_body *rb );
  void generate_partitions_and_stuff( phys_transient_allocator *transient_buffer );
  void solver_priority_sort( phys_transient_allocator *transient_buffer );
  void frame_advance( const float delta_t );
  void time_step( const float outside_delta_t, const bool last_step );
  physics_system();
  static physics_system *create_physics_system();
  static void destroy_physics_system( physics_system *psys );
  static void initialize();
  static void shutdown();
};

extern physics_system *g_physics_system;

inline uint get_physics_system_flag( const uint f )
{
  return g_physics_system->get_flag( f );
}

inline float get_physics_system_outside_sub_delta_t()
{
  return g_physics_system->m_outside_sub_delta_t;
}

void verify_is_in_physics_system( rigid_body_constraint_contact *rbc, rigid_body *b1_, rigid_body *b2_ );