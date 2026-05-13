#pragma once

#include "phys_base.h"
#include "phys_math.h"

class pulse_sum_node;
class rigid_body;

class rigid_body_constraint_point;
class rigid_body_constraint_hinge;
class rigid_body_constraint_distance;
class rigid_body_constraint_ragdoll;
class rigid_body_constraint_wheel;
class rigid_body_constraint_angular_actuator;
class rigid_body_constraint_upright;
class rigid_body_constraint_custom_orientation;
class rigid_body_constraint_custom_path;
class rigid_body_constraint_contact;

class rb_inplace_partition_node
{
public:
  rigid_body_constraint_point *m_rbc_point_first;
  rigid_body_constraint_hinge *m_rbc_hinge_first;
  rigid_body_constraint_distance *m_rbc_dist_first;
  rigid_body_constraint_ragdoll *m_rbc_ragdoll_first;
  rigid_body_constraint_wheel *m_rbc_wheel_first;
  rigid_body_constraint_angular_actuator *m_rbc_angular_actuator_first;
  rigid_body_constraint_upright *m_rbc_upright_first;
  rigid_body_constraint_custom_orientation *m_rbc_custom_orientation_first;
  rigid_body_constraint_custom_path *m_rbc_custom_path_first;
  rigid_body_constraint_contact *m_rbc_contact_first;
  rigid_body *m_partition_head;
  rigid_body *m_partition_tail;
  rigid_body *m_next_node;
  int m_partition_size;
};

class rigid_body
{
public:
  void set( const float mass,
            const phys_vec3 &inertia,
            const phys_mat44 &mat,
            const phys_vec3 &t_vel,
            const phys_vec3 &a_vel,
            const int stable_min_contact_count );
  void set_inertia( const phys_vec3 &inertia );
  void set_mass( const float mass );
  void set_max_avel( const float max_avel ) { m_max_avel = max_avel; }
  void set_min_stable_contact_count( const int count ) { m_stable_min_contact_count = count; }

  enum rigid_body_flags_e
  {
    FLAG_DISABLE_FORCES = ( 1 << 0 ),
    FLAG_STABLE = ( 1 << 2 ),
    FLAG_GROUP_STABLE = ( 1 << 3 ),
    FLAG_ENVIRONMENT_RIGID_BODY = ( 1 << 4 ),
    FLAG_USER_RIGID_BODY = ( 1 << 5 ),
    FLAG_NO_AUTO_REMOVE = ( 1 << 6 ),
    FLAG_DANGEROUS = ( 1 << 7 ),
    FLAG_CLIENT_FLAGS_START = ( 1 << 8 )
  };

  void set_flag( const uint f, const int b ) { b ? m_flags |= f : m_flags &= ~f; }
  const uint get_flag( const uint f ) const { return ( m_flags & f ) != 0; }
  void set_client_flag( const uint f, const int b ) { b ? m_flags |= ( f << 8 ) : m_flags &= ~( f << 8 ); }
  const uint get_client_flag( const uint f ) const { return ( m_flags & ( f << 8 ) ) != 0; }

  const uint is_environment_rigid_body() const { return get_flag( FLAG_ENVIRONMENT_RIGID_BODY ); }
  const uint is_user_rigid_body() const { return get_flag( FLAG_USER_RIGID_BODY ); }

  const phys_mat44 &get_mat() const { return m_mat; }
  phys_mat44 &dangerous_get_mat() { return m_mat; }

  phys_vec3 m_last_position;
  phys_vec3 m_moved_vec;
  float m_smallest_lambda;

  void swap_last_position()
  {
    phys_vec3 temp = m_last_position;
    m_last_position = m_mat.GetW();
    m_mat.SetW( temp );

    VALIDATE_POSITION_VECTOR( m_last_position, this );
    VALIDATE_POSITION_VECTOR( m_mat.GetW(), this );
  }
  void update_last_position()
  {
    m_last_position = m_smallest_lambda * m_moved_vec;
    VALIDATE_POSITION_VECTOR( m_last_position, this );
  }

  const phys_vec3 get_col_moved_dist() const { return m_moved_vec; }
  void adjust_col_moved_vec( const float lambda )
  {
    m_mat.SetW( m_mat.GetW() + ( m_moved_vec * lambda ) );
    m_moved_vec *= ( 1.0f - lambda );
    VALIDATE_POSITION_VECTOR( m_moved_vec, this );
    VALIDATE_POSITION_VECTOR( m_mat.GetW(), this );
  }
  const phys_vec3 &get_t_vel() const { return m_t_vel; }
  const phys_vec3 &get_a_vel() const { return m_a_vel; }
  void dangerous_set_t_vel( const phys_vec3 &t_vel ) { m_t_vel = t_vel; }
  void dangerous_set_a_vel( const phys_vec3 &a_vel ) { m_a_vel = a_vel; }
  const phys_vec3 get_force() const { return m_force_sum; }
  const phys_vec3 get_torque() const { return m_torque_sum; }
  void set_force( const phys_vec3 &force ) { m_force_sum = force; }
  void set_torque( const phys_vec3 &torque ) { m_torque_sum = torque; }
  void add_force( const phys_vec3 &force, const phys_vec3 &point, const float torque_mult );
  void add_force( const phys_vec3 &force ) { m_force_sum += force; }
  void add_torque( const phys_vec3 &torque ) { m_torque_sum += torque; }
  const phys_vec3 get_gravity_acc_vec() const { return m_gravity_acc_vec; }
  void set_gravity_acc_vec( const phys_vec3 &gravity_acc_vec ) { m_gravity_acc_vec = gravity_acc_vec; }
  const float get_max_delta_t() const { return m_max_delta_t; }
  void set_max_delta_t( const float max_delta_t ) { m_max_delta_t = max_delta_t; }
  const float get_inv_mass() const { return m_inv_mass; }
  const phys_vec3 &get_inv_inertia() const { return m_inv_inertia; }
  const float get_stable_energy_time() const { return m_stable_energy_time; }
  const uint is_stable() const { return get_flag( FLAG_STABLE ); }
  const uint is_dangerous() const { return get_flag( FLAG_DANGEROUS ); }
  const uint is_group_stable() const { return get_flag( FLAG_GROUP_STABLE ); }
  const uint is_no_auto_remove() const { return get_flag( FLAG_NO_AUTO_REMOVE ); }
  const int get_constraint_count() const { return m_constraint_count; }
  const int get_contact_count() const { return m_contact_count; }
  void set_user_data( void *d ) { m_userdata = d; }
  void *get_user_data() const { return m_userdata; }
  void set_largest_vel_sq( const float largest_vel_sq ) { m_largest_vel_sq = largest_vel_sq; }
  const float get_largest_vel_sq() const { return m_largest_vel_sq; }
  float *get_largest_vel_sq_ptr() { return &m_largest_vel_sq; }
  void set_t_drag_coef( const float t_drag_coef ) { m_t_drag_coef = t_drag_coef; }
  const float get_t_drag_coef() const { return m_t_drag_coef; }
  void set_a_drag_coef( const float a_drag_coef ) { m_a_drag_coef = a_drag_coef; }
  const float get_a_drag_coef() const { return m_a_drag_coef; }

protected:
  phys_mat44 m_mat;
  phys_vec3 m_inv_inertia;
  phys_vec3 m_gravity_acc_vec;
  phys_vec3 m_t_vel;
  phys_vec3 m_a_vel;
  phys_vec3 m_last_t_vel;
  phys_vec3 m_last_a_vel;
  phys_vec3 m_force_sum;
  phys_vec3 m_torque_sum;
  float m_inv_mass;
  float m_max_avel;
  float m_max_delta_t;
  uint m_flags;
  uint m_tick;
  pulse_sum_node *m_node;
  int m_constraint_count;
  int m_contact_count;
  int m_stable_min_contact_count;
  float m_stable_energy_time;
  float m_largest_vel_sq;
  float m_t_drag_coef;
  float m_a_drag_coef;
  void *m_userdata;

public:
  rb_inplace_partition_node m_partition_node;

  friend class rbint;
};

class user_rigid_body : public rigid_body
{
public:
  const phys_mat44 *m_dictator;
  __declspec( align( 16 ) ) phys_mat44 m_dictator_mat;

public:
  void set( const phys_mat44 *const dictator );
  void setPosition( const phys_mat44 *const dictator );
  const phys_mat44 *get_dictator() const;
};

class environment_rigid_body : public rigid_body
{
public:
  void set();
  environment_rigid_body();
};
