#include "physics_system.h"

#include "physics_system_internal.h"
#include "rbc_defs/rbc_def_internal.h"
#include "rbc_defs/rbc_def_ragdoll.h"
#include "rbc_defs/rbc_def_custom.h"

class phys_list_condition_functor_has_user_rigid_body
{
public:
  bool test( const rigid_body_constraint &c )
  {
    return ( c.get_b1() && c.get_b1()->is_user_rigid_body() ) || ( c.get_b2() && c.get_b2()->is_user_rigid_body() );
  }
};

class phys_list_condition_functor_has_rigid_body
{
  rigid_body *m_rb;

public:
  phys_list_condition_functor_has_rigid_body( rigid_body *const rb )
    : m_rb( rb )
  {
  }
  bool test( rigid_body_constraint &c ) { return ( c.get_b1() && c.get_b1() == m_rb ) || ( c.get_b2() && c.get_b2() == m_rb ); }
};

class phys_list_condition_functor_has_rigid_body_and_user_rigid_body
{
  rigid_body *m_rb;

public:
  phys_list_condition_functor_has_rigid_body_and_user_rigid_body( rigid_body *const rb )
    : m_rb( rb )
  {
  }
  bool test( rigid_body_constraint &c )
  {
    return ( ( c.get_b1() && c.get_b1() == m_rb && c.get_b2()->is_user_rigid_body() ) ||
             ( c.get_b2() && c.get_b2() == m_rb && c.get_b1()->is_user_rigid_body() ) );
  }
};

class phys_list_condition_functor_has_no_constraints
{
public:
  bool test( rigid_body &c ) { return c.get_constraint_count() == 0; }
};

template <typename list_type, typename T>
T *create( list_type *list_name, rigid_body *const b1, rigid_body *const b2, const int no_error, const char *error_msg )
{
  g_physics_system->validate_member( b1 );
  g_physics_system->validate_member( b2 );
  T *rbc = list_name->add( no_error, error_msg );
  if ( rbc )
  {
    rbcint::set( rbc, b1, b2 );
  }
  return rbc;
}

user_rigid_body *phys_sys::get_user_rigid_body( const phys_mat44 *const dictactor )
{
  for ( phys_free_list<user_rigid_body>::iterator it = g_physics_system->m_list_user_rigid_body.begin(); it != g_physics_system->m_list_user_rigid_body.end(); ++it )
  {
    user_rigid_body &urb = *it;
    if ( urb.get_dictator() == dictactor )
    {
      return &urb;
    }
  }
  return NULL;
}

environment_rigid_body *phys_sys::get_environment_rigid_body()
{
  return &g_physics_system->m_environment_rigid_body;
}

rigid_body_constraint_contact *phys_sys::get_rbc_contact( rigid_body *const b1, rigid_body *const b2 )
{
  g_physics_system->validate_member( b1 );
  g_physics_system->validate_member( b2 );

  rigid_body_pair_key key( b1, b2 );
  return g_physics_system->m_search_tree_rbc_contact.find( key );
}

void phys_sys::destroy_all_user_rigid_body()
{
  phys_list_condition_functor_has_user_rigid_body condition;

  for ( phys_free_list<rigid_body_constraint_point>::iterator i = g_physics_system->m_list_rbc_point.begin(); i != g_physics_system->m_list_rbc_point.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_point>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_point.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_hinge>::iterator i = g_physics_system->m_list_rbc_hinge.begin(); i != g_physics_system->m_list_rbc_hinge.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_hinge>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_hinge.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_distance>::iterator i = g_physics_system->m_list_rbc_dist.begin(); i != g_physics_system->m_list_rbc_dist.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_distance>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_dist.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_ragdoll>::iterator i = g_physics_system->m_list_rbc_ragdoll.begin(); i != g_physics_system->m_list_rbc_ragdoll.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_ragdoll>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_ragdoll.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_wheel>::iterator i = g_physics_system->m_list_rbc_wheel.begin(); i != g_physics_system->m_list_rbc_wheel.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_wheel>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_wheel.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_angular_actuator>::iterator i = g_physics_system->m_list_rbc_angular_actuator.begin(); i != g_physics_system->m_list_rbc_angular_actuator.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_angular_actuator>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_angular_actuator.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_upright>::iterator i = g_physics_system->m_list_rbc_upright.begin(); i != g_physics_system->m_list_rbc_upright.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_upright>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_upright.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_custom_orientation>::iterator i = g_physics_system->m_list_rbc_custom_orientation.begin(); i != g_physics_system->m_list_rbc_custom_orientation.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_custom_orientation>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_custom_orientation.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_custom_path>::iterator i = g_physics_system->m_list_rbc_custom_path.begin(); i != g_physics_system->m_list_rbc_custom_path.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_custom_path>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_custom_path.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_contact>::iterator i = g_physics_system->m_list_rbc_contact.begin(); i != g_physics_system->m_list_rbc_contact.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_contact>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_contact.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  g_physics_system->m_list_user_rigid_body.remove_all();
}

void phys_sys::destroy_all_constraint( rigid_body *const rb )
{
  fixup_wheel_constraints( rb );

  phys_list_condition_functor_has_rigid_body condition( rb );

  for ( phys_free_list<rigid_body_constraint_point>::iterator i = g_physics_system->m_list_rbc_point.begin(); i != g_physics_system->m_list_rbc_point.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_point>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_point.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_hinge>::iterator i = g_physics_system->m_list_rbc_hinge.begin(); i != g_physics_system->m_list_rbc_hinge.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_hinge>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_hinge.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_distance>::iterator i = g_physics_system->m_list_rbc_dist.begin(); i != g_physics_system->m_list_rbc_dist.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_distance>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_dist.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_ragdoll>::iterator i = g_physics_system->m_list_rbc_ragdoll.begin(); i != g_physics_system->m_list_rbc_ragdoll.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_ragdoll>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_ragdoll.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_wheel>::iterator i = g_physics_system->m_list_rbc_wheel.begin(); i != g_physics_system->m_list_rbc_wheel.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_wheel>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_wheel.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_angular_actuator>::iterator i = g_physics_system->m_list_rbc_angular_actuator.begin(); i != g_physics_system->m_list_rbc_angular_actuator.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_angular_actuator>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_angular_actuator.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_upright>::iterator i = g_physics_system->m_list_rbc_upright.begin(); i != g_physics_system->m_list_rbc_upright.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_upright>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_upright.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_custom_orientation>::iterator i = g_physics_system->m_list_rbc_custom_orientation.begin(); i != g_physics_system->m_list_rbc_custom_orientation.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_custom_orientation>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_custom_orientation.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_custom_path>::iterator i = g_physics_system->m_list_rbc_custom_path.begin(); i != g_physics_system->m_list_rbc_custom_path.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_custom_path>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_custom_path.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_contact>::iterator i = g_physics_system->m_list_rbc_contact.begin(); i != g_physics_system->m_list_rbc_contact.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_contact>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_contact.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }
}

void phys_sys::destroy_all_constraint_with_user_rigid_body( rigid_body *const rb )
{
  fixup_wheel_constraints( rb );

  phys_list_condition_functor_has_rigid_body_and_user_rigid_body condition( rb );

  for ( phys_free_list<rigid_body_constraint_point>::iterator i = g_physics_system->m_list_rbc_point.begin(); i != g_physics_system->m_list_rbc_point.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_point>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_point.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_hinge>::iterator i = g_physics_system->m_list_rbc_hinge.begin(); i != g_physics_system->m_list_rbc_hinge.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_hinge>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_hinge.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_distance>::iterator i = g_physics_system->m_list_rbc_dist.begin(); i != g_physics_system->m_list_rbc_dist.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_distance>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_dist.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_ragdoll>::iterator i = g_physics_system->m_list_rbc_ragdoll.begin(); i != g_physics_system->m_list_rbc_ragdoll.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_ragdoll>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_ragdoll.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_wheel>::iterator i = g_physics_system->m_list_rbc_wheel.begin(); i != g_physics_system->m_list_rbc_wheel.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_wheel>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_wheel.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_angular_actuator>::iterator i = g_physics_system->m_list_rbc_angular_actuator.begin(); i != g_physics_system->m_list_rbc_angular_actuator.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_angular_actuator>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_angular_actuator.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_upright>::iterator i = g_physics_system->m_list_rbc_upright.begin(); i != g_physics_system->m_list_rbc_upright.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_upright>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_upright.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_custom_orientation>::iterator i = g_physics_system->m_list_rbc_custom_orientation.begin(); i != g_physics_system->m_list_rbc_custom_orientation.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_custom_orientation>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_custom_orientation.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_custom_path>::iterator i = g_physics_system->m_list_rbc_custom_path.begin(); i != g_physics_system->m_list_rbc_custom_path.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_custom_path>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_custom_path.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }

  for ( phys_free_list<rigid_body_constraint_contact>::iterator i = g_physics_system->m_list_rbc_contact.begin(); i != g_physics_system->m_list_rbc_contact.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<rigid_body_constraint_contact>::iterator next = i.next_after_remove();
      g_physics_system->m_list_rbc_contact.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }
}

void phys_sys::destroy_all_unused_user_rigid_body()
{
  phys_list_condition_functor_has_no_constraints condition;

  for ( phys_free_list<user_rigid_body>::iterator i = g_physics_system->m_list_user_rigid_body.begin(); i != g_physics_system->m_list_user_rigid_body.end(); )
  {
    if ( condition.test( *i ) )
    {
      phys_free_list<user_rigid_body>::iterator next = i.next_after_remove();
      g_physics_system->m_list_user_rigid_body.remove( &*i );
      i = next;
    }
    else
    {
      ++i;
    }
  }
}

void phys_sys::fixup_wheel_constraints( rigid_body *const rb )
{
  for ( phys_free_list<rigid_body_constraint_wheel>::iterator it = g_physics_system->m_list_rbc_wheel.begin(); it != g_physics_system->m_list_rbc_wheel.end(); ++it )
  {
    rigid_body_constraint_wheel &rbc_wheel = *it;
    if ( rbc_wheel.get_b2() == rb )
    {
      rbc_wheel.set_no_collision();
    }
  }
}

void phys_sys::update_constraint_infos()
{
  rbint::constraint_info_reset( &g_physics_system->m_environment_rigid_body );

  for ( phys_free_list<user_rigid_body>::iterator it = g_physics_system->m_list_user_rigid_body.begin(); it != g_physics_system->m_list_user_rigid_body.end(); ++it )
  {
    user_rigid_body &i = *it;
    rbint::constraint_info_reset( &i );
  }

  for ( phys_free_list<rigid_body>::iterator it = g_physics_system->m_list_rigid_body.begin(); it != g_physics_system->m_list_rigid_body.end(); ++it )
  {
    rigid_body &i = *it;
    rbint::constraint_info_reset( &i );
  }

  for ( phys_free_list<rigid_body_constraint_point>::iterator it = g_physics_system->m_list_rbc_point.begin(); it != g_physics_system->m_list_rbc_point.end(); ++it )
  {
    rigid_body_constraint_point &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_hinge>::iterator it = g_physics_system->m_list_rbc_hinge.begin(); it != g_physics_system->m_list_rbc_hinge.end(); ++it )
  {
    rigid_body_constraint_hinge &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_distance>::iterator it = g_physics_system->m_list_rbc_dist.begin(); it != g_physics_system->m_list_rbc_dist.end(); ++it )
  {
    rigid_body_constraint_distance &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_ragdoll>::iterator it = g_physics_system->m_list_rbc_ragdoll.begin(); it != g_physics_system->m_list_rbc_ragdoll.end(); ++it )
  {
    rigid_body_constraint_ragdoll &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_wheel>::iterator it = g_physics_system->m_list_rbc_wheel.begin(); it != g_physics_system->m_list_rbc_wheel.end(); ++it )
  {
    rigid_body_constraint_wheel &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_angular_actuator>::iterator it = g_physics_system->m_list_rbc_angular_actuator.begin(); it != g_physics_system->m_list_rbc_angular_actuator.end(); ++it )
  {
    rigid_body_constraint_angular_actuator &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_upright>::iterator it = g_physics_system->m_list_rbc_upright.begin(); it != g_physics_system->m_list_rbc_upright.end(); ++it )
  {
    rigid_body_constraint_upright &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_custom_orientation>::iterator it = g_physics_system->m_list_rbc_custom_orientation.begin(); it != g_physics_system->m_list_rbc_custom_orientation.end(); ++it )
  {
    rigid_body_constraint_custom_orientation &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_custom_path>::iterator it = g_physics_system->m_list_rbc_custom_path.begin(); it != g_physics_system->m_list_rbc_custom_path.end(); ++it )
  {
    rigid_body_constraint_custom_path &i = *it;
    rbcint::process_constraint_info( &i );
  }

  for ( phys_free_list<rigid_body_constraint_contact>::iterator it = g_physics_system->m_list_rbc_contact.begin(); it != g_physics_system->m_list_rbc_contact.end(); ++it )
  {
    rigid_body_constraint_contact &i = *it;
    rbcint::process_constraint_info( &i );
  }
}

void phys_sys::set_max_delta_t( const float max_delta_t )
{
  g_physics_system->m_max_delta_t = max_delta_t;
}

float phys_sys::get_max_delta_t()
{
  return g_physics_system->m_max_delta_t;
}

void phys_sys::set_v_tol( const int max_v_iters )
{
  g_physics_system->m_max_vel_iters = max_v_iters;
}

void phys_sys::get_v_tol( int *max_v_iters )
{
  *max_v_iters = g_physics_system->m_max_vel_iters;
}

void phys_sys::set_vp_tol( const int max_vp_iters )
{
  g_physics_system->m_max_vel_pos_iters = max_vp_iters;
}

void phys_sys::get_vp_tol( int *max_vp_iters )
{
  *max_vp_iters = g_physics_system->m_max_vel_pos_iters;
}

void phys_sys::set_collision_callback( phys_collision_callback_t collision_callback )
{
  g_physics_system->m_collision_callback = collision_callback;
}

int phys_sys::get_rigid_body_alloc_count()
{
  return g_physics_system->m_list_rigid_body.get_count();
}

int phys_sys::get_user_rigid_body_alloc_count()
{
  return g_physics_system->m_list_user_rigid_body.get_count();
}

void phys_sys::phys_frame_advance( float delta_t )
{
  g_physics_system->frame_advance( delta_t );
}

void phys_sys::phys_init()
{
  physics_system::initialize();
}

void phys_sys::phys_shutdown()
{
  physics_system::shutdown();
}
