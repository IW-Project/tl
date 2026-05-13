#pragma once

#include "phys_base.h"
#include <phys_mem.h>

#include "rbc_def_contact.h"
#include "rbc_def_base.h"
#include "rbc_def_generic.h"
#include "rbc_def_vehicle.h"
#include "rbc_def_ragdoll.h"
#include "rbc_def_custom.h"
#include "rbc_def_wad.h"

#include "rigid_body.h"
#include "rigid_body_internal.h"

class rbcint
{
public:
  static phys_simple_link_list<contact_point_info> *get_list_cpi( rigid_body_constraint_contact *rbc )
  {
    return &rbc->m_list_contact_point_info_buffer_1;
  }

  static phys_simple_link_list<contact_point_info> *get_list_cached_cpi( rigid_body_constraint_contact *rbc )
  {
    return &rbc->m_list_contact_point_info_buffer_2;
  }

  static user_rigid_body **get_urb( rigid_body_constraint *rbc )
  {
    if ( rbc->get_b1() && rbc->get_b1()->is_user_rigid_body() )
    {
      return reinterpret_cast<user_rigid_body **>( rbc->get_b1() );
    }
    if ( rbc->get_b2() && rbc->get_b2()->is_user_rigid_body() )
    {
      return reinterpret_cast<user_rigid_body **>( rbc->get_b2() );
    }
    return NULL;
  }

  static void process_constraint_info( rigid_body_constraint_wheel *rbc_wheel )
  {
    process_constraint_info( reinterpret_cast<rigid_body_constraint *>( rbc_wheel ) );
    if ( rbc_wheel->get_wheel_flag( rigid_body_constraint_wheel::WHEEL_FLAG_IS_COLLIDING ) )
    {
      if ( rbc_wheel->get_b1() )
      {
        rbint::increment_contact_count( rbc_wheel->get_b1(), 1 );
      }
      if ( rbc_wheel->get_b2() )
      {
        rbint::increment_contact_count( rbc_wheel->get_b2(), 1 );
      }
    }
  }

  static void process_constraint_info( rigid_body_constraint_contact *rbc_contact )
  {
    int a = rbc_contact->get_cached_point_count();
    int b = rbc_contact->get_point_count();
    int c = tl_max( a, b );
    if ( rbc_contact->get_b1() )
    {
      rbint::increment_contact_count( rbc_contact->get_b1(), c );
      rbc_contact->get_b1()->set_flag( rigid_body::FLAG_CLIENT_FLAGS_START, 1 );
    }
    if ( rbc_contact->get_b2() )
    {
      rbint::increment_contact_count( rbc_contact->get_b2(), c );
      rbc_contact->get_b2()->set_flag( rigid_body::FLAG_CLIENT_FLAGS_START, 1 );
    }
  }

  static void process_constraint_info( rigid_body_constraint *rbc )
  {
    if ( rbc->get_b1() )
    {
      rbint::increment_constraint_count( rbc->get_b1() );
    }
    if ( rbc->get_b2() )
    {
      rbint::increment_constraint_count( rbc->get_b2() );
    }
  }

  static void set_b1( rigid_body_constraint *rbc, rigid_body *const rb ) { rbc->b1 = rb; }

  static void set_b2( rigid_body_constraint *rbc, rigid_body *const rb ) { rbc->b2 = rb; }

  static void set( rigid_body_constraint *rbc, rigid_body *const b1, rigid_body *const b2 )
  {
    rbc->b1 = b1;
    rbc->b2 = b2;
  }

  static void set_next( rigid_body_constraint *rbc, rigid_body_constraint *next ) { rbc->m_next = next; }

  template <typename T>
  static T *get_next( T *rbc )
  {
    return reinterpret_cast<T *>( rbc->m_next );
  }
};