#pragma once

#include "rigid_body.h"
#include "constraint_solver/pulse_sum_base.h"
#include "phys_util.h"

class rbint
{
public:
  static const phys_vec3 inv_L( rigid_body *rb, phys_vec3 &v, const float delta_t )
  {
    return delta_t * phys_multiply( rb->m_node->m_world_inv_inertia, v );
  }
  static const phys_vec3 mul_inv_L( rigid_body *rb, const phys_vec3 &t )
  {
    return phys_inv_diag_multiply( rb->m_inv_inertia, phys_inv_multiply( rb->get_mat(), t ) );
  }
  static const phys_vec3 mul_L( rigid_body *rb, const phys_vec3 &t )
  {
    return phys_multiply( rb->get_mat(), phys_inv_diag_multiply( rb->m_inv_inertia, phys_inv_multiply( rb->get_mat(), t ) ) );
  }

  static pulse_sum_node *get_pulse_sum_node( rigid_body *rb ) { return rb->m_node; }

  static const phys_mat44 *get_dictator( const user_rigid_body *rb )
  {
    tlAssert( rb->is_user_rigid_body() );
    return rb->m_dictator;
  }

  static void prolog_frame_advance( user_rigid_body *rb, const float delta_t )
  {
    tlAssert( rb->is_user_rigid_body() );
    tlAssert( rb->m_dictator );

    nuge::calc_velocities( rb->m_mat, *rb->m_dictator, delta_t, &rb->m_t_vel, &rb->m_a_vel );

    VALIDATE_POSITION_VECTOR( rb->m_t_vel, rb );
    VALIDATE_POSITION_VECTOR( rb->m_a_vel, rb );
  }

  static void collision_prolog( rigid_body *const rb, const float delta_t )
  {
    rb->m_moved_vec = ( rb->m_mat.GetW() + ( delta_t * rb->m_t_vel ) ) - rb->m_last_position;
    VALIDATE_POSITION_VECTOR( rb->m_moved_vec, rb );
    rb->m_smallest_lambda = 1.0f;
    rb->swap_last_position();
    rb->set_largest_vel_sq( 0.0f );
  }
  static void collision_prolog( user_rigid_body *rb, const float delta_t )
  {
    tlAssert( rb->is_user_rigid_body() );
    tlAssert( rb->m_dictator );

    rb->m_moved_vec = delta_t * rb->m_t_vel;
    VALIDATE_POSITION_VECTOR( rb->m_moved_vec, rb );
    rb->m_smallest_lambda = 0.0f;
  }
  static void substep( user_rigid_body *rb, const float delta_t )
  {
    phys_mat44 rv;

    rb->m_mat.GetW() += delta_t * rb->m_t_vel;
    make_rotate( &rv, rb->m_a_vel, delta_t );
    phys_multiply_mat( rb->m_mat, rv, rb->m_mat );
  }
  static void take_next_step( user_rigid_body *rb, const float delta_t )
  {
    tlAssert( rb->is_user_rigid_body() );
    tlAssert( rb->m_dictator );

    rb->m_mat.GetW() += delta_t * rb->m_t_vel;
    phys_mat44 rv;
    make_rotate( &rv, rb->m_a_vel, delta_t );
    phys_multiply_mat( rb->m_mat, rv, rb->m_mat );
  }
  static void take_last_step( user_rigid_body *rb, const float delta_t )
  {
    tlAssert( rb->is_user_rigid_body() );
    tlAssert( rb->m_dictator );

    rb->m_mat = *rb->m_dictator;
  }
  static void solver_prolog( rigid_body *rb, const float delta_t )
  {
    rb->m_force_sum /= delta_t;
    rb->m_torque_sum /= delta_t;

    VALIDATE_POSITION_VECTOR( rb->m_force_sum, rb );
    VALIDATE_POSITION_VECTOR( rb->m_torque_sum, rb );
  }
  static void solver_epilog( rigid_body *const rb, const float delta_t )
  {
    rb->m_force_sum = PHYS_ZERO_VEC;
    rb->m_torque_sum = PHYS_ZERO_VEC;
  }
  static void euler_integrate_velocity( rigid_body *const rb, const float delta_t )
  {
    VALIDATE_POSITION_VECTOR( rb->m_t_vel, rb );
    VALIDATE_POSITION_VECTOR( rb->m_a_vel, rb );

    rb->m_last_t_vel = rb->m_t_vel;
    rb->m_last_a_vel = rb->m_a_vel;

    const float inv_mass = rb->get_inv_mass();
    const phys_vec3 accel = ( inv_mass * rb->m_force_sum ) + rb->m_gravity_acc_vec;
    rb->m_t_vel += delta_t * accel;

    rb->m_a_vel += rbint::inv_L( rb, rb->m_torque_sum, delta_t );

    const phys_vec3 a_vel_loc1 = phys_inv_multiply( rb->m_mat, rb->m_a_vel );
    const phys_vec3 a_mom_loc1 = phys_inv_diag_multiply( rb->m_inv_inertia, a_vel_loc1 );
    const float ake_times_two_1 = phys_dot( a_mom_loc1, a_vel_loc1 );

    const phys_vec3 gyro_loc = phys_cross( a_mom_loc1, a_vel_loc1 );
    const phys_vec3 gyro_scaled_loc = phys_inv_diag_multiply( rb->m_inv_inertia, gyro_loc );
    rb->m_a_vel += phys_multiply( rb->m_mat, delta_t * gyro_scaled_loc );

    const float navel_sq = AbsSquared( rb->m_a_vel );
    if ( navel_sq > phys_sqr( rb->m_max_avel ) )
    {
      if ( navel_sq <= phys_sqr( 0.00001f ) )
      {
        tlAssert( navel_sq > phys_sqr( 0.00001f ) );
      }

      rb->m_a_vel *= rb->m_max_avel / sqrtf( navel_sq );
    }

    const phys_vec3 a_vel_loc2 = phys_inv_multiply( rb->m_mat, rb->m_a_vel );
    const phys_vec3 a_mom_loc2 = phys_inv_diag_multiply( rb->m_inv_inertia, a_vel_loc2 );
    const float ake_times_two_2 = phys_dot( a_mom_loc2, a_vel_loc2 );

    if ( ake_times_two_2 > ake_times_two_1 && ake_times_two_2 > 0.00001f )
    {
      rb->m_a_vel *= sqrtf( ake_times_two_1 / ake_times_two_2 );
    }

    const float dthresh = 0.89999998f;
    const float t_drag = tl_min( delta_t * rb->m_t_drag_coef, dthresh );
    rb->m_t_vel *= ( 1.0f - t_drag );

    const float a_drag = tl_min( delta_t * rb->m_a_drag_coef, dthresh );
    rb->m_a_vel *= ( 1.0f - a_drag );

    VALIDATE_POSITION_VECTOR( rb->m_t_vel, rb );
    VALIDATE_POSITION_VECTOR( rb->m_a_vel, rb );
  }
  static void euler_integrate_pos( rigid_body *const rb, const float delta_t )
  {
    rb->m_mat.GetW() += delta_t * rb->m_t_vel;

    phys_mat44 rv;
    make_rotate( &rv, rb->m_a_vel, delta_t, 1000.0f );
    phys_multiply_mat( rb->m_mat, rv, rb->m_mat );

    if ( ++rb->m_tick > 5 )
    {
      rb->m_tick = 0;
      orthonormalize( &rb->m_mat );
    }

    VALIDATE_POSITION_VECTOR( rb->m_mat.GetW(), rb );
  }
  static void setup_constraint( rigid_body *rb, pulse_sum_node *psn )
  {
    tlAssert( rb );
    tlAssert( rb->get_inv_mass() > 0.00001f );

    nuge::tensor_transform_principle( rb->m_inv_inertia, rb->m_mat, &psn->m_world_inv_inertia );
    rb->m_node = psn;
    psn->m_rb = rb;
    psn->m_inv_mass = rb->get_inv_mass();
  }
  static void add_vel( rigid_body *rb, phys_vec3 &t, phys_vec3 &a )
  {
    rb->m_t_vel += t;
    rb->m_a_vel += a;

    VALIDATE_POSITION_VECTOR( rb->m_t_vel, rb );
    VALIDATE_POSITION_VECTOR( rb->m_a_vel, rb );

    const float ntvel_sq = AbsSquared( rb->m_t_vel );
    const float navel_sq = AbsSquared( rb->m_a_vel );
    if ( ( ntvel_sq + navel_sq ) > phys_sqr( 100000.0f ) )
    {
      rb->set_flag( rigid_body::FLAG_DANGEROUS, 1 );
    }
  }
  static void add_pos_t( rigid_body *rb, phys_vec3 &r ) { r += rb->m_mat.GetW(); }
  static void update_stability( rigid_body *const rb, const float delta_t )
  {
    float max_stable_energy_sq = phys_sqr( 11.9f );
    if ( !rb->m_stable_min_contact_count )
    {
      max_stable_energy_sq = phys_sqr( 5.4400001f );
    }

    if ( rb->m_largest_vel_sq <= max_stable_energy_sq )
    {
      if ( !rb->is_stable() )
      {
        rb->m_stable_energy_time = rb->m_stable_energy_time + delta_t;
        if ( rb->m_stable_energy_time >= 1.0f ||
             ( rb->m_stable_energy_time >= 0.5f && rb->m_contact_count >= rb->m_stable_min_contact_count ) )
        {
          rb->set_flag( rigid_body::FLAG_STABLE, 1 );
        }
      }
    }
    else
    {
      rb->m_stable_energy_time = 0.0f;
      rb->set_flag( rigid_body::FLAG_STABLE, 0 );
    }
  }
  static const phys_vec3 multiply( const rigid_body *rb, const phys_vec3 &r ) { return phys_multiply( rb->get_mat(), r ); }
  static const phys_vec3 full_multiply( const rigid_body *rb, phys_vec3 &r ) { return phys_full_multiply( rb->get_mat(), r ); }

  static const phys_vec3 add_pos( rigid_body *rb, const phys_vec3 &r ) { return rb->get_mat().GetW() + r; }

  static const phys_vec3 sub_pos( const rigid_body *rb, phys_vec3 &r ) { return r - rb->get_mat().GetW(); }

  static const phys_vec3 gtv( rigid_body *rb, const phys_vec3 &r )
  {
    phys_vec3 _a = rb->m_a_vel;
    phys_vec3 _b = phys_cross( _a, r );
    return rb->m_t_vel + _b;
  }

  static const phys_vec3 last_gtv( const rigid_body *rb, phys_vec3 &r )
  {
    phys_vec3 _a = rb->m_last_a_vel;
    phys_vec3 _b = phys_cross( _a, r );
    return rb->m_last_t_vel + _b;
  }

  static int verify_pulse_sum_node( rigid_body *rb )
  {
    if ( rb->m_node )
    {
      return rb->get_flag( rigid_body::FLAG_ENVIRONMENT_RIGID_BODY | rigid_body::FLAG_USER_RIGID_BODY ) == 0;
    }
    else
    {
      return rb->get_flag( rigid_body::FLAG_ENVIRONMENT_RIGID_BODY | rigid_body::FLAG_USER_RIGID_BODY );
    }
  }

  static phys_vec3 &get_last_t_vel( rigid_body *rb ) { return rb->m_last_t_vel; }

  static phys_vec3 &get_last_a_vel( rigid_body *rb ) { return rb->m_last_a_vel; }

  static void constraint_info_reset( rigid_body *rb )
  {
    rb->m_constraint_count = 0;
    rb->m_contact_count = 0;
    rb->set_flag( 0x100u, 0 );
  }
  static void increment_constraint_count( rigid_body *rb ) { ++rb->m_constraint_count; }
  static void increment_contact_count( rigid_body *rb, const int c ) { rb->m_contact_count += c; }
};

#define LAST_T_VEL( ptr ) rbint::get_last_t_vel( ptr )
#define LAST_A_VEL( ptr ) rbint::get_last_a_vel( ptr )
