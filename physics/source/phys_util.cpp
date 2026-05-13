#include "phys_util.h"

#include "rigid_body.h"

void nuge::get_ballistic_info( rigid_body *const list_rigid_body[],
                               const int rbodies_count,
                               phys_vec3 *center_of_mass,
                               phys_vec3 *total_momentum,
                               float *total_mass )
{
  PHYS_ASSERT_ALIGNED( *center_of_mass );
  PHYS_ASSERT_ALIGNED( *total_momentum );

  *center_of_mass = PHYS_ZERO_VEC;
  *total_momentum = PHYS_ZERO_VEC;
  *total_mass = 0.0f;

  for ( int i = 0; i < rbodies_count; ++i )
  {
    if ( list_rigid_body[i] )
    {
      const float mass = 1.0f / list_rigid_body[i]->get_inv_mass();
      *center_of_mass += ( mass * list_rigid_body[i]->get_mat().GetW() );
      *total_momentum += ( mass * list_rigid_body[i]->get_t_vel() );
      *total_mass += mass;
    }
  }

  tlAssert( *total_mass > 0.00001f );
  *center_of_mass /= *total_mass;
}

void nuge::apply_ballistic_target( rigid_body *const list_rigid_body[],
                                   const int rbodies_count,
                                   const phys_vec3 &target,
                                   float *const dist_sq )
{
  phys_vec3 center_of_mass, total_momentum;
  float total_mass;
  get_ballistic_info( list_rigid_body, rbodies_count, &center_of_mass, &total_momentum, &total_mass );

  const phys_vec3 total_velocity = total_momentum / total_mass;
  const phys_vec3 delta_pos = target - center_of_mass;
  *dist_sq = AbsSquared( delta_pos );
  float delta_z = delta_pos.GetZ();
  float delta_x = math::Sqrt( ( delta_pos.GetX() * delta_pos.GetX() ) + ( delta_pos.GetY() * delta_pos.GetY() ) );

  if ( delta_x < 0.0001f )
    return;

  const float g = list_rigid_body[0]->get_gravity_acc_vec().GetZ();
  const float A = ( 0.5f * g ) * delta_x;
  const float B = delta_z * delta_x;

  phys_vec3 dir( delta_pos.GetX(), delta_pos.GetY(), 0.0f );
  dir /= delta_x;
  phys_vec3 vt( phys_dot( total_velocity, dir ), 0.0f, total_velocity.GetZ() );
  float s_ = 1.0f;

  int iters = 0;
  bool done = false;
  while ( !done )
  {
    phys_vec3 vs( s_, 0.0f, ( B * s_ ) - ( A / s_ ) );
    phys_vec3 dvs( 1.0f, 0.0f, B + ( A / ( s_ * s_ ) ) );
    phys_vec3 ddvs( 0.0f, 0.0f, ( A * -2.0f ) / ( ( s_ * s_ ) * s_ ) );

    const float fs = phys_dot( dvs, vs - vt );
    const float dfs = phys_dot( ddvs, vs - vt ) + phys_dot( dvs, dvs );
    const float s1_ = s_ - ( fs / dfs );
    done = math::Abs( s1_ - s_ ) < 0.001f;
    s_ = s1_;
    if ( ++iters > 25 )
    {
      done = true;
    }
  }

  if ( math::Abs( s_ ) < 1.0 && math::Abs( s_ ) > 0.001f )
  {
    if ( s_ < 0.0f )
      s_ = -1.0f;
    else
      s_ = 1.0f;
  }

  phys_vec3 new_velocity = ( s_ * dir ) + phys_vec3( 0.0f, 0.0f, ( B * s_ ) - ( A / s_ ) );
  float total_square_mass = 0.0f;
  for ( int rb = 0; rb < rbodies_count; ++rb )
  {
    if ( list_rigid_body[rb] )
    {
      const float inv_mass = list_rigid_body[rb]->get_inv_mass();
      total_square_mass = total_square_mass + ( 1.0f / ( phys_sqr( inv_mass ) ) );
    }
  }
  tlAssert( total_square_mass > 0.00001f );
  phys_vec3 C = ( total_mass / total_square_mass ) * ( new_velocity - total_velocity );
  for ( int rb = 0; rb < rbodies_count; ++rb )
  {
    if ( list_rigid_body[rb] )
    {
      list_rigid_body[rb]->add_force( C / phys_sqr( list_rigid_body[rb]->get_inv_mass() ) );
    }
  }
}

void nuge::calc_velocities( const phys_mat44 &mat0, const phys_mat44 &mat1, const float delta_t, phys_vec3 *t_vel, phys_vec3 *a_vel )
{
  PHYS_ASSERT_ALIGNED( *t_vel );
  PHYS_ASSERT_ALIGNED( *a_vel );

  const float oo_delta_t = 1.0f / delta_t;
  phys_mat44 vel_mat;
  phys_transpose( vel_mat, mat0 );
  phys_multiply_mat( vel_mat, mat1, vel_mat );
  Phys_UnitQuaternion qvel = Phys_GetQuaternion( vel_mat );
  float nqvec = Abs( qvel.GetImag() );
  if ( nqvec > 0.00001f )
  {
    *a_vel = ( ( -oo_delta_t / nqvec ) * ( math::ACos( qvel.GetReal() ) * 2.0f ) ) * qvel.GetImag();
  }
  else
  {
    *a_vel = PHYS_ZERO_VEC;
  }
  *t_vel = oo_delta_t * ( mat1.GetW() - mat0.GetW() );
}

void nuge::calc_velocities( const phys_mat44 &mat0,
                            const phys_mat44 &mat1,
                            const phys_vec3 &center_offset_loc,
                            const float delta_t,
                            phys_vec3 *t_vel,
                            phys_vec3 *a_vel )
{
  PHYS_ASSERT_ALIGNED( *t_vel );
  PHYS_ASSERT_ALIGNED( *a_vel );

  calc_velocities( mat0, mat1, delta_t, t_vel, a_vel );
  *t_vel += phys_cross( *a_vel, phys_multiply( mat1, center_offset_loc ) );
}

void nuge::calc_sphere_inertia( const float radius, phys_vec3 *unit_inertia, float *volume )
{
  PHYS_ASSERT_ALIGNED( *unit_inertia );

  *volume = ( ( ( ( PHYS_PI_ * radius ) * radius ) * radius ) * 4.0 ) / 3.0;
  const float d = ( ( ( *volume * radius ) * radius ) * 2.0 ) / 5.0;
  *unit_inertia = phys_vec3( d, d, d );
}

void nuge::calc_box_inertia( const phys_vec3 &dim, phys_vec3 *unit_inertia, float *volume )
{
  PHYS_ASSERT_ALIGNED( *unit_inertia );

  *volume = dim.GetX() * dim.GetY() * dim.GetZ();
  phys_vec3 moments = ( ( ( dim.GetX() * 0.083333336 ) * dim.GetY() ) * dim.GetZ() ) *
                      ( phys_vec3( phys_sqr( dim.GetX() ), phys_sqr( dim.GetY() ), phys_sqr( dim.GetZ() ) ) );

  *unit_inertia = phys_vec3( moments.GetY() + moments.GetZ(), moments.GetX() + moments.GetZ(), moments.GetX() + moments.GetY() );
}

void nuge::calc_bound_sphere( const phys_vec3 *vert_list, const int vert_count, float *radius, phys_vec3 *com )
{
  PHYS_ASSERT_ALIGNED( *com );
  PHYS_ASSERT_ALIGNED( *vert_list );

  phys_vec3 dir3_com = PHYS_ZERO_VEC;
  for ( int i = 0; i < vert_count; ++i )
  {
    dir3_com += vert_list[i];
  }
  dir3_com /= float( vert_count );
  *radius = 0.0f;
  for ( int i = 0; i < vert_count; ++i )
  {
    const float r = AbsSquared( vert_list[i] - dir3_com );
    if ( r > *radius )
    {
      *radius = r;
    }
  }
  *radius = math::Sqrt( *radius );
  *com = dir3_com;
}

void nuge::calc_bound_box( const phys_vec3 *vert_list, const int vert_count, phys_vec3 *dim, phys_vec3 *com )
{
  PHYS_ASSERT_ALIGNED( *vert_list );
  PHYS_ASSERT_ALIGNED( *dim );
  PHYS_ASSERT_ALIGNED( *com );

  phys_vec3 vmin( 10000000.0, 10000000.0, 10000000.0 );
  phys_vec3 vmax( -10000000.0, -10000000.0, -10000000.0 );

  for ( int i = 0; i < vert_count; ++i )
  {
    phys_vec3 v = vert_list[i];
    if ( vmin.GetX() > v.GetX() )
      vmin.SetX( v.GetX() );
    if ( vmax.GetX() < v.GetX() )
      vmax.SetX( v.GetX() );
    if ( vmin.GetY() > v.GetY() )
      vmin.SetY( v.GetY() );
    if ( vmax.GetY() < v.GetY() )
      vmax.SetY( v.GetY() );
    if ( vmin.GetZ() > v.GetZ() )
      vmin.SetZ( v.GetZ() );
    if ( vmax.GetZ() < v.GetZ() )
      vmax.SetZ( v.GetZ() );
  }

  *com = 0.5f * ( vmin + vmax );
  *dim = vmax - vmin;
}
