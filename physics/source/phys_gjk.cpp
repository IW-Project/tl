#include "collision/phys_gjk.h"

const int BIT_COUNT[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4 };

int backup1( phys_gjk_info *gjk_info, const int new_index, const bool seed_simplex )
{
  phys_vec3 *new_w = &gjk_info->m_w_verts[new_index];
  float new_dist_sq = AbsSquared( *new_w );
  int new_w_set = 1 << new_index;
  gjk_info->m_set_list[new_w_set].m_lamda[new_index] = 1.0f;

  const int last_w_set = ~( 1 << new_index ) & gjk_info->m_w_set;

  int w_inds[3] = { 0, 0, 0 };
  float dotps[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
  phys_vec3 difs[3];
  for ( int i = 0; i < 3; ++i )
    difs[i] = phys_vec3( 0.0f );

  int w_count = 0;
  int mask = 1;
  for ( int i = 0; i < 4; ++i )
  {
    if ( ( mask & last_w_set ) != 0 )
    {
      w_inds[w_count] = i;
      difs[w_count] = gjk_info->m_w_verts[i] - *new_w;
      dotps[w_count + 3] = AbsSquared( difs[w_count] );
      dotps[w_count] = phys_dot( *new_w, difs[w_count] );

      if ( dotps[w_count + 3] > 9.9999994e-11f )
      {
        const float c0 = -dotps[w_count] / dotps[w_count + 3];
        const float c1 = 1.0f - c0;
        if ( c0 > 0.0f && c1 > 0.0f )
        {
          const phys_vec3 ww = *new_w + ( difs[w_count] * c0 );
          const float nww_sq = AbsSquared( ww );
          if ( new_dist_sq > nww_sq )
          {
            new_w_set = ( 1 << new_index ) | mask;
            new_dist_sq = nww_sq;
            phys_gjk_info::phys_gjk_set_info *lams = &gjk_info->m_set_list[new_w_set];
            lams->m_lamda[i] = c0;
            lams->m_lamda[new_index] = c1;
          }
        }
      }

      if ( seed_simplex )
      {
        const float nww_sq_0 = AbsSquared( gjk_info->m_w_verts[i] );
        if ( new_dist_sq > nww_sq_0 )
        {
          new_w_set = mask;
          new_dist_sq = nww_sq_0;
          gjk_info->m_set_list[mask].m_lamda[i] = 1.0f;
        }
      }

      ++w_count;
    }

    mask *= 2;
  }

  if ( seed_simplex && w_count == 2 )
  {
    const phys_vec3 dif = gjk_info->m_w_verts[2] - gjk_info->m_w_verts[1];
    const float ndif_sq = AbsSquared( dif );
    if ( ndif_sq > 9.9999994e-11f )
    {
      const float c0 = -phys_dot( gjk_info->m_w_verts[1], dif ) / ndif_sq;
      const float c1 = 1.0f - c0;
      if ( c0 > 0.0f && c1 > 0.0f )
      {
        const phys_vec3 ww = gjk_info->m_w_verts[1] + ( dif * c0 );
        const float nww_sq = AbsSquared( ww );
        if ( new_dist_sq > nww_sq )
        {
          new_w_set = 6;
          new_dist_sq = nww_sq;
          gjk_info->m_set_list[6].m_lamda[1] = c1;
          gjk_info->m_set_list[6].m_lamda[2] = c0;
        }
      }
    }
  }

  for ( int i = 0; i < w_count - 1; ++i )
  {
    for ( int j = i + 1; j < w_count; ++j )
    {
      const float a00 = dotps[i + 3];
      const float a01 = phys_dot( difs[i], difs[j] );
      const float a11 = dotps[j + 3];
      const float b0 = -dotps[i];
      const float b1 = -dotps[j];
      const float denom = ( a00 * a11 ) - ( a01 * a01 );
      if ( denom > 0.0000099999997f )
      {
        const float c0 = ( ( b0 * a11 ) - ( b1 * a01 ) ) / denom;
        const float c1 = ( ( a00 * b1 ) - ( a01 * b0 ) ) / denom;
        const float c2 = ( 1.0f - c0 ) - c1;
        if ( c0 > 0.0f && c1 > 0.0f && c2 > 0.0f )
        {
          const phys_vec3 ww = *new_w + ( difs[i] * c0 ) + ( difs[j] * c1 );
          const float nww_sq = AbsSquared( ww );
          if ( new_dist_sq > nww_sq )
          {
            new_w_set = ( 1 << new_index ) | ( 1 << w_inds[j] ) | ( 1 << w_inds[i] );
            new_dist_sq = nww_sq;
            phys_gjk_info::phys_gjk_set_info *lams = &gjk_info->m_set_list[new_w_set];
            lams->m_lamda[w_inds[i]] = c0;
            lams->m_lamda[w_inds[j]] = c1;
            lams->m_lamda[new_index] = c2;
          }
        }
      }
    }
  }

  if ( w_count == 3 && gjk_info->m_lower_dist_sq <= 0.0f )
  {
    const phys_vec3 cr01 = phys_cross( difs[0], difs[1] );
    const float denom = phys_dot( cr01, difs[2] );
    if ( fabsf( denom ) > 0.0000099999997f )
    {
      const float c2 = -phys_dot( cr01, *new_w ) / denom;
      if ( c2 >= 0.0f )
      {
        const phys_vec3 crw2 = phys_cross( *new_w, difs[2] );
        const float c0 = phys_dot( crw2, difs[1] ) / denom;
        if ( c0 >= 0.0f )
        {
          const float c1 = -phys_dot( crw2, difs[0] ) / denom;
          if ( c1 >= 0.0f && ( ( ( 1.0f - c0 ) - c1 ) - c2 ) >= 0.0f )
            return 15;
        }
      }
    }
  }

  return new_w_set;
}

void phys_gjk_info::set_flag( const int f, const int b )
{
  if ( b )
    m_flags |= f;
  else
    m_flags &= ~f;
}

int phys_gjk_info::get_flag( const int f )
{
  return ( m_flags & f ) != 0;
}

phys_gjk_info::phys_gjk_set_info *phys_gjk_info::get_set_info( const int w_set )
{
  return &m_set_list[w_set & 0xF];
}

const int phys_gjk_info::compress_verts( const int w_set )
{
  for ( int i = 0, packed = 0; i < 4; ++i )
  {
    if ( ( w_set & ( 1 << i ) ) != 0 )
    {
      m_a_verts[packed] = m_a_verts[i];
      m_b_verts[packed] = m_b_verts[i];
      m_a_inds[packed] = m_a_inds[i];
      m_b_inds[packed] = m_b_inds[i];
      ++packed;
    }
    if ( i == 3 )
      return packed;
  }

  return 0;
}

void phys_gjk_info::comp_v( const int w_set, phys_vec3 *v )
{
  tlAssert( w_set > 0 && w_set < 15 );

  phys_vec3 list_w_vert[3];
  int list_w_vert_count = 0;
  int i_index = 0;
  int bit = 1;

  while ( i_index < 4 )
  {
    if ( ( bit & w_set ) != 0 )
    {
      list_w_vert[list_w_vert_count] = m_w_verts[i_index];
      ++list_w_vert_count;
    }
    ++i_index;
    bit *= 2;
  }

  tlAssert( list_w_vert_count > 0 && list_w_vert_count < 4 );

  if ( list_w_vert_count == 1 )
  {
    *v = list_w_vert[0];
    return;
  }

  phys_vec3 e1 = list_w_vert[1] - list_w_vert[0];
  float ne1_sq = phys_dot( e1, e1 );

  if ( list_w_vert_count == 2 )
  {
    tlAssert( ne1_sq > 0.0f );
    float t = phys_dot( e1, list_w_vert[0] ) / ne1_sq;
    phys_vec3 closest_on_line = e1 * t;
    *v = list_w_vert[0] - closest_on_line;
    return;
  }

  tlAssert( list_w_vert_count == 3 );

  phys_vec3 e2 = list_w_vert[2] - list_w_vert[0];
  phys_vec3 normal = phys_cross( e1, e2 );
  float nnormal_sq = phys_dot( normal, normal );

  if ( nnormal_sq > 0.01f )
  {
    float t = phys_dot( normal, list_w_vert[0] ) / nnormal_sq;
    *v = normal * t;
    return;
  }

  phys_vec3 side[3];
  side[0] = e1;
  side[1] = list_w_vert[2] - list_w_vert[1];
  side[2] = -e2;

  float nside_sq[3];
  nside_sq[0] = phys_dot( side[0], side[0] );
  nside_sq[1] = phys_dot( side[1], side[1] );
  nside_sq[2] = phys_dot( side[2], side[2] );

  int side_i = 0;
  if ( nside_sq[0] < nside_sq[1] )
  {
    if ( nside_sq[1] < nside_sq[2] )
      side_i = 2;
    else
      side_i = 1;
  }
  else if ( nside_sq[0] < nside_sq[2] )
  {
    side_i = 2;
  }

  tlAssert( nside_sq[side_i] > 0.0f );

  float t = phys_dot( side[side_i], list_w_vert[side_i] ) / nside_sq[side_i];
  phys_vec3 closest_on_side = side[side_i] * t;
  *v = list_w_vert[side_i] - closest_on_side;
}

void phys_gjk_info::comp_closest_points( const int w_set, phys_vec3 *a, phys_vec3 *b )
{
  *a = phys_vec3( 0.0f );
  *b = phys_vec3( 0.0f );

  float lambda_sum = 0.0f;
  phys_gjk_set_info *si_w_set = get_set_info( w_set );

  int i_index = 0;
  int bit = 1;

  while ( i_index < 4 )
  {
    if ( ( bit & w_set ) != 0 )
    {
      float lambda = si_w_set->m_lamda[i_index];
      *a = *a + m_a_verts[i_index] * lambda;
      *b = *b + m_b_verts[i_index] * lambda;
      lambda_sum = lambda_sum + lambda;
    }
    ++i_index;
    bit *= 2;
  }

  tlAssert( lambda_sum >= 0.0f );

  float lambda_inv = 1.0f / lambda_sum;
  *a = *a * lambda_inv;
  *b = *b * lambda_inv;
}

void phys_gjk_info::comp_lambda_1( const int w_set )
{
  phys_gjk_set_info *si = get_set_info( w_set );
  for ( int k = 0; k < 4; ++k )
    si->m_lamda[k] = 0.0f;

  int i = 0;
  while ( i < 4 && ( ( w_set & ( 1 << i ) ) == 0 ) )
    ++i;

  if ( i < 4 )
    si->m_lamda[i] = 1.0f;
  si->m_candidate = 1;
}

void phys_gjk_info::comp_lambda_2( const int w_set, const int i )
{
  phys_gjk_set_info *si = get_set_info( w_set );
  for ( int k = 0; k < 4; ++k )
    si->m_lamda[k] = 0.0f;

  int j = -1;
  for ( int k = 0; k < 4; ++k )
  {
    if ( k != i && ( w_set & ( 1 << k ) ) != 0 )
    {
      j = k;
      break;
    }
  }

  if ( j < 0 )
  {
    si->m_lamda[i] = 1.0f;
    si->m_candidate = 0;
    return;
  }

  const phys_vec3 dif = m_w_verts[j] - m_w_verts[i];
  const float denom = AbsSquared( dif );
  if ( denom <= 9.9999994e-11f )
  {
    si->m_lamda[i] = 1.0f;
    si->m_candidate = 0;
    return;
  }

  const float c_j = -phys_dot( m_w_verts[i], dif ) / denom;
  const float c_i = 1.0f - c_j;
  si->m_lamda[i] = c_i;
  si->m_lamda[j] = c_j;
  si->m_candidate = ( c_i >= 0.0f && c_j >= 0.0f );
}

void phys_gjk_info::comp_lambda_3( const int w_set, const int i, const int j )
{
  phys_gjk_set_info *si = get_set_info( w_set );
  for ( int k = 0; k < 4; ++k )
    si->m_lamda[k] = 0.0f;

  int k = -1;
  for ( int n = 0; n < 4; ++n )
  {
    if ( n != i && n != j && ( w_set & ( 1 << n ) ) != 0 )
    {
      k = n;
      break;
    }
  }

  if ( k < 0 )
  {
    si->m_lamda[i] = 1.0f;
    si->m_candidate = 0;
    return;
  }

  const phys_vec3 e0 = m_w_verts[j] - m_w_verts[i];
  const phys_vec3 e1 = m_w_verts[k] - m_w_verts[i];
  const phys_vec3 rhs = -m_w_verts[i];

  const float d00 = phys_dot( e0, e0 );
  const float d01 = phys_dot( e0, e1 );
  const float d11 = phys_dot( e1, e1 );
  const float r0 = phys_dot( rhs, e0 );
  const float r1 = phys_dot( rhs, e1 );
  const float denom = ( d00 * d11 ) - ( d01 * d01 );

  if ( fabsf( denom ) <= 0.0000099999997f )
  {
    si->m_lamda[i] = 1.0f;
    si->m_candidate = 0;
    return;
  }

  const float c_j = ( ( r0 * d11 ) - ( r1 * d01 ) ) / denom;
  const float c_k = ( ( d00 * r1 ) - ( d01 * r0 ) ) / denom;
  const float c_i = ( 1.0f - c_j ) - c_k;

  si->m_lamda[i] = c_i;
  si->m_lamda[j] = c_j;
  si->m_lamda[k] = c_k;
  si->m_candidate = ( c_i >= 0.0f && c_j >= 0.0f && c_k >= 0.0f );
}

void phys_gjk_info::comp_lambda_4()
{
  phys_gjk_set_info *si = get_set_info( m_w_set );
  for ( int k = 0; k < 4; ++k )
    si->m_lamda[k] = 0.0f;

  int inds[4];
  int n = 0;
  for ( int i = 0; i < 4; ++i )
  {
    if ( ( m_w_set & ( 1 << i ) ) != 0 )
      inds[n++] = i;
  }

  if ( n != 4 )
  {
    si->m_candidate = 0;
    if ( n > 0 )
      si->m_lamda[inds[0]] = 1.0f;
    return;
  }

  const phys_vec3 &p0 = m_w_verts[inds[0]];
  const phys_vec3 &p1 = m_w_verts[inds[1]];
  const phys_vec3 &p2 = m_w_verts[inds[2]];
  const phys_vec3 &p3 = m_w_verts[inds[3]];

  const phys_vec3 a = p1 - p0;
  const phys_vec3 b = p2 - p0;
  const phys_vec3 c = p3 - p0;
  const phys_vec3 rhs = -p0;

  const float det = phys_dot( a, phys_cross( b, c ) );
  if ( fabsf( det ) <= 0.0000099999997f )
  {
    si->m_lamda[inds[0]] = 1.0f;
    si->m_candidate = 0;
    return;
  }

  const float l1 = phys_dot( rhs, phys_cross( b, c ) ) / det;
  const float l2 = phys_dot( a, phys_cross( rhs, c ) ) / det;
  const float l3 = phys_dot( a, phys_cross( b, rhs ) ) / det;
  const float l0 = ( ( 1.0f - l1 ) - l2 ) - l3;

  si->m_lamda[inds[0]] = l0;
  si->m_lamda[inds[1]] = l1;
  si->m_lamda[inds[2]] = l2;
  si->m_lamda[inds[3]] = l3;
  si->m_candidate = ( l0 >= 0.0f && l1 >= 0.0f && l2 >= 0.0f && l3 >= 0.0f );
}

const int phys_gjk_info::gjk_subalgorithm( const int w_set, const int new_index )
{
  (void)w_set;
  return backup1( this, new_index, false );
}

const int phys_gjk_info::seed_simplex( const int cached_vert_count )
{
  tlAssert( cached_vert_count > 0 );
  tlAssert( cached_vert_count < 4 );

  m_w_set = 0;
  int bit = 1;

  for ( int i = 0; i < cached_vert_count; ++i )
  {
    phys_vec3 w = m_a_verts[i] - m_b_verts[i];
    m_w_verts[i] = w - m_gjk_origin;
    m_w_set |= bit;
    bit *= 2;
  }

  return backup1( this, 0, true );
}

const int phys_gjk_info::init_gjk( const phys_gjk_input *d, phys_vec3 &initial_support_dir, const bool in_separation_loop )
{
  int cached_vert_count;

  if ( in_separation_loop )
  {
    cached_vert_count = compress_verts( m_w_set );
  }
  else
  {
    phys_gjk_cache_info *gjk_ci = d->gjk_ci;
    if ( gjk_ci && gjk_ci->is_simplex_valid() )
    {
      get_simplex( d->gjk_cg1, d->gjk_cg2, gjk_ci, m_a_verts, m_a_inds, m_b_verts, m_b_inds, &cached_vert_count );
      for ( int i = 0; i < cached_vert_count; ++i )
        m_b_verts[i] = phys_full_multiply( cg2_to_cg1_xform, m_b_verts[i] );
    }
    else
    {
      cached_vert_count = 0;
    }
  }

  if ( cached_vert_count )
  {
    m_w_set = seed_simplex( cached_vert_count );
    comp_v( m_w_set, &m_support_dir );
    m_last_w_set = m_w_set;
    return 1;
  }

  m_support_dir = initial_support_dir;
  m_last_w_set = 0;
  m_w_set = 0;
  return 0;
}

phys_gjk_info::gjk_retval_e phys_gjk_info::gjk( const phys_gjk_input *d, phys_vec3 &initial_support_dir, const bool in_separation_loop )
{
  m_lower_dist_sq = -34.0f;
  m_upper_dist_sq = 34.0f;
  m_gjk_iter = init_gjk( d, initial_support_dir, in_separation_loop );

  do
  {
    m_upper_dist_sq = AbsSquared( m_support_dir );
    if ( m_gjk_iter && m_gjk_pen_thresh_sq > m_upper_dist_sq )
      return GJK_PENETRATING;

    int new_index;
    if ( ( m_w_set & 1 ) != 0 )
    {
      if ( ( m_w_set & 2 ) != 0 )
        new_index = ( ( m_w_set & 4 ) != 0 ) ? 3 : 2;
      else
        new_index = 1;
    }
    else
    {
      new_index = 0;
    }

    d->gjk_cg1->support( -m_support_dir, &m_a_verts[new_index], &m_a_inds[new_index] );
    d->gjk_cg2->support( phys_inv_multiply( cg2_to_cg1_xform, m_support_dir ), &m_b_verts[new_index], &m_b_inds[new_index] );
    m_b_verts[new_index] = phys_full_multiply( cg2_to_cg1_xform, m_b_verts[new_index] );

    const phys_vec3 w = ( m_a_verts[new_index] - m_b_verts[new_index] ) - m_gjk_origin;
    const float dotvw = phys_dot( m_support_dir, w );
    if ( dotvw > 0.0f && m_upper_dist_sq > 0.0f )
    {
      const float lower_dist_sq = phys_sqr( dotvw ) / m_upper_dist_sq;
      if ( lower_dist_sq > m_lower_dist_sq )
      {
        m_lower_dist_sq = lower_dist_sq;
        if ( get_flag( FLAG_EXIT_ON_SEP_THRESH ) && m_lower_dist_sq > phys_sqr( m_gjk_sep_thresh ) )
          return GJK_SEPARATED;
      }
    }

    if ( m_gjk_iter && m_lower_dist_sq > 0.0f )
    {
      if ( m_lower_dist_sq > m_upper_dist_sq * phys_sqr( 1.0f - 0.001f ) )
        return GJK_VALID;

      for ( int i = 0, bit = 1; i < 4; ++i, bit *= 2 )
      {
        if ( ( bit & m_last_w_set ) != 0 )
        {
          const float n = AbsSquared( w - m_w_verts[i] );
          if ( phys_sqr( 0.001f ) > n )
            return GJK_VALID;
        }
      }
    }

    m_w_verts[new_index] = w;
    m_w_set |= 1 << new_index;
    m_last_w_set = m_w_set;
    m_w_set = gjk_subalgorithm( m_w_set, new_index );
    if ( m_w_set == 15 )
      return GJK_PENETRATING;

    comp_v( m_w_set, &m_support_dir );
    ++m_gjk_iter;
  } while ( m_gjk_iter < 30 );

  return GJK_VALID;
}

phys_gjk_info::gjk_retval_e phys_gjk_info::collide( const phys_gjk_input *d )
{
  phys_vec3 initial_support_dir = get_initial_support_dir( d );
  return gjk( d, initial_support_dir, false );
}

phys_gjk_info::gjk_retval_e
phys_gjk_info::gjk_ray_cast( const phys_gjk_input *d, phys_vec3 &initial_support_dir, const bool in_separation_loop )
{
  m_continuous_collision_lambda = d->m_start_time;
  m_gjk_origin = m_cg1_relative_translation_loc * ( -m_continuous_collision_lambda );
  m_lower_dist_sq = -34.0f;
  m_upper_dist_sq = 34.0f;
  m_gjk_iter = init_gjk( d, initial_support_dir, in_separation_loop );

  set_flag( FLAG_EXIT_ON_SEP_THRESH, 0 );
  set_flag( FLAG_TEST_CONVERGENCE, 0 );
  set_flag( FLAG_IS_SEPARATED, 0 );
  set_flag( FLAG_TEST_UD_LT_SP, d->gjk_ci && d->gjk_ci->is_support_dir_valid() );

  m_cc_reset_iter = 0;
  const float support_dir_moveback = tl_max( 0.050999999f, m_geom_radii_sum );
  tlAssert( support_dir_moveback < m_gjk_sep_thresh );

  float nsupport_dir;
  float lambda_denom;
  float lambda_numer;
  float dotp_origin_support_dir;
  float lambda;
  float dotvw;
  int new_index;
  phys_vec3 w;

  while ( true )
  {
    while ( true )
    {
      m_upper_dist_sq = AbsSquared( m_support_dir );

      if ( m_gjk_iter )
      {
        if ( get_flag( FLAG_TEST_UD_LT_SP ) && phys_sqr( m_gjk_sep_thresh ) > m_upper_dist_sq )
          set_flag( FLAG_EXIT_ON_SEP_THRESH, 1 );

        if ( m_gjk_pen_thresh_sq > m_upper_dist_sq )
        {
          if ( get_flag( CONTINUOUS_COLLISION ) && m_continuous_collision_lambda != 0.0f && !get_flag( INTERSECTION_TEST_ONLY ) )
            tlWarning( "m_continuous_collision_lambda problem\n" );
          return GJK_PENETRATING;
        }
      }

      if ( ( m_w_set & 1 ) != 0 )
      {
        if ( ( m_w_set & 2 ) != 0 )
          new_index = ( ( m_w_set & 4 ) != 0 ) ? 3 : 2;
        else
          new_index = 1;
      }
      else
      {
        new_index = 0;
      }

      d->gjk_cg1->support( -m_support_dir, &m_a_verts[new_index], &m_a_inds[new_index] );
      d->gjk_cg2->support( phys_inv_multiply( cg2_to_cg1_xform, m_support_dir ), &m_b_verts[new_index], &m_b_inds[new_index] );
      m_b_verts[new_index] = phys_full_multiply( cg2_to_cg1_xform, m_b_verts[new_index] );

      w = ( m_a_verts[new_index] - m_b_verts[new_index] ) - m_gjk_origin;
      dotvw = phys_dot( m_support_dir, w );

      if ( dotvw > 0.0f && m_upper_dist_sq > 0.0f )
      {
        set_flag( FLAG_IS_SEPARATED, 1 );
        const float lower_dist_sq = phys_sqr( dotvw ) / m_upper_dist_sq;
        if ( lower_dist_sq > m_lower_dist_sq )
        {
          m_lower_dist_sq = lower_dist_sq;
          if ( get_flag( FLAG_EXIT_ON_SEP_THRESH ) && m_lower_dist_sq > phys_sqr( m_gjk_sep_thresh ) )
            return GJK_SEPARATED;
        }
      }

      bool has_converged = false;
      if ( m_gjk_iter && m_lower_dist_sq > 0.0f )
      {
        has_converged = m_lower_dist_sq > ( m_upper_dist_sq * phys_sqr( 1.0f - 0.001f ) );
        for ( int i = 0, mask = 1; i < 4 && !has_converged; ++i, mask *= 2 )
        {
          if ( ( mask & m_last_w_set ) != 0 )
            has_converged = phys_sqr( 0.001f ) > AbsSquared( w - m_w_verts[i] );
        }
      }

      if ( !get_flag( FLAG_TEST_CONVERGENCE ) && dotvw > 0.0f &&
           phys_sqr( dotvw ) >= ( phys_sqr( support_dir_moveback ) * m_upper_dist_sq ) )
      {
        nsupport_dir = sqrtf( m_upper_dist_sq );
        tlAssert( nsupport_dir > 0.0f );
        lambda_denom = -phys_dot( m_cg1_relative_translation_loc, m_support_dir );
        lambda_numer = dotvw - ( support_dir_moveback * nsupport_dir );

        if ( lambda_denom <= 0.0f )
        {
          if ( lambda_numer > 0.0f )
          {
            if ( m_lower_dist_sq > phys_sqr( m_gjk_sep_thresh ) )
              return GJK_SEPARATED;
            set_flag( FLAG_TEST_CONVERGENCE, 1 );
            set_flag( FLAG_EXIT_ON_SEP_THRESH, 1 );
          }
        }
        else if ( lambda_numer >= 0.0f )
        {
          dotp_origin_support_dir = phys_dot( m_gjk_origin, m_support_dir );
          lambda = ( lambda_numer + dotp_origin_support_dir ) / lambda_denom;
          if ( lambda > ( m_continuous_collision_lambda + 0.000099999997f ) )
            break;
        }
      }

      if ( has_converged )
        return GJK_VALID;

      m_w_verts[new_index] = w;
      m_w_set |= 1 << new_index;
      m_last_w_set = m_w_set;
      m_w_set = gjk_subalgorithm( m_w_set, new_index );
      if ( m_w_set == 15 )
      {
        if ( get_flag( CONTINUOUS_COLLISION ) && m_continuous_collision_lambda != 0.0f )
          tlWarning( "m_continuous_collision_lambda problem\n" );
        return GJK_PENETRATING;
      }

      comp_v( m_w_set, &m_support_dir );
      if ( ++m_gjk_iter >= 30 )
      {
        tlWarning( "gjk reached the maximum number of iterations.\n" );
        return ( m_lower_dist_sq > 0.0f ) ? GJK_VALID : GJK_PENETRATING;
      }
    }

    if ( !get_flag( FLAG_TEST_CONVERGENCE ) && dotvw > 0.0f && phys_sqr( dotvw ) >= ( phys_sqr( support_dir_moveback ) * m_upper_dist_sq ) )
    {
      const float nsupport_dir = sqrtf( m_upper_dist_sq );
      const float lambda_denom = -phys_dot( m_cg1_relative_translation_loc, m_support_dir );
      const float lambda_numer = dotvw - ( support_dir_moveback * nsupport_dir );
      const float dotp_origin_support_dir = phys_dot( m_gjk_origin, m_support_dir );
      const float lambda = ( lambda_numer + dotp_origin_support_dir ) / lambda_denom;

      if ( lambda > d->m_end_time )
      {
        const float ray_end_dist_numer = ( lambda_numer - ( d->m_end_time * lambda_denom ) ) + dotp_origin_support_dir;
        if ( ray_end_dist_numer <= ( m_gjk_sep_thresh * nsupport_dir ) )
        {
          const float lambda_end = d->m_end_time;
          if ( ( lambda_end - m_continuous_collision_lambda ) <= 0.000099999997f )
          {
            set_flag( FLAG_TEST_CONVERGENCE, 1 );
            set_flag( FLAG_EXIT_ON_SEP_THRESH, 1 );
          }
          m_continuous_collision_lambda = lambda_end;
          m_gjk_origin = m_cg1_relative_translation_loc * ( -lambda_end );
          m_lower_dist_sq = -34.0f;
          m_upper_dist_sq = 34.0f;
          if ( ++m_cc_reset_iter > 1 )
            set_flag( FLAG_TEST_UD_LT_SP, 1 );
          if ( m_cc_reset_iter >= 10 )
          {
            set_flag( FLAG_TEST_CONVERGENCE, 1 );
            set_flag( FLAG_EXIT_ON_SEP_THRESH, 1 );
          }
          if ( !m_w_set )
            m_w_set = 1 << new_index;
          tlAssert( init_gjk( d, m_support_dir, true ) );
          tlAssert( m_w_set != 0 );
          tlAssert( m_last_w_set == m_w_set );
          ++m_gjk_iter;
          continue;
        }
        tlAssert( ray_end_dist_numer > 0.0f );
        return GJK_SEPARATED;
      }

      if ( ( lambda - m_continuous_collision_lambda ) <= 0.000099999997f )
      {
        set_flag( FLAG_TEST_CONVERGENCE, 1 );
        set_flag( FLAG_EXIT_ON_SEP_THRESH, 1 );
      }
      m_continuous_collision_lambda = lambda;
      m_gjk_origin = m_cg1_relative_translation_loc * ( -lambda );
      m_lower_dist_sq = -34.0f;
      m_upper_dist_sq = 34.0f;
      if ( ++m_cc_reset_iter > 1 )
        set_flag( FLAG_TEST_UD_LT_SP, 1 );
      if ( m_cc_reset_iter >= 10 )
      {
        set_flag( FLAG_TEST_CONVERGENCE, 1 );
        set_flag( FLAG_EXIT_ON_SEP_THRESH, 1 );
      }
      if ( !m_w_set )
        m_w_set = 1 << new_index;
      tlAssert( init_gjk( d, m_support_dir, true ) );
      tlAssert( m_w_set != 0 );
      tlAssert( m_last_w_set == m_w_set );
      ++m_gjk_iter;
    }
  }
}

const phys_vec3 phys_gjk_info::get_initial_support_dir( const phys_gjk_input *d )
{
  phys_gjk_cache_info *gjk_ci = d->gjk_ci;

  if ( gjk_ci && ( gjk_ci->m_flags & 4 ) != 0 )
  {
    return phys_inv_multiply( *d->cg1_to_world_xform, gjk_ci->m_support_dir );
  }

  phys_vec3 cg2_center = d->gjk_cg2->get_center( *d->cg2_to_world_xform );
  phys_vec3 cg2_center_local = phys_full_multiply( cg2_to_cg1_xform, cg2_center );
  phys_vec3 cg1_center = d->gjk_cg1->get_center();
  phys_vec3 dir = cg1_center - cg2_center_local;
  float dir_len_sq = phys_dot( dir, dir );

  if ( dir_len_sq < 0.0000000099999991f )
  {
    tlAssert( false && "degenerate gjk initial support dir." );
    return phys_vec3( 1.0f, 0.0f, 0.0f );
  }

  return dir;
}

void phys_gjk_info::gjk_cache_update_invalid( const phys_gjk_input *d )
{
  if ( d->gjk_ci )
  {
    d->gjk_ci->m_flags &= ~1;
  }
}

void phys_gjk_info::gjk_cache_update_separated( const phys_gjk_input *d )
{
  if ( d->gjk_ci )
  {
    d->gjk_ci->m_flags |= 1;
    d->gjk_ci->m_flags |= 4;
    d->gjk_ci->m_support_dir = phys_multiply( *d->cg1_to_world_xform, m_support_dir );
    d->gjk_ci->m_flags &= ~8;
  }
}

void phys_gjk_info::gjk_cache_update_colliding( const phys_gjk_input *d )
{
  if ( d->gjk_ci )
  {
    d->gjk_ci->set_flag_was_touched();
    d->gjk_ci->set_support_dir( phys_multiply( *d->cg1_to_world_xform, cg1_cinfo_loc.m_n ) );
    set_simplex( d->gjk_cg1,
                 d->gjk_cg2,
                 d->gjk_ci,
                 &-cg1_cinfo_loc.m_n,
                 &phys_inv_multiply( cg2_to_cg1_xform, cg1_cinfo_loc.m_n ),
                 m_a_inds,
                 m_b_inds,
                 m_w_set );
  }
}

void phys_gjk_info::gjk_cache_update_test_only_valid( const phys_gjk_input *d )
{
  if ( d->gjk_ci )
  {
    d->gjk_ci->m_flags |= 1;
    d->gjk_ci->m_flags |= 4;
    d->gjk_ci->m_support_dir = phys_multiply( *d->cg1_to_world_xform, m_support_dir );
    d->gjk_ci->m_flags &= ~8;
  }
}

void phys_gjk_info::gjk_cache_update_test_only_penetrating( const phys_gjk_input *d )
{
  if ( d->gjk_ci )
  {
    d->gjk_ci->m_flags |= 1;
    d->gjk_ci->m_flags &= ~4;
    d->gjk_ci->m_flags &= ~8;
  }
}

const bool phys_gjk_info::phys_collide_do_gjk_collide( const phys_gjk_input *d )
{
  return collide( d ) == GJK_PENETRATING;
}
