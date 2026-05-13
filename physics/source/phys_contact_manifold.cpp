#include "collision/phys_contact_manifold.h"
#include "collision/phys_broad_phase_base.h"

#include "rbc_defs/rbc_def_contact.h"
#include "phys_avl_tree.h"

void set_cpi_params( contact_point_info *cpi, phys_collision_pair *pcp );

const char *g_contact_manifold_error_msg;

void mesh_point_swap( contact_manifold_mesh_point **mp1, contact_manifold_mesh_point **mp2 )
{
  contact_manifold_mesh_point *temp = *mp1;
  *mp1 = *mp2;
  *mp2 = temp;
}

bool convex_poly_point_compare( contact_manifold_mesh_point *elem, contact_manifold_mesh_point *pivot )
{
  return phys_v2_le( elem->m_contact_p, pivot->m_contact_p );
}

bool feature_point_compare( contact_manifold_mesh_point *elem, contact_manifold_mesh_point *pivot )
{
  float elem_cos_sq = elem->m_contact_p.GetX();
  float pivot_cos_sq = pivot->m_contact_p.GetX();
  return elem_cos_sq > pivot_cos_sq;
}

void feature_qsort( contact_manifold_mesh_point **list, const intptr_t list_count )
{
  if ( list_count == 2 )
  {
    if ( feature_point_compare( list[1], list[0] ) )
    {
      mesh_point_swap( list, list + 1 );
    }
  }
  else if ( list_count > 2 )
  {
    contact_manifold_mesh_point **list_mid = list;
    contact_manifold_mesh_point *pivot = list[0];
    contact_manifold_mesh_point **list_end = &list[list_count - 1];
    for ( contact_manifold_mesh_point **elem_i = list + 1; elem_i <= list_end; ++elem_i )
    {
      if ( feature_point_compare( *elem_i, pivot ) )
      {
        mesh_point_swap( elem_i, ++list_mid );
      }
    }
    *list = *list_mid;
    *list_mid = pivot;
    feature_qsort( list, list_mid - list );
    feature_qsort( list_mid + 1, list_end - list_mid );
  }
}

void convex_poly_qsort( contact_manifold_mesh_point **list, const intptr_t list_count )
{
  if ( list_count == 2 )
  {
    if ( convex_poly_point_compare( list[1], list[0] ) )
    {
      mesh_point_swap( list, list + 1 );
    }
  }
  else if ( list_count > 2 )
  {
    contact_manifold_mesh_point **list_mid = list;
    contact_manifold_mesh_point *pivot = list[0];
    contact_manifold_mesh_point **list_end = &list[list_count - 1];
    for ( contact_manifold_mesh_point **elem_i = list + 1; elem_i <= list_end; ++elem_i )
    {
      if ( convex_poly_point_compare( *elem_i, pivot ) )
      {
        mesh_point_swap( elem_i, ++list_mid );
      }
    }
    *list = *list_mid;
    *list_mid = pivot;
    convex_poly_qsort( list, list_mid - list );
    convex_poly_qsort( list_mid + 1, list_end - list_mid );
  }
}

void phys_contact_manifold::generate_convex_poly_internal()
{
  convex_poly_qsort( m_list_sorted_mesh_point, m_list_mesh_point_count );
  float min_length2 = get_convex_poly_min_length_sq();
  float min_sin_sq = get_convex_poly_min_sin_sq();
  contact_manifold_mesh_point **i_smp = m_list_sorted_mesh_point;
  contact_manifold_mesh_point **cp_mp = m_list_contact_point;
  contact_manifold_mesh_point **min_cp_mp = cp_mp + 1;
  contact_manifold_mesh_point **last_i_smp = &m_list_sorted_mesh_point[m_list_mesh_point_count];

  while ( i_smp < last_i_smp )
  {
    // Grow lower hull, pruning non-right-hand turns.
    contact_manifold_mesh_point *mp = *i_smp;
    while ( cp_mp > min_cp_mp )
    {
      contact_manifold_mesh_point *prev_mp = *( cp_mp - 1 );
      phys_vec2 e1 = prev_mp->m_contact_p - ( *( cp_mp - 2 ) )->m_contact_p;
      phys_vec2 e2 = mp->m_contact_p - prev_mp->m_contact_p;
      if ( rht( e1, e2, min_length2, min_sin_sq ) )
      {
        break;
      }
      --cp_mp;
    }

    tlAssertMsg( m_allocator->fast_is_within_buffer_limits( cp_mp, sizeof( contact_manifold_mesh_point * ) ),
                 g_contact_manifold_error_msg );

    *cp_mp++ = mp;
    ++i_smp;
  }

  contact_manifold_mesh_point **min_cp_mp_rev = cp_mp;
  contact_manifold_mesh_point **last_i_smp_rev = m_list_sorted_mesh_point;
  for ( contact_manifold_mesh_point **i_smp_rev = i_smp - 2; i_smp_rev >= last_i_smp_rev; --i_smp_rev )
  {
    // Grow upper hull in reverse order.
    contact_manifold_mesh_point *mp = *i_smp_rev;
    while ( cp_mp > min_cp_mp_rev )
    {
      contact_manifold_mesh_point *prev_mp = *( cp_mp - 1 );
      phys_vec2 e1 = mp->m_contact_p - prev_mp->m_contact_p;
      phys_vec2 e2 = prev_mp->m_contact_p - ( *( cp_mp - 2 ) )->m_contact_p;
      if ( rht( e2, e1, min_length2, min_sin_sq ) )
      {
        break;
      }
      --cp_mp;
    }

    tlAssertMsg( m_allocator->fast_is_within_buffer_limits( cp_mp, sizeof( contact_manifold_mesh_point * ) ),
                 g_contact_manifold_error_msg );

    *cp_mp++ = mp;
  }

  m_list_contact_point_count = cp_mp - 1 - m_list_contact_point;
  tlAssert( m_list_contact_point_count <= m_list_mesh_point_count );
}

void phys_contact_manifold::generate_convex_poly( const phys_mat44 &contact_mat )
{
  tlAssert( m_list_sorted_mesh_point != NULL );
  tlAssert( m_list_contact_point == NULL );
  tlAssert( m_list_mesh_point_count > 1 );

  // Allocate memory for contact points array
  m_list_contact_point = reinterpret_cast<contact_manifold_mesh_point **>(
      m_allocator->fast_align_start( PHYS_ALIGNOF( contact_manifold_mesh_point * ), g_contact_manifold_error_msg ) );

  tlAssertMsg(
      m_allocator->fast_is_within_buffer_limits( m_list_contact_point, sizeof( contact_manifold_mesh_point * ) * m_list_mesh_point_count ),
      g_contact_manifold_error_msg );

  // Transform all mesh points from 3D to 2D using contact matrix
  contact_manifold_mesh_point **last_mp_i = &m_list_sorted_mesh_point[m_list_mesh_point_count];
  for ( contact_manifold_mesh_point **mp_i = m_list_sorted_mesh_point; mp_i != last_mp_i; ++mp_i )
  {
    contact_manifold_mesh_point *mp = *mp_i;
    phys_vec2 contact_p = phys_v3_to_v2_inv_multiply( contact_mat, mp->m_p );
    mp->m_contact_p = contact_p;
  }

  // Compute convex hull using Graham scan
  generate_convex_poly_internal();

  // Validate result, especially for 2-point case
  if ( m_list_contact_point_count == 2 )
  {
    phys_vec2 delta = m_list_contact_point[0]->m_contact_p - m_list_contact_point[1]->m_contact_p;
    float distance_sq = AbsSquared( delta );
    if ( get_convex_poly_min_length_sq() > distance_sq )
    {
      m_list_contact_point_count = 1;
    }
  }

  tlAssert( m_list_contact_point_count > 0 );

  // Finalize allocation to correct size
  contact_manifold_mesh_point **temp = reinterpret_cast<contact_manifold_mesh_point **>(
      m_allocator->fast_allocate( sizeof( contact_manifold_mesh_point * ) * m_list_contact_point_count, g_contact_manifold_error_msg ) );
  tlAssert( m_list_contact_point == temp );
}

const float phys_contact_manifold::compute_convex_poly_area()
{
  float area = 0.0f;

  contact_manifold_mesh_point **last_cp_i = &m_list_contact_point[m_list_contact_point_count - 1];
  contact_manifold_mesh_point *v0 = *m_list_contact_point;

  for ( contact_manifold_mesh_point **cp_i = m_list_contact_point + 1; cp_i != last_cp_i; ++cp_i )
  {
    contact_manifold_mesh_point *v1 = *cp_i;
    contact_manifold_mesh_point *v2 = cp_i[1];
    phys_vec2 e1 = v1->m_contact_p - v0->m_contact_p;
    phys_vec2 e2 = v2->m_contact_p - v1->m_contact_p;
    area = area + phys_v2_cross( e1, e2 );
  }

  float result = area * 0.5f;
  if ( result < 0.0f )
  {
    result = -result;
  }

  return result;
}

const float phys_contact_manifold::compute_convex_poly_perimeter()
{
  float perimeter = 0.0f;

  contact_manifold_mesh_point **last_cp_i = &m_list_contact_point[m_list_contact_point_count];
  contact_manifold_mesh_point **prev_cp_i = last_cp_i - 1;

  for ( contact_manifold_mesh_point **cp_i = m_list_contact_point; cp_i != last_cp_i; ++cp_i )
  {
    phys_vec2 delta = ( *cp_i )->m_contact_p - ( *prev_cp_i )->m_contact_p;
    perimeter = perimeter + Abs( delta );
    prev_cp_i = cp_i;
  }

  return perimeter;
}

void phys_contact_manifold::comp_feature_normal()
{
  tlAssert( m_list_mesh_point != NULL );
  tlAssert( m_list_sorted_mesh_point == NULL );

  // Allocate sorted mesh point array
  m_list_sorted_mesh_point = reinterpret_cast<contact_manifold_mesh_point **>(
      m_allocator->fast_align_start( PHYS_ALIGNOF( contact_manifold_mesh_point * ), g_contact_manifold_error_msg ) );

  m_allocator->fast_allocate( sizeof( contact_manifold_mesh_point * ) * m_list_mesh_point_count, g_contact_manifold_error_msg );

  // Copy mesh point pointers to sorted array
  contact_manifold_mesh_point **last_mp_it = &m_list_sorted_mesh_point[m_list_mesh_point_count];
  contact_manifold_mesh_point *mp_i = m_list_mesh_point;
  for ( contact_manifold_mesh_point **smp_i = m_list_sorted_mesh_point; smp_i < last_mp_it; ++smp_i )
  {
    *smp_i = mp_i++;
  }

  m_list_mesh_point = NULL;

  const float LENGTH_TOLERANCE_SQ = get_LENGTH_TOLERANCE_SQ();

  tlAssert( m_list_mesh_point_count >= 2 );

  if ( m_list_mesh_point_count == 2 )
  {
    // Handle 2-point case
    phys_vec3 edge = ( *( m_list_sorted_mesh_point + 1 ) )->m_p - ( *m_list_sorted_mesh_point )->m_p;
    float nedge_sq = AbsSquared( edge );

    if ( nedge_sq <= LENGTH_TOLERANCE_SQ )
    {
      m_feature_normal = m_feature_hitn;
    }
    else
    {
      float dot_val = phys_dot( m_feature_hitn, edge );
      phys_vec3 proj = ( dot_val / nedge_sq ) * edge;
      phys_vec3 normal = m_feature_hitn - proj;
      m_feature_normal = normal;

      if ( LENGTH_TOLERANCE_SQ >= AbsSquared( m_feature_normal ) )
      {
        m_feature_normal = m_feature_hitn;
      }
    }

    ( *m_list_sorted_mesh_point )->m_p += m_feature_hitp;
    ( *( m_list_sorted_mesh_point + 1 ) )->m_p += m_feature_hitp;
  }
  else
  {
    // Handle > 2 point case
    feature_qsort( m_list_sorted_mesh_point, m_list_mesh_point_count );
    m_feature_normal = m_feature_hitn;
    float best_cos_sq = -2.0f;
    float IS_VALID_DIST_THRESH_SQ = phys_sqr( 0.034f );

    contact_manifold_mesh_point **start_mp = &m_list_sorted_mesh_point[m_close_mesh_point_count];

    // Find best feature normal via cross products
    for ( contact_manifold_mesh_point **mp_i_it = start_mp; mp_i_it < last_mp_it - 1 && GET_COS_SQ( *mp_i_it ) >= best_cos_sq; ++mp_i_it )
    {
      contact_manifold_mesh_point *mp_i_0 = *mp_i_it;
      for ( contact_manifold_mesh_point **mp_j_it = mp_i_it + 1; mp_j_it < last_mp_it && GET_COS_SQ( *mp_j_it ) >= best_cos_sq; ++mp_j_it )
      {
        contact_manifold_mesh_point *mp_j = *mp_j_it;
        phys_vec3 normal = phys_cross( mp_i_0->m_p, mp_j->m_p );
        float nnormal_sq = AbsSquared( normal );

        if ( nnormal_sq > LENGTH_TOLERANCE_SQ )
        {
          float dotp = phys_dot( m_feature_hitn, normal );
          float cos_sq = phys_sqr( dotp ) / nnormal_sq;

          if ( cos_sq > best_cos_sq )
          {
            bool is_valid = true;
            if ( dotp < 0.0f )
            {
              normal = -normal;
            }

            // Validate normal against other points
            for ( contact_manifold_mesh_point **mp_k_it = start_mp; mp_k_it < last_mp_it && GET_COS_SQ( *mp_k_it ) >= cos_sq && is_valid;
                  ++mp_k_it )
            {
              contact_manifold_mesh_point *mp_k = *mp_k_it;
              if ( mp_k_it != &mp_i_0 && mp_k != mp_j )
              {
                float dp = phys_dot( mp_k->m_p, normal );
                if ( dp < 0.0f )
                {
                  float orth_dist_sq = phys_sqr( dp ) / nnormal_sq;
                  if ( orth_dist_sq > IS_VALID_DIST_THRESH_SQ && orth_dist_sq > ( 0.0000068538761f * GET_NP_SQ( mp_k ) ) )
                  {
                    is_valid = false;
                  }
                }
              }
            }

            if ( is_valid )
            {
              best_cos_sq = cos_sq;
              m_feature_normal = normal;
            }
          }
        }
      }
    }

    // Adjust mesh point positions and filter by distance thresholds
    float nfeature_normal_sq = AbsSquared( m_feature_normal );
    tlAssert( nfeature_normal_sq > 0.0f );

    contact_manifold_mesh_point **list_mp_i = m_list_sorted_mesh_point;
    for ( contact_manifold_mesh_point **mp_i_it_0 = m_list_sorted_mesh_point; mp_i_it_0 < last_mp_it; ++mp_i_it_0 )
    {
      contact_manifold_mesh_point *mp_i_1 = *mp_i_it_0;
      float dot_p_fn = phys_dot( mp_i_1->m_p, m_feature_normal );
      float orth_dist_sq_0 = phys_sqr( dot_p_fn ) / nfeature_normal_sq;

      if ( phys_sqr( m_feature_distance_eps ) >= orth_dist_sq_0 ||
           ( GET_NP_SQ( mp_i_1 ) * m_sin_feautre_angular_eps_sq ) >= orth_dist_sq_0 )
      {
        phys_vec3 proj = ( dot_p_fn / nfeature_normal_sq ) * m_feature_normal;
        mp_i_1->m_p += m_feature_hitp - proj;
        *list_mp_i++ = mp_i_1;
      }
    }
    m_list_mesh_point_count = list_mp_i - m_list_sorted_mesh_point;
  }

  tlAssert( AbsSquared( m_feature_normal ) > 0.0f );
}

void phys_contact_manifold_process::isect_info::init( phys_contact_manifold *cman )
{
  m_cman = cman;
  m_i = cman->m_list_contact_point;
  m_next_i = m_i + 1;
  m_last_i = m_i + cman->m_list_contact_point_count - 1;
  m_edge = get_contact_p( m_next_i ) - get_contact_p( m_i );
}
void phys_contact_manifold_process::isect_info::update()
{
  m_i = m_next_i;
  m_next_i = next_pv( m_next_i );
  m_edge = get_contact_p( m_next_i ) - get_contact_p( m_i );
}
void phys_contact_manifold_process::intersect_poly_segment( phys_contact_manifold *cman, phys_vec2 &p0, phys_vec2 &p1 )
{
  phys_vec2 dir = p1 - p0;
  float t_enter = 0.0f;
  float t_exit = 1.0f;
  poly_vert_id cp_mp = cman->m_list_contact_point;
  poly_vert_id last_cp_mp = cp_mp + ( cman->m_list_contact_point_count - 1 );
  contact_manifold_mesh_point *v0 = *last_cp_mp;
  while ( cp_mp <= last_cp_mp )
  {
    contact_manifold_mesh_point *v1 = *cp_mp;
    const phys_vec2 normal = phys_v2_rotr( v0->m_contact_p - v1->m_contact_p );
    const float numer = phys_v2_dot( v0->m_contact_p - p0, normal );
    const float denom = phys_v2_dot( dir, normal );
    if ( denom <= -0.0001f || denom >= 0.0001f )
    {
      const float t_ = numer / denom;
      if ( denom >= 0.0f )
      {
        if ( t_exit > t_ )
        {
          t_exit = t_;
        }
      }
      else if ( t_ > t_enter )
      {
        t_enter = t_;
      }
      if ( t_enter > t_exit )
      {
        // No intersection
        return;
      }
    }
    else if ( numer < 0.0f )
    {
      // Segment is parallel and outside the polygon, no intersection
      return;
    }
    v0 = *cp_mp++;
  }

  tlAssert( t_enter <= t_exit )
  tlAssert( cman->m_list_contact_point_count >= 2 )
  m_list_isect_point = cman->m_list_contact_point;
  m_contact_point_count = 2;
  ( *m_list_isect_point )->m_contact_p = p0 + ( t_enter * dir );
  ( *( m_list_isect_point + 1 ) )->m_contact_p = p0 + ( t_exit * dir );
}

bool phys_contact_manifold_process::find_bottom( phys_contact_manifold_process::bridge *b,
                                                 phys_contact_manifold_process::isect_info *left_cman,
                                                 phys_contact_manifold_process::isect_info *right_cman )
{
  poly_vert_id &left_i = b->m_left_i;
  poly_vert_id &right_i = b->m_right_i;
  const poly_vert_id start_left_i = left_i;
  const poly_vert_id start_right_i = right_i;
  phys_vec2 left_edge;
  phys_vec2 right_edge;
  const phys_vec2 *next_left_v;
  const phys_vec2 *prev_right_v;
  while ( true )
  {
    const poly_vert_id l_i = left_i;
    const poly_vert_id r_i = right_i;
    next_left_v = &get_contact_p( left_cman->next_pv( l_i ) );
    left_edge = *next_left_v - get_contact_p( l_i );
    while ( true )
    {
      const poly_vert_id prev_right_i = right_cman->prev_pv( right_i );
      if ( phys_v2_cross( left_edge, get_contact_p( prev_right_i ) - *next_left_v ) >= 0.0f )
      {
        break;
      }
      right_i = prev_right_i;
      if ( right_i == start_right_i )
      {
        return false;
      }
    }
    prev_right_v = &get_contact_p( right_cman->prev_pv( right_i ) );
    right_edge = *prev_right_v - get_contact_p( right_i );
    while ( true )
    {
      const poly_vert_id next_left_i = left_cman->next_pv( left_i );
      if ( phys_v2_cross( right_edge, get_contact_p( next_left_i ) - *prev_right_v ) <= 0.0f )
      {
        break;
      }
      left_i = next_left_i;
      if ( left_i == start_left_i )
      {
        return false;
      }
    }
    if ( left_i == l_i && right_i == r_i )
    {
      break;
    }
  }
  const float det = phys_v2_cross( left_edge, right_edge );
  if ( det <= 0.0001f )
  {
    b->m_intersection_p = get_contact_p( right_i );
  }
  else
  {
    b->m_intersection_p = *next_left_v + ( ( phys_v2_cross( *prev_right_v - *next_left_v, right_edge ) / det ) * left_edge );
  }
  return true;
}

void phys_contact_manifold_process::intersect_poly_poly()
{
  tlAssert( cman1.get_poly_vert_count() >= 2 && cman2.get_poly_vert_count() >= 2 );
  tlAssert( cman1.get_poly_vert_count() > 2 || cman2.get_poly_vert_count() > 2 );

  m_list_isect_point = NULL;
  m_contact_point_count = 0;

  if ( cman1.get_poly_vert_count() == 2 )
  {
    phys_vec2 &p0 = cman1.get_poly_vert( 0 );
    phys_vec2 &p1 = cman1.get_poly_vert( 1 );
    intersect_poly_segment( &cman2, p0, p1 );
  }
  else if ( cman2.get_poly_vert_count() == 2 )
  {
    phys_vec2 &p0 = cman2.get_poly_vert( 0 );
    phys_vec2 &p1 = cman2.get_poly_vert( 1 );
    intersect_poly_segment( &cman1, p0, p1 );
  }
  else
  {
    isect_info gb_cman1, gb_cman2;
    gb_cman1.init( &cman1 );
    gb_cman2.init( &cman2 );

    isect_info *left_gb = &gb_cman1;
    isect_info *right_gb = &gb_cman2;

    bridge *list_bridge = (bridge *)m_allocator.fast_align_start(
        tl_max( PHYS_ALIGNOF( bridge ), PHYS_ALIGNOF( contact_manifold_mesh_point * ) ),
        g_contact_manifold_error_msg );
    bridge *b_cur = list_bridge;

    int max_ctr = gb_cman1.m_cman->get_poly_vert_count() + gb_cman2.m_cman->get_poly_vert_count();

    for ( int ctr = 0; ctr <= max_ctr; ++ctr )
    {
      b_cur->m_left_i = left_gb->m_i;
      b_cur->m_right_i = right_gb->m_i;

      phys_vec2 caliper_dir;
      if ( phys_v2_cross( left_gb->m_edge, right_gb->m_edge ) < 0.0f )
      {
        caliper_dir = right_gb->m_edge;
        right_gb->update();
      }
      else
      {
        caliper_dir = left_gb->m_edge;
        left_gb->update();
      }

      phys_vec2 v36 = get_contact_p( right_gb->m_i ) - get_contact_p( left_gb->m_i );
      float cr = phys_v2_cross( caliper_dir, v36 );

      if ( cr <= 0.0f )
      {
        if ( cr >= 0.0f )
        {
          phys_vec2 vdisplace = phys_v2_rotr( caliper_dir );
          float nvdisplace = Abs( vdisplace );
          tlAssertMsg( nvdisplace > 0.0001f, "nvdisplace > 0.0001f" );
          vdisplace *= get_poly_vert_displacement_factor() / nvdisplace;
          displace_contact_p( right_gb->m_i, vdisplace, contact_mat );
        }
        else
        {
          if ( ctr )
          {
            tlAssertMsg( m_allocator.fast_is_within_buffer_limits( b_cur, sizeof( bridge ) ), g_contact_manifold_error_msg );
            if ( !find_bottom( b_cur, left_gb, right_gb ) )
              return;
            ++b_cur;
          }
          isect_info *temp = left_gb;
          left_gb = right_gb;
          right_gb = temp;
        }
      }
    }

    int list_bridge_count = b_cur - list_bridge;

    if ( list_bridge_count % 2 )
    {
      tlWarning( "contact manifold intersect poly poly failed.\n" );
      phys_contact_manifold *cman;
      if ( cman2.compute_convex_poly_perimeter() < cman1.compute_convex_poly_perimeter() )
        cman = &cman2;
      else
        cman = &cman1;
      copy_poly( cman );
    }
    else
    {
      void *temp_ptr = m_allocator.fast_allocate( sizeof( bridge ) * list_bridge_count, g_contact_manifold_error_msg );
      tlAssert( temp_ptr == list_bridge );

      if ( list_bridge_count == 0 )
      {
        copy_poly( right_gb->m_cman );
      }
      else
      {
        m_list_isect_point = (contact_manifold_mesh_point **)m_allocator.fast_align_start(
            tl_max( PHYS_ALIGNOF( contact_manifold_mesh_point * ), PHYS_ALIGNOF( contact_manifold_mesh_point * ) ),
            g_contact_manifold_error_msg );
        contact_manifold_mesh_point **ip_i = m_list_isect_point;

        for ( int bridge_i = 0; bridge_i < list_bridge_count; ++bridge_i )
        {
          bridge *bridge_ptr = &list_bridge[bridge_i];

          tlAssertMsg( m_allocator.fast_is_within_buffer_limits( ip_i, sizeof( contact_manifold_mesh_point * ) ),
                       g_contact_manifold_error_msg );

          *ip_i = *bridge_ptr->m_left_i;

          for ( contact_manifold_mesh_point **test_i = m_list_isect_point; test_i != ip_i; ++test_i )
          {
            if ( *test_i == *bridge_ptr->m_left_i )
              tlWarning( "contact manifold failure." );
          }

          ( *ip_i )->m_contact_p = bridge_ptr->m_intersection_p;
          ++ip_i;

          poly_vert_id term_left_i = list_bridge[( bridge_i + 1 ) % list_bridge_count].m_right_i;
          for ( poly_vert_id left_i = left_gb->next_pv( bridge_ptr->m_left_i ); left_i != term_left_i; left_i = left_gb->next_pv( left_i ) )
          {
            tlAssertMsg( m_allocator.fast_is_within_buffer_limits( ip_i, sizeof( contact_manifold_mesh_point * ) ),
                         g_contact_manifold_error_msg );

            for ( contact_manifold_mesh_point **test_i = m_list_isect_point; test_i != ip_i; ++test_i )
            {
              if ( *test_i == *left_i )
                tlWarning( "contact manifold failure." );
            }

            *ip_i++ = *left_i;
          }

          isect_info *temp = left_gb;
          left_gb = right_gb;
          right_gb = temp;
        }

        m_contact_point_count = (int)( ip_i - m_list_isect_point );
        void *temp_ptr2 = m_allocator.fast_allocate( sizeof( contact_manifold_mesh_point * ) * m_contact_point_count,
                                                     g_contact_manifold_error_msg );
        tlAssert( temp_ptr2 == m_list_isect_point );
      }
    }
  }
}

void phys_contact_manifold_process::process( phys_collision_pair *pcp, phys_gjk_info *gjk_info )
{
  nullify_pointers();
  m_allocator.reset();

  const phys_mat44 *cg1_to_world_xform = pcp->m_bpi1->get_cg_to_world_xform();
  const phys_mat44 *cg1_to_rb1_xform = pcp->m_bpi1->get_cg_to_rb_xform();
  m_cpi = NULL;

  const phys_mat44 *rb2_to_world_xform = pcp->m_bpi2->get_rb_to_world_xform();
  phys_full_inv_multiply_mat( cg1_to_rb2_xform, *rb2_to_world_xform, *cg1_to_world_xform );

  float half_min_sep = rigid_body_constraint_contact::get_std_contact_min_sep_dist() * 0.5f;
  phys_vec3 contact_displacement = half_min_sep * gjk_info->cg1_cinfo_loc.m_n;

  phys_vec3 p1_displaced = gjk_info->cg1_cinfo_loc.m_p1;
  phys_vec3 p2_displaced = gjk_info->cg1_cinfo_loc.m_p2;

  gjk_info->cg1_cinfo_loc.m_p1 = gjk_info->cg1_cinfo_loc.m_p1 + contact_displacement;
  gjk_info->cg1_cinfo_loc.m_p2 = gjk_info->cg1_cinfo_loc.m_p2 - contact_displacement;

  phys_vec3 diff = p1_displaced - p2_displaced;
  float dist_p1_p2_n = phys_dot( diff, gjk_info->cg1_cinfo_loc.m_n );
  float penetration_t = phys_contact_manifold::get_STD_PENETRATION_T( dist_p1_p2_n );
  float add_point_eps = phys_contact_manifold::get_STD_GET_FEATURE_DISTANCE_EPS( penetration_t );
  float sin_angular_eps_sq = phys_contact_manifold::get_STD_GET_FEATURE_SIN_ANGULAR_EPS_SQ( penetration_t );

  cman1.set_get_feature_params( gjk_info->cg1_cinfo_loc.m_p1, gjk_info->cg1_cinfo_loc.m_n, add_point_eps, sin_angular_eps_sq );
  pcp->m_bpi1->get_gjk_cg()->get_feature( &cman1 );

  phys_vec3 hitp = phys_full_inv_multiply( gjk_info->cg2_to_cg1_xform, gjk_info->cg1_cinfo_loc.m_p2 );
  phys_vec3 neg_n = -gjk_info->cg1_cinfo_loc.m_n;
  phys_vec3 hitn = phys_inv_multiply( gjk_info->cg2_to_cg1_xform, neg_n );

  cman2.set_get_feature_params( hitp, hitn, add_point_eps, sin_angular_eps_sq );
  pcp->m_bpi2->get_gjk_cg()->get_feature( &cman2 );

  phys_vec3 cg1_relative_translation_loc = gjk_info->m_continuous_collision_lambda * gjk_info->m_cg1_relative_translation_loc;

  bool single_point = true;
  if ( cman1.get_mesh_point_count() >= 2 && cman2.get_mesh_point_count() >= 2 )
  {
    single_point = ( cman1.get_mesh_point_count() == 2 && cman2.get_mesh_point_count() == 2 );
  }

  if ( !single_point )
  {
    float leps = phys_contact_manifold::get_STD_COMP_FEATURE_NORMAL_DISTANCE_EPS( penetration_t );
    float aeps = phys_contact_manifold::get_STD_COMP_FEATURE_NORMAL_SIN_ANGULAR_EPS_SQ( penetration_t );
    cman1.set_comp_feature_normal_eps( leps, aeps );
    cman1.comp_feature_normal();
    cman2.set_comp_feature_normal_eps( leps, aeps );
    cman2.comp_feature_normal();

    single_point = true;
    if ( cman1.get_mesh_point_count() >= 2 && cman2.get_mesh_point_count() >= 2 )
    {
      single_point = ( cman1.get_mesh_point_count() == 2 && cman2.get_mesh_point_count() == 2 );
    }

    if ( !single_point )
    {
      comp_contact_mat( gjk_info->cg1_cinfo_loc.m_n );
      cman1.generate_convex_poly( contact_mat );

      phys_vec3 neg_cg1_rel_trans = -cg1_relative_translation_loc;
      cman2.xform_and_translate_mesh_points( gjk_info->cg2_to_cg1_xform, neg_cg1_rel_trans );
      cman2.generate_convex_poly( contact_mat );

      single_point = true;
      if ( cman1.get_poly_vert_count() >= 2 && cman2.get_poly_vert_count() >= 2 )
      {
        single_point = ( cman1.get_poly_vert_count() == 2 && cman2.get_poly_vert_count() == 2 );
      }

      if ( !single_point )
      {
        intersect_poly_poly();
        single_point = ( m_contact_point_count == 0 );
      }
    }
  }

  if ( single_point )
  {
    m_cpi = contact_point_info::create_cpi( 1, false, m_cpi_allocator );
    if ( !m_cpi )
    {
      return;
    }

    phys_vec3 p1_rb1 = phys_full_multiply( *cg1_to_rb1_xform, p1_displaced );
    m_cpi->m_list_b1_r_loc[0] = p1_rb1;

    phys_vec3 p2_rb2 = phys_full_multiply( cg1_to_rb2_xform, p2_displaced );
    m_cpi->m_list_b2_r_loc[0] = p2_rb2;

    m_contact_point_count = 1;
    goto label_fill_cpi;
  }

  tlAssert( m_contact_point_count > 0 );
  tlAssert( m_list_isect_point );

  {
    float d1 = phys_dot( gjk_info->cg1_cinfo_loc.m_n, cman1.m_feature_normal );
    float d2 = phys_dot( gjk_info->cg1_cinfo_loc.m_n, cman2.m_feature_normal );
    tlAssert( fabsf( d1 ) > 0.0000001f );
    tlAssert( fabsf( d2 ) > 0.0000001f );

    float n1 = phys_dot( p1_displaced, cman1.m_feature_normal );
    float n2 = phys_dot( p2_displaced, cman2.m_feature_normal );

    if ( m_contact_point_count > 5 )
    {
      phys_vec3 axis_3d = ( cman2.m_feature_normal / d2 ) - ( cman1.m_feature_normal / d1 );
      phys_vec2 A_ = phys_v3_to_v2_inv_multiply( contact_mat, axis_3d );

      contact_manifold_mesh_point *closest_mp = NULL;
      float closest_dist = 1.0e7f;

      contact_manifold_mesh_point **last_mp_i = &m_list_isect_point[m_contact_point_count];
      contact_manifold_mesh_point **prev_mp_i = last_mp_i - 2;
      contact_manifold_mesh_point **cur_mp_i = last_mp_i - 1;

      for ( contact_manifold_mesh_point **next_mp_i = m_list_isect_point; next_mp_i != last_mp_i; ++next_mp_i )
      {
        float area = CALC_AREA( *prev_mp_i, *cur_mp_i, *next_mp_i );
        SET_VERTEX_AREA( *cur_mp_i, area );

        float dist = phys_v2_dot( ( *cur_mp_i )->m_contact_p, A_ );
        if ( closest_dist > dist )
        {
          closest_dist = dist;
          closest_mp = *cur_mp_i;
        }

        prev_mp_i = cur_mp_i;
        cur_mp_i = next_mp_i;
      }

      tlAssert( closest_mp );

      contact_manifold_mesh_point **MAX_MP_I = m_list_isect_point + 5;
      while ( last_mp_i > MAX_MP_I )
      {
        contact_manifold_mesh_point **smallest_area_mp_i = m_list_isect_point;
        for ( contact_manifold_mesh_point **cur = smallest_area_mp_i + 1; cur != last_mp_i; ++cur )
        {
          if ( *cur != closest_mp && GET_VERTEX_AREA( *cur ) < GET_VERTEX_AREA( *smallest_area_mp_i ) )
            smallest_area_mp_i = cur;
        }

        --last_mp_i;
        contact_manifold_mesh_point **prev = GET_PREV_MP( smallest_area_mp_i, m_list_isect_point, last_mp_i );
        contact_manifold_mesh_point **next = GET_NEXT_MP( smallest_area_mp_i, m_list_isect_point, last_mp_i );
        contact_manifold_mesh_point **prev_prev = GET_PREV_MP( prev, m_list_isect_point, last_mp_i );
        contact_manifold_mesh_point **next_next = GET_NEXT_MP( next, m_list_isect_point, last_mp_i );

        SET_VERTEX_AREA( *prev, CALC_AREA( *prev_prev, *prev, *next ) );
        SET_VERTEX_AREA( *next, CALC_AREA( *prev, *next, *next_next ) );

        while ( smallest_area_mp_i < last_mp_i )
        {
          *smallest_area_mp_i = smallest_area_mp_i[1];
          ++smallest_area_mp_i;
        }
      }

      m_contact_point_count = 5;
    }

    m_cpi = contact_point_info::create_cpi( m_contact_point_count, false, m_cpi_allocator );
    if ( !m_cpi )
      return;

    {
      phys_vec3 *b1_r_i = m_cpi->m_list_b1_r_loc;
      phys_vec3 *b2_r_i = m_cpi->m_list_b2_r_loc;
      contact_manifold_mesh_point **ip_i = m_list_isect_point;
      contact_manifold_mesh_point **last_ip_i = &ip_i[m_contact_point_count];

      while ( ip_i != last_ip_i )
      {
        phys_vec3 ip_3d_ = phys_v2_to_v3_multiply( contact_mat, ( *ip_i )->m_contact_p );

        float v20 = phys_dot( ip_3d_, cman1.m_feature_normal );
        phys_vec3 v104 = ( ( n1 - v20 ) / d1 ) * gjk_info->cg1_cinfo_loc.m_n;
        phys_vec3 b1_3d = ip_3d_ + v104;
        *b1_r_i = phys_full_multiply( *cg1_to_rb1_xform, b1_3d );

        phys_vec3 ip_3d_2 = ip_3d_ + cg1_relative_translation_loc;
        float v23 = phys_dot( ip_3d_2, cman2.m_feature_normal );
        phys_vec3 v100 = ( ( n2 - v23 ) / d2 ) * gjk_info->cg1_cinfo_loc.m_n;
        phys_vec3 b2_3d = ip_3d_2 + v100;
        *b2_r_i = phys_full_multiply( cg1_to_rb2_xform, b2_3d );

        ++ip_i;
        ++b1_r_i;
        ++b2_r_i;
      }
    }
  }

label_fill_cpi:
  tlAssert( m_cpi );
  {
    phys_vec3 normal_world = phys_multiply( *cg1_to_world_xform, -gjk_info->cg1_cinfo_loc.m_n );
    m_cpi->set_normal( normal_world );

    float MIN_LAMBDA_ACTIVE_DIST = rigid_body_constraint_contact::get_MIN_LAMBDA_ACTIVE_DIST();
    float translation_along_normal = phys_dot( gjk_info->m_cg1_relative_translation_loc, gjk_info->cg1_cinfo_loc.m_n );

    if ( translation_along_normal >= -MIN_LAMBDA_ACTIVE_DIST )
    {
      m_cpi->set_translation_lambda( 1.0f );
    }
    else
    {
      float lambda = ( -dist_p1_p2_n ) / translation_along_normal;
      m_cpi->set_translation_lambda( tl_clamp( lambda, 0.0f, 1.0f ) );
    }

    set_cpi_params( m_cpi, pcp );
    m_list_cpi.add( m_cpi );

    rigid_body *rb1 = pcp->m_bpi1->get_rb();
    rigid_body *rb2 = pcp->m_bpi2->get_rb();
    m_cpi->m_pcp = pcp;

    rigid_body_pair_key key( rb1, rb2 );
    rigid_body_constraint_contact *rbc =
        avl_tree_find<rigid_body_pair_key, rigid_body_constraint_contact, rigid_body_constraint_contact::avl_tree_accessor>(
            m_rbc_contact_search_tree_root,
            &key );

    m_cpi->m_rbc_contact = rbc;

    const contact_point_info *cached_cpi = NULL;
    if ( rbc )
    {
      if ( rbc->is_swapped( rb1, rb2 ) )
        m_cpi->swap();
      cached_cpi = rbc->get_cached_cpi();
    }

    m_cpi->set_closest_cached_psc( cached_cpi );
  }
}
