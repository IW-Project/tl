#pragma once

#include "phys_math.h"
#include "phys_mem.h"

class phys_convex_hull
{
public:
  struct ch_triangle
  {
    phys_vec3 m_normal;
    phys_vec3 *m_verts[3];
    const float get_dist( const phys_vec3 & );
  };
  struct ch_edge
  {
    phys_vec3 *m_verts[2];
  };

public:
  enum
  {
    MAX_INPUT_VERTICES = 6144
  };

  typedef phys_static_array<phys_vec3, MAX_INPUT_VERTICES> vertex_buffer_t;
  typedef phys_static_array<phys_vec3 *, MAX_INPUT_VERTICES> intermediate_vertex_list_t;
  typedef phys_static_array<ch_triangle, 256> intermediate_triangle_list_t;
  typedef phys_static_array<ch_edge, 128> intermediate_edge_list_t;
  typedef phys_static_array<phys_vec3 *, 64> convex_hull_vertex_list_t;
  typedef phys_static_array<ch_triangle, 64> convex_hull_triangle_list_t;

  vertex_buffer_t m_vertex_buffer;
  intermediate_vertex_list_t m_intermediate_vertex_list;
  intermediate_triangle_list_t m_intermediate_triangle_list;
  intermediate_edge_list_t m_intermediate_edge_list;
  convex_hull_vertex_list_t m_convex_hull_vert_list;
  convex_hull_triangle_list_t m_convex_hull_triangle_list;

  inline void reset_vertex_buffer() { m_vertex_buffer.remove_all(); }
  inline void add_vertex_to_buffer( const phys_vec3 &vert ) { *m_vertex_buffer.add() = vert; }
  inline void add_convex_hull_triangle( ch_triangle *tri ) { *m_convex_hull_triangle_list.add() = *tri; }
  inline void add_convex_hull_vert( phys_vec3 **vert )
  {
    tlAssert( vert );
    *m_convex_hull_vert_list.add() = *vert;
    m_intermediate_vertex_list.remove( vert );
  }

  inline void add_intermediate_edge( phys_vec3 *v0, phys_vec3 *v1 )
  {
    tlAssert( v0 && v1 && v0 != v1 );
    for ( intermediate_edge_list_t::iterator it = m_intermediate_edge_list.begin(); it != m_intermediate_edge_list.end(); it++ )
    {
      ch_edge *edge = &( *it );
      if ( edge->m_verts[0] == v0 && edge->m_verts[1] == v1 || edge->m_verts[0] == v1 && edge->m_verts[1] == v0 )
      {
        m_intermediate_edge_list.remove_slow( edge );
        return;
      }
    }

    ch_edge *edge = m_intermediate_edge_list.add();
    edge->m_verts[0] = v0;
    edge->m_verts[1] = v1;
  }

  inline void add_triangle_edges( ch_triangle *tri )
  {
    add_intermediate_edge( tri->m_verts[0], tri->m_verts[1] );
    add_intermediate_edge( tri->m_verts[1], tri->m_verts[2] );
    add_intermediate_edge( tri->m_verts[2], tri->m_verts[0] );
  }

  inline void create_intermediate_triangle( phys_vec3 *v0, phys_vec3 *v1, phys_vec3 *v2 )
  {
    ch_triangle *tri = m_intermediate_triangle_list.add();
    tri->m_verts[0] = v0;
    tri->m_verts[1] = v1;
    tri->m_verts[2] = v2;

    phys_vec3 normal = phys_cross( *v1 - *v0, *v2 - *v0 );
    float nnormal = Abs( normal );
    tlAssert( nnormal > 0.00001f );
    tri->m_normal = normal * ( 1.0f / nnormal );
  }

  void destroy_intermediate_triangle( ch_triangle *tri ) { m_intermediate_triangle_list.remove_slow( tri ); }

  inline phys_vec3 **support_intermediate_verts( const phys_vec3 &dir )
  {
    phys_vec3 **best_vert = NULL;
    float best_dotp = -1000000.0f;
    for ( intermediate_vertex_list_t::iterator it = m_intermediate_vertex_list.begin(); it != m_intermediate_vertex_list.end(); it++ )
    {
      phys_vec3 **vert = &( *it );
      float dotp = phys_dot( **vert, dir );
      if ( dotp > best_dotp )
      {
        best_dotp = dotp;
        best_vert = vert;
      }
    }
    tlAssert( best_vert );
    return best_vert;
  }

  inline void calculate_initial_triangle_vertices()
  {
    float largest_twice_area_sq = 0.0f;
    phys_vec3 **best_verts[3] = { NULL, NULL, NULL };

    const int NUM_SPHERE_VERTS = m_intermediate_vertex_list.get_count();

    for ( int v0 = 0; v0 < NUM_SPHERE_VERTS - 2; ++v0 )
    {
      phys_vec3 **z0 = &m_intermediate_vertex_list[v0];

      for ( int v1 = v0 + 1; v1 < NUM_SPHERE_VERTS - 1; ++v1 )
      {
        phys_vec3 **z1 = &m_intermediate_vertex_list[v1];

        for ( int v2 = v1 + 1; v2 < NUM_SPHERE_VERTS; ++v2 )
        {
          phys_vec3 **z2 = &m_intermediate_vertex_list[v2];

          float twice_area_sq = AbsSquared( phys_cross( **z1 - **z0, **z2 - **z0 ) );

          if ( twice_area_sq > largest_twice_area_sq )
          {
            largest_twice_area_sq = twice_area_sq;
            best_verts[0] = z0;
            best_verts[1] = z1;
            best_verts[2] = z2;
          }
        }
      }
    }

    tlAssert( largest_twice_area_sq > 0.0f );

    phys_vec3 nn = phys_cross( **best_verts[1] - **best_verts[0], **best_verts[2] - **best_verts[0] );
    nn = phys_Unitize( nn );

    *m_vertex_buffer.add() = **best_verts[0] + nn;

    *m_intermediate_vertex_list.add() = &m_vertex_buffer[m_vertex_buffer.get_count() - 1];

    bool not_done = true;
    while ( not_done )
    {
      not_done = false;
      for ( int i = 0; i < 2; ++i )
      {
        if ( best_verts[i] < best_verts[i + 1] )
        {
          phys_vec3 **temp = best_verts[i];
          best_verts[i] = best_verts[i + 1];
          best_verts[i + 1] = temp;
          not_done = true;
        }
      }
    }

    add_convex_hull_vert( best_verts[0] );
    add_convex_hull_vert( best_verts[1] );
    add_convex_hull_vert( best_verts[2] );
  }

  inline void init_convex_hull()
  {
    m_intermediate_vertex_list.remove_all();
    for ( vertex_buffer_t::iterator vert_i = m_vertex_buffer.begin(); vert_i != m_vertex_buffer.end(); vert_i++ )
    {
      *m_intermediate_vertex_list.add() = &( *vert_i );
    }
    m_convex_hull_vert_list.remove_all();
    m_intermediate_triangle_list.remove_all();
    m_convex_hull_triangle_list.remove_all();
    calculate_initial_triangle_vertices();
    tlAssert( m_convex_hull_vert_list.get_count() == 3 );
    create_intermediate_triangle( m_convex_hull_vert_list[0], m_convex_hull_vert_list[1], m_convex_hull_vert_list[2] );
    create_intermediate_triangle( m_convex_hull_vert_list[1], m_convex_hull_vert_list[0], m_convex_hull_vert_list[2] );
  }

  inline const float tetrahedron_volume( const phys_vec3 &a, const phys_vec3 &b, const phys_vec3 &c, const phys_vec3 &d )
  {
    return math::Abs( phys_dot( a - d, phys_cross( b - d, c - d ) ) ) / 6.0f;
  }

  inline const float calc_expansion_volume( const phys_vec3 &vert )
  {
    float volume = 0.0f;
    for ( intermediate_triangle_list_t::iterator tri_i = m_intermediate_triangle_list.begin(); tri_i != m_intermediate_triangle_list.end();
          tri_i++ )
    {
      ch_triangle *tri = &( *tri_i );
      if ( tri->get_dist( vert ) > 0.034000002f )
      {
        volume += tetrahedron_volume( vert, *tri->m_verts[0], *tri->m_verts[1], *tri->m_verts[2] );
      }
    }
    tlAssert( volume > 0.0f );
    return volume;
  }

  inline void create_edge_list( const phys_vec3 &vert )
  {
    m_intermediate_edge_list.remove_all();
    intermediate_triangle_list_t::iterator tri_i = m_intermediate_triangle_list.begin();
    while ( tri_i != m_intermediate_triangle_list.end() )
    {
      ch_triangle *tri = &( *tri_i );
      if ( phys_dot( tri->m_normal, vert - *tri->m_verts[0] ) > 0.034000002f )
      {
        add_triangle_edges( tri );
        m_intermediate_triangle_list.remove_slow( tri );
      }
      else
      {
        tri_i++;
      }
    }
    tlAssert( m_intermediate_edge_list.get_count() >= 3 );
  }

  inline void remove_inside_verts()
  {
    intermediate_vertex_list_t::iterator vert_i = m_intermediate_vertex_list.begin();
    while ( vert_i != m_intermediate_vertex_list.end() )
    {
      phys_vec3 **vert = &( *vert_i );
      bool inside = true;

      intermediate_triangle_list_t::iterator tri_i = m_intermediate_triangle_list.begin();
      intermediate_triangle_list_t::iterator tri_i_end = m_intermediate_triangle_list.end();
      while ( tri_i != tri_i_end && inside )
      {
        inside = ( *tri_i ).get_dist( **vert ) <= 0.034000002f;
        tri_i++;
      }

      if ( inside )
      {
        m_intermediate_vertex_list.remove( vert );
      }
      else
      {
        vert_i++;
      }
    }
  }
  
  inline void compute_convex_hull( const int max_verts, const float min_expansion_volume_percent )
  {
    tlAssert( max_verts >= 3 );

    init_convex_hull();

    float total_volume = 0.0f;
    while ( true )
    {
      bool can_continue = false;
      if ( m_convex_hull_vert_list.get_count() < max_verts )
      {
        if ( m_intermediate_triangle_list.get_count() > 0 )
        {
          can_continue = m_intermediate_vertex_list.get_count() > 0;
        }
      }

      if ( !can_continue )
      {
        break;
      }

      ch_triangle *best_tri = NULL;
      phys_vec3 **best_vert = NULL;
      float best_volume = 0.0f;

      intermediate_triangle_list_t::iterator tri_i = m_intermediate_triangle_list.begin();
      while ( tri_i != m_intermediate_triangle_list.end() )
      {
        ch_triangle *tri = &( *tri_i );
        phys_vec3 **support_vert = support_intermediate_verts( tri->m_normal );
        if ( tri->get_dist( **support_vert ) <= 0.034000002f )
        {
          add_convex_hull_triangle( tri );
          destroy_intermediate_triangle( tri );
        }
        else
        {
          float volume = calc_expansion_volume( **support_vert );
          if ( volume > best_volume )
          {
            best_tri = tri;
            best_vert = support_vert;
            best_volume = volume;
          }
          tri_i++;
        }
      }

      if ( !best_tri )
      {
        break;
      }

      tlAssert( best_vert );
      tlAssert( best_volume > 0.0f );
      tlAssert( best_tri->get_dist( **best_vert ) > 0.034000002f );

      const float increase_percent = total_volume <= 0.000099999997f ? 100000.0f : best_volume / total_volume;
      if ( min_expansion_volume_percent > increase_percent )
      {
        break;
      }

      total_volume += best_volume;

      create_edge_list( **best_vert );

      for ( intermediate_edge_list_t::iterator edge_i = m_intermediate_edge_list.begin(); edge_i != m_intermediate_edge_list.end(); edge_i++ )
      {
        ch_edge *edge = &( *edge_i );
        create_intermediate_triangle( *best_vert, edge->m_verts[0], edge->m_verts[1] );
      }

      add_convex_hull_vert( best_vert );
      remove_inside_verts();
    }

    while ( m_intermediate_triangle_list.get_count() > 0 )
    {
      ch_triangle *const tri = m_intermediate_triangle_list.get_list_head();
      add_convex_hull_triangle( tri );
      destroy_intermediate_triangle( tri );
    }
  }
};
