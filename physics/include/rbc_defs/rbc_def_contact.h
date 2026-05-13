#pragma once

#include "phys_base.h"
#include "phys_assert.h"
#include "phys_mem.h"
#include "rbc_def_base.h"
#include "phys_avl_tree.h"
#include "rigid_body.h"

#include <constraint_solver/pulse_sum_cache.h>

class phys_collision_pair;
class rigid_body;
class phys_transient_allocator;
class contact_point_info;

class rigid_body_pair_key
{
private:
  rigid_body *m_b1;
  rigid_body *m_b2;

public:
  rigid_body_pair_key( rigid_body *const b1, rigid_body *const b2 )
  {
    tlAssert( b1 != b2 );
    tlAssert( b1 );
    tlAssert( b2 );

    if ( b1 >= b2 )
    {
      m_b1 = b2;
      m_b2 = b1;
    }
    else
    {
      m_b1 = b1;
      m_b2 = b2;
    }
  }

  rigid_body_pair_key() {}

  int operator<( const rigid_body_pair_key &k ) const
  {
    if ( m_b1 == k.m_b1 )
    {
      return m_b2 < k.m_b2;
    }
    else
    {
      return m_b1 < k.m_b1;
    }
  }

  int operator>( const rigid_body_pair_key &k ) const
  {
    if ( m_b1 == k.m_b1 )
    {
      return m_b2 > k.m_b2;
    }
    else
    {
      return m_b1 > k.m_b1;
    }
  }

  int operator==( const rigid_body_pair_key &k ) const { return ( m_b1 == k.m_b1 ) && ( m_b2 == k.m_b2 ); }

  int operator>=( const rigid_body_pair_key &k ) const { return !( *this < k ); }
};

class contact_point_info
{
public:
  enum ps_cache_e
  {
    PSC_N = 0,
    PSC_F1 = 1,
    PSC_F2 = 2,
    NUM_PSC = 3
  };

  enum flags_e
  {
    FLAG_SOLVER_PRIORITY_MASK = 7,
    FLAG_NO_OVERFLOW_ERROR = 8,
    FLAG_HAS_VALID_RB2_ENTITY = 16
  };

  struct pulse_sum_cache_info
  {
    pulse_sum_cache m_ps_cache_list[3];
  };

  phys_vec3 m_normal;
  float m_fric_coef;
  float m_bounce_coef;
  float m_max_restitution_vel;
  int m_flags;
  int m_point_pair_count;
  phys_vec3 *m_list_b1_r_loc;
  phys_vec3 *m_list_b2_r_loc;
  pulse_sum_cache_info *m_list_pulse_sum_cache_info;
  contact_point_info *m_next_link;
  void *m_rb2_entity;
  float m_translation_lambda;

  inline void set_translation_lambda( const float translation_lambda ) { m_translation_lambda = translation_lambda; }

  phys_collision_pair *m_pcp;
  class rigid_body_constraint_contact *m_rbc_contact;

  inline void set_solver_priority( const unsigned int priority )
  {
    tlAssert( ( priority & FLAG_SOLVER_PRIORITY_MASK ) == priority );
    m_flags &= ~FLAG_SOLVER_PRIORITY_MASK;
    m_flags |= priority;
  }

  inline const unsigned int get_solver_priority() { return m_flags & FLAG_SOLVER_PRIORITY_MASK; }

  inline void swap()
  {
    m_normal = -m_normal;
    phys_vec3 *temp = m_list_b1_r_loc;
    m_list_b1_r_loc = m_list_b2_r_loc;
    m_list_b2_r_loc = temp;
  }

  inline void set_rb2_entity( void *rb2_entity )
  {
    set_flag( FLAG_HAS_VALID_RB2_ENTITY, 1 );
    m_rb2_entity = rb2_entity;
  }

  inline void *get_rb2_entity() { return m_rb2_entity; }

  void set_flag( const unsigned int f, const int b )
  {
    if ( b )
      m_flags |= f;
    else
      m_flags &= ~f;
  }

  inline const unsigned int get_flag( const unsigned int f ) { return m_flags & f; }

  inline void set_next_link( contact_point_info *const p ) { m_next_link = p; }

  inline contact_point_info *get_next_link() const { return m_next_link; }

  inline void get_closest_psc( const phys_vec3 &normal,
                               const phys_vec3 &b1_r_loc,
                               const phys_vec3 &b2_r_loc,
                               float *closest_error,
                               const pulse_sum_cache_info **closest_psc ) const
  {
    float normal_error_sq = AbsSquared( normal - m_normal );
    const phys_vec3 *pp_b1_r_loc = m_list_b1_r_loc;
    const phys_vec3 *pp_b2_r_loc = m_list_b2_r_loc;
    for ( int pp_i = 0; pp_i < m_point_pair_count; ++pp_i )
    {
      float error = ( normal_error_sq + AbsSquared( pp_b1_r_loc[pp_i] - b1_r_loc ) ) + AbsSquared( pp_b2_r_loc[pp_i] - b2_r_loc );
      if ( *closest_error > error )
      {
        *closest_error = error;
        *closest_psc = &m_list_pulse_sum_cache_info[pp_i];
      }
    }
  }

  inline void set_normal( phys_vec3 &normal )
  {
    m_normal = normal;
    PHYS_ASSERT_UNIT( m_normal );
  }

  inline void fake_constructor()
  {
    m_flags = 0;
    m_fric_coef = -1.0f;
    m_bounce_coef = -1.0f;
    m_max_restitution_vel = -1.0f;
    m_rb2_entity = NULL;
  }

  inline void check_surface_properties()
  {
    tlAssert( m_fric_coef >= 0.0f );
    tlAssert( m_bounce_coef <= 1.0f && m_bounce_coef >= 0.0f );
    tlAssert( m_max_restitution_vel >= 0.0f );
  }

  inline void set( const float fric_coef, const float bounce_coef, const float max_restitution_vel )
  {
    m_fric_coef = fric_coef;
    m_bounce_coef = bounce_coef;
    m_max_restitution_vel = max_restitution_vel;
    check_surface_properties();
  }

  inline void set_no_overflow_error( const bool no_overflow_error ) { set_flag( FLAG_NO_OVERFLOW_ERROR, no_overflow_error ); }

  static phys_transient_allocator *get_cpi_allocator();

  static inline contact_point_info *create_cpi( const int point_pair_count, const bool no_error, phys_transient_allocator *allocator )
  {
    tlAssert( point_pair_count > 0 );

    int vec3_offset = tl_align( sizeof( contact_point_info ), sizeof( phys_vec3 ) );
    int PSC_LIST_ALIGNMENT = 16;
    int psc_offset = tl_align( 32 * point_pair_count + vec3_offset, PSC_LIST_ALIGNMENT );
    int alloc_size = tl_align( 12 * point_pair_count, PSC_LIST_ALIGNMENT ) + psc_offset;

    int alloc_alignment = tl_max( sizeof( phys_vec3 ), PSC_LIST_ALIGNMENT );

    contact_point_info *cpi =
        (contact_point_info *)allocator->mt_allocate( alloc_size, alloc_alignment, 1, "contact_point_info buffer overflow" );
    PHYS_ASSERT( pai_create_cpi, cpi || no_error, "contact_point_info buffer overflow" );
    if ( cpi )
    {
      cpi->m_list_b1_r_loc = (phys_vec3 *)( (unsigned char *)cpi + vec3_offset );
      cpi->m_list_b2_r_loc = (phys_vec3 *)( (unsigned char *)cpi + vec3_offset + 4 * point_pair_count );
      cpi->m_list_pulse_sum_cache_info = (pulse_sum_cache_info *)( (unsigned char *)cpi + psc_offset );
      cpi->m_point_pair_count = point_pair_count;
      cpi->fake_constructor();
    }
    return cpi;
  }

  inline void set_closest_cached_psc( const contact_point_info *cached_cpi )
  {
    const phys_vec3 *b1_r_loc = m_list_b1_r_loc;
    const phys_vec3 *b2_r_loc = m_list_b2_r_loc;
    pulse_sum_cache_info *psci_i = m_list_pulse_sum_cache_info;
    pulse_sum_cache_info *last_psci_i = &psci_i[m_point_pair_count];
    while ( psci_i != last_psci_i )
    {
      set_closest_cached_psc( cached_cpi, m_normal, *b1_r_loc, *b2_r_loc, psci_i );
      ++psci_i;
      ++b1_r_loc;
      ++b2_r_loc;
    }
  }

  inline void set_closest_cached_psc( const contact_point_info *cached_cpi,
                                      const phys_vec3 &normal,
                                      const phys_vec3 &b1_r_loc,
                                      const phys_vec3 &b2_r_loc,
                                      pulse_sum_cache_info *psc )
  {
    const pulse_sum_cache_info *closest_psc = NULL;
    float closest_error = 1.0e7f;
    while ( cached_cpi )
    {
      cached_cpi->get_closest_psc( normal, b1_r_loc, b2_r_loc, &closest_error, &closest_psc );
      cached_cpi = cached_cpi->get_next_link();
    }

    if ( closest_psc )
    {
      psc->m_ps_cache_list[PSC_N].set_pulse_sum( closest_psc->m_ps_cache_list[PSC_N].get_pulse_sum() );
      psc->m_ps_cache_list[PSC_F1].set_pulse_sum( closest_psc->m_ps_cache_list[PSC_F1].get_pulse_sum() );
      psc->m_ps_cache_list[PSC_F2].set_pulse_sum( closest_psc->m_ps_cache_list[PSC_F2].get_pulse_sum() );
    }
    else if ( psc )
    {
      psc->m_ps_cache_list[PSC_N].zero_pulse_sum();
      psc->m_ps_cache_list[PSC_F1].zero_pulse_sum();
      psc->m_ps_cache_list[PSC_F2].zero_pulse_sum();
    }
  }
};

class rigid_body_constraint_contact : public rigid_body_constraint
{
  friend class rbcint;

private:
  unsigned char __align0[12];
  phys_simple_link_list<contact_point_info> m_list_contact_point_info_buffer_1;
  phys_simple_link_list<contact_point_info> m_list_contact_point_info_buffer_2;
  unsigned int m_solver_priority;
  void verify_constraint( rigid_body *const b1_, rigid_body *const b2_ );

public:
  phys_inplace_avl_tree_node<rigid_body_constraint_contact> m_avl_tree_node;
  rigid_body_pair_key m_avl_key;

  struct avl_tree_accessor
  {
    static inline phys_inplace_avl_tree_node<rigid_body_constraint_contact> *get_avl_node( rigid_body_constraint_contact *rbc )
    {
      return &rbc->m_avl_tree_node;
    }

    static inline const rigid_body_pair_key &get_avl_key( rigid_body_constraint_contact *rbc ) { return rbc->m_avl_key; }

    static inline void set_avl_key( rigid_body_constraint_contact *rbc, const rigid_body_pair_key &avl_key ) { rbc->m_avl_key = avl_key; }
  };

  rigid_body_constraint_contact();
  ~rigid_body_constraint_contact();

  inline void set_solver_priority( const int solver_priority ) { m_solver_priority = solver_priority; }

  inline const unsigned int get_solver_priority() { return m_solver_priority; }

  inline const bool is_swapped( rigid_body *const b1_, rigid_body *const b2_ ) { return b1 != b1_; }

  inline void add_cpi_simple( contact_point_info *cpi, rigid_body *const b1_, rigid_body *const b2_ )
  {
    tlAssert( cpi );
    tlAssert( cpi->m_list_b1_r_loc );
    tlAssert( cpi->m_list_b2_r_loc );
    tlAssert( cpi->m_list_pulse_sum_cache_info );
    tlAssert( cpi->m_point_pair_count > 0 );
    verify_constraint( b1_, b2_ );
    m_list_contact_point_info_buffer_1.add( cpi );
  }

  inline contact_point_info *get_cpi() { return m_list_contact_point_info_buffer_1.get_first(); }

  inline contact_point_info *get_cached_cpi() { return m_list_contact_point_info_buffer_2.get_first(); }

  inline void set_cpi( contact_point_info *cpi ) { m_list_contact_point_info_buffer_1.set_first( cpi ); }

  inline void set_cached_cpi( contact_point_info *cpi ) { m_list_contact_point_info_buffer_2.set_first( cpi ); }

  inline const int get_cached_point_count()
  {
    int count = 0;
    phys_simple_link_list<contact_point_info>::iterator cached_cpi_i = m_list_contact_point_info_buffer_2.begin();
    phys_simple_link_list<contact_point_info>::iterator cached_cpi_end = m_list_contact_point_info_buffer_2.end();
    while ( cached_cpi_i != cached_cpi_end )
    {
      count += (*cached_cpi_i).m_point_pair_count;
      ++cached_cpi_i;
    }
    return count;
  }

  inline const int get_point_count()
  {
    int count = 0;
    phys_simple_link_list<contact_point_info>::iterator cpi_i = m_list_contact_point_info_buffer_1.begin();
    phys_simple_link_list<contact_point_info>::iterator cpi_end = m_list_contact_point_info_buffer_1.end();
    while ( cpi_i != cpi_end )
    {
      count += ( *cpi_i ).m_point_pair_count;
      ++cpi_i;
    }
    return count;
  }

  inline void update_smallest_lambda()
  {
    tlAssert( b1->is_environment_rigid_body() == false );
    if ( b2->is_environment_rigid_body() )
    {
      for ( contact_point_info *cpi = get_cpi(); cpi; cpi = cpi->get_next_link() )
      {
        b1->m_smallest_lambda = tl_min( b1->m_smallest_lambda, cpi->m_translation_lambda );
      }
    }
  }

  inline void zero_pulse_sums() { ; }

  void setup_constraint( pulse_sum_constraint_solver *psys, const float delta_t );

  inline void epilog_vel_constraint( const float delta_t )
  {
    update_smallest_lambda();
    contact_point_info *temp = get_cpi();
    set_cpi( get_cached_cpi() );
    set_cached_cpi( temp );
    m_list_contact_point_info_buffer_1.remove_all();
  }

  inline static const float get_std_max_restitution_vel() { return 3400.f; }

  inline static const float get_std_active_limit_distance_eps() { return 1.02f; }

  inline static const float get_half_std_active_limit_distance_eps() { return 0.50999999f; }

  inline static const phys_vec3 get_std_active_limit_distance_eps_vec() { return phys_vec3( get_std_active_limit_distance_eps() ); }

  inline static const phys_vec3 get_half_std_active_limit_distance_eps_vec()
  {
    return phys_vec3( get_half_std_active_limit_distance_eps() );
  }

  inline static const float get_std_contact_min_sep_dist() { return 0.68000001f; }

  inline static const float get_MIN_LAMBDA_ACTIVE_DIST() { return 0.17f; }
};
