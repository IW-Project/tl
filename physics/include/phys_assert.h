#pragma once

#include "tl_defs.h"

void phys_set_debug_callback( void ( *debug_callback )( void * ) );
void phys_exec_debug_callback( void *data );

inline intptr_t phys_assert_condition( intptr_t condition )
{
  return condition;
}

void PHYS_WARNING( const char *file, int line, const char *expr, const char *desc );

class phys_assert_info
{
public:
  int m_hits_total_count;
  int m_hits_frame_count;
  int m_max_hits_total;
  int m_max_hits_per_frame;
  bool m_use_warnings_only;
  phys_assert_info *m_next;

  phys_assert_info( int max_hits_total, int max_hits_per_frame, bool use_warnings_only );
  void frame_advance();
  static void phys_assert_info_frame_advance_all();
};

extern phys_assert_info *g_list_phys_assert_info;
extern phys_assert_info pai_create_cpi;
extern phys_assert_info pai_gjk_cache_system_max_num_gjk_ci;
extern phys_assert_info pai_max_num_sap_active_pair;
extern phys_assert_info pai_create_sap_active_pair;
extern phys_assert_info pai_check_terrain_query_params;
extern phys_assert_info pai_create_pulse_sum_contact;

#define PHYS_ASSERT( pai_obj, condition, message )                                                               \
  do                                                                                                             \
  {                                                                                                              \
    if ( !phys_assert_condition( (intptr_t)condition ) )                                                         \
    {                                                                                                            \
      if ( ( ( pai_obj ).m_hits_total_count < ( pai_obj ).m_max_hits_total || !( pai_obj ).m_max_hits_total ) && \
           ( pai_obj ).m_hits_frame_count < ( pai_obj ).m_max_hits_per_frame )                                   \
      {                                                                                                          \
        if ( ( pai_obj ).m_use_warnings_only )                                                                   \
        {                                                                                                        \
          PHYS_WARNING( __FILE__, __LINE__, #condition, message );                                               \
        }                                                                                                        \
        else if ( !( condition ) && _tlAssert( __FILE__, __LINE__, #condition, message ) )                       \
        {                                                                                                        \
          tlDebugBreak();                                                                                        \
        }                                                                                                        \
      }                                                                                                          \
      tlAtomicIncrement( (volatile u32 *)&( pai_obj ).m_hits_total_count );                                                      \
      tlAtomicIncrement( (volatile u32 *)&( pai_obj ).m_hits_frame_count );                                                      \
    }                                                                                                            \
  } while ( 0 )
