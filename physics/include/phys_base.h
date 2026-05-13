#pragma once

#include "phys_mem.h"
#include "phys_assert.h"

inline const bool rel_cmp_ge( const float v1, const float v2, const float e )
{
  return v1 >= ( ( 1.0f - e ) * v2 );
}

inline bool IS_NANF( float x )
{
  return ( *(unsigned int *)&x & 0x7F800000 ) == 0x7F800000;
}
