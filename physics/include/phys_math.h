#pragma once

#include <tl_system.h>
#include <vmath.h>
#include <phys_base.h>
#include <phys_mem_new.h>
#include <old_phys_math.h>

const phys_mat44 PHYS_IDENTITY_MATRIX( PHYS_X_VEC, PHYS_Y_VEC, PHYS_Z_VEC, PHYS_ZERO_VEC );

inline void PHYS_ASSERT_UNIT( const phys_vec3 &v )
{
  const float nv = Abs( v );
  tlAssert( fabsf( nv - 1.0f ) < 0.001f );
}

inline void PHYS_ASSERT_ORTHOGONAL( const phys_vec3 &v1, const phys_vec3 &v2 )
{
  const float d = phys_dot( v1, v2 );
  tlAssert( fabsf( d ) < 0.001f );
}

inline void PHYS_ASSERT_ORTHONORMAL( const phys_mat44 &m )
{
  const float nx = Abs( m.GetX() );
  const float ny = Abs( m.GetY() );
  const float nz = Abs( m.GetZ() );
  const float dxy = phys_dot( m.GetX(), m.GetY() );
  const float dxz = phys_dot( m.GetX(), m.GetZ() );
  const float dyz = phys_dot( m.GetY(), m.GetZ() );
  tlAssert( fabsf( nx - 1.0f ) < 0.001f );
  tlAssert( fabsf( ny - 1.0f ) < 0.001f );
  tlAssert( fabsf( nz - 1.0f ) < 0.001f );
  tlAssert( fabsf( dxy ) < 0.001f );
  tlAssert( fabsf( dxz ) < 0.001f );
  tlAssert( fabsf( dyz ) < 0.001f );
}

inline void PHYS_ASSERT_ALIGNED( const phys_vec3 &v )
{
  tlAssert( uint( &v ) % PHYS_ALIGNOF( phys_vec3 ) == 0 );
}

inline phys_vec3 phys_min( const phys_vec3 &v1, const phys_vec3 &v2 )
{
  return phys_vec3( tl_min( v1.GetX(), v2.GetX() ), tl_min( v1.GetY(), v2.GetY() ), tl_min( v1.GetZ(), v2.GetZ() ) );
}

inline phys_vec3 phys_max( const phys_vec3 &v1, const phys_vec3 &v2 )
{
  return phys_vec3( tl_max( v1.GetX(), v2.GetX() ), tl_max( v1.GetY(), v2.GetY() ), tl_max( v1.GetZ(), v2.GetZ() ) );
}

#define VALIDATE_POSITION_VECTOR( pos, data )                                                                                        \
  if ( ( pos )[0] != ( pos )[0] || fabsf( ( pos )[0] ) > 100000.0f || ( pos )[1] != ( pos )[1] || fabsf( ( pos )[1] ) > 100000.0f || \
       ( pos )[2] != ( pos )[2] || fabsf( ( pos )[2] ) > 100000.0f )                                                                 \
  {                                                                                                                                  \
    phys_exec_debug_callback( data );                                                                                                \
  }
