#pragma once

#include <math.h>

namespace math
{
  struct Float4
  {
    float x;
    float y;
    float z;
    float w;
  };

  inline float Abs( float a )
  {
    return fabsf( a );
  }

  inline float Sqrt( float a )
  {
    return sqrtf( a );
  }

  inline float RSqrt( float a )
  {
    return 1.0f / sqrtf( a );
  }

  template <typename T>
  inline T RSqrtAbs( const T &a )
  {
    return RSqrt( Abs( a ) );
  }

  template <typename T>
  inline T SqrtAbs( const T &a )
  {
    return Sqrt( Abs( a ) );
  }
}