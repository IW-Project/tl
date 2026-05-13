#pragma once

#include <tl_system.h>
#include <vmath.h>

inline const float phys_sqr( const float x )
{
  return x * x;
}

ALIGN(16) class phys_vec3
{
  float x, y, z, w;

public:
  float GetX() const { return x; }
  float GetY() const { return y; }
  float GetZ() const { return z; }

  inline void SetX( const float v ) { x = v; }
  inline void SetY( const float v ) { y = v; }
  inline void SetZ( const float v ) { z = v; }
  void Set( const float _x, const float _y, const float _z )
  {
    x = _x;
    y = _y;
    z = _z;
  }

  template <typename T>
  const float &operator[]( const T i ) const
  {
    return ( (const float *)this )[i];
  }

  template <typename T>
  float &operator[]( const T i )
  {
    return ( (float *)this )[i];
  }

  phys_vec3() {}
  phys_vec3( float _x, float _y, float _z )
    : x( _x ),
      y( _y ),
      z( _z )
  {
  }
  phys_vec3( const float xyz )
    : x( xyz ),
      y( xyz ),
      z( xyz )
  {
  }

  phys_vec3 &operator=( const phys_vec3 &v )
  {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }

  phys_vec3 &operator+=( const phys_vec3 &v )
  {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }

  phys_vec3 &operator-=( const phys_vec3 &v )
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  phys_vec3 &operator*=( const float f )
  {
    x *= f;
    y *= f;
    z *= f;
    return *this;
  }

  phys_vec3 &operator/=( const float f )
  {
    const float d_inv = 1.0f / f;
    x *= d_inv;
    y *= d_inv;
    z *= d_inv;
    return *this;
  }

  phys_vec3 operator-() const { return phys_vec3( -x, -y, -z ); }

  friend inline phys_vec3 operator+( const phys_vec3 &a, const phys_vec3 &b ) { return phys_vec3( a.x + b.x, a.y + b.y, a.z + b.z ); }

  friend inline phys_vec3 operator-( const phys_vec3 &a, const phys_vec3 &b ) { return phys_vec3( a.x - b.x, a.y - b.y, a.z - b.z ); }

  friend inline phys_vec3 operator*( const phys_vec3 &v, const float f ) { return phys_vec3( v.x * f, v.y * f, v.z * f ); }

  friend inline phys_vec3 operator*( const phys_vec3 &a, const phys_vec3 &b ) { return phys_vec3( a.x * b.x, a.y * b.y, a.z * b.z ); }

  friend inline phys_vec3 operator*( const float f, const phys_vec3 &v ) { return phys_vec3( v.x * f, v.y * f, v.z * f ); }

  friend inline phys_vec3 operator/( const phys_vec3 &v, const float f )
  {
    const float d_inv = 1.0f / f;
    return phys_vec3( v.x * d_inv, v.y * d_inv, v.z * d_inv );
  }

  friend inline float AbsSquared( const phys_vec3 &v ) { return ( v.x * v.x ) + ( v.y * v.y ) + ( v.z * v.z ); }

  friend inline float Abs( const phys_vec3 &v ) { return sqrtf( v.x * v.x + v.y * v.y + v.z * v.z ); }

  friend inline phys_vec3 phys_Unitize( const phys_vec3 &v )
  {
    const float v2 = AbsSquared( v );
    if ( v2 > phys_sqr( 0.00001f ) )
      return v / sqrtf( v2 );
    else
      return v;
  }

  friend inline const float phys_dot( const phys_vec3 &a, const phys_vec3 &b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }

  friend inline const phys_vec3 phys_cross( const phys_vec3 &a, const phys_vec3 &b )
  {
    return phys_vec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
  }

  friend inline phys_vec3 phys_cross_i( const phys_vec3 &r ) { return phys_vec3( 0, r.z, -r.y ); }
  friend inline phys_vec3 phys_cross_j( const phys_vec3 &r ) { return phys_vec3( -r.z, 0, r.x ); }
  friend inline phys_vec3 phys_cross_k( const phys_vec3 &r ) { return phys_vec3( r.y, -r.x, 0 ); }

  friend inline phys_vec3 phys_Inv( const phys_vec3 &v ) { return phys_vec3( 1.0f / v.x, 1.0f / v.y, 1.0f / v.z ); }

  friend inline phys_vec3 phys_inv_diag_multiply( const phys_vec3 &v1, const phys_vec3 &v2 )
  {
    return phys_vec3( v2.x / v1.x, v2.y / v1.y, v2.z / v1.z );
  }

  friend inline void phys_add_x( phys_vec3 &v1, const phys_vec3 &v2 ) { v1.x += v2.x; }

  friend inline void phys_add_y( phys_vec3 &v1, const phys_vec3 &v2 ) { v1.y += v2.y; }

  friend inline void phys_add_z( phys_vec3 &v1, const phys_vec3 &v2 ) { v1.z += v2.z; }

  friend class phys_mat44;
};

ALIGN(16) class phys_mat44
{
private:
  phys_vec3 x, y, z, w;

public:
  void fix_w_column()
  {
    x.w = 0.0f;
    y.w = 0.0f;
    z.w = 0.0f;
    w.w = 1.0f;
  }

  phys_mat44() {}

  phys_mat44( const phys_vec3 &_x, const phys_vec3 &_y, const phys_vec3 &_z, const phys_vec3 &_w )
  {
    x = _x;
    y = _y;
    z = _z;
    w = _w;
  }

  phys_vec3 &GetX() { return *(phys_vec3 *)&x; }
  phys_vec3 &GetY() { return *(phys_vec3 *)&y; }
  phys_vec3 &GetZ() { return *(phys_vec3 *)&z; }
  phys_vec3 &GetW() { return *(phys_vec3 *)&w; }

  const phys_vec3 &GetX() const { return *(phys_vec3 *)&x; }
  const phys_vec3 &GetY() const { return *(phys_vec3 *)&y; }
  const phys_vec3 &GetZ() const { return *(phys_vec3 *)&z; }
  const phys_vec3 &GetW() const { return *(phys_vec3 *)&w; }

  inline void SetX( const phys_vec3 &v ) { GetX() = v; }
  inline void SetY( const phys_vec3 &v ) { GetY() = v; }
  inline void SetZ( const phys_vec3 &v ) { GetZ() = v; }
  inline void SetW( const phys_vec3 &v ) { GetW() = v; }

  inline const phys_vec3 &operator[]( int i ) const { return reinterpret_cast<const phys_vec3 &>( ( (float *)this )[4 * i] ); }
  inline phys_vec3 &operator[]( int i ) { return reinterpret_cast<phys_vec3 &>( ( (float *)this )[4 * i] ); }
};

inline const phys_vec3 phys_multiply( const phys_mat44 &mat, const phys_vec3 &v )
{
  return v.GetX() * mat.GetX() + v.GetY() * mat.GetY() + v.GetZ() * mat.GetZ();
}

inline const phys_vec3 phys_full_multiply( const phys_mat44 &mat, const phys_vec3 &v )
{
  return phys_multiply( mat, v ) + mat.GetW();
}

inline const phys_vec3 phys_inv_multiply( const phys_mat44 &mat, const phys_vec3 &v )
{
  return phys_vec3( phys_dot( v, mat.GetX() ), phys_dot( v, mat.GetY() ), phys_dot( v, mat.GetZ() ) );
}

inline const phys_vec3 phys_full_inv_multiply( const phys_mat44 &mat, const phys_vec3 &v )
{
  return phys_inv_multiply( mat, v - mat.GetW() );
}

inline void phys_multiply_mat( phys_mat44 &dest, const phys_mat44 &left, const phys_mat44 &right )
{
  if ( &dest != &left )
  {
    dest.GetX() = phys_multiply( left, right.GetX() );
    dest.GetY() = phys_multiply( left, right.GetY() );
    dest.GetZ() = phys_multiply( left, right.GetZ() );
  }
  else
  {
    const phys_mat44 temp = left;
    phys_multiply_mat( dest, temp, right );
  }
}

inline void phys_full_multiply_mat( phys_mat44 &dest, const phys_mat44 &left, const phys_mat44 &right )
{
  if ( &dest != &left )
  {
    dest.GetX() = phys_multiply( left, right.GetX() );
    dest.GetY() = phys_multiply( left, right.GetY() );
    dest.GetZ() = phys_multiply( left, right.GetZ() );
    dest.GetW() = phys_full_multiply( left, right.GetW() );
  }
  else
  {
    const phys_mat44 temp = left;
    phys_full_multiply_mat( dest, temp, right );
  }
}

inline void phys_inv_multiply_mat( phys_mat44 &dest, const phys_mat44 &left, const phys_mat44 &right )
{
  if ( &dest != &left )
  {
    dest[0] = phys_inv_multiply( left, right.GetX() );
    dest[1] = phys_inv_multiply( left, right.GetY() );
    dest[2] = phys_inv_multiply( left, right.GetZ() );
  }
  else
  {
    const phys_mat44 temp = left;
    phys_inv_multiply_mat( dest, temp, right );
  }
}

inline void phys_full_inv_multiply_mat( phys_mat44 &dest, const phys_mat44 &left, const phys_mat44 &right )
{
  if ( &dest != &left )
  {
    dest[0] = phys_inv_multiply( left, right.GetX() );
    dest[1] = phys_inv_multiply( left, right.GetY() );
    dest[2] = phys_inv_multiply( left, right.GetZ() );
    dest[3] = phys_full_inv_multiply( left, right.GetW() );
  }
  else
  {
    const phys_mat44 temp = left;
    phys_full_inv_multiply_mat( dest, temp, right );
  }
}

const phys_vec3 PHYS_ZERO_VEC( 0.0f, 0.0f, 0.0f );
const phys_vec3 PHYS_X_VEC( 1.0f, 0.0f, 0.0f );
const phys_vec3 PHYS_Y_VEC( 0.0f, 1.0f, 0.0f );
const phys_vec3 PHYS_Z_VEC( 0.0f, 0.0f, 1.0f );

inline void SetIdentity( phys_mat44 &m )
{
  m[0][0] = 1.0f;
  m[0][1] = 0.0f;
  m[0][2] = 0.0f;
  m[0][3] = 0.0f;
  m[1][0] = 0.0f;
  m[1][1] = 1.0f;
  m[1][2] = 0.0f;
  m[1][3] = 0.0f;
  m[2][0] = 0.0f;
  m[2][1] = 0.0f;
  m[2][2] = 1.0f;
  m[2][3] = 0.0f;
  m[3][0] = 0.0f;
  m[3][1] = 0.0f;
  m[3][2] = 0.0f;
  m[3][3] = 1.0f;
}

inline void FixupWRow( phys_mat44 & ) {}

inline void orthonormalize( phys_mat44 *mat )
{
  const float nx = Abs( mat->GetX() );
  mat->GetX() *= 1.0f / nx;
  mat->GetY() -= phys_dot( mat->GetY(), mat->GetX() ) * mat->GetX();
  const float ny = Abs( mat->GetY() );
  mat->GetY() *= 1.0f / ny;
  mat->GetZ() = phys_cross( mat->GetX(), mat->GetY() );
}

inline const phys_vec3 construct_orth_ud( const phys_vec3 &ud )
{
  const float nud = Abs( ud );
  tlAssert( fabsf( nud - 1.0f ) < .0001f );
  phys_vec3 orth_ud = phys_cross_i( ud );
  float north_ud = Abs( orth_ud );
  if ( north_ud < .0001f )
  {
    orth_ud = phys_cross_j( ud );
    north_ud = Abs( orth_ud );
    if ( north_ud < .0001f )
    {
      orth_ud = phys_cross_k( ud );
      north_ud = Abs( orth_ud );
      tlAssert( north_ud >= .0001f );
    }
  }
  orth_ud *= 1.0f / north_ud;
  return orth_ud;
}

inline void phys_swap_float( float &l, float &r )
{
  float t = l;
  l = r;
  r = t;
}

inline void phys_transpose( phys_mat44 &dest, const phys_mat44 &source )
{
  if ( &dest != &source )
  {
    dest[0][0] = source[0][0];
    dest[0][1] = source[1][0];
    dest[0][2] = source[2][0];

    dest[1][0] = source[0][1];
    dest[1][1] = source[1][1];
    dest[1][2] = source[2][1];

    dest[2][0] = source[0][2];
    dest[2][1] = source[1][2];
    dest[2][2] = source[2][2];
  }
  else
  {
    phys_swap_float( dest[0][1], dest[1][0] );
    phys_swap_float( dest[0][2], dest[2][0] );
    phys_swap_float( dest[1][2], dest[2][1] );
  }
}

inline void phys_full_inverse( phys_mat44 &dest, const phys_mat44 &source )
{
  if ( &dest != &source )
  {
    phys_transpose( dest, source );
    dest[3] = -phys_multiply( dest, source[3] );
  }
  else
  {
    phys_mat44 temp;
    phys_full_inverse( temp, source );
    dest = temp;
  }
}

inline void make_rotate( phys_mat44 &m, const phys_vec3 &u, const float ca, const float sa )
{
  float omca = 1.0f - ca;
  float xx = u[0] * u[0] * omca;
  float xy = u[0] * u[1] * omca;
  float xz = u[0] * u[2] * omca;
  float yy = u[1] * u[1] * omca;
  float yz = u[1] * u[2] * omca;
  float zz = u[2] * u[2] * omca;
  float xsa = u[0] * sa;
  float ysa = u[1] * sa;
  float zsa = u[2] * sa;

  m[0][0] = xx + ca;
  m[1][0] = xy - zsa;
  m[2][0] = xz + ysa;
  m[3][0] = 0.0f;

  m[0][1] = xy + zsa;
  m[1][1] = yy + ca;
  m[2][1] = yz - xsa;
  m[3][1] = 0.0f;

  m[0][2] = xz - ysa;
  m[1][2] = yz + xsa;
  m[2][2] = zz + ca;
  m[3][2] = 0.0f;

  m[0][3] = 0.0f;
  m[1][3] = 0.0f;
  m[2][3] = 0.0f;
  m[3][3] = 1.0f;
}

const float PHYS_PI_ = 3.141592653f;
const float PHYS_PI_TIMES_2 = PHYS_PI_ * 2.0f;
const float PHYS_PI_OVER_2 = PHYS_PI_ / 2.0f;

inline float phys_acosf_safe( float co_ )
{
  if ( co_ >= 1.0f )
    return 0.0f;
  else if ( co_ <= -1.0f )
    return PHYS_PI_;
  else
    return acosf( co_ );
}

inline void make_rotate( phys_mat44 *mat, const phys_vec3 &v, const float theta_factor = 1.0f, const float max_rotation_radians = 1000.0f )
{
  float nv = Abs( v );
  if ( nv < .00001f )
  {
    SetIdentity( *mat );
  }
  else
  {
    const phys_vec3 ud = ( 1.0f / nv ) * v;
    nv *= theta_factor;
    if ( max_rotation_radians < nv )
    {
      nv = max_rotation_radians;
    }
    make_rotate( *mat, ud, cosf( nv ), sinf( nv ) );
  }
}

inline void make_rotate( phys_mat44 *mat, const phys_vec3 &v1, const phys_vec3 &v2 )
{
  phys_vec3 ud = phys_cross( v1, v2 );
  float si_ = AbsSquared( ud );
  if ( si_ < .00001f )
  {
    SetIdentity( *mat );
  }
  else
  {
    si_ = sqrtf( si_ );
    ud *= 1.0f / si_;
    const float co_ = phys_dot( v1, v2 );

    const float len = sqrtf( co_ * co_ + si_ * si_ );
    make_rotate( *mat, ud, co_ / len, si_ / len );
  }
}

class Phys_UnitQuaternion
{
protected:
  float x, y, z, w;

public:
  float GetX() const { return x; }
  float GetY() const { return y; }
  float GetZ() const { return z; }
  float GetW() const { return w; }
  const phys_vec3 GetImag() const { return phys_vec3( x, y, z ); }
  float GetReal() const { return w; }

  template <typename T>
  const float &operator[]( T i ) const
  {
    return ( (const float *)this )[i];
  }

  template <typename T>
  float &operator[]( T i )
  {
    return ( (float *)this )[i];
  }

  Phys_UnitQuaternion() {}
  Phys_UnitQuaternion( float _x, float _y, float _z, float _w )
    : x( _x ),
      y( _y ),
      z( _z ),
      w( _w )
  {
  }
};

inline Phys_UnitQuaternion Phys_GetQuaternion( const phys_mat44 &M )
{
  int i, j, k;
  float tr, s, q[4];
  float a, b, c, d;

  tr = M[0][0] + M[1][1] + M[2][2];
  if ( tr > 0.0f )
  {
    s = sqrtf( tr + 1.0f );
    a = s * 0.5f;
    s = 0.5f / s;
    b = ( M[2][1] - M[1][2] ) * s;
    c = ( M[0][2] - M[2][0] ) * s;
    d = ( M[1][0] - M[0][1] ) * s;
  }
  else
  {
    i = 0;
    j = 1;
    k = 2;
    if ( M[1][1] > M[0][0] )
    {
      i = 1;
      j = 2;
      k = 0;
    }
    if ( M[2][2] > M[i][i] )
    {
      i = 2;
      j = 0;
      k = 1;
    }

    s = sqrtf( M[i][i] - M[j][j] - M[k][k] + 1.0f );
    q[i + 1] = s * 0.5f;
    if ( s != 0.0f )
      s = 0.5f / s;
    q[0] = ( M[k][j] - M[j][k] ) * s;
    q[j + 1] = ( M[j][i] + M[i][j] ) * s;
    q[k + 1] = ( M[k][i] + M[i][k] ) * s;

    a = q[0];
    b = q[1];
    c = q[2];
    d = q[3];
  }
  b = -b;
  c = -c;
  d = -d;
  return Phys_UnitQuaternion( b, c, d, a );
}
