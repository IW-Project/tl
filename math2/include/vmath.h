#pragma once

#include "float4_f.h"

namespace math
{
  static const float MATH_PI = 3.14159265f;

  inline float ASinUpper( float y )
  {
    register float a;

    register float x1 = y;
    if ( x1 < 0.5f )
    {
      register float x2 = x1 * x1;
      register float x3 = x2 * x1;
      register float x5 = x2 * x3;
      register float x7 = x2 * x5;
      a = 5.398124e-2f * x7;
      a += 0.075f * x5;
      a += .1666667f * x3;
      a += x1;
    }
    else
    {
      x1 = SqrtAbs( ( 1.0f - x1 ) * 0.5f );
      register float x2 = x1 * x1;
      register float x3 = x2 * x1;
      register float x5 = x2 * x3;
      register float x7 = x2 * x5;
      a = -.1079625f * x7;
      a += -.15f * x5;
      a += -.3333333f * x3;
      a += -2.0f * x1;
      a += 1.570796f;
    }
    return a;
  }

  inline float ASin( float y )
  {
    float a = ASinUpper( Abs( y ) );
    if ( y < 0.0f )
    {
      return -a;
    }
    return a;
  }

  inline float ACos( float x )
  {
    return ( MATH_PI / 2.0f ) - ASin( x );
  }

  /*
  class Vector4;
  class Mat43;
  class Mat44;
  class Mat33;
  class Mat34;
  class TranMat43;
  class DiagMat44;
  class DiagMat33;
  class Dir3;
  class Position3;
  class Quaternion;
  typedef Dir3 UnitDir3;

  typedef const Dir3 &Dir3Arg;
  typedef const Position3 &Position3Arg;
  typedef const Quaternion &QuaternionArg;
  typedef const Vector4 &Vector4Arg;
  typedef const Mat43 &Mat43Arg;
  typedef const Mat44 &Mat44Arg;
  typedef const Mat33 &Mat33Arg;
  typedef const Mat34 &Mat34Arg;
  typedef const TranMat43 &TranMat43Arg;
  typedef const DiagMat44 &DiagMat44Arg;
  typedef const DiagMat33 &DiagMat33Arg;
  typedef const UnitDir3 &UnitDir3Arg;

  enum SinCosType
  {
    _cos,
    _negsin,
    _negcos,
    _sin,
    _any
  };

  const Float4 Float4_XAxis = { 1.0f, 0.0f, 0.0f, 0.0f };
  const Float4 Float4_YAxis = { 0.0f, 1.0f, 0.0f, 0.0f };
  const Float4 Float4_ZAxis = { 0.0f, 0.0f, 1.0f, 0.0f };
  const Float4 Float4_Zero = { 0.0f, 0.0f, 0.0f, 0.0f };

  class Vector4
  {
  public:
    struct Constant
    {
      float x;
      float y;
      float z;
      float w;
    };

    struct Packed
    {
      Packed( float _x, float _y, float _z, float _w ) { Set( _x, _y, _z, _w ); }

      Packed( Vector4Arg _v ) { Set( _v ); }

      Packed()
      {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 0.0f;
      }

      float x;
      float y;
      float z;
      float w;

      void Set( Vector4Arg _v )
      {
        x = _v.v.x;
        y = _v.v.y;
        z = _v.v.z;
        w = _v.v.w;
      }

      void Set( const float _x, const float _y, const float _z, const float _w )
      {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
      }

      void SetX( const float _x ) { x = _x; }

      void SetY( const float _y ) { y = _y; }

      void SetZ( const float _z ) { z = _z; }

      void SetW( const float _w ) { w = _w; }

      float GetX() { return x; }

      float GetY() { return y; }

      float GetZ() { return z; }

      float GetW() { return w; }

      float &operator[]( int i ) { return *( &x + i ); }

      const float &operator[]( int i ) const { return *( &x + i ); }
    };

  public:
    Float4 v;

  public:
    Vector4( const Vector4::Packed &_p )
    {
      v.x = _p.x;
      v.y = _p.y;
      v.z = _p.z;
      v.w = _p.w;
    }

    Vector4( const Vector4::Constant &_c )
    {
      v.x = _c.x;
      v.y = _c.y;
      v.z = _c.z;
      v.w = _c.w;
    }

    Vector4( QuaternionArg _q )
    {
      v.x = _q.v.x;
      v.y = _q.v.y;
      v.z = _q.v.z;
      v.w = _q.v.w;
    }

    Vector4( Position3Arg _p, const float _w );

    Vector4( Position3Arg _p );

    Vector4( Dir3Arg _d, const float _f );

    Vector4( Dir3Arg _d );

    Vector4( const float *_f )
    {
      v.x = _f[0];
      v.y = _f[1];
      v.z = _f[2];
      v.w = _f[3];
    }

    Vector4( const float & );

    Vector4( const float _x, const float _y, const float _z, const float _w )
    {
      v.x = _x;
      v.y = _y;
      v.z = _z;
      v.w = _w;
    }

    Vector4()
    {
      v.x = 0.0f;
      v.y = 0.0f;
      v.z = 0.0f;
      v.w = 0.0f;
    }

    float GetX() const { return v.x; }

    float GetY() const { return v.y; }

    float GetZ() const { return v.z; }

    float GetW() const { return v.w; }

    void SetX( const float _f ) { v.x = _f; }

    void SetY( const float _f ) { v.y = _f; }

    void SetZ( const float _f ) { v.z = _f; }

    void SetW( const float _f ) { v.w = _f; }

    void Set( const float _x, const float _y, const float _z, const float _w )
    {
      v.x = _x;
      v.y = _y;
      v.z = _z;
      v.w = _w;
    }

    Vector4 &operator+=( const float _f )
    {
      v.x += _f;
      v.y += _f;
      v.z += _f;
      v.w += _f;
      return *this;
    }

    Vector4 &operator+=( Vector4Arg _v )
    {
      v.x += _v.v.x;
      v.y += _v.v.y;
      v.z += _v.v.z;
      v.w += _v.v.w;
      return *this;
    }

    Vector4 &operator-=( const float _f )
    {
      v.x -= _f;
      v.y -= _f;
      v.z -= _f;
      v.w -= _f;
      return *this;
    }

    Vector4 &operator-=( Vector4Arg _v )
    {
      v.x -= _v.v.x;
      v.y -= _v.v.y;
      v.z -= _v.v.z;
      v.w -= _v.v.w;
      return *this;
    }

    Vector4 &operator*=( Mat44Arg _mat );

    Vector4 &operator*=( DiagMat44Arg _mat )
    {
      v.x *= _mat.v.x;
      v.y *= _mat.v.y;
      v.z *= _mat.v.z;
      v.w *= _mat.v.w;
      return *this;
    }

    Vector4 &operator*=( Mat43Arg _mat )
    {
      v.x = v.x * _mat.x.v.x + v.y * _mat.y.v.x + v.z * _mat.z.v.x;
      v.y = v.x * _mat.x.v.y + v.y * _mat.y.v.y + v.z * _mat.z.v.y;
      v.z = v.x * _mat.x.v.z + v.y * _mat.y.v.z + v.z * _mat.z.v.z;
      return *this;
    }

    Vector4 &operator*=( TranMat43Arg _mat )
    {
      v.x *= _mat.v.x;
      v.y *= _mat.v.y;
      v.z *= _mat.v.z;
      return *this;
    }

    Vector4 &operator*=( Mat33Arg _mat )
    {
      v.x = v.x * _mat.x.v.x + v.y * _mat.y.v.x + v.z * _mat.z.v.x;
      v.y = v.x * _mat.x.v.y + v.y * _mat.y.v.y + v.z * _mat.z.v.y;
      v.z = v.x * _mat.x.v.z + v.y * _mat.y.v.z + v.z * _mat.z.v.z;
      return *this;
    }

    Vector4 &operator*=( DiagMat33Arg _mat )
    {
      v.x *= _mat.v.x;
      v.y *= _mat.v.y;
      v.z *= _mat.v.z;
      return *this;
    }

    Vector4 &operator*=( const float _f )
    {
      v.x *= _f;
      v.y *= _f;
      v.z *= _f;
      v.w *= _f;
      return *this;
    }

    Vector4 &operator*=( Vector4Arg _v )
    {
      v.x *= _v.v.x;
      v.y *= _v.v.y;
      v.z *= _v.v.z;
      v.w *= _v.v.w;
      return *this;
    }

    Vector4 &operator/=( Mat44Arg _mat );

    Vector4 &operator/=( DiagMat44Arg _mat )
    {
      v.x /= _mat.v.x;
      v.y /= _mat.v.y;
      v.z /= _mat.v.z;
      v.w /= _mat.v.w;
      return *this;
    }

    Vector4 &operator/=( Mat43Arg _mat )
    {
      v.x = v.x / _mat.x.v.x + v.y / _mat.y.v.x + v.z / _mat.z.v.x;
      v.y = v.x / _mat.x.v.y + v.y / _mat.y.v.y + v.z / _mat.z.v.y;
      v.z = v.x / _mat.x.v.z + v.y / _mat.y.v.z + v.z / _mat.z.v.z;
      return *this;
    }

    Vector4 &operator/=( TranMat43Arg _mat )
    {
      v.x /= _mat.v.x;
      v.y /= _mat.v.y;
      v.z /= _mat.v.z;
      return *this;
    }

    Vector4 &operator/=( Mat33Arg _mat )
    {
      v.x = v.x / _mat.x.v.x + v.y / _mat.y.v.x + v.z / _mat.z.v.x;
      v.y = v.x / _mat.x.v.y + v.y / _mat.y.v.y + v.z / _mat.z.v.y;
      v.z = v.x / _mat.x.v.z + v.y / _mat.y.v.z + v.z / _mat.z.v.z;
      return *this;
    }

    Vector4 &operator/=( DiagMat33Arg _mat )
    {
      v.x /= _mat.v.x;
      v.y /= _mat.v.y;
      v.z /= _mat.v.z;
      return *this;
    }

    Vector4 &operator/=( const float _f )
    {
      float invF = 1.0f / _f;
      v.x *= invF;
      v.y *= invF;
      v.z *= invF;
      v.w *= invF;
      return *this;
    }

    Vector4 &operator/=( Vector4Arg _v )
    {
      v.x /= _v.v.x;
      v.y /= _v.v.y;
      v.z /= _v.v.z;
      v.w /= _v.v.w;
      return *this;
    }

    float &operator[]( unsigned int i ) { return *( &v.x + i ); }

    const float &operator[]( unsigned int i ) const { return *( &v.x + i ); }
  };

  Vector4 Vector4_Zero()
  {
    Vector4 tmp;
    tmp.v = Float4_Zero;
    return tmp;
  }

  Vector4 RSqrt( Vector4Arg _v )
  {
    Vector4 tmp;
    tmp.v.x = sqrtf( _v.v.x );
    tmp.v.y = sqrtf( _v.v.y );
    tmp.v.z = sqrtf( _v.v.z );
    tmp.v.w = sqrtf( _v.v.w );
    return tmp;
  }

  class Dir3
  {
  public:
    Float4 v;

    struct Constant
    {
      float x;
      float y;
      float z;
      float w;
    };

    struct Packed
    {
      Packed( const float _x, const float _y, const float _z ) { Set( _x, _y, _z ); }

      Packed( Dir3Arg dir ) { Set( dir ); }

      Packed()
      {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
      }

      float x;
      float y;
      float z;

      void Set( Dir3Arg dir )
      {
        x = dir.v.x;
        y = dir.v.y;
        z = dir.v.z;
      }

      void Set( const float _x, const float _y, const float _z )
      {
        x = _x;
        y = _y;
        z = _z;
      }

      void SetX( const float _x ) { x = _x; }

      void SetY( const float _y ) { y = _y; }

      void SetZ( const float _z ) { z = _z; }

      float GetX() const { return x; }

      float GetY() const { return y; }

      float GetZ() const { return z; }

      float &operator[]( int i ) { return *( &x + i ); }

      float &operator[]( int i ) { return *( &x + i ); }

      Packed &operator+=( Dir3Arg dir )
      {
        x += dir.v.x;
        y += dir.v.y;
        z += dir.v.z;
        return *this;
      }
    };

    Dir3( const Dir3::Packed &packed )
    {
      v.x = packed.x;
      v.y = packed.y;
      v.z = packed.z;
      v.w = 0.0f;
    }

    Dir3( const Dir3::Constant &constant )
    {
      v.x = constant.x;
      v.y = constant.y;
      v.z = constant.z;
      v.w = constant.w;
    }

    Dir3( Vector4Arg vec )
    {
      v.x = vec.v.x;
      v.y = vec.v.y;
      v.z = vec.v.z;
      v.w = 0.0f;
    }

    Dir3( Position3Arg pos )
    {
      v.x = pos.v.x;
      v.y = pos.v.y;
      v.z = pos.v.z;
      v.w = 0.0f;
    }

    Dir3( const float *floatPtr )
    {
      v.x = floatPtr[0];
      v.y = floatPtr[1];
      v.z = floatPtr[2];
      v.w = 0.0f;
    }

    Dir3( const float &_f )
    {
      v.x = _f;
      v.y = _f;
      v.z = _f;
      v.w = 0.0f;
    }

    Dir3( const float _x, const float _y, const float _z )
    {
      v.x = _x;
      v.y = _y;
      v.z = _z;
      v.w = 0.0f;
    }

    Dir3()
    {
      v.x = 0.0f;
      v.y = 0.0f;
      v.z = 0.0f;
      v.w = 0.0f;
    }

    float GetX() const { return v.x; }

    float GetY() const { return v.y; }

    float GetZ() const { return v.z; }

    float GetW() const { return v.w; }

    void SetX( const float _x ) { v.x = _x; }

    void SetY( const float _y ) { v.y = _y; }

    void SetZ( const float _z ) { v.z = _z; }

    void Set( const float _x, const float _y, const float _z )
    {
      v.x = _x;
      v.y = _y;
      v.z = _z;
    }

    Dir3 &operator+=( const float f )
    {
      v.x += f;
      v.y += f;
      v.z += f;
      return *this;
    }

    Dir3 &operator+=( Vector4Arg _v )
    {
      v.x += _v.v.x;
      v.y += _v.v.y;
      v.z += _v.v.z;
      return *this;
    }

    Dir3 &operator+=( Position3Arg _v )
    {
      v.x += _v.v.x;
      v.y += _v.v.y;
      v.z += _v.v.z;
      return *this;
    }

    Dir3 &operator+=( Dir3Arg _v )
    {
      v.x += _v.v.x;
      v.y += _v.v.y;
      v.z += _v.v.z;
      return *this;
    }

    Dir3 &operator-=( const float f )
    {
      v.x -= f;
      v.y -= f;
      v.z -= f;
      return *this;
    }

    Dir3 &Dir3::operator-=( Vector4Arg _v )
    {
      v.x -= _v.v.x;
      v.y -= _v.v.y;
      v.z -= _v.v.z;
      return *this;
    }

    Dir3 &operator-=( Position3Arg _v )
    {
      v.x -= _v.v.x;
      v.y -= _v.v.y;
      v.z -= _v.v.z;
      return *this;
    }

    Dir3 &operator-=( Dir3Arg _v )
    {
      v.x -= _v.v.x;
      v.y -= _v.v.y;
      v.z -= _v.v.z;
      return *this;
    }

    Dir3 &operator*=( Mat43Arg _mat )
    {
      v.x = v.x * _mat.x.v.x + v.y * _mat.y.v.x + v.z * _mat.z.v.x;
      v.y = v.x * _mat.x.v.y + v.y * _mat.y.v.y + v.z * _mat.z.v.y;
      v.z = v.x * _mat.x.v.z + v.y * _mat.y.v.z + v.z * _mat.z.v.z;
      return *this;
    }

    Dir3 &operator*=( TranMat43Arg _mat )
    {
      v.x *= _mat.v.x;
      v.y *= _mat.v.y;
      v.z *= _mat.v.z;
      return *this;
    }

    Dir3 &operator*=( Mat33Arg _mat )
    {
      v.x = v.x * _mat.x.v.x + v.y * _mat.y.v.x + v.z * _mat.z.v.x;
      v.y = v.x * _mat.x.v.y + v.y * _mat.y.v.y + v.z * _mat.z.v.y;
      v.z = v.x * _mat.x.v.z + v.y * _mat.y.v.z + v.z * _mat.z.v.z;
      return *this;
    }

    Dir3 &operator*=( DiagMat33Arg _mat )
    {
      v.x *= _mat.v.x;
      v.y *= _mat.v.y;
      v.z *= _mat.v.z;
      return *this;
    }

    Dir3 &operator*=( const float _f )
    {
      v.x *= _f;
      v.y *= _f;
      v.z *= _f;
      return *this;
    }

    Dir3 &operator/=( Mat43Arg _mat )
    {
      v.x = v.x / _mat.x.v.x + v.y / _mat.y.v.x + v.z / _mat.z.v.x;
      v.y = v.x / _mat.x.v.y + v.y / _mat.y.v.y + v.z / _mat.z.v.y;
      v.z = v.x / _mat.x.v.z + v.y / _mat.y.v.z + v.z / _mat.z.v.z;
      return *this;
    }

    Dir3 &operator/=( TranMat43Arg _mat )
    {
      v.x /= _mat.v.x;
      v.y /= _mat.v.y;
      v.z /= _mat.v.z;
      return *this;
    }

    Dir3 &operator/=( Mat33Arg _mat )
    {
      v.x = v.x / _mat.x.v.x + v.y / _mat.y.v.x + v.z / _mat.z.v.x;
      v.y = v.x / _mat.x.v.y + v.y / _mat.y.v.y + v.z / _mat.z.v.y;
      v.z = v.x / _mat.x.v.z + v.y / _mat.y.v.z + v.z / _mat.z.v.z;
      return *this;
    }

    Dir3 &operator/=( DiagMat33Arg _mat )
    {
      v.x /= _mat.v.x;
      v.y /= _mat.v.y;
      v.z /= _mat.v.z;
      return *this;
    }

    Dir3 &operator/=( const float f )
    {
      float invF = 1.0f / f;
      v.x *= invF;
      v.y *= invF;
      v.z *= invF;

      return *this;
    }

    Dir3 &operator/=( Vector4Arg _v )
    {
      v.x /= _v.v.x;
      v.y /= _v.v.y;
      v.z /= _v.v.z;

      return *this;
    }

    Dir3 &operator/=( Position3Arg _v )
    {
      v.x /= _v.v.x;
      v.y /= _v.v.y;
      v.z /= _v.v.z;

      return *this;
    }

    Dir3 &operator/=( Dir3Arg _v )
    {
      v.x /= _v.v.x;
      v.y /= _v.v.y;
      v.z /= _v.v.z;

      return *this;
    }

    float &operator[]( unsigned int i ) { return *( &v.x + i ); }

    const float &operator[]( unsigned int i ) const { return *( &v.x + i ); }

    Vector4 val34();
  };

  // Out-of-line definitions for Vector4 constructors that take Dir3Arg
  inline Vector4::Vector4( Dir3Arg _d, const float _f )
  {
    v.x = _d.v.x;
    v.y = _d.v.y;
    v.z = _d.v.z;
    v.w = _f;
  }

  inline Vector4::Vector4( Dir3Arg _d )
  {
    v.x = _d.v.x;
    v.y = _d.v.y;
    v.z = _d.v.z;
    v.w = 0.0f;
  }

  inline Dir3 UnitDirX()
  {
    Dir3 tmp;
    tmp.v = Float4_XAxis;
    return tmp;
  }

  inline Dir3 UnitDirY()
  {
    Dir3 tmp;
    tmp.v = Float4_YAxis;
    return tmp;
  }

  inline Dir3 UnitDirZ()
  {
    Dir3 tmp;
    tmp.v = Float4_ZAxis;
    return tmp;
  }

  inline Dir3 Dir3_Zero()
  {
    Dir3 tmp;
    tmp.v = Float4_Zero;
    return tmp;
  }

  typedef Mat43 RotTranMat43;

  class Mat43
  {
  public:
    struct Packed
    {
      Dir3::Packed x;
      Dir3::Packed y;
      Dir3::Packed z;
      Position3::Packed w;

      Dir3::Packed &GetX() { return x; }

      Dir3::Packed &GetY() { return y; }

      Dir3::Packed &GetZ() { return z; }

      Position3::Packed &GetW() { return w; }

      void Set( Mat43Arg mat )
      {
        x.Set( mat.x );
        y.Set( mat.y );
        z.Set( mat.z );
        w.Set( mat.w );
      }
    };

  public:
    Dir3 x;
    Dir3 y;
    Dir3 z;
    Position3 w;

    Mat43( const Mat43::Packed &_p )
    {
      x.Set( _p.x.GetX(), _p.x.GetY(), _p.x.GetZ() );
      y.Set( _p.y.GetX(), _p.y.GetY(), _p.y.GetZ() );
      z.Set( _p.z.GetX(), _p.z.GetY(), _p.z.GetZ() );
      w.Set( _p.w.GetX(), _p.w.GetY(), _p.w.GetZ() );
    }

    Mat43( Mat43Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      w = _mat.w;
    }

    Mat43( Mat44Arg _mat );

    Mat43( DiagMat44Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z );
      w.Set( 0.0f, 0.0f, 0.0f );
    }

    Mat43( TranMat43Arg _mat ) {}

    Mat43( Mat33Arg _mat, Position3Arg _pos )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      w = _pos;
    }

    Mat43( DiagMat33Arg _mat, Position3Arg _pos )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z );
      w = _pos;
    }

    Mat43( Mat33Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      w.Set( 0.0f, 0.0f, 0.0f );
    }

    Mat43( DiagMat33Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z );
      w.Set( 0.0f, 0.0f, 0.0f );
    }

    Mat43( Dir3Arg _x, Dir3Arg _y, Dir3Arg _z, Position3Arg _pos )
    {
      x = _x;
      y = _y;
      z = _z;
      w = _pos;
    }

    Mat43()
    {
      x.Set( 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f );
    }

    Mat43 &operator=( Mat43Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      w = _mat.w;
      return *this;
    }

    Mat43 &operator=( Mat44Arg _mat );

    Mat43 &operator=( DiagMat44Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z );
      w.Set( 0.0f, 0.0f, 0.0f );
      return *this;
    }

    Mat43 &operator=( TranMat43Arg _mat );

    Mat43 &operator=( Mat33Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      w.Set( 0.0f, 0.0f, 0.0f );
      return *this;
    }

    Mat43 &operator=( DiagMat33Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z );
      w.Set( 0.0f, 0.0f, 0.0f );
      return *this;
    }

    Dir3 &GetX() { return x; }

    const Dir3 &GetX() const { return x; }

    Dir3 &GetY() { return y; }

    const Dir3 &GetY() const { return y; }

    Dir3 &GetZ() { return z; }

    const Dir3 &GetZ() const { return z; }

    Position3 &GetW() { return w; }

    const Position3 &GetW() const { return w; }

    void SetX( Dir3Arg _x ) { x = _x; }

    void SetY( Dir3Arg _y ) { y = _y; }

    void SetZ( Dir3Arg _z ) { z = _z; }

    void SetW( Position3Arg _w ) { w = _w; }

    void SetM33( Mat44Arg _mat );

    void SetM33( DiagMat44Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z );
    }

    void SetM33( Mat43Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
    }

    void SetM33( Mat33Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
    }

    void SetM33( DiagMat33Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z );
    }

    Mat43 &operator*=( Mat43Arg _mat )
    {
      // Create temporary variables to store the original matrix values
      Dir3 origX = x;
      Dir3 origY = y;
      Dir3 origZ = z;
      Position3 origW = w;

      // Calculate new x, y and z columns
      x.v.x = origX.v.x * _mat.x.v.x + origY.v.x * _mat.y.v.x + origZ.v.x * _mat.z.v.x;
      x.v.y = origX.v.y * _mat.x.v.x + origY.v.y * _mat.y.v.x + origZ.v.y * _mat.z.v.x;
      x.v.z = origX.v.z * _mat.x.v.x + origY.v.z * _mat.y.v.x + origZ.v.z * _mat.z.v.x;

      y.v.x = origX.v.x * _mat.x.v.y + origY.v.x * _mat.y.v.y + origZ.v.x * _mat.z.v.y;
      y.v.y = origX.v.y * _mat.x.v.y + origY.v.y * _mat.y.v.y + origZ.v.y * _mat.z.v.y;
      y.v.z = origX.v.z * _mat.x.v.y + origY.v.z * _mat.y.v.y + origZ.v.z * _mat.z.v.y;

      z.v.x = origX.v.x * _mat.x.v.z + origY.v.x * _mat.y.v.z + origZ.v.x * _mat.z.v.z;
      z.v.y = origX.v.y * _mat.x.v.z + origY.v.y * _mat.y.v.z + origZ.v.y * _mat.z.v.z;
      z.v.z = origX.v.z * _mat.x.v.z + origY.v.z * _mat.y.v.z + origZ.v.z * _mat.z.v.z;

      // Calculate new translation component
      w.v.x = origX.v.x * _mat.w.v.x + origY.v.x * _mat.w.v.y + origZ.v.x * _mat.w.v.z + origW.v.x;
      w.v.y = origX.v.y * _mat.w.v.x + origY.v.y * _mat.w.v.y + origZ.v.y * _mat.w.v.z + origW.v.y;
      w.v.z = origX.v.z * _mat.w.v.x + origY.v.z * _mat.w.v.y + origZ.v.z * _mat.w.v.z + origW.v.z;

      return *this;
    }

    Mat43 &operator*=( TranMat43Arg _mat )
    {
      // Scale the rotation part
      x.v.x *= _mat.v.x;
      x.v.y *= _mat.v.x;
      x.v.z *= _mat.v.x;

      y.v.x *= _mat.v.y;
      y.v.y *= _mat.v.y;
      y.v.z *= _mat.v.y;

      z.v.x *= _mat.v.z;
      z.v.y *= _mat.v.z;
      z.v.z *= _mat.v.z;

      return *this;
    }

    Mat43 &operator*=( Mat33Arg _mat )
    {
      // Create temporary variables to store the original matrix values
      Dir3 origX = x;
      Dir3 origY = y;
      Dir3 origZ = z;

      // Calculate new x, y and z columns by multiplying the 3x3 part
      x.v.x = origX.v.x * _mat.x.v.x + origY.v.x * _mat.y.v.x + origZ.v.x * _mat.z.v.x;
      x.v.y = origX.v.y * _mat.x.v.x + origY.v.y * _mat.y.v.x + origZ.v.y * _mat.z.v.x;
      x.v.z = origX.v.z * _mat.x.v.x + origY.v.z * _mat.y.v.x + origZ.v.z * _mat.z.v.x;

      y.v.x = origX.v.x * _mat.x.v.y + origY.v.x * _mat.y.v.y + origZ.v.x * _mat.z.v.y;
      y.v.y = origX.v.y * _mat.x.v.y + origY.v.y * _mat.y.v.y + origZ.v.y * _mat.z.v.y;
      y.v.z = origX.v.z * _mat.x.v.y + origY.v.z * _mat.y.v.y + origZ.v.z * _mat.z.v.y;

      z.v.x = origX.v.x * _mat.x.v.z + origY.v.x * _mat.y.v.z + origZ.v.x * _mat.z.v.z;
      z.v.y = origX.v.y * _mat.x.v.z + origY.v.y * _mat.y.v.z + origZ.v.y * _mat.z.v.z;
      z.v.z = origX.v.z * _mat.x.v.z + origY.v.z * _mat.y.v.z + origZ.v.z * _mat.z.v.z;

      // Translation component remains unchanged

      return *this;
    }

    Mat43 &operator*=( DiagMat33Arg _mat )
    {
      // Multiply by diagonal matrix (simple scaling)
      x.v.x *= _mat.v.x;
      x.v.y *= _mat.v.x;
      x.v.z *= _mat.v.x;

      y.v.x *= _mat.v.y;
      y.v.y *= _mat.v.y;
      y.v.z *= _mat.v.y;

      z.v.x *= _mat.v.z;
      z.v.y *= _mat.v.z;
      z.v.z *= _mat.v.z;

      return *this;
    }
  };

  class Mat44
  {
  public:
    struct Packed
    {
      Vector4::Packed x;
      Vector4::Packed y;
      Vector4::Packed z;
      Vector4::Packed w;

      Vector4::Packed &GetX() { return x; }

      Vector4::Packed &GetY() { return y; }

      Vector4::Packed &GetZ() { return z; }

      Vector4::Packed &GetW() { return w; }

      void Set( Mat44Arg _mat )
      {
        x.Set( _mat.x );
        y.Set( _mat.y );
        z.Set( _mat.z );
        w.Set( _mat.w );
      }

      Packed()
      {
        x.Set( 0.0f, 0.0f, 0.0f, 0.0f );
        y.Set( 0.0f, 0.0f, 0.0f, 0.0f );
        z.Set( 0.0f, 0.0f, 0.0f, 0.0f );
        w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
      }
    };

    Vector4 x;
    Vector4 y;
    Vector4 z;
    Vector4 w;

  public:
    Mat44( Mat44Arg _p )
    {
      x = _p.x;
      y = _p.y;
      z = _p.z;
      w = _p.w;
    }

    // Constructors
    Mat44( DiagMat44Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, _mat.v.w );
    }

    Mat44( Mat34Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
    }

    Mat44( Mat43Arg _mat )
    {
      x.Set( _mat.x.v.x, _mat.x.v.y, _mat.x.v.z, 0.0f );
      y.Set( _mat.y.v.x, _mat.y.v.y, _mat.y.v.z, 0.0f );
      z.Set( _mat.z.v.x, _mat.z.v.y, _mat.z.v.z, 0.0f );
      w.Set( _mat.w.v.x, _mat.w.v.y, _mat.w.v.z, 1.0f );
    }

    Mat44( TranMat43Arg _mat )
    {
      // Assuming TranMat43 is a transposed/scaling matrix
      x.Set( _mat.v.x, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
    }

    Mat44( Mat33Arg _mat )
    {
      x.Set( _mat.x.v.x, _mat.x.v.y, _mat.x.v.z, 0.0f );
      y.Set( _mat.y.v.x, _mat.y.v.y, _mat.y.v.z, 0.0f );
      z.Set( _mat.z.v.x, _mat.z.v.y, _mat.z.v.z, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
    }

    Mat44( DiagMat33Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
    }

    Mat44( Dir3Arg _x, Dir3Arg _y, Dir3Arg _z, Position3Arg _w )
    {
      x = _x;
      y = _y;
      z = _z;
      w = _w;
    }

    Mat44( Vector4Arg _x, Vector4Arg _y, Vector4Arg _z, Vector4Arg _w )
    {
      x = _x;
      y = _y;
      z = _z;
      w = _w;
    }

    Mat44()
    {
      x.Set( 1.0f, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, 1.0f, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, 1.0f, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
    }

    // Assignment operators
    Mat44 &operator=( Mat44Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      w = _mat.w;
      return *this;
    }

    Mat44 &operator=( DiagMat44Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, _mat.v.w );
      return *this;
    }

    Mat44 &operator=( Mat34Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
      return *this;
    }

    Mat44 &operator=( Mat43Arg _mat )
    {
      x.Set( _mat.x.v.x, _mat.x.v.y, _mat.x.v.z, 0.0f );
      y.Set( _mat.y.v.x, _mat.y.v.y, _mat.y.v.z, 0.0f );
      z.Set( _mat.z.v.x, _mat.z.v.y, _mat.z.v.z, 0.0f );
      w.Set( _mat.w.v.x, _mat.w.v.y, _mat.w.v.z, 1.0f );
      return *this;
    }

    Mat44 &operator=( TranMat43Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
      return *this;
    }

    Mat44 &operator=( Mat33Arg _mat )
    {
      x.Set( _mat.x.v.x, _mat.x.v.y, _mat.x.v.z, 0.0f );
      y.Set( _mat.y.v.x, _mat.y.v.y, _mat.y.v.z, 0.0f );
      z.Set( _mat.z.v.x, _mat.z.v.y, _mat.z.v.z, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
      return *this;
    }

    Mat44 &operator=( DiagMat33Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z, 0.0f );
      w.Set( 0.0f, 0.0f, 0.0f, 1.0f );
      return *this;
    }

    // Getters
    Vector4 &GetX() { return x; }

    const Vector4 &GetX() const { return x; }

    Vector4 &GetY() { return y; }

    const Vector4 &GetY() const { return y; }

    Vector4 &GetZ() { return z; }

    const Vector4 &GetZ() const { return z; }

    Vector4 &GetW() { return w; }

    const Vector4 &GetW() const { return w; }

    // Setters
    void SetX( Vector4Arg _x ) { x = _x; }

    void SetY( Vector4Arg _y ) { y = _y; }

    void SetZ( Vector4Arg _z ) { z = _z; }

    void SetW( Vector4Arg _w ) { w = _w; }

    // Set 3x3 matrix part
    void SetM33( Mat44Arg _mat )
    {
      x.Set( _mat.x.v.x, _mat.x.v.y, _mat.x.v.z, x.v.w );
      y.Set( _mat.y.v.x, _mat.y.v.y, _mat.y.v.z, y.v.w );
      z.Set( _mat.z.v.x, _mat.z.v.y, _mat.z.v.z, z.v.w );
    }

    void SetM33( DiagMat44Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, x.v.w );
      y.Set( 0.0f, _mat.v.y, 0.0f, y.v.w );
      z.Set( 0.0f, 0.0f, _mat.v.z, z.v.w );
    }

    void SetM33( Mat43Arg _mat )
    {
      x.Set( _mat.x.v.x, _mat.x.v.y, _mat.x.v.z, x.v.w );
      y.Set( _mat.y.v.x, _mat.y.v.y, _mat.y.v.z, y.v.w );
      z.Set( _mat.z.v.x, _mat.z.v.y, _mat.z.v.z, z.v.w );
    }

    void SetM33( Mat33Arg _mat )
    {
      x.Set( _mat.x.v.x, _mat.x.v.y, _mat.x.v.z, x.v.w );
      y.Set( _mat.y.v.x, _mat.y.v.y, _mat.y.v.z, y.v.w );
      z.Set( _mat.z.v.x, _mat.z.v.y, _mat.z.v.z, z.v.w );
    }

    void SetM33( DiagMat33Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, x.v.w );
      y.Set( 0.0f, _mat.v.y, 0.0f, y.v.w );
      z.Set( 0.0f, 0.0f, _mat.v.z, z.v.w );
    }

    // Matrix multiplication operators
    Mat44 &operator*=( Mat44Arg _mat )
    {
      // Store original values before modifying
      Vector4 origX = x;
      Vector4 origY = y;
      Vector4 origZ = z;
      Vector4 origW = w;

      // Calculate each component of the result
      x.v.x = origX.v.x * _mat.x.v.x + origY.v.x * _mat.x.v.y + origZ.v.x * _mat.x.v.z + origW.v.x * _mat.x.v.w;
      x.v.y = origX.v.y * _mat.x.v.x + origY.v.y * _mat.x.v.y + origZ.v.y * _mat.x.v.z + origW.v.y * _mat.x.v.w;
      x.v.z = origX.v.z * _mat.x.v.x + origY.v.z * _mat.x.v.y + origZ.v.z * _mat.x.v.z + origW.v.z * _mat.x.v.w;
      x.v.w = origX.v.w * _mat.x.v.x + origY.v.w * _mat.x.v.y + origZ.v.w * _mat.x.v.z + origW.v.w * _mat.x.v.w;

      y.v.x = origX.v.x * _mat.y.v.x + origY.v.x * _mat.y.v.y + origZ.v.x * _mat.y.v.z + origW.v.x * _mat.y.v.w;
      y.v.y = origX.v.y * _mat.y.v.x + origY.v.y * _mat.y.v.y + origZ.v.y * _mat.y.v.z + origW.v.y * _mat.y.v.w;
      y.v.z = origX.v.z * _mat.y.v.x + origY.v.z * _mat.y.v.y + origZ.v.z * _mat.y.v.z + origW.v.z * _mat.y.v.w;
      y.v.w = origX.v.w * _mat.y.v.x + origY.v.w * _mat.y.v.y + origZ.v.w * _mat.y.v.z + origW.v.w * _mat.y.v.w;

      z.v.x = origX.v.x * _mat.z.v.x + origY.v.x * _mat.z.v.y + origZ.v.x * _mat.z.v.z + origW.v.x * _mat.z.v.w;
      z.v.y = origX.v.y * _mat.z.v.x + origY.v.y * _mat.z.v.y + origZ.v.y * _mat.z.v.z + origW.v.y * _mat.z.v.w;
      z.v.z = origX.v.z * _mat.z.v.x + origY.v.z * _mat.z.v.y + origZ.v.z * _mat.z.v.z + origW.v.z * _mat.z.v.w;
      z.v.w = origX.v.w * _mat.z.v.x + origY.v.w * _mat.z.v.y + origZ.v.w * _mat.z.v.z + origW.v.w * _mat.z.v.w;

      w.v.x = origX.v.x * _mat.w.v.x + origY.v.x * _mat.w.v.y + origZ.v.x * _mat.w.v.z + origW.v.x * _mat.w.v.w;
      w.v.y = origX.v.y * _mat.w.v.x + origY.v.y * _mat.w.v.y + origZ.v.y * _mat.w.v.z + origW.v.y * _mat.w.v.w;
      w.v.z = origX.v.z * _mat.w.v.x + origY.v.z * _mat.w.v.y + origZ.v.z * _mat.w.v.z + origW.v.z * _mat.w.v.w;
      w.v.w = origX.v.w * _mat.w.v.x + origY.v.w * _mat.w.v.y + origZ.v.w * _mat.w.v.z + origW.v.w * _mat.w.v.w;

      return *this;
    }

    Mat44 &operator*=( DiagMat44Arg _mat )
    {
      // For diagonal matrix, just scale each column
      x.v.x *= _mat.v.x;
      x.v.y *= _mat.v.x;
      x.v.z *= _mat.v.x;
      x.v.w *= _mat.v.x;

      y.v.x *= _mat.v.y;
      y.v.y *= _mat.v.y;
      y.v.z *= _mat.v.y;
      y.v.w *= _mat.v.y;

      z.v.x *= _mat.v.z;
      z.v.y *= _mat.v.z;
      z.v.z *= _mat.v.z;
      z.v.w *= _mat.v.z;

      w.v.x *= _mat.v.w;
      w.v.y *= _mat.v.w;
      w.v.z *= _mat.v.w;
      w.v.w *= _mat.v.w;

      return *this;
    }

    Mat44 &operator*=( Mat43Arg _mat )
    {
      // Store original values
      Vector4 origX = x;
      Vector4 origY = y;
      Vector4 origZ = z;
      Vector4 origW = w;

      // Multiply the 3x3 part
      x.v.x = origX.v.x * _mat.x.v.x + origY.v.x * _mat.y.v.x + origZ.v.x * _mat.z.v.x;
      x.v.y = origX.v.y * _mat.x.v.x + origY.v.y * _mat.y.v.x + origZ.v.y * _mat.z.v.x;
      x.v.z = origX.v.z * _mat.x.v.x + origY.v.z * _mat.y.v.x + origZ.v.z * _mat.z.v.x;
      x.v.w = origX.v.w * _mat.x.v.x + origY.v.w * _mat.y.v.x + origZ.v.w * _mat.z.v.x;

      y.v.x = origX.v.x * _mat.x.v.y + origY.v.x * _mat.y.v.y + origZ.v.x * _mat.z.v.y;
      y.v.y = origX.v.y * _mat.x.v.y + origY.v.y * _mat.y.v.y + origZ.v.y * _mat.z.v.y;
      y.v.z = origX.v.z * _mat.x.v.y + origY.v.z * _mat.y.v.y + origZ.v.z * _mat.z.v.y;
      y.v.w = origX.v.w * _mat.x.v.y + origY.v.w * _mat.y.v.y + origZ.v.w * _mat.z.v.y;

      z.v.x = origX.v.x * _mat.x.v.z + origY.v.x * _mat.y.v.z + origZ.v.x * _mat.z.v.z;
      z.v.y = origX.v.y * _mat.x.v.z + origY.v.y * _mat.y.v.z + origZ.v.y * _mat.z.v.z;
      z.v.z = origX.v.z * _mat.x.v.z + origY.v.z * _mat.y.v.z + origZ.v.z * _mat.z.v.z;
      z.v.w = origX.v.w * _mat.x.v.z + origY.v.w * _mat.y.v.z + origZ.v.w * _mat.z.v.z;

      // Incorporate translation
      w.v.x = origX.v.x * _mat.w.v.x + origY.v.x * _mat.w.v.y + origZ.v.x * _mat.w.v.z + origW.v.x;
      w.v.y = origX.v.y * _mat.w.v.x + origY.v.y * _mat.w.v.y + origZ.v.y * _mat.w.v.z + origW.v.y;
      w.v.z = origX.v.z * _mat.w.v.x + origY.v.z * _mat.w.v.y + origZ.v.z * _mat.w.v.z + origW.v.z;
      w.v.w = origX.v.w * _mat.w.v.x + origY.v.w * _mat.w.v.y + origZ.v.w * _mat.w.v.z + origW.v.w;

      return *this;
    }

    Mat44 &operator*=( TranMat43Arg _mat )
    {
      // Scale the matrix by diagonal elements
      x.v.x *= _mat.v.x;
      x.v.y *= _mat.v.x;
      x.v.z *= _mat.v.x;
      x.v.w *= _mat.v.x;

      y.v.x *= _mat.v.y;
      y.v.y *= _mat.v.y;
      y.v.z *= _mat.v.y;
      y.v.w *= _mat.v.y;

      z.v.x *= _mat.v.z;
      z.v.y *= _mat.v.z;
      z.v.z *= _mat.v.z;
      z.v.w *= _mat.v.z;

      return *this;
    }

    Mat44 &operator*=( Mat33Arg _mat )
    {
      // Store original values
      Vector4 origX = x;
      Vector4 origY = y;
      Vector4 origZ = z;

      // Only multiply the 3x3 part
      x.v.x = origX.v.x * _mat.x.v.x + origY.v.x * _mat.y.v.x + origZ.v.x * _mat.z.v.x;
      x.v.y = origX.v.y * _mat.x.v.x + origY.v.y * _mat.y.v.x + origZ.v.y * _mat.z.v.x;
      x.v.z = origX.v.z * _mat.x.v.x + origY.v.z * _mat.y.v.x + origZ.v.z * _mat.z.v.x;
      x.v.w = origX.v.w * _mat.x.v.x + origY.v.w * _mat.y.v.x + origZ.v.w * _mat.z.v.x;

      y.v.x = origX.v.x * _mat.x.v.y + origY.v.x * _mat.y.v.y + origZ.v.x * _mat.z.v.y;
      y.v.y = origX.v.y * _mat.x.v.y + origY.v.y * _mat.y.v.y + origZ.v.y * _mat.z.v.y;
      y.v.z = origX.v.z * _mat.x.v.y + origY.v.z * _mat.y.v.y + origZ.v.z * _mat.z.v.y;
      y.v.w = origX.v.w * _mat.x.v.y + origY.v.w * _mat.y.v.y + origZ.v.w * _mat.z.v.y;

      z.v.x = origX.v.x * _mat.x.v.z + origY.v.x * _mat.y.v.z + origZ.v.x * _mat.z.v.z;
      z.v.y = origX.v.y * _mat.x.v.z + origY.v.y * _mat.y.v.z + origZ.v.y * _mat.z.v.z;
      z.v.z = origX.v.z * _mat.x.v.z + origY.v.z * _mat.y.v.z + origZ.v.z * _mat.z.v.z;
      z.v.w = origX.v.w * _mat.x.v.z + origY.v.w * _mat.y.v.z + origZ.v.w * _mat.z.v.z;

      return *this;
    }

    Mat44 &operator*=( DiagMat33Arg _mat )
    {
      // Scale only the 3x3 part
      x.v.x *= _mat.v.x;
      x.v.y *= _mat.v.x;
      x.v.z *= _mat.v.x;
      x.v.w *= _mat.v.x;

      y.v.x *= _mat.v.y;
      y.v.y *= _mat.v.y;
      y.v.z *= _mat.v.y;
      y.v.w *= _mat.v.y;

      z.v.x *= _mat.v.z;
      z.v.y *= _mat.v.z;
      z.v.z *= _mat.v.z;
      z.v.w *= _mat.v.z;

      return *this;
    }

    Vector4 &operator[]( unsigned int i ) { return ( (Vector4 *)&x )[i]; }

    const Vector4 &operator[]( unsigned int i ) const { return ( (const Vector4 *)&x )[i]; }
  };

  // Out-of-line definitions for methods using Mat44Arg

  // Vector4 operators
  inline Vector4 &Vector4::operator*=( Mat44Arg _mat )
  {
    v.x = v.x * _mat.x.v.x + v.y * _mat.y.v.x + v.z * _mat.z.v.x + v.w * _mat.w.v.x;
    v.y = v.x * _mat.x.v.y + v.y * _mat.y.v.y + v.z * _mat.z.v.y + v.w * _mat.w.v.y;
    v.z = v.x * _mat.x.v.z + v.y * _mat.y.v.z + v.z * _mat.z.v.z + v.w * _mat.w.v.z;
    v.w = v.x * _mat.x.v.w + v.y * _mat.y.v.w + v.z * _mat.z.v.w + v.w * _mat.w.v.w;
    return *this;
  }

  inline Vector4 &Vector4::operator/=( Mat44Arg _mat )
  {
    v.x = v.x / _mat.x.v.x + v.y / _mat.y.v.x + v.z / _mat.z.v.x + v.w / _mat.w.v.x;
    v.y = v.x / _mat.x.v.y + v.y / _mat.y.v.y + v.z / _mat.z.v.y + v.w / _mat.w.v.y;
    v.z = v.x / _mat.x.v.z + v.y / _mat.y.v.z + v.z / _mat.z.v.z + v.w / _mat.w.v.z;
    v.w = v.x / _mat.x.v.w + v.y / _mat.y.v.w + v.z / _mat.z.v.w + v.w / _mat.w.v.w;
    return *this;
  }

  // Mat43 constructors and operators
  inline Mat43::Mat43( Mat44Arg _mat )
  {
    x = _mat.x;
    y = _mat.y;
    z = _mat.z;
    w.Set( _mat.w.GetX(), _mat.w.GetY(), _mat.w.GetZ() );
  }

  inline Mat43 &Mat43::operator=( Mat44Arg _mat )
  {
    x = _mat.x;
    y = _mat.y;
    z = _mat.z;
    w.Set( _mat.w.GetX(), _mat.w.GetY(), _mat.w.GetZ() );
    return *this;
  }

  inline void Mat43::SetM33( Mat44Arg _mat )
  {
    x = _mat.x;
    y = _mat.y;
    z = _mat.z;
  }

  class DiagMat44
  {
  public:
    struct Packed
    {
      float x;
      float y;
      float z;
      float w;

      void Set( DiagMat44Arg _mat )
      {
        x = _mat.v.x;
        y = _mat.v.y;
        z = _mat.v.z;
        w = _mat.v.w;
      }

      void Set( float _x, float _y, float _z, float _w )
      {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
      }

      float GetX() { return x; }

      float GetY() { return y; }

      float GetZ() { return z; }

      float GetW() { return w; }
    };

    Float4 v;

  public:
    DiagMat44( const DiagMat44::Packed &_p )
    {
      v.x = _p.x;
      v.y = _p.y;
      v.z = _p.z;
      v.w = _p.w;
    }

    DiagMat44( DiagMat44Arg _mat )
    {
      v.x = _mat.v.x;
      v.y = _mat.v.y;
      v.z = _mat.v.z;
      v.w = _mat.v.w;
    }

    DiagMat44( DiagMat33Arg _mat )
    {
      v.x = _mat.v.x;
      v.y = _mat.v.y;
      v.z = _mat.v.z;
      v.w = 1.0f;
    }

    DiagMat44( Vector4Arg _v )
    {
      v.x = _v.v.x;
      v.y = _v.v.y;
      v.z = _v.v.z;
      v.w = _v.v.w;
    }

    DiagMat44( const float _x, const float _y, const float _z, const float _w )
    {
      v.x = _x;
      v.y = _y;
      v.z = _z;
      v.w = _w;
    }

    DiagMat44()
    {
      v.x = 1.0f;
      v.y = 1.0f;
      v.z = 1.0f;
      v.w = 1.0f;
    }

    DiagMat44 &operator=( DiagMat44Arg );
    DiagMat44 &operator=( DiagMat33Arg );
    Vector4 GetX();
    Vector4 GetY();
    Vector4 GetZ();
    Vector4 GetW();
    void SetM33( DiagMat44Arg );
    void SetM33( DiagMat33Arg );
    DiagMat44 &operator*=( DiagMat44Arg );
    DiagMat44 &operator*=( DiagMat33Arg );
  };

  // Out-of-line definitions for methods using Mat44Arg

  // Vector4 operators
  inline Vector4 &Vector4::operator*=( DiagMat44Arg _mat )
  {
    v.x *= _mat.v.x;
    v.y *= _mat.v.y;
    v.z *= _mat.v.z;
    v.w *= _mat.v.w;
    return *this;
  }

  inline Vector4 &Vector4::operator/=( DiagMat44Arg _mat )
  {
    v.x /= _mat.v.x;
    v.y /= _mat.v.y;
    v.z /= _mat.v.z;
    v.w /= _mat.v.w;
    return *this;
  }

  // Mat43 constructors and operators
  inline Mat43::Mat43( DiagMat44Arg _mat )
  {
    x.Set( _mat.v.x, 0.0f, 0.0f );
    y.Set( 0.0f, _mat.v.y, 0.0f );
    z.Set( 0.0f, 0.0f, _mat.v.z );
    w.Set( 0.0f, 0.0f, 0.0f );
  }

  inline Mat43 &Mat43::operator=( DiagMat44Arg _mat )
  {
    x.Set( _mat.v.x, 0.0f, 0.0f );
    y.Set( 0.0f, _mat.v.y, 0.0f );
    z.Set( 0.0f, 0.0f, _mat.v.z );
    w.Set( 0.0f, 0.0f, 0.0f );
    return *this;
  }

  inline void Mat43::SetM33( DiagMat44Arg _mat )
  {
    x.Set( _mat.v.x, 0.0f, 0.0f );
    y.Set( 0.0f, _mat.v.y, 0.0f );
    z.Set( 0.0f, 0.0f, _mat.v.z );
  }

  class Mat33
  {
  public:
    Dir3 x;
    Dir3 y;
    Dir3 z;

  public:
    // Constructors
    inline Mat33( Mat33Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
    }

    inline Mat33( QuaternionArg _q )
    {
      // Convert quaternion to rotation matrix
      float xx = _q.v.x * _q.v.x;
      float xy = _q.v.x * _q.v.y;
      float xz = _q.v.x * _q.v.z;
      float xw = _q.v.x * _q.v.w;
      float yy = _q.v.y * _q.v.y;
      float yz = _q.v.y * _q.v.z;
      float yw = _q.v.y * _q.v.w;
      float zz = _q.v.z * _q.v.z;
      float zw = _q.v.z * _q.v.w;

      x.v.x = 1.0f - 2.0f * ( yy + zz );
      x.v.y = 2.0f * ( xy + zw );
      x.v.z = 2.0f * ( xz - yw );

      y.v.x = 2.0f * ( xy - zw );
      y.v.y = 1.0f - 2.0f * ( xx + zz );
      y.v.z = 2.0f * ( yz + xw );

      z.v.x = 2.0f * ( xz + yw );
      z.v.y = 2.0f * ( yz - xw );
      z.v.z = 1.0f - 2.0f * ( xx + yy );
    }

    inline Mat33( Mat44Arg _mat )
    {
      x.v.x = _mat.x.v.x;
      x.v.y = _mat.x.v.y;
      x.v.z = _mat.x.v.z;

      y.v.x = _mat.y.v.x;
      y.v.y = _mat.y.v.y;
      y.v.z = _mat.y.v.z;

      z.v.x = _mat.z.v.x;
      z.v.y = _mat.z.v.y;
      z.v.z = _mat.z.v.z;
    }

    inline Mat33( DiagMat44Arg _mat )
    {
      x.v.x = _mat.v.x;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = _mat.v.y;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = _mat.v.z;
    }

    inline Mat33( Mat43Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
    }

    inline Mat33( TranMat43Arg _mat )
    {
      // Diagonal matrix
      x.v.x = _mat.v.x;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = _mat.v.y;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = _mat.v.z;
    }

    inline Mat33( DiagMat33Arg _mat )
    {
      x.v.x = _mat.v.x;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = _mat.v.y;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = _mat.v.z;
    }

    inline Mat33( Dir3Arg _x, Dir3Arg _y, Dir3Arg _z )
    {
      x = _x;
      y = _y;
      z = _z;
    }

    inline Mat33()
    {
      x.v.x = 1.0f;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = 1.0f;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = 1.0f;
    }

    // Assignment operators
    inline Mat33 &operator=( Mat33Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      return *this;
    }

    inline Mat33 &operator=( QuaternionArg _q )
    {
      // Convert quaternion to rotation matrix
      float xx = _q.v.x * _q.v.x;
      float xy = _q.v.x * _q.v.y;
      float xz = _q.v.x * _q.v.z;
      float xw = _q.v.x * _q.v.w;
      float yy = _q.v.y * _q.v.y;
      float yz = _q.v.y * _q.v.z;
      float yw = _q.v.y * _q.v.w;
      float zz = _q.v.z * _q.v.z;
      float zw = _q.v.z * _q.v.w;

      x.v.x = 1.0f - 2.0f * ( yy + zz );
      x.v.y = 2.0f * ( xy + zw );
      x.v.z = 2.0f * ( xz - yw );

      y.v.x = 2.0f * ( xy - zw );
      y.v.y = 1.0f - 2.0f * ( xx + zz );
      y.v.z = 2.0f * ( yz + xw );

      z.v.x = 2.0f * ( xz + yw );
      z.v.y = 2.0f * ( yz - xw );
      z.v.z = 1.0f - 2.0f * ( xx + yy );

      return *this;
    }

    inline Mat33 &operator=( Mat44Arg _mat )
    {
      x.v.x = _mat.x.v.x;
      x.v.y = _mat.x.v.y;
      x.v.z = _mat.x.v.z;

      y.v.x = _mat.y.v.x;
      y.v.y = _mat.y.v.y;
      y.v.z = _mat.y.v.z;

      z.v.x = _mat.z.v.x;
      z.v.y = _mat.z.v.y;
      z.v.z = _mat.z.v.z;

      return *this;
    }

    inline Mat33 &operator=( DiagMat44Arg _mat )
    {
      x.v.x = _mat.v.x;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = _mat.v.y;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = _mat.v.z;

      return *this;
    }

    inline Mat33 &operator=( Mat43Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      return *this;
    }

    inline Mat33 &operator=( TranMat43Arg _mat )
    {
      // Diagonal matrix
      x.v.x = _mat.v.x;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = _mat.v.y;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = _mat.v.z;

      return *this;
    }

    inline Mat33 &operator=( DiagMat33Arg _mat )
    {
      x.v.x = _mat.v.x;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = _mat.v.y;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = _mat.v.z;

      return *this;
    }

    // Getters
    inline const Dir3 &GetX() const { return x; }

    inline Dir3 &GetX() { return x; }

    inline const Dir3 &GetY() const { return y; }

    inline Dir3 &GetY() { return y; }

    inline const Dir3 &GetZ() const { return z; }

    inline Dir3 &GetZ() { return z; }

    // Setters
    inline void SetX( Dir3Arg _x ) { x = _x; }

    inline void SetY( Dir3Arg _y ) { y = _y; }

    inline void SetZ( Dir3Arg _z ) { z = _z; }

    // Set 3x3 matrix part
    inline void SetM33( Mat44Arg _mat )
    {
      x.v.x = _mat.x.v.x;
      x.v.y = _mat.x.v.y;
      x.v.z = _mat.x.v.z;

      y.v.x = _mat.y.v.x;
      y.v.y = _mat.y.v.y;
      y.v.z = _mat.y.v.z;

      z.v.x = _mat.z.v.x;
      z.v.y = _mat.z.v.y;
      z.v.z = _mat.z.v.z;
    }

    inline void SetM33( DiagMat44Arg _mat )
    {
      x.v.x = _mat.v.x;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = _mat.v.y;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = _mat.v.z;
    }

    inline void SetM33( Mat43Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
    }

    inline void SetM33( Mat33Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
    }

    inline void SetM33( DiagMat33Arg _mat )
    {
      x.v.x = _mat.v.x;
      x.v.y = 0.0f;
      x.v.z = 0.0f;

      y.v.x = 0.0f;
      y.v.y = _mat.v.y;
      y.v.z = 0.0f;

      z.v.x = 0.0f;
      z.v.y = 0.0f;
      z.v.z = _mat.v.z;
    }

    // Matrix multiplication operators
    inline Mat33 &operator*=( Mat33Arg _mat )
    {
      // Store original values
      Dir3 origX = x;
      Dir3 origY = y;
      Dir3 origZ = z;

      // Multiply matrices
      x.v.x = origX.v.x * _mat.x.v.x + origY.v.x * _mat.y.v.x + origZ.v.x * _mat.z.v.x;
      x.v.y = origX.v.y * _mat.x.v.x + origY.v.y * _mat.y.v.x + origZ.v.y * _mat.z.v.x;
      x.v.z = origX.v.z * _mat.x.v.x + origY.v.z * _mat.y.v.x + origZ.v.z * _mat.z.v.x;

      y.v.x = origX.v.x * _mat.x.v.y + origY.v.x * _mat.y.v.y + origZ.v.x * _mat.z.v.y;
      y.v.y = origX.v.y * _mat.x.v.y + origY.v.y * _mat.y.v.y + origZ.v.y * _mat.z.v.y;
      y.v.z = origX.v.z * _mat.x.v.y + origY.v.z * _mat.y.v.y + origZ.v.z * _mat.z.v.y;

      z.v.x = origX.v.x * _mat.x.v.z + origY.v.x * _mat.y.v.z + origZ.v.x * _mat.z.v.z;
      z.v.y = origX.v.y * _mat.x.v.z + origY.v.y * _mat.y.v.z + origZ.v.y * _mat.z.v.z;
      z.v.z = origX.v.z * _mat.x.v.z + origY.v.z * _mat.y.v.z + origZ.v.z * _mat.z.v.z;

      return *this;
    }

    inline Mat33 &operator*=( DiagMat33Arg _mat )
    {
      // For diagonal matrix, just scale each column
      x.v.x *= _mat.v.x;
      x.v.y *= _mat.v.x;
      x.v.z *= _mat.v.x;

      y.v.x *= _mat.v.y;
      y.v.y *= _mat.v.y;
      y.v.z *= _mat.v.y;

      z.v.x *= _mat.v.z;
      z.v.y *= _mat.v.z;
      z.v.z *= _mat.v.z;

      return *this;
    }
  };

  class DiagMat33
  {
  public:
    struct Packed
    {
      float x;
      float y;
      float z;

      // Packed methods
      void Set( DiagMat33Arg _mat )
      {
        x = _mat.v.x;
        y = _mat.v.y;
        z = _mat.v.z;
      }

      void Set( const float _x, const float _y, const float _z )
      {
        x = _x;
        y = _y;
        z = _z;
      }

      float GetX() { return x; }

      float GetY() { return y; }

      float GetZ() { return z; }
    };

    Float4 v;

  public:
    // DiagMat33 constructor implementations
    DiagMat33( const Packed &_p )
    {
      v.x = _p.x;
      v.y = _p.y;
      v.z = _p.z;
      v.w = 0.0f;
    }

    DiagMat33( DiagMat33Arg _mat )
    {
      v.x = _mat.v.x;
      v.y = _mat.v.y;
      v.z = _mat.v.z;
      v.w = 0.0f;
    }

    DiagMat33( DiagMat44Arg _mat )
    {
      v.x = _mat.v.x;
      v.y = _mat.v.y;
      v.z = _mat.v.z;
      v.w = 0.0f;
    }

    DiagMat33( Vector4Arg _v )
    {
      v.x = _v.v.x;
      v.y = _v.v.y;
      v.z = _v.v.z;
      v.w = 0.0f;
    }

    DiagMat33( Dir3Arg _d )
    {
      v.x = _d.v.x;
      v.y = _d.v.y;
      v.z = _d.v.z;
      v.w = 0.0f;
    }

    DiagMat33( const float _x, const float _y, const float _z )
    {
      v.x = _x;
      v.y = _y;
      v.z = _z;
      v.w = 0.0f;
    }

    DiagMat33()
    {
      v.x = 1.0f;
      v.y = 1.0f;
      v.z = 1.0f;
      v.w = 0.0f;
    }

    // Assignment operators
    DiagMat33 &operator=( DiagMat33Arg _mat )
    {
      v.x = _mat.v.x;
      v.y = _mat.v.y;
      v.z = _mat.v.z;
      v.w = _mat.v.w;
      return *this;
    }

    DiagMat33 &operator=( DiagMat44Arg _mat )
    {
      v.x = _mat.v.x;
      v.y = _mat.v.y;
      v.z = _mat.v.z;
      v.w = 0.0f;
      return *this;
    }

    // Getters for diagonal vectors
    Dir3 GetX() { return Dir3( v.x, 0.0f, 0.0f ); }

    Dir3 GetY() { return Dir3( 0.0f, v.y, 0.0f ); }

    Dir3 GetZ() { return Dir3( 0.0f, 0.0f, v.z ); }

    // Set 3x3 matrix part
    void SetM33( DiagMat44Arg _mat )
    {
      v.x = _mat.v.x;
      v.y = _mat.v.y;
      v.z = _mat.v.z;
    }

    void SetM33( DiagMat33Arg _mat )
    {
      v.x = _mat.v.x;
      v.y = _mat.v.y;
      v.z = _mat.v.z;
    }

    // Multiplication operator
    DiagMat33 &operator*=( DiagMat33Arg _mat )
    {
      v.x *= _mat.v.x;
      v.y *= _mat.v.y;
      v.z *= _mat.v.z;
      return *this;
    }
  };

  class Mat34
  {
  public:
    Vector4 x;
    Vector4 y;
    Vector4 z;

  public:
    Mat34( Mat34Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
    }

    Mat34( DiagMat33Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z, 0.0f );
    }

    Mat34( Vector4Arg _x, Vector4Arg _y, Vector4Arg _z )
    {
      x = _x;
      y = _y;
      z = _z;
    }

    Mat34()
    {
      x.Set( 1.0f, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, 1.0f, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, 1.0f, 0.0f );
    }

    Mat34 &operator=( Mat34Arg _mat )
    {
      x = _mat.x;
      y = _mat.y;
      z = _mat.z;
      return *this;
    }

    Mat34 &operator=( DiagMat33Arg _mat )
    {
      x.Set( _mat.v.x, 0.0f, 0.0f, 0.0f );
      y.Set( 0.0f, _mat.v.y, 0.0f, 0.0f );
      z.Set( 0.0f, 0.0f, _mat.v.z, 0.0f );
      return *this;
    }

    const Vector4 &GetX() const { return x; }

    Vector4 &GetX() { return x; }

    const Vector4 &GetY() const { return y; }

    Vector4 &GetY() { return y; }

    const Vector4 &GetZ() const { return z; }

    Vector4 &GetZ() { return z; }

    void SetX( Vector4Arg _x ) { x = _x; }

    void SetY( Vector4Arg _y ) { y = _y; }

    void SetZ( Vector4Arg _z ) { z = _z; }

    Vector4 &operator[]( unsigned int i ) { return ( (Vector4 *)&x )[i]; }

    const Vector4 &operator[]( unsigned int i ) const { return ( (const Vector4 *)&x )[i]; }
  };

  class TranMat43
  {
  public:
    struct Packed
    {
      float x;
      float y;
      float z;
      void Set( TranMat43Arg );
      void Set( float, float, float );
      float GetX();
      float GetY();
      float GetZ();
    };

    Float4 v;

  public:
    TranMat43( const TranMat43::Packed & );
    TranMat43( TranMat43Arg );
    TranMat43( Mat44Arg );
    TranMat43( DiagMat44Arg );
    TranMat43( Mat43Arg );
    TranMat43( Mat33Arg );
    TranMat43( DiagMat33Arg );
    TranMat43( Dir3Arg );
    TranMat43( Position3Arg );
    TranMat43( const float, const float, const float );
    TranMat43();
    TranMat43 &operator=( TranMat43Arg );
    TranMat43 &operator=( Mat44Arg );
    TranMat43 &operator=( DiagMat44Arg );
    TranMat43 &operator=( Mat43Arg );
    TranMat43 &operator=( Mat33Arg );
    TranMat43 &operator=( DiagMat33Arg );
    Dir3 GetX();
    Dir3 GetY();
    Dir3 GetZ();
    Position3 GetW();
    void SetW( Position3Arg );
    TranMat43 &operator*=( Mat43Arg );
    TranMat43 &operator*=( TranMat43Arg );
  };

  class Position3
  {
  public:
    Float4 v;

    struct Constant
    {
      float x;
      float y;
      float z;
      float w;
    };

    struct Packed
    {
      Packed( float x, float y, float z ) { Set( x, y, z ); }

      Packed( const Position3 &pos ) { Set( pos ); }

      float x;
      float y;
      float z;

      void Set( Position3Arg pos )
      {
        x = pos.v.x;
        y = pos.v.y;
        z = pos.v.z;
      }

      void Set( const float x, const float y, const float z )
      {
        this->x = x;
        this->y = y;
        this->z = z;
      }

      void SetX( const float x ) { this->x = x; }

      void SetY( const float y ) { this->y = y; }

      void SetZ( const float z ) { this->z = z; }

      float GetX() const { return x; }

      float GetY() const { return y; }

      float GetZ() const { return z; }

      float &operator[]( int i ) { return *( &x + i ); }

      const float &operator[]( int i ) const { return *( &x + i ); }

      Packed &operator+=( const Packed &other )
      {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
      }
    };

  public:
    Position3( const Packed &packed ) {}

    Position3( const Constant &constant )
    {
      v.x = constant.x;
      v.y = constant.y;
      v.z = constant.z;
      v.w = constant.w;
    }

    Position3( const float x, const float y, const float z )
    {
      v.x = x;
      v.y = y;
      v.z = z;
      v.w = 1.0f;
    }

    Position3( const float *ptr )
    {
      v.x = ptr[0];
      v.y = ptr[1];
      v.z = ptr[2];
      v.w = 1.0f;
    }

    Position3()
    {
      v.x = 0.0f;
      v.y = 0.0f;
      v.z = 0.0f;
      v.w = 1.0f;
    }

    float GetX() const { return v.x; }

    float GetY() const { return v.y; }

    float GetZ() const { return v.z; }

    void SetX( const float _x ) { v.x = _x; }

    void SetY( const float _y ) { v.y = _y; }

    void SetZ( const float _z ) { v.z = _z; }

    void Set( const float _x, const float _y, const float _z )
    {
      v.x = _x;
      v.y = _y;
      v.z = _z;
    }

    Position3 &operator+=( const float scalar )
    {
      v.x += scalar;
      v.y += scalar;
      v.z += scalar;
      return *this;
    }

    Position3 &operator+=( Vector4Arg vec )
    {
      v.x += vec.v.x;
      v.y += vec.v.y;
      v.z += vec.v.z;
      return *this;
    }
  };

  // Out-of-line definitions for Vector4 constructors that take Position3Arg
  inline Vector4::Vector4( Position3Arg _p, const float _w )
  {
    v.x = _p.v.x;
    v.y = _p.v.y;
    v.z = _p.v.z;
    v.w = _w;
  }

  inline Vector4::Vector4( Position3Arg _p )
  {
    v.x = _p.v.x;
    v.y = _p.v.y;
    v.z = _p.v.z;
    v.w = 1.0f;
  }

  class Quaternion
  {
  public:
    struct Constant
    {
      float x;
      float y;
      float z;
      float w;
    };

    struct Packed
    {
      float x;
      float y;
      float z;
      float w;

      void Set( const float _x, const float _y, const float _z, const float _w )
      {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
      }

      float GetX() { return x; }

      float GetY() { return y; }

      float GetZ() { return z; }

      float GetW() { return w; }
    };

    Float4 v;

  public:
    Quaternion( const Packed &_p )
    {
      v.x = _p.x;
      v.y = _p.y;
      v.z = _p.z;
      v.w = _p.w;
    }

    Quaternion( const Constant &_c )
    {
      v.x = _c.x;
      v.y = _c.y;
      v.z = _c.z;
      v.w = _c.w;
    }

    Quaternion( Dir3Arg &_d )
    {
      // Create a quaternion from a direction vector (pure quaternion)
      v.x = _d.v.x;
      v.y = _d.v.y;
      v.z = _d.v.z;
      v.w = 0.0f;
    }

    Quaternion( Vector4Arg &_v )
    {
      v.x = _v.v.x;
      v.y = _v.v.y;
      v.z = _v.v.z;
      v.w = _v.v.w;
    }

    Quaternion( const float _x, const float _y, const float _z, const float _w )
    {
      v.x = _x;
      v.y = _y;
      v.z = _z;
      v.w = _w;
    }

    Quaternion()
    {
      // Initialize to identity quaternion
      v.x = 0.0f;
      v.y = 0.0f;
      v.z = 0.0f;
      v.w = 1.0f;
    }

    // Getters for individual components
    float GetX() { return v.x; }

    float GetY() { return v.y; }

    float GetZ() { return v.z; }

    float GetW() { return v.w; }

    float &operator[]( unsigned int i ) { return *( &v.x + i ); }

    const float &operator[]( unsigned int i ) const { return *( &v.x + i ); }

    Dir3 GetImag() { return Dir3( v.x, v.y, v.z ); }

    float GetReal() { return v.w; }
  };

  Dir3 operator+( Dir3Arg _a, Dir3Arg _b )
  {
    Dir3 tmp;
    tmp.SetX( _a.GetX() + _b.GetX() );
    tmp.SetY( _a.GetY() + _b.GetY() );
    tmp.SetZ( _a.GetZ() + _b.GetZ() );
    tmp.v.w = _a.GetW() + _b.GetX();
    return tmp;
  }

  Dir3 operator-( Dir3Arg _v )
  {
    Dir3 tmp;
    tmp.SetX( -_v.GetX() );
    tmp.SetY( -_v.GetY() );
    tmp.SetZ( -_v.GetZ() );
    tmp.v.w = -_v.GetW();
    return tmp;
  }

  Dir3 operator-( Dir3Arg _a, Dir3Arg _b )
  {
    Dir3 tmp;
    tmp.SetX( _a.GetX() - _b.GetX() );
    tmp.SetY( _a.GetY() - _b.GetY() );
    tmp.SetZ( _a.GetZ() - _b.GetZ() );
    tmp.v.w = _a.GetW() - _b.GetW();
    return tmp;
  }

  Dir3 operator*( Dir3Arg _v, const Mat44 *_m )
  {
    return Mul( _v, _m );
  }

  Dir3 operator*( float _a, Dir3Arg _b )
  {
    Dir3 tmp;
    tmp.SetX( _a * _b.GetX() );
    tmp.SetY( _a * _b.GetY() );
    tmp.SetZ( _a * _b.GetZ() );
    tmp.v.w = _a * _b.GetW();
    return tmp;
  }

  Dir3 operator/( Dir3Arg _a, float _b )
  {
    Dir3 tmp;
    tmp.SetX( _a.GetX() / _b );
    tmp.SetY( _a.GetY() / _b );
    tmp.SetZ( _a.GetZ() / _b );
    return tmp;
  }

  Vector4 DotVec( Dir3Arg _a, Dir3Arg _b )
  {
    Vector4 ret;
    ret.v.w = ( _a.GetX() * _b.GetX() ) + ( _a.GetY() * _b.GetY() ) + ( _a.GetZ() * _b.GetZ() );
    ret.v.z = ret.v.w;
    ret.v.y = ret.v.w;
    ret.v.x = ret.v.w;
    return ret;
  }

  Dir3 Mul( Dir3Arg _v, const Mat44 *_m )
  {
    Dir3 tmp;
    tmp.SetX( _v.GetX() * _m->GetX().GetX() + _v.GetY() * _m->GetY().GetX() + _v.GetZ() * _m->GetZ().GetX() );
    tmp.SetY( _v.GetX() * _m->GetX().GetY() + _v.GetY() * _m->GetY().GetY() + _v.GetZ() * _m->GetZ().GetY() );
    tmp.SetZ( _v.GetX() * _m->GetX().GetZ() + _v.GetY() * _m->GetY().GetZ() + _v.GetZ() * _m->GetZ().GetZ() );
    return tmp;
  }

  Dir3 Mul( Dir3Arg _v, DiagMat33Arg _m )
  {
    Dir3 tmp;
    tmp.SetX( _v.GetX() * _m.v.x );
    tmp.SetY( _v.GetY() * _m.v.y );
    tmp.SetZ( _v.GetZ() * _m.v.z );
    tmp.v.w = _v.GetW() * _m.v.w;
    return tmp;
  }

  Dir3 operator*( Dir3Arg _a, DiagMat33Arg _b )
  {
    return Mul( _a, _b );
  }

  float Abs( Dir3Arg _v )
  {
    return sqrtf( _v.v.x * _v.v.x + _v.v.y * _v.v.y + _v.v.z * _v.v.z );
  }

  float AbsSquared( Dir3Arg _v )
  {
    return ( ( _v.v.x * _v.v.x ) + ( _v.v.y * _v.v.y ) + ( _v.v.z * _v.v.z ) );
  }

  float Dot( Dir3Arg _a, Dir3Arg _b )
  {
    return _a.GetX() * _b.GetX() + _a.GetY() * _b.GetY() + _a.GetZ() * _b.GetZ();
  }

  Dir3 Cross( Dir3Arg _a, Dir3Arg _b )
  {
    Dir3 tmp;
    tmp.SetX( _a.GetY() * _b.GetZ() - _a.GetZ() * _b.GetY() );
    tmp.SetY( _a.GetZ() * _b.GetX() - _a.GetX() * _b.GetZ() );
    tmp.SetZ( _a.GetX() * _b.GetY() - _a.GetY() * _b.GetX() );
    tmp.v.w = 0.0f;

    return tmp;
  }

  Dir3 SafeNormalize( Dir3Arg _v )
  {
    Vector4 lensqr = DotVec( _v, _v );
    if ( lensqr.GetX() <= 0.0f )
    {
      return Vector4_Zero();
    }
    else
    {
      return Mul( _v, RSqrt( lensqr ) );
    }
  }

  Dir3 Unitize( Dir3Arg _v )
  {
    return SafeNormalize( _v );
  }

#define VALIDATE_POSITION_VECTOR( pos )                                                                                              \
  if ( ( pos )[0] != ( pos )[0] || fabsf( ( pos )[0] ) > 100000.0f || ( pos )[1] != ( pos )[1] || fabsf( ( pos )[1] ) > 100000.0f || \
       ( pos )[2] != ( pos )[2] || fabsf( ( pos )[2] ) > 100000.0f )
        */
}
