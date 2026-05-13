#pragma once

#ifdef _WINDOWS
  #include <intrin.h>
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <Windows.h>

  #include <stdio.h>
  #include <stdarg.h>
  #include <Psapi.h>
  #include <new>
#endif

typedef __int32  int32_t;
typedef unsigned __int32 uint32_t;
typedef __int64  int64_t;
typedef unsigned __int64 uint64_t;
typedef unsigned int uint;
typedef unsigned char u8;
typedef unsigned short u16;
typedef signed char s8;
typedef signed short s16;
typedef signed char i8;
typedef signed short i16;
typedef float f32;
typedef double f64;

typedef int32_t i32;
typedef int32_t s32;
typedef uint32_t u32;
typedef int64_t i64;
typedef int64_t s64;
typedef uint64_t u64;

#define tlUNIMPLEMENTED( x )                       \
  {                                                \
    static bool inited = false;                    \
    if ( !inited )                                 \
    {                                              \
      OutputDebugStringA( "----- " );              \
      OutputDebugStringA( x );                     \
      OutputDebugStringA( " not implemented.\n" ); \
      inited = true;                               \
    }                                              \
  }
#define tlAssert( cond )                                            \
  if ( !( cond ) && !_tlAssert( __FILE__, __LINE__, "%s", #cond ) ) \
  {                                                                 \
    __debugbreak();                                                 \
  }
#define tlAssertMsg( cond, msg )                                    \
  if ( !( cond ) && !_tlAssert( __FILE__, __LINE__, #cond, #msg ) ) \
  {                                                                 \
    __debugbreak();                                                 \
  }

#define ALIGN( x ) __declspec( align( x ) )

typedef u64 tlThreadId;

struct tlFileBuf
{
  u8 *Buf;
  uint Size;
  uint UserData;
};

struct tlSystemCallbacks
{
  bool ( *ReadFile )( const char *FileName, tlFileBuf *File, uint Align, uint Flags );
  void ( *ReleaseFile )( tlFileBuf *File );
  void ( *CriticalError )( const char *Text );
  void ( *Warning )( const char *Text );
  void ( *DebugPrint )( const char *Text );
  void *( *MemAlloc )( uint Size, uint Align, uint Flags );
  void *( *MemRealloc )( void *Ptr, uint Size, uint Align, uint Flags );
  void ( *MemFree )( void *Ptr );
};

inline u32 tlEndianSwap32(u32 x) { return (x >> 24) | ((x >> 8) & 0xff00) | ((x & 0xff00) << 8) | (x << 24); }

inline i32 tlEndianSwap32(i32 x) { return (x >> 24) | ((x >> 8) & 0xff00) | ((x & 0xff00) << 8) | (x << 24); }

inline u16 tlEndianSwap16(u16 x) { return (x >> 8) | (x << 8); }

inline i16 tlEndianSwap16(i16 x) { return (x >> 8) | (x << 8); }

inline float tlEndianSwap32(float x)
{
  u32 y = (u32 &)x;
  y = tlEndianSwap32(y);
  return (float &)y;
}
