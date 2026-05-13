#pragma once

#include "tl_defs.h"

#pragma pack( push, 8 )
typedef struct tagTHREADNAME_INFO
{
  unsigned int dwType;
  const char *szName;
  unsigned int dwThreadID;
  unsigned int dwFlags;
} THREADNAME_INFO;
#pragma pack( pop )

struct _SCOPETABLE_ENTRY
{
  unsigned int enclosing_level;
  unsigned int filter;
  unsigned int specific_handler;
};

typedef struct _SCOPETABLE_ENTRY *PSCOPETABLE_ENTRY;

typedef struct _EH3_EXCEPTION_REGISTRATION
{
  struct _EH3_EXCEPTION_REGISTRATION *Next;
  PVOID ExceptionHandler;
  PSCOPETABLE_ENTRY ScopeTable;
  DWORD TryLevel;
} _EH3_EXCEPTION_REGISTRATION;

typedef struct CPPEH_RECORD
{
  DWORD old_esp;
  EXCEPTION_POINTERS *exc_ptr;
  struct _EH3_EXCEPTION_REGISTRATION registration;
} CPPEH_RECORD;

const DWORD MS_VC_EXCEPTION = 0x406D1388;

tlThreadId tlGetCurrentThreadId();

FORCEINLINE void tlMemoryFence()
{
  LONG Fence = 0;
  InterlockedExchange( &Fence, 0 );
}

inline void SetThreadName( unsigned int dwThreadID, const char *szThreadName )
{
  THREADNAME_INFO info;
  CPPEH_RECORD ms_exc;

  info.dwType = 4096;
  info.szName = szThreadName;
  info.dwThreadID = dwThreadID;
  info.dwFlags = 0;
  ms_exc.registration.TryLevel = 0;

#pragma warning( push )
#pragma warning( disable : 6320 6322 )
  __try
  {
    RaiseException( MS_VC_EXCEPTION, 0, sizeof( info ) / sizeof( ULONG_PTR ), (ULONG_PTR *)&info );
  }
  __except ( EXCEPTION_EXECUTE_HANDLER )
  {
  }
#pragma warning( pop )
}

inline void tlPrefetch( void* addr, u32 offset = 0 )
{
  _mm_prefetch( (const char *)addr + offset, _MM_HINT_T0 );
}

inline int tlAtomicIncrement( volatile u32 *var )
{
  return InterlockedExchangeAdd( (volatile LONG*)var, 1 );
}

inline int tlAtomicDecrement( volatile u32 *var )
{
  return InterlockedExchangeAdd( (volatile LONG*)var, -1 ) - 1;
}

inline bool tlAtomicCompareAndSwap( volatile u32 *var, u32 exchange, u32 compare )
{
  return InterlockedCompareExchange( (volatile LONG*)var, (LONG)exchange, (LONG)compare ) == (LONG)compare;
}

inline bool tlAtomicCompareAndSwap( volatile u64 *var, u64 newvalue, u64 compare )
{
  return InterlockedCompareExchange64( (volatile LONGLONG *)var, (LONGLONG)newvalue, (LONGLONG)compare ) == (LONGLONG)compare;
}

inline u32 tlAtomicAdd( volatile u32 *var, u32 value )
{
  volatile u32 i;

  for ( i = *var; !tlAtomicCompareAndSwap( var, i + value, i ); i = *var )
  {
    Sleep( 0 );
  }
  return i + value;
}

inline u64 tlAtomicAnd( volatile u64 *var, u64 value )
{
  u64 v;

  for ( v = *var; !tlAtomicCompareAndSwap( var, value & v, v ); v = *var )
  {
    Sleep( 0 );
  }
  return value & v;
}

inline u64 tlAtomicOr( volatile u64 *var, u64 value )
{
  u64 v;

  for ( v = *var; !tlAtomicCompareAndSwap( var, value | v, v ); v = *var )
  {
    Sleep( 0 );
  }
  return value | v;
}

inline tlThreadId tlGetCurrentThreadId()
{
  return GetCurrentThreadId();
}

inline void tlYield()
{
  SwitchToThread();
}

class ALIGN( 8 ) tlAtomicReadWriteMutex
{
private:
  volatile u64 WriteThreadId;
  volatile u32 ReadLockCount;
  volatile u32 WriteLockCount;
  tlAtomicReadWriteMutex *ThisPtr;

public:
  void WriteLock()
  {
    tlThreadId CurThread;

    CurThread = tlGetCurrentThreadId();
    if ( tlAtomicCompareAndSwap( &this->ThisPtr->WriteThreadId, CurThread, CurThread ) )
    {
      tlAtomicIncrement( &this->ThisPtr->WriteLockCount );
      return;
    }
    while ( 1 )
    {
      if ( !tlAtomicCompareAndSwap( &this->ThisPtr->WriteThreadId, CurThread, 0ULL ) )
      {
        tlYield();
        continue;
      }
      if ( tlAtomicCompareAndSwap( &this->ThisPtr->ReadLockCount, 0, 0 ) )
        break;
      while ( !tlAtomicCompareAndSwap( &this->ThisPtr->WriteThreadId, 0ULL, CurThread ) )
        ;
      tlYield();
    }
    tlAtomicIncrement( &this->ThisPtr->WriteLockCount );
    tlMemoryFence();
  }

  void ReadLock()
  {
    tlThreadId CurThread;

    CurThread = tlGetCurrentThreadId();
    if ( tlAtomicCompareAndSwap( &this->ThisPtr->WriteThreadId, CurThread, CurThread ) )
    {
      tlAtomicIncrement( &this->ThisPtr->ReadLockCount );
    }
    else
    {
      while ( !tlAtomicCompareAndSwap( &this->ThisPtr->WriteThreadId, CurThread, 0ULL ) )
        tlYield();
      tlAtomicIncrement( &this->ThisPtr->ReadLockCount );
      while ( !tlAtomicCompareAndSwap( &this->ThisPtr->WriteThreadId, 0ULL, CurThread ) )
        ;
    }
    tlMemoryFence();
  }

  void WriteUnlock()
  {
    tlThreadId CurThread;

    CurThread = tlGetCurrentThreadId();
    if ( !tlAtomicDecrement( &this->ThisPtr->WriteLockCount ) )
    {
      tlMemoryFence();
      while ( !tlAtomicCompareAndSwap( &this->ThisPtr->WriteThreadId, 0ULL, CurThread ) )
        ;
    }
  }

  void ReadUnlock() { tlAtomicDecrement( &this->ThisPtr->ReadLockCount ); }
};

class tlAtomicMutex
{
public:
  tlThreadId ThreadId;
  u32 LockCount;
  tlAtomicMutex *ThisPtr;

  ~tlAtomicMutex()
  {
    this->ThreadId = 0;
    this->ThisPtr = NULL;
  }

  void Lock()
  {
    tlThreadId CurThread;

    CurThread = tlGetCurrentThreadId();
    if ( this->ThreadId == CurThread )
    {
      ++this->LockCount;
    }
    else
    {
      while ( !tlAtomicCompareAndSwap( &this->ThisPtr->ThreadId, CurThread, 0ULL ) )
        tlYield();
      tlMemoryFence();
      this->LockCount = 1;
    }
  }

  void Unlock()
  {
    if ( LockCount-- == 1 )
    {
      tlMemoryFence();
      ThreadId = 0;
    }
  }

  bool TryLock()
  {
    tlThreadId CurThread;

    CurThread = tlGetCurrentThreadId();
    if ( this->ThreadId == CurThread )
    {
      ++this->LockCount;
      return true;
    }
    else
    {
      if ( tlAtomicCompareAndSwap( &this->ThisPtr->ThreadId, CurThread, 0ULL ) )
      {
        tlMemoryFence();
        this->LockCount = 1;
        return true;
      }
    }
    return false;
  }

  void Create()
  {
    ThisPtr = this;
    ThreadId = 0;
    LockCount = 0;
  }

  void Destroy()
  {
    ThreadId = 0;
    ThisPtr = 0;
  }
};

class tlSharedAtomicMutex
{
public:
  volatile u64 ThreadId;
  volatile u32 LockCount;
  tlSharedAtomicMutex *ThisPtr;

  void Lock()
  {
    tlThreadId CurThread;

    CurThread = tlGetCurrentThreadId();
    if ( this->ThisPtr->ThreadId == CurThread )
    {
      ++this->ThisPtr->LockCount;
    }
    else
    {
      while ( !tlAtomicCompareAndSwap( &this->ThisPtr->ThreadId, CurThread, 0ULL ) )
        tlYield();
      tlMemoryFence();
      this->ThisPtr->LockCount = 1;
    }
  }
  void Unlock()
  {
    if ( ThisPtr->LockCount-- == 1 )
    {
      tlMemoryFence();
      ThisPtr->ThreadId = 0;
    }
  }
};
