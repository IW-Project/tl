#include "jobqueue.h"

#define JQ_MAX_TEMP_WORKERS       16
#define JQ_ATOMIC_HEAP_MAX_LEVELS 11
#define JQ_MAX_QUEUES             8

jqBatchGroup::jqBatchGroup()
{
  BatchCount = 0;
}

jqModule::jqModule( const char *Name, jqWorkerType Type, jqModuleCallback Code, jqBatchGroup Group )
{
  this->Name = Name;
  this->Type = Type;
  this->Code = Code;
  this->Group = Group;
}

jqBatch::jqBatch()
{
  p3x_info = 0;
  Input = 0;
  Output = 0;
  Module = 0;
  ConditionalAddress = 0;
  ConditionalValue = 0;
  GroupID = 0;
  memset( ParamData, 0, sizeof( ParamData ) );
}

bool jqAtomicHeap::GetAvailableBlock( LevelInfo *FitLevel, int *FitSlot )
{
  int c;
  u64 cellContents;
  u64 newCellContents;
  u64 bit;

  for ( c = 0; c < FitLevel->NCells; ++c )
  {
    cellContents = FitLevel->CellAvailable[c];
    if ( cellContents )
    {
      break;
    }
  }

  if ( c >= FitLevel->NCells )
  {
    return false;
  }

  bit = 1i64 << 63;
  while ( ( cellContents & bit ) == 0 )
  {
    bit >>= 1;
  }

  *FitSlot = 64 * c;
  while ( bit != 1 )
  {
    bit >>= 1;
    ++*FitSlot;
  }

  newCellContents = cellContents & ~( 1i64 << ( *FitSlot & 0x3F ) );
  return tlAtomicCompareAndSwap( (volatile u64 *)&FitLevel->CellAvailable[c], newCellContents, cellContents );
}

bool jqAtomicHeap::AllocBlock( LevelInfo **FitLevel, int *FitSlot )
{
  if ( *FitLevel >= &Levels[NLevels] )
  {
    return false;
  }

  while ( !GetAvailableBlock( *FitLevel, FitSlot ) )
  {
    if ( ++*FitLevel >= &Levels[NLevels] )
    {
      return false;
    }
  }
  return true;
}

int jqAtomicHeap::SplitBlock( LevelInfo *Level, int Slot, LevelInfo *LevelTo )
{
  jqAtomicHeap::LevelInfo *i;

  for ( i = Level; i > LevelTo; tlAtomicOr( &i->CellAvailable[( Slot + 1 ) / 64], 1i64 << ( ( Slot + 1 ) & 0x3F ) ) )
  {
    Slot *= 2;
    --i;
  }
  tlAtomicOr( &i->CellAllocated[Slot / 64], 1i64 << ( Slot & 0x3F ) );
  return Slot;
}

char *jqAtomicHeap::AllocLevel( int LevelIdx )
{
  jqAtomicHeap::LevelInfo *LevelTo;
  jqAtomicHeap::LevelInfo *FitLevel;
  int blockPos;

  LevelTo = &Levels[LevelIdx];
  FitLevel = LevelTo;
  LevelIdx = 0;
  if ( !AllocBlock( &FitLevel, &LevelIdx ) )
  {
    return 0;
  }
  blockPos = SplitBlock( FitLevel, LevelIdx, LevelTo );
  tlAtomicAdd( &ThisPtr->TotalBlocks, 1 );
  tlAtomicAdd( &ThisPtr->TotalUsed, LevelTo->BlockSize );
  return &HeapBase[blockPos * LevelTo->BlockSize];
}

int jqAtomicHeap::FindLevelForSize( unsigned int Size )
{
  return ( BlockSize < Size ) + ( 2 * BlockSize < Size ) + ( 4 * BlockSize < Size ) + ( 8 * BlockSize < Size ) + ( 16 * BlockSize < Size ) +
         ( 32 * BlockSize < Size ) + ( BlockSize << 6 < Size ) + ( BlockSize << 7 < Size ) + ( BlockSize << 8 < Size ) +
         ( BlockSize << 9 < Size ) + ( BlockSize << 10 < Size );
}

char *jqAtomicHeap::Alloc( unsigned int Size, unsigned int Align )
{
  int LevelForSize;
  char *alloc;

  Mutex.Lock();
  Size = ( Size < Align ) ? Align : Size;
  if ( Size <= HeapSize )
  {
    LevelForSize = FindLevelForSize( Size );
    alloc = AllocLevel( LevelForSize );
    Mutex.Unlock();
    return alloc;
  }
  else
  {
    tlPrintf( "Size (%d) > HeapSize (%d), return NULL\n", Size, HeapSize );
    return 0;
  }
}

void jqAtomicHeap::FindAllocatedBlock( unsigned int Offset, LevelInfo **FitLevel, int *FitSlot )
{
  *FitLevel = Levels;

  if ( *FitLevel < &Levels[NLevels] )
  {
    for ( ; *FitLevel < &Levels[NLevels]; ++*FitLevel )
    {
      *FitSlot = Offset / ( *FitLevel )->BlockSize;
      if ( ( ( *FitLevel )->CellAllocated[*FitSlot / 64] & ( 1i64 << ( *FitSlot & 0x3F ) ) ) != 0 )
      {
        break;
      }
    }
  }
  tlAssert( *FitLevel < &Levels[NLevels] );
}

void jqAtomicHeap::MergeBlocks( LevelInfo **FitLevel, int *FitSlot )
{
  int buddySlot;
  int cell;
  u64 cellContents;
  u64 bit;
  u64 newCellContents;

  while ( *FitLevel < &Levels[NLevels - 1] )
  {
    buddySlot = *FitSlot ^ 1;
    cell = buddySlot / 64;
    bit = 1i64 << ( buddySlot & 0x3F );

    for ( ;; )
    {
      cellContents = ( *FitLevel )->CellAvailable[cell];
      if ( ( cellContents & bit ) == 0 )
      {
        return;
      }

      newCellContents = cellContents & ~bit;
      if ( tlAtomicCompareAndSwap( (volatile u64 *)&( *FitLevel )->CellAvailable[cell], newCellContents, cellContents ) )
      {
        break;
      }
    }

    ++*FitLevel;
    *FitSlot >>= 1;
  }
}

void jqAtomicHeap::Free( void *Ptr )
{
  int FitSlot = 0;
  LevelInfo *FitLevel = (LevelInfo *)Ptr;

  Mutex.Lock();
  FindAllocatedBlock( (char *)Ptr - HeapBase, &FitLevel, &FitSlot );
  tlAtomicAnd( &FitLevel->CellAllocated[BlockCell( FitSlot )], BlockBit( FitSlot ) );
  tlAtomicAdd( &ThisPtr->TotalBlocks, 0xFFFFFFFF );
  tlAtomicAdd( &ThisPtr->TotalUsed, -FitLevel->BlockSize );
  MergeBlocks( &FitLevel, &FitSlot );

  tlAssert( ( jqGet( &FitLevel->CellAvailable[BlockCell( FitSlot )] ) & BlockBit( FitSlot ) ) == 0 );
  tlAtomicOr( &FitLevel->CellAvailable[BlockCell( FitSlot )], BlockBit( FitSlot ) );
  Mutex.Unlock();
}

jqAtomicHeap::~jqAtomicHeap()
{
  if ( LevelData )
  {
    tlMemFree( LevelData );
  }
  Mutex.ThreadId = 0;
  Mutex.ThisPtr = NULL;
}

void jqAtomicHeap::Init( void *_HeapBase, unsigned int _HeapSize, unsigned int _BlockSize )
{
  int i, j, k;
  unsigned __int64 *nextCell;
  int align;

  HeapBase = reinterpret_cast<char *>( _HeapBase );
  HeapSize = _HeapSize;
  BlockSize = _BlockSize;
  ThisPtr = this;
  TotalUsed = 0;
  TotalBlocks = 0;
  NLevels = 1;
  for ( i = BlockSize; i < HeapSize; i *= 2 )
  {
    ++NLevels;
  }
  tlAssert( ( BlockSize << ( NLevels - 1 ) ) == HeapSize );
  tlAssert( NLevels <= JQ_ATOMIC_HEAP_MAX_LEVELS );
  i = 0;
  for ( j = 0; j < NLevels; ++j )
  {
    Levels[j].BlockSize = BlockSize << j;
    Levels[j].NBlocks = 1 << ( NLevels - 1 - j );
    Levels[j].NCells = tlCeilDiv( Levels[j].NBlocks, 64 );
    i += tl_align( ( Levels[j].NBlocks ), 1024 ) / 8;
  }
  LevelData = (unsigned char *)tlMemAlloc( 2 * i, 128, 0 );
  memset( LevelData, 0, 2 * i );
  nextCell = (unsigned __int64 *)&LevelData[i];
  for ( k = 0; k < NLevels; ++k )
  {
    align = tl_align( ( Levels[k].NBlocks ), 1024 ) / 8;
    Levels[k].CellAvailable = (unsigned __int64 *)LevelData;
    Levels[k].CellAllocated = nextCell;
    nextCell += align;
    LevelData += align;
  }
  Levels[NLevels - 1].CellAvailable[0] = 1;
}

jqQueue::~jqQueue()
{
  Queue.TailLock.ThreadId = 0;
  Queue.TailLock.ThisPtr = 0;
  Queue.HeadLock.ThreadId = 0;
  Queue.HeadLock.ThisPtr = 0;
}

jqBatchPool::~jqBatchPool()
{
  BatchDataHeap.~jqAtomicHeap();
  BaseQueue.Queue.TailLock.ThreadId = 0;
  BaseQueue.Queue.TailLock.ThisPtr = NULL;
  BaseQueue.Queue.HeadLock.ThreadId = 0;
  BaseQueue.Queue.HeadLock.ThisPtr = 0;
}

unsigned int jqProcessorsMask = 255;
int jqNWorkers;
unsigned __int64 jqMainThreadID;

jqWorker *jqWorkers;
jqWorker *jqTempWorkers;
jqBatchPool jqPool = jqBatchPool();
jqQueue jqGlobalQueue;
jqQueue jqHighPriorityQueue;
void( __cdecl *jqWorkerInitFn )( int );
int jqKeepWorkersAwakeCount;
int jqSleepingWorkersCount;
int jqPoolLock;
int jqBatchPoolExternallyLockedCount;
bool jqStopSignal;
HANDLE jqNewJobAdded;
const char *jqCheckContext;
int jqNextAvailTempWorker;

__declspec( thread ) jqQueue *jqCurQueue;
__declspec( thread ) jqWorker *jqCurWorker;
__declspec( thread ) jqBatch *jqCurBatch;

void jqAttachQueueToWorkers( jqQueue *Queue, unsigned int ProcessorMask )
{
  int numQueues;
  jqWorker *Worker;
  u32 Processor;
  u32 BaseProcessorsMask;
  int id;

  id = -1;
  BaseProcessorsMask = jqProcessorsMask;
  Processor = 1;
  while ( BaseProcessorsMask )
  {
    while ( ( Processor & BaseProcessorsMask ) == 0 )
      Processor *= 2;
    ++id;
    BaseProcessorsMask ^= Processor;
    if ( ( ProcessorMask & Processor ) != 0 )
    {
      Worker = &jqWorkers[id];
      do
      {
        numQueues = Worker->NumQueues;
        tlAssert( numQueues < JQ_MAX_QUEUES );
      } while ( !tlAtomicCompareAndSwap( (volatile u32 *)&Worker->Queues[numQueues], (u32)Queue, 0 ) );
      tlAtomicIncrement( (volatile u32 *)&Worker->NumQueues );
      Queue->ProcessorsMask |= Worker->Processor;
    }
  }
}

void jqEnableWorkers( unsigned int ProcessorsMask )
{
  jqProcessorsMask = ProcessorsMask;
}

int jqGetNumWorkers()
{
  return jqNWorkers;
}

unsigned __int64 jqGetCurrentThreadID()
{
  return GetCurrentThreadId();
}

unsigned __int64 jqGetMainThreadID()
{
  return jqMainThreadID;
}

jqBatchPool *jqGetPool()
{
  return &jqPool;
}

jqBatch *jqGetCurrentBatch()
{
  return jqCurBatch;
}

jqWorker *jqGetCurrentWorker()
{
  return jqCurWorker;
}

jqQueue *jqGetWorkerQueue( int worker )
{
  int i;

  if ( jqNWorkers <= 0 )
  {
    return 0;
  }

  for ( i = 0; i < jqNWorkers; ++i )
  {
    if ( jqWorkers[i].Processor == worker )
    {
      return &jqWorkers[i].WorkerSpecific;
    }
  }

  return 0;
}

void jqShutdownWorker()
{
  ;
}

int jqGetQueuedBatchCount( jqBatchGroup *GroupID )
{
  if ( GroupID )
  {
    return GroupID->QueuedBatchCount;
  }
  else
  {
    return jqPool.GroupID.QueuedBatchCount;
  }
}

int jqGetExecutingBatchCount( jqBatchGroup *GroupID )
{
  if ( GroupID )
  {
    return GroupID->ExecutingBatchCount;
  }
  else
  {
    return jqPool.GroupID.ExecutingBatchCount;
  }
}

jqWorker *jqFindWorkerForProcessor( jqProcessor Processor )
{
  int i;

  if ( jqNWorkers <= 0 )
  {
    return 0;
  }

  for ( i = 0; i < jqNWorkers; ++i )
  {
    if ( jqWorkers[i].Processor == Processor )
    {
      return &jqWorkers[i];
    }
  }
  return 0;
}

jqBoolean jqPoll( jqBatchGroup *GroupID )
{
  jqBatchGroup *BatchCount;

  if ( GroupID )
  {
    BatchCount = GroupID;
  }
  else
  {
    BatchCount = &jqPool.GroupID;
  }
  tlAssert( ( (unsigned int)BatchCount & 0x7 ) == 0 );
  return BatchCount->BatchCount != 0;
}

bool jqAreJobsQueued( jqBatchGroup *GroupID )
{
  if ( GroupID )
  {
    return GroupID->QueuedBatchCount != 0;
  }
  else
  {
    return jqPool.GroupID.QueuedBatchCount != 0;
  }
}

void jqSetWorkerInitFunction( void ( *fn )( int ) )
{
  jqWorkerInitFn = fn;
}

void jqLetWorkersSleep()
{
  tlAssert( jqKeepWorkersAwakeCount > 0 );
  tlAtomicDecrement( (volatile u32 *)&jqKeepWorkersAwakeCount );
}

char *jqAllocBatchData( unsigned int Size )
{
  return jqPool.BatchDataHeap.Alloc( Size, 16 );
}

void jqFreeBatchData( void *Ptr )
{
  jqPool.BatchDataHeap.Free( Ptr );
}

unsigned int jqGetBatchDataAvailable()
{
  return jqPool.BatchDataHeap.HeapSize - jqPool.BatchDataHeap.TotalUsed;
}

int jqExecuteBatch( jqWorker *Worker, jqBatch *Batch )
{
  return Batch->Module->Code( Batch );
}

bool jqCanBatchExecute()
{
  return true;
}

jqBoolean jqWorkerSleep( jqWorker *Worker )
{
  while ( !jqPool.GroupID.QueuedBatchCount )
  {
    if ( jqStopSignal )
    {
      break;
    }
    if ( jqKeepWorkersAwakeCount )
    {
      break;
    }
    tlAtomicIncrement( (volatile u32 *)&jqSleepingWorkersCount );
    WaitForSingleObject( jqNewJobAdded, 1u );
    tlAtomicDecrement( (volatile u32 *)&jqSleepingWorkersCount );
  }
  tlYield();
  tlMemoryFence();
  return !jqStopSignal;
}

void jqSetCheckContext( const char *desc )
{
  jqCheckContext = desc;
}

void jqCheckDMALS( const void *addr )
{
  if ( !addr )
  {
    tlFatal( "%s (LS) is NULL.", jqCheckContext );
  }
  if ( ( (unsigned __int8)addr & 0xF ) != 0 )
  {
    tlFatal( "%s 0x%x (LS) not 16byte aligned.", jqCheckContext, addr );
  }
  if ( (unsigned int)addr > 0x40000 )
  {
    tlFatal( "%s 0x%x (LS) is > 256k.", jqCheckContext, addr );
  }
  if ( (unsigned int)addr < 0x4000 )
  {
    tlFatal( "%s 0x%x (LS) is in kernel memory.", jqCheckContext, addr );
  }
}

void jqCheckDMAMain( const void *addr )
{
  if ( !addr )
  {
    tlFatal( "%s (Main) is NULL.", jqCheckContext );
  }
  if ( ( (unsigned __int8)addr & 0xF ) != 0 )
  {
    tlFatal( "%s 0x%x (Main) not 16byte aligned.", jqCheckContext, addr );
  }
  if ( (unsigned int)addr < 0x40000 )
  {
    tlFatal( "%s 0x%x (Main) is < 256k.", jqCheckContext, addr );
  }
}

void jqCheckDMASize( unsigned int size )
{
  if ( ( size & 0xF ) != 0 )
  {
    tlFatal( "%s size %d not 16byte aligned.", jqCheckContext, size );
  }
}

void jqCheckDMATag( int tag )
{
  if ( tag < 0 || tag >= 31 )
  {
    tlFatal( "%s invalid DMA tag %d.", jqCheckContext, tag );
  }
}

void jqCheckRange( int val, int mn, int mx )
{
  if ( val < mn || val > mx )
  {
    tlFatal( "%s %d out of range [%d-%d].", jqCheckContext, val, mn, mx );
  }
}

void jqCheckStack()
{
  ;
}

void *jqFetch( void *dest, const void *src, unsigned int size )
{
  if ( size )
  {
    memcpy( dest, (char *)src, size );
  }
  return dest;
}

void jqStore( void *dest, const void *src, unsigned int size )
{
  if ( size )
  {
    memcpy( dest, src, size );
  }
}

void *jqFetchAsync( void *dest, const void *src, unsigned int size )
{
  if ( size )
  {
    memcpy( dest, src, size );
  }
  return dest;
}

void jqStoreAsync( void *dest, const void *src, unsigned int size )
{
  if ( size )
  {
    memcpy( dest, src, size );
  }
}

void jqWait()
{
  ;
}

void jqWaitMultiple()
{
  ;
}

void jqSetMemBase()
{
  ;
}

void jqSetStackSize()
{
  ;
}

int jqGetMemAvailable()
{
  return 0;
}

void *jqAlloc()
{
  return 0;
}

void *jqGetMemBase()
{
  return 0;
}

void _jqInit()
{
  SYSTEM_INFO SystemInfo;

  jqNewJobAdded = CreateEventA( 0, 1, 0, 0 );
  jqProcessorsMask = 0;
  GetSystemInfo( &SystemInfo );

  for ( int i = 0; i < SystemInfo.dwNumberOfProcessors; ++i )
  {
    jqProcessorsMask |= 1 << i;
  }
}

void _jqShutdown()
{
  CloseHandle( jqNewJobAdded );
}

void _jqStart()
{
  HANDLE thr;
  DWORD ThreadId;

  for ( int i = 0; i < jqNWorkers; ++i )
  {
    jqWorkers[i].Type = JQ_WORKER_GENERIC;
    if ( jqWorkers[i].Processor != 1 )
    {
      thr = CreateThread( 0, 0x10000, (LPTHREAD_START_ROUTINE)jqWorkerThread, &jqWorkers[i], 4, &ThreadId );
      tlAssert( thr != 0 );
      jqWorkers[i].Thread = thr;
      ResumeThread( thr );
    }
  }
}

void _jqStop()
{
  jqStopSignal = true;
  SetEvent( jqNewJobAdded );

  if ( jqNWorkers )
  {
    for ( int i = 0; i < jqNWorkers; ++i )
    {
      if ( jqWorkers[i].Thread )
      {
        WaitForSingleObject( jqWorkers[i].Thread, INFINITE );
        CloseHandle( jqWorkers[i].Thread );
      }
    }
  }

  jqStopSignal = false;
  ResetEvent( jqNewJobAdded );
}

void _jqAddBatch()
{
  ;
}

void jqAlertWorkers()
{
  PulseEvent( jqNewJobAdded );
}

void jqLockBatchPoolInternal()
{
  while ( !tlAtomicCompareAndSwap( (volatile u32 *)&jqPoolLock, 1, 0 ) )
    tlYield();
  tlMemoryFence();
}

void jqUnlockBatchPoolInternal()
{
  tlMemoryFence();
  while ( !tlAtomicCompareAndSwap( (volatile u32 *)&jqPoolLock, 0, 1 ) )
    ;
}

void jqKeepWorkersAwake()
{
  tlAtomicIncrement( (volatile u32 *)&jqKeepWorkersAwakeCount );
  if ( jqKeepWorkersAwakeCount > 0 )
  {
    PulseEvent( jqNewJobAdded );
  }
}

void jqLockBatchPool()
{
  jqLockBatchPoolInternal();
  tlAtomicIncrement( (volatile u32 *)&jqBatchPoolExternallyLockedCount );
}

void jqUnlockBatchPool()
{
  tlAssert( jqBatchPoolExternallyLockedCount > 0 );
  tlAtomicDecrement( (volatile u32 *)&jqBatchPoolExternallyLockedCount );
  if ( !jqBatchPoolExternallyLockedCount )
  {
    PulseEvent( jqNewJobAdded );
  }
  jqUnlockBatchPoolInternal();
}

void jqSetBatchDataHeapSize( unsigned int Size, unsigned int BlockSize )
{
  void *alloc;

  tlAssert( !jqNWorkers );
  tlAssert( Size > 0 );

  if ( jqPool.BatchDataHeap.LevelData )
  {
    tlMemFree( jqPool.BatchDataHeap.LevelData );
  }
  alloc = tlMemAlloc( Size, 0x80u, 0 );
  jqPool.BatchDataHeap.Init( alloc, Size, BlockSize );
}

void jqInit()
{
  jqAtomicQueue<jqBatch, 32>::NodeType *Node;

  jqPool.ThisPtr = &jqPool;
  jqPool.BaseQueue.Queue.ThisPtr = &jqPool.BaseQueue.Queue;
  jqPool.BaseQueue.Queue._FreeList = 0;
  jqPool.BaseQueue.Queue.FreeListPtr = &jqPool.BaseQueue.Queue._FreeList;
  jqPool.BaseQueue.Queue.FreeLock.ThisPtr = &jqPool.BaseQueue.Queue.FreeLock;
  jqPool.BaseQueue.Queue.FreeLock.ThreadId = 0;
  jqPool.BaseQueue.Queue.FreeLock.LockCount = 0;
  jqPool.BaseQueue.Queue.NodeBlockListHead = 0;
  jqPool.BaseQueue.Queue.HeadLock.ThisPtr = &jqPool.BaseQueue.Queue.HeadLock;
  jqPool.BaseQueue.Queue.HeadLock.ThreadId = 0;
  jqPool.BaseQueue.Queue.HeadLock.LockCount = 0;
  jqPool.BaseQueue.Queue.TailLock.ThisPtr = &jqPool.BaseQueue.Queue.TailLock;
  jqPool.BaseQueue.Queue.TailLock.ThreadId = 0;
  jqPool.BaseQueue.Queue.TailLock.LockCount = 0;
  Node = jqPool.BaseQueue.Queue.AllocateNode();
  Node->Next = NULL;
  jqPool.BaseQueue.Queue.Tail = Node;
  jqPool.BaseQueue.Queue.Head = Node;
  jqPool.BaseQueue.QueuedBatchCount = 0;
  jqPool.BaseQueue.Queue.AllocateNodeBlock( 128 );
  jqMainThreadID = tlGetCurrentThreadId();
  jqSleepingWorkersCount = 0;
  jqMainThreadID = GetCurrentThreadId();
  _jqInit();
}

void jqInitQueue( jqQueue *Queue )
{
  Queue->ThisPtr = Queue;
  Queue->QueuedBatchCount = 0;
  Queue->Queue.Init( &jqPool.BaseQueue.Queue );
  Queue->ProcessorsMask = 0;
}

void jqInitWorker( jqWorker *Worker )
{
  u32 i;

  Worker->ThisPtr = Worker;
  jqInitQueue( &Worker->WorkerSpecific );
  Worker->NumQueues = 0;
  for ( i = 0; i < 8; ++i )
    Worker->Queues[i] = 0;
}

void jqAddBatchToQueue( const jqBatch *Batch, jqQueue *Queue )
{
  tlAssert( Batch->Module != NULL );
  if ( Batch->GroupID )
  {
    tlAtomicIncrement( (volatile u32 *)&Batch->GroupID->QueuedBatchCount );
  }
  tlAtomicIncrement( (volatile u32 *)&Batch->Module->Group.QueuedBatchCount );
  tlAtomicIncrement( (volatile u32 *)&jqPool.GroupID.QueuedBatchCount );
  tlAtomicIncrement( (volatile u32 *)&Queue->QueuedBatchCount );
  Queue->Queue.Push( Batch );
  PulseEvent( jqNewJobAdded );
}

void jqAddBatch( const jqBatch *Batch, jqQueue *Queue )
{
  if ( !Queue )
  {
    Queue = &jqGlobalQueue;
  }
  tlAssert( Batch->Module != NULL );
  if ( Batch->GroupID )
  {
    tlAtomicIncrement( (volatile u32 *)&Batch->GroupID->QueuedBatchCount );
  }
  tlAtomicIncrement( (volatile u32 *)&Batch->Module->Group.QueuedBatchCount );
  tlAtomicIncrement( (volatile u32 *)&jqPool.GroupID.QueuedBatchCount );
  tlAtomicIncrement( (volatile u32 *)&Queue->QueuedBatchCount );
  Queue->Queue.Push( Batch );
  PulseEvent( jqNewJobAdded );
}

void jqAddBatch( const jqModule *Module, void *Input, void *Output, jqBatchGroup *GroupID, jqQueue *Queue, void *ParamData, int ParamSize )
{
  jqBatch Batch;

  Batch.Module = (jqModule *)Module;
  Batch.p3x_info = 0;
  Batch.ConditionalAddress = 0;
  Batch.ConditionalValue = 0;
  Batch.Input = Input;
  Batch.Output = Output;
  Batch.GroupID = GroupID;

  tlAssert( ParamSize >= 0 && ParamSize <= (int)sizeof( Batch.ParamData ) );

  if ( ParamData && ParamSize )
  {
    memcpy( Batch.ParamData, ParamData, ParamSize );
  }
  memset( (char *)Batch.ParamData + ParamSize, 205, 92 - ParamSize );
  jqAddBatch( &Batch, Queue );
}

void jqSkipBatch()
{
  jqAddBatch( jqCurBatch, jqCurQueue );
}

bool jqPopNextBatchFromQueue( jqWorker *Worker, jqQueue *Queue, jqBatchGroup *GroupID, jqBatch *PoppedBatch )
{
  int CheckedBatches;
  int BatchCount;
  jqBatchGroup *PoppedGroup;
  jqAtomicQueue<jqBatch, 32>::NodeType *Node;
  jqAtomicQueue<jqBatch, 32>::NodeType **FreeListPtr;
  LONG Target;
  LONG v8;

  CheckedBatches = 0;
  BatchCount = Queue->ThisPtr->QueuedBatchCount;
  if ( !BatchCount )
  {
    return false;
  }
  while ( CheckedBatches <= BatchCount )
  {
    ++CheckedBatches;
    if ( !Queue->Queue.Pop( PoppedBatch ) )
    {
      break;
    }
    PoppedGroup = PoppedBatch->GroupID;
    if ( !PoppedGroup )
    {
      PoppedGroup = &PoppedBatch->Module->Group;
    }
    jqCurQueue = Queue;
    if ( !GroupID || GroupID == PoppedGroup )
    {
      goto found;
    }
    Queue->Queue.FreeLock.Lock();
    FreeListPtr = Queue->Queue.FreeListPtr;
    Node = *FreeListPtr;
    if ( !Node )
    {
      Queue->Queue.AllocateNodeBlock( 32 );
      FreeListPtr = Queue->Queue.FreeListPtr;
      Node = *FreeListPtr;
    }
    *FreeListPtr = Node->Next;
    if ( Queue->Queue.FreeLock.ThisPtr->LockCount-- == 1 )
    {
      Target = 0;
      InterlockedExchange( &Target, 0 );
      Queue->Queue.FreeLock.ThisPtr->ThreadId = 0;
    }
    memcpy( &Node->Data, PoppedBatch, sizeof( Node->Data ) );
    Node->Next = 0;
    Queue->Queue.TailLock.Lock();
    Queue->Queue.ThisPtr->Tail->Next = Node;
    Queue->Queue.ThisPtr->Tail = Node;
    if ( Queue->Queue.TailLock.LockCount-- == 1 )
    {
      v8 = 0;
      InterlockedExchange( &v8, 0 );
      Queue->Queue.TailLock.ThreadId = 0;
    }
    BatchCount = Queue->ThisPtr->QueuedBatchCount;
    if ( !BatchCount )
    {
      return false;
    }
  }
  return false;
found:
  if ( PoppedBatch->GroupID )
  {
    tlAtomicIncrement( (volatile u32 *)&PoppedBatch->GroupID->ExecutingBatchCount );
    tlAtomicDecrement( (volatile u32 *)&PoppedBatch->GroupID->QueuedBatchCount );
    tlAssert( PoppedBatch->GroupID->QueuedBatchCount >= 0 );
  }
  tlAtomicIncrement( (volatile u32 *)&PoppedBatch->Module->Group.ExecutingBatchCount );
  tlAtomicDecrement( (volatile u32 *)&PoppedBatch->Module->Group.QueuedBatchCount );
  tlAtomicIncrement( (volatile u32 *)&jqGetPool()->GroupID.ExecutingBatchCount );
  tlAtomicDecrement( (volatile u32 *)&jqGetPool()->GroupID.QueuedBatchCount );
  tlAtomicDecrement( (volatile u32 *)&Queue->ThisPtr->QueuedBatchCount );
  tlAssert( jqGetPool()->GroupID.QueuedBatchCount >= 0 );
  tlAssert( Queue->ThisPtr->QueuedBatchCount >= 0 );
  return true;
}

bool jqPopNextBatch( jqWorker *Worker, bool *doHighPriority, jqBatchGroup *GroupID, jqBatch *PoppedBatch )
{
  jqQueue *queue;
  int i;

  if ( jqPopNextBatchFromQueue( Worker, &Worker->WorkerSpecific, GroupID, PoppedBatch ) )
  {
    return true;
  }

  if ( jqHighPriorityQueue.ThisPtr && *doHighPriority && jqPopNextBatchFromQueue( Worker, &jqHighPriorityQueue, GroupID, PoppedBatch ) )
  {
    *doHighPriority = false;
    return true;
  }
  else
  {
    for ( i = 0; i < 8; ++i )
    {
      queue = Worker->Queues[i];
      if ( queue && jqPopNextBatchFromQueue( Worker, queue, GroupID, PoppedBatch ) )
      {
        return true;
      }
    }
    return false;
  }
}

void jqWorkerLoop( jqWorker *Worker, jqBatchGroup *GroupID, bool BreakWhenEmpty, unsigned __int64 *batchCount )
{
  u64 deltaTime;
  u64 WorkerStartTime;
  int ret;
  u32 CachedConditionalValue;
  void *CachedConditionalAddress;
  u64 lastConditionalCheckTime;
  bool doHighPriority;
  jqBatch CurBatch;

  if ( jqWorkerInitFn && Worker->WorkerID > 0 )
  {
    jqWorkerInitFn( Worker->WorkerID );
  }
  lastConditionalCheckTime = 0;
  CachedConditionalAddress = 0;
  CachedConditionalValue = 0;
  jqCurWorker = Worker;
  doHighPriority = 1;
  CurBatch = jqBatch();
  do
  {
    do
    {
      if ( !jqPopNextBatch( Worker, &doHighPriority, GroupID, &CurBatch ) )
      {
        break;
      }
      jqCurBatch = &CurBatch;
      ret = 1;
      WorkerStartTime = tlPcGetTick().QuadPart;
      if ( CachedConditionalAddress == CurBatch.ConditionalAddress && CachedConditionalValue == CurBatch.ConditionalValue )
      {
        CurBatch.ConditionalAddress = 0;
      }
      if ( CurBatch.ConditionalAddress )
      {
        if ( WorkerStartTime - lastConditionalCheckTime > 50000 )
        {
          if ( *(unsigned int *)CurBatch.ConditionalAddress >= CurBatch.ConditionalValue )
          {
            CachedConditionalAddress = CurBatch.ConditionalAddress;
            CachedConditionalValue = CurBatch.ConditionalValue;
            CurBatch.ConditionalAddress = 0;
            ret = jqExecuteBatch( Worker, &CurBatch );
          }
          lastConditionalCheckTime = WorkerStartTime;
        }
      }
      else
      {
        ret = jqExecuteBatch( Worker, &CurBatch );
      }

      if ( !ret )
      {
        deltaTime = tlPcGetTick().QuadPart - WorkerStartTime;
        Worker->WorkTime += deltaTime;
        doHighPriority = 1;
      }
      if ( ret == 2 )
      {
        doHighPriority = 1;
      }
      jqCurBatch = 0;
      if ( ret )
      {
        tlAtomicIncrement( (volatile u32 *)&jqGetPool()->GroupID.QueuedBatchCount );
        tlAtomicIncrement( (volatile u32 *)&CurBatch.Module->Group.QueuedBatchCount );
        tlAtomicIncrement( (volatile u32 *)&jqCurQueue->QueuedBatchCount );
        if ( CurBatch.GroupID )
        {
          tlAtomicIncrement( (volatile u32 *)&CurBatch.GroupID->QueuedBatchCount );
        }
        {
          jqAtomicQueue<jqBatch, 32>::NodeType *Node;
          jqAtomicQueue<jqBatch, 32>::NodeType **FreeListPtr;
          LONG Target;
          LONG v23;

          jqCurQueue->Queue.FreeLock.Lock();
          FreeListPtr = jqCurQueue->Queue.FreeListPtr;
          Node = *FreeListPtr;
          if ( !Node )
          {
            jqCurQueue->Queue.AllocateNodeBlock( 32 );
            FreeListPtr = jqCurQueue->Queue.FreeListPtr;
            Node = *FreeListPtr;
          }
          *FreeListPtr = Node->Next;
          if ( jqCurQueue->Queue.FreeLock.ThisPtr->LockCount-- == 1 )
          {
            Target = 0;
            InterlockedExchange( &Target, 0 );
            jqCurQueue->Queue.FreeLock.ThisPtr->ThreadId = 0;
          }
          memcpy( &Node->Data, &CurBatch, sizeof( Node->Data ) );
          Node->Next = 0;
          jqCurQueue->Queue.TailLock.Lock();
          jqCurQueue->Queue.ThisPtr->Tail->Next = Node;
          jqCurQueue->Queue.ThisPtr->Tail = Node;
          if ( jqCurQueue->Queue.TailLock.LockCount-- == 1 )
          {
            v23 = 0;
            InterlockedExchange( &v23, 0 );
            jqCurQueue->Queue.TailLock.ThreadId = 0;
          }
        }
      }
      tlAtomicDecrement( (volatile u32 *)&jqGetPool()->GroupID.ExecutingBatchCount );
      tlAtomicDecrement( (volatile u32 *)&CurBatch.Module->Group.ExecutingBatchCount );
      if ( CurBatch.GroupID )
      {
        tlAtomicDecrement( (volatile u32 *)&CurBatch.GroupID->ExecutingBatchCount );
      }

    } while ( ( !BreakWhenEmpty || !ret ) && ( !batchCount || *batchCount ) );
    doHighPriority = 1;

  } while ( !BreakWhenEmpty && jqWorkerSleep( Worker ) );
  jqCurWorker = 0;
}

void jqTempWorkerLoop( jqWorker *Worker, jqBatchGroup *GroupID, bool( __cdecl *callback )( void * ), void *context )
{
  int ret;
  int numJobs;
  bool doHighPriority;
  jqBatch CurBatch;

  jqCurWorker = Worker;
  doHighPriority = 1;
  CurBatch = jqBatch();

  while ( ( callback( context ) & 1 ) == 0 )
  {
    numJobs = jqGetQueuedBatchCount( 0 );
    while ( jqPopNextBatch( Worker, &doHighPriority, GroupID, &CurBatch ) )
    {
      jqCurBatch = &CurBatch;
      ret = jqExecuteBatch( Worker, &CurBatch );
      jqCurBatch = 0;
      if ( ret )
      {
        tlAtomicIncrement( (volatile u32 *)&jqGetPool()->GroupID.QueuedBatchCount );
        tlAtomicIncrement( (volatile u32 *)&CurBatch.Module->Group.QueuedBatchCount );
        tlAtomicIncrement( (volatile u32 *)&jqCurQueue->QueuedBatchCount );
        if ( CurBatch.GroupID )
        {
          tlAtomicIncrement( (volatile u32 *)&CurBatch.GroupID->QueuedBatchCount );
        }
        {
          jqAtomicQueue<jqBatch, 32>::NodeType *Node;
          jqAtomicQueue<jqBatch, 32>::NodeType **FreeListPtr;
          LONG Target;
          LONG v18;

          jqCurQueue->Queue.FreeLock.Lock();
          FreeListPtr = jqCurQueue->Queue.FreeListPtr;
          Node = *FreeListPtr;
          if ( !Node )
          {
            jqCurQueue->Queue.AllocateNodeBlock( 32 );
            FreeListPtr = jqCurQueue->Queue.FreeListPtr;
            Node = *FreeListPtr;
          }
          *FreeListPtr = Node->Next;
          if ( jqCurQueue->Queue.FreeLock.ThisPtr->LockCount-- == 1 )
          {
            Target = 0;
            InterlockedExchange( &Target, 0 );
            jqCurQueue->Queue.FreeLock.ThisPtr->ThreadId = 0;
          }
          memcpy( &Node->Data, &CurBatch, sizeof( Node->Data ) );
          Node->Next = 0;
          jqCurQueue->Queue.TailLock.Lock();
          jqCurQueue->Queue.ThisPtr->Tail->Next = Node;
          jqCurQueue->Queue.ThisPtr->Tail = Node;
          if ( jqCurQueue->Queue.TailLock.LockCount-- == 1 )
          {
            v18 = 0;
            InterlockedExchange( &v18, 0 );
            jqCurQueue->Queue.TailLock.ThreadId = 0;
          }
        }
      }
      if ( !ret || ret == 2 )
      {
        doHighPriority = 1;
      }
      tlAtomicDecrement( (volatile u32 *)&jqGetPool()->GroupID.ExecutingBatchCount );
      tlAtomicDecrement( (volatile u32 *)&CurBatch.Module->Group.ExecutingBatchCount );
      if ( CurBatch.GroupID )
      {
        tlAtomicDecrement( (volatile u32 *)&CurBatch.GroupID->ExecutingBatchCount );
      }
      if ( ret )
      {
        if ( --numJobs > 0 )
        {
          continue;
        }
      }
      if ( callback( context ) )
      {
        jqCurWorker = 0;
        return;
      }
      numJobs = jqGetQueuedBatchCount( 0 );
    }
    doHighPriority = 1;
  }
  jqCurWorker = 0;
}

unsigned int jqWorkerThread( void *_this )
{
  jqWorker *worker = (jqWorker *)_this;
  char ThreadName[256];

  sprintf( ThreadName, "JQ Worker %d (Processor 0x%x)", worker->WorkerID, worker->Processor );
  SetThreadName( 0xFFFFFFFF, ThreadName );
  worker->ThreadId = GetCurrentThreadId();
  jqWorkerLoop( worker, 0, 0, 0 );
  return 0;
}

void jqFlush( jqBatchGroup *GroupID, unsigned __int64 batchCount )
{
  unsigned __int64 *BatchCount;
  volatile int *ExecutingBatchCount;
  unsigned __int64 *workerBatchCount;
  unsigned __int64 zero = 0;

  PulseEvent( jqNewJobAdded );

  if ( GroupID )
  {
    BatchCount = &GroupID->BatchCount;
    ExecutingBatchCount = &GroupID->ExecutingBatchCount;
  }
  else
  {
    BatchCount = &jqPool.GroupID.BatchCount;
    GroupID = &jqPool.GroupID;
    ExecutingBatchCount = &jqPool.GroupID.ExecutingBatchCount;
  }

  tlAssert( ( (u32)BatchCount & 7 ) == 0 );
  workerBatchCount = BatchCount;
  if ( batchCount )
  {
    workerBatchCount = &zero;
  }
  while ( 1 )
  {
    tlMemoryFence();
    if ( !( GroupID->QueuedBatchCount + *ExecutingBatchCount ) && GroupID->BatchCount <= batchCount )
    {
      break;
    }
    jqWorkerLoop( jqWorkers, GroupID, 1, workerBatchCount );
  }
}

void jqStop()
{
  tlAssert( jqGetCurrentThreadID() == jqGetMainThreadID() );
  if ( jqWorkers )
  {
    jqFlush( 0, 0 );
    tlAssert( jqPool.GroupID.QueuedBatchCount == 0 );
    _jqStop();
    tlMemFree( jqTempWorkers );
    jqNWorkers = 0;
    tlMemFree( jqWorkers );
    jqWorkers = NULL;
  }
}

void jqAssistWithBatches( bool( __cdecl *callback )( void * ), void *context, jqBatchGroup *GroupID )
{
  jqWorker *tmpWorker;

  if ( jqTempWorkers )
  {
    tlAssert( callback );
    tlAssert( jqNextAvailTempWorker < JQ_MAX_TEMP_WORKERS );
    tmpWorker = &jqTempWorkers[tlAtomicIncrement( (volatile u32 *)&jqNextAvailTempWorker )];
    tmpWorker->Processor = -1;
    tmpWorker->NumQueues = 1;
    tmpWorker->Queues[0] = &jqGlobalQueue;
    jqTempWorkerLoop( tmpWorker, GroupID, callback, context );
    tlAtomicDecrement( (volatile u32 *)&jqNextAvailTempWorker );
  }
}

void jqShutdown()
{
  void *mem;

  tlAssert( jqGetCurrentThreadID() == jqGetMainThreadID() );
  jqStop();
  while ( jqPool.BaseQueue.Queue.NodeBlockListHead )
  {
    mem = jqPool.BaseQueue.Queue.NodeBlockListHead;
    jqPool.BaseQueue.Queue.NodeBlockListHead = jqPool.BaseQueue.Queue.NodeBlockListHead->Next;
    tlMemFree( mem );
  }
  CloseHandle( jqNewJobAdded );
  jqSleepingWorkersCount = 0;
}

void jqStart()
{
  int id, j, Processor, ProcessorsMask;

  tlAssert( jqGetCurrentThreadID() == jqGetMainThreadID() );

  jqStop();

  // Init needed workers
  jqProcessorsMask |= 1;
  jqNWorkers = tlCountOnes( jqProcessorsMask );
  jqWorkers = (jqWorker *)tlMemAlloc( sizeof( jqWorker ) * jqNWorkers, 8, 0 );
  memset( jqWorkers, 0, sizeof( jqWorker ) * jqNWorkers );
  id = 0;
  ProcessorsMask = jqProcessorsMask;
  Processor = 1;
  while ( ProcessorsMask )
  {
    while ( ( Processor & ProcessorsMask ) == 0 )
      Processor *= 2;
    ProcessorsMask ^= Processor;
    jqInitWorker( &jqWorkers[id] );
    jqWorkers[id].Processor = Processor;
    jqWorkers[id].WorkerID = id;
    ++id;
  }
  tlAssert( id == jqNWorkers );

  // Init temp workers
  jqTempWorkers = (jqWorker *)tlMemAlloc( sizeof( jqWorker ) * JQ_MAX_TEMP_WORKERS, 8, 0 );
  memset( jqTempWorkers, 0, sizeof( jqWorker ) * JQ_MAX_TEMP_WORKERS );
  for ( id = 0; id < JQ_MAX_TEMP_WORKERS; ++id )
  {
    jqInitWorker( &jqTempWorkers[id] );
  }

  // Init global queue and high priority queue
  jqInitQueue( &jqGlobalQueue );
  jqAttachQueueToWorkers( &jqGlobalQueue, 255 );
  jqInitQueue( &jqHighPriorityQueue );
  jqAttachQueueToWorkers( &jqHighPriorityQueue, 255 );
  _jqStart();
}
