#pragma once

#include <tl_defs.h>
#include <tl_system.h>
#include <tl_thread.h>

typedef bool( __cdecl *jqDoneAssistingWithBatchesFn )( void * );
typedef void( __cdecl *jqWorkerInitFnType )( int );
typedef int( __cdecl *jqModuleCallback )( class jqBatch * );
typedef int jqBoolean;

enum jqWorkerType
{
  JQ_WORKER_GENERIC = 0x0,
  JQ_WORKER_MAX = 0x1,
  JQ_WORKER_DEFAULT = 0x0,
};

enum jqProcessor
{
  JQ_CORE_0 = 0x1,
  JQ_CORE_1 = 0x2,
  JQ_CORE_2 = 0x4,
  JQ_CORE_3 = 0x8,
  JQ_CORE_4 = 0x10,
  JQ_CORE_5 = 0x20,
  JQ_CORE_6 = 0x40,
  JQ_CORE_7 = 0x80,
  JQ_CORE_ALL = 0xFF,
};

class jqBatchGroup
{
public:
  union
  {
    struct
    {
      int QueuedBatchCount;
      int ExecutingBatchCount;
    };
    u64 BatchCount;
  };

  bool operator==( jqBatchGroup comperand )
  {
    if ( QueuedBatchCount == comperand.QueuedBatchCount && ExecutingBatchCount == comperand.ExecutingBatchCount &&
         BatchCount == comperand.BatchCount )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  jqBatchGroup();
};

class jqModule
{
public:
  const char *Name;
  jqWorkerType Type;
  jqModuleCallback Code;
  jqBatchGroup Group;

  jqModule( const char *Name, jqWorkerType Type, jqModuleCallback Code, jqBatchGroup Group );
};

class _jqBatch
{
};

class ALIGN( 4 ) jqBatch
{
public:
  void *p3x_info;
  void *Input;
  void *Output;
  jqModule *Module;
  jqBatchGroup *GroupID;
  void *ConditionalAddress;
  unsigned int ConditionalValue;
  unsigned int ParamData[23];
  _jqBatch _Batch;

  jqBatch();
};

template <typename T, unsigned int I>
class ALIGN( 8 ) jqAtomicQueue
{
public:
  class NodeType
  {
  public:
    NodeType *Next;
    jqBatch Data;
  };
  struct NodeBlockEntry
  {
    void *Addr;
    NodeBlockEntry *Next;
  };

  NodeType **FreeListPtr;
  NodeType *_FreeList;
  NodeBlockEntry *NodeBlockListHead;
  NodeType *Head;
  NodeType *Tail;
  tlSharedAtomicMutex FreeLock;
  tlAtomicMutex HeadLock;
  tlAtomicMutex TailLock;
  jqAtomicQueue<T, I> *ThisPtr;

  void AllocateNodeBlock( int nodeCount )
  {
    int blockSizeBytes;
    NodeType *nodeBlock;
    NodeType *node;
    int i;
    NodeType **blockEntry;

    blockSizeBytes = nodeCount << 7;
    nodeBlock = (NodeType *)tlMemAlloc( blockSizeBytes + sizeof( NodeBlockEntry ), 4u, 0 );
    if ( nodeCount - 1 > 0 )
    {
      node = nodeBlock;
      i = nodeCount - 1;
      do
      {
        --i;
        node->Next = node + 1;
        ++node;
      }
      while ( i );
    }
    blockEntry = (NodeType **)( (char *)&nodeBlock->Next + blockSizeBytes );
    *( blockEntry - 32 ) = 0;
    *blockEntry = nodeBlock;
    blockEntry[1] = (NodeType *)NodeBlockListHead;
    NodeBlockListHead = (NodeBlockEntry *)( (char *)nodeBlock + blockSizeBytes );
    *FreeListPtr = nodeBlock;
  }

  NodeType *AllocateNode()
  {
    // Acquire the free list lock to ensure thread safety
    FreeLock.Lock();

    // Get current free list pointer and the first available node
    NodeType *availableNode = *FreeListPtr;

    // If no nodes are available, allocate a new block
    if ( !availableNode )
    {
      // Create a new block with 32 nodes
      AllocateNodeBlock( 32 );

      // Get the updated free list pointer and first available node
      availableNode = *FreeListPtr;
    }

    // Update free list to point to the next available node
    *FreeListPtr = availableNode->Next;

    // Handle mutex unlocking
    tlSharedAtomicMutex *lockPtr = FreeLock.ThisPtr;
    if ( lockPtr->LockCount-- == 1 )
    {
      // Reset thread ID when the lock is fully released
      LONG unusedTarget = 0;
      InterlockedExchange( &unusedTarget, 0 );
      lockPtr->ThreadId = 0;
    }

    // Return the allocated node
    return availableNode;
  }

  void Init( jqAtomicQueue<T, I> *SharedFreeList )
  {
    NodeType *Node;

    ThisPtr = this;
    _FreeList = 0;
    if ( SharedFreeList )
    {
      FreeListPtr = SharedFreeList->FreeListPtr;
      FreeLock.ThisPtr = &SharedFreeList->FreeLock;
    }
    else
    {
      FreeListPtr = &_FreeList;
      FreeLock.ThisPtr = &FreeLock;
    }
    FreeLock.ThreadId = 0;
    FreeLock.LockCount = 0;
    NodeBlockListHead = 0;
    HeadLock.ThisPtr = &HeadLock;
    HeadLock.ThreadId = 0;
    HeadLock.LockCount = 0;
    TailLock.ThisPtr = &TailLock;
    TailLock.ThreadId = 0;
    TailLock.LockCount = 0;
    Node = AllocateNode();
    Node->Next = 0;
    Tail = Node;
    Head = Node;
  }
  void Push( const jqBatch *Data )
  {
    NodeType *Node;
    LONG Target;

    Node = AllocateNode();
    memcpy( &Node->Data, Data, sizeof( Node->Data ) );
    Node->Next = 0;

    TailLock.Lock();
    ThisPtr->Tail->Next = Node;
    ThisPtr->Tail = Node;
    if ( TailLock.LockCount-- == 1 )
    {
      Target = 0;
      InterlockedExchange( &Target, 0 );
      TailLock.ThreadId = 0;
    }
  }
  bool Pop( jqBatch *p )
  {
    NodeType *Node;
    NodeType *Next;
    LONG Target;

    HeadLock.Lock();
    Next = ThisPtr->Head->Next;
    Node = ThisPtr->Head;

    if ( Next )
    {
      memcpy( p, &Next->Data, sizeof( jqBatch ) );
      ThisPtr->Head = Next;
      if ( HeadLock.LockCount-- == 1 )
      {
        Target = 0;
        InterlockedExchange( &Target, 0 );
        HeadLock.ThreadId = 0;
      }

      FreeLock.Lock();
      Node->Next = *FreeListPtr;
      *FreeListPtr = Node;
      if ( FreeLock.ThisPtr->LockCount-- == 1 )
      {
        Node = 0;
        InterlockedExchange( (volatile LONG *)&Node, 0 );
        FreeLock.ThisPtr->ThreadId = 0;
      }
      return true;
    }
    else
    {
      if ( HeadLock.LockCount-- == 1 )
      {
        Target = 0;
        InterlockedExchange( &Target, 0 );
        HeadLock.ThreadId = 0;
      }
      return false;
    }
  }
};

class jqAtomicHeap
{
public:
  struct LevelInfo
  {
    unsigned int BlockSize;
    int NBlocks;
    int NCells;
    u64 *CellAvailable;
    u64 *CellAllocated;
  };

  jqAtomicHeap *ThisPtr;
  tlAtomicMutex Mutex;
  char *HeapBase;
  unsigned int HeapSize;
  unsigned int BlockSize;
  volatile unsigned int TotalUsed;
  volatile unsigned int TotalBlocks;
  int NLevels;
  LevelInfo Levels[11];
  unsigned char *LevelData;

  inline int BlockCell( int FitSlot ) { return FitSlot / 64; }
  inline u64 BlockBit( int FitSlot ) { return ~( 1i64 << ( FitSlot & 0x3F ) ); }
  bool GetAvailableBlock( LevelInfo *FitLevel, int *FitSlot );
  bool AllocBlock( LevelInfo **FitLevel, int *FitSlot );
  int SplitBlock( LevelInfo *Level, int Slot, LevelInfo *LevelTo );
  char *AllocLevel( int LevelIdx );
  int FindLevelForSize( unsigned int Size );
  char *Alloc( unsigned int Size, unsigned int Align );
  void FindAllocatedBlock( unsigned int Offset, LevelInfo **FitLevel, int *FitSlot );
  void MergeBlocks( LevelInfo **FitLevel, int *FitSlot );
  void Free( void *Ptr );
  ~jqAtomicHeap();
  void Init( void *_HeapBase, unsigned int _HeapSize, unsigned int _BlockSize );
};

class jqMemBaseMarker
{
public:
  void *MemBaseRestore;
};

class jqQueue
{
public:
  jqQueue *ThisPtr;
  jqAtomicQueue<jqBatch, 32> Queue;
  int QueuedBatchCount;
  unsigned int ProcessorsMask;
  ~jqQueue();
};

class jqBatchPool
{
public:
  jqBatchPool *ThisPtr;
  jqQueue BaseQueue;
  jqBatchGroup GroupID;
  jqAtomicHeap BatchDataHeap;

  ~jqBatchPool();
};

struct ALIGN( 4 ) _jqWorker
{
  jqWorkerType Type;
  void *Thread;
  unsigned int ThreadId;
  bool Idle;
};

class jqWorker : public _jqWorker
{
public:
  jqWorker *ThisPtr;
  int Processor;
  int WorkerID;
  int NumQueues;
  jqQueue WorkerSpecific;
  jqQueue *Queues[8];
  unsigned __int64 WorkTime;
};

struct ALIGN(8) jqWorkerCmd
{
  jqModule *module;
  u32 dataSize;
  volatile int ppu_fence;
  volatile int spu_fence;
  volatile int *spuThreadLimit;
  jqQueue *queue;
  u32 string_table;
};

extern jqWorker *jqWorkers;

void jqAttachQueueToWorkers( jqQueue *Queue, unsigned int ProcessorMask );
void jqEnableWorkers( unsigned int ProcessorsMask );
int jqGetNumWorkers();
unsigned __int64 jqGetCurrentThreadID();
unsigned __int64 jqGetMainThreadID();
jqBatchPool *jqGetPool();
jqBatch *jqGetCurrentBatch();
jqWorker *jqGetCurrentWorker();
jqQueue *jqGetWorkerQueue( int worker );
void jqShutdownWorker();
int jqGetQueuedBatchCount( jqBatchGroup *GroupID );
int jqGetExecutingBatchCount( jqBatchGroup *GroupID );
jqWorker *jqFindWorkerForProcessor( jqProcessor Processor );
jqBoolean jqPoll( jqBatchGroup *GroupID );
bool jqAreJobsQueued( jqBatchGroup *GroupID );
void jqSetWorkerInitFunction( void ( *fn )( int ) );
void jqLetWorkersSleep();
char *jqAllocBatchData( unsigned int Size );
void jqFreeBatchData( void *Ptr );
unsigned int jqGetBatchDataAvailable();
int jqExecuteBatch( jqWorker *Worker, jqBatch *Batch );
bool jqCanBatchExecute();
jqBoolean jqWorkerSleep( jqWorker *Worker );
void jqSetCheckContext( const char *desc );
void jqCheckDMALS( const void *addr );
void jqCheckDMAMain( const void *addr );
void jqCheckDMASize( unsigned int size );
void jqCheckDMATag( int tag );
void jqCheckRange( int val, int mn, int mx );
void jqCheckStack();
void *jqFetch( void *dest, const void *src, unsigned int size );
void jqStore( void *dest, const void *src, unsigned int size );
void *jqFetchAsync( void *dest, const void *src, unsigned int size );
void jqStoreAsync( void *dest, const void *src, unsigned int size );
void jqWait();
void jqWaitMultiple();
void jqSetMemBase();
void jqSetStackSize();
int jqGetMemAvailable();
void *jqAlloc();
void *jqGetMemBase();

void _jqInit();
void _jqShutdown();
void _jqStart();
void _jqStop();
void _jqAddBatch();

void jqAlertWorkers();
void jqLockBatchPoolInternal();
void jqUnlockBatchPoolInternal();
void jqKeepWorkersAwake();
void jqLockBatchPool();
void jqUnlockBatchPool();
void jqSetBatchDataHeapSize( unsigned int Size, unsigned int BlockSize );
void jqInit();
void jqInitQueue( jqQueue *Queue );
void jqInitWorker( jqWorker *Worker );
void jqAddBatchToQueue( const jqBatch *Batch, jqQueue *Queue );
void jqAddBatch( const jqBatch *Batch, jqQueue *Queue );
void jqAddBatch( const jqModule *Module, void *Input, void *Output, jqBatchGroup *GroupID, jqQueue *Queue, void *ParamData, int ParamSize );
void jqSkipBatch();
bool jqPopNextBatchFromQueue( jqWorker *Worker, jqQueue *Queue, jqBatchGroup *GroupID, jqBatch *PoppedBatch );
bool jqPopNextBatch( jqWorker *Worker, bool *doHighPriority, jqBatchGroup *GroupID, jqBatch *PoppedBatch );
void jqWorkerLoop( jqWorker *Worker, jqBatchGroup *GroupID, bool BreakWhenEmpty, unsigned __int64 *batchCount );
void jqTempWorkerLoop( jqWorker *Worker, jqBatchGroup *GroupID, bool( __cdecl *callback )( void * ), void *context );
unsigned int jqWorkerThread( void *_this );
void jqFlush( jqBatchGroup *GroupID, unsigned __int64 batchCount );
void jqStop();
void jqAssistWithBatches( bool( __cdecl *callback )( void * ), void *context, jqBatchGroup *GroupID );
void jqShutdown();
void jqStart();

inline u64 jqGet( u64 *Cell )
{
  return *Cell;
}
