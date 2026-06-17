/*
The MIT License (MIT)

Copyright (c) 2024-2026, Jacco Bikker / Breda University of Applied Sciences.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
	tiny_bvh_threadpool.h - default parallel build backend for tiny_bvh.h.

	Implements BVHContext's spawn / barrier / parallel_for hooks on the bundled
	job system. Included automatically from the TINYBVH_IMPLEMENTATION translation
	unit unless TINYBVH_NO_BUILTIN_POOL is set.
*/

#pragma once

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace tinybvh {

// Wicked job system, condensed. https://github.com/turanszkij/WickedEngine
// Removed: Thread priority, Dispatch, graceful shutdown; not needed in TinyBVH.
// A task is a raw function pointer plus an inline payload copy: pool-agnostic, with
// no per-task allocation.
class JobSystem
{
public:
	JobSystem() { Initialize(); }
	~JobSystem()
	{
		if (res.numThreads == 0) return;
		res.alive.store( false );
		bool wake_loop = true;
		std::thread waker( [&] { while (wake_loop) res.sleepingCondition.notify_all(); } );
		for (auto& thread : res.threads) thread.join();
		wake_loop = false;
		waker.join();
		res.jobQueue.reset();
		res.threads.clear();
		res.numThreads = 0;
	}
	struct context { std::atomic<uint32_t> counter{ 0 }; } ctx;
	static const uint32_t JOB_PAYLOAD_MAX = 64; // builder payloads are <= 32B, + spawn header
	struct Job
	{
		void (*fn)(void*) = nullptr;
		alignas( 16 ) uint8_t payload[JOB_PAYLOAD_MAX];	// inline copy; scheduling never allocates
		inline uint32_t execute( context& ctx )
		{
			fn( payload );
			return ctx.counter.fetch_sub( 1 );
		}
	};
	struct JobQueue
	{
		std::deque<Job> queue;
		std::mutex locker;
		inline void push_back( const Job& item ) { std::scoped_lock lock( locker ); queue.push_back( item ); }
		inline bool pop_front( Job& item )
		{
			std::scoped_lock lock( locker );
			if (queue.empty()) return false; else item = std::move( queue.front() );
			queue.pop_front();
			return true;
		}
	};
	struct Resources
	{
		uint32_t numThreads = 0;
		std::vector<std::thread> threads;
		std::unique_ptr<JobQueue[]> jobQueue;
		std::atomic<uint32_t> nextQueue{ 0 };
		std::condition_variable sleepingCondition;
		std::mutex sleepingMutex;
		std::condition_variable waitingCondition;
		std::mutex waitingMutex;
		std::atomic_bool alive{ true };
		inline void work( context& ctx, uint32_t startingQueue )
		{
			Job job;
			for (uint32_t i = 0; i < numThreads; ++i) while (jobQueue[startingQueue++ % numThreads].pop_front( job ))
				if (job.execute( ctx ) == 1) { std::unique_lock<std::mutex> lock( waitingMutex ); waitingCondition.notify_all(); }
		}
	} res;
	void Initialize()
	{
		res.numThreads = std::thread::hardware_concurrency();
		res.jobQueue.reset( new JobQueue[res.numThreads] );
		res.threads.reserve( res.numThreads );
		context& c = ctx;
		Resources& r = res;
		for (uint32_t threadID = 0; threadID < res.numThreads; threadID++)
			res.threads.emplace_back( [&c, threadID, &r]
				{	while (r.alive.load()) {
			r.work( c, threadID );
			std::unique_lock<std::mutex> lock( r.sleepingMutex );
			r.sleepingCondition.wait( lock );
		} } );
	}
	void Execute( void (*fn)(void*), const void* payload, uint32_t size )
	{
		assert( size <= JOB_PAYLOAD_MAX );
		ctx.counter.fetch_add( 1 );
		Job job;
		job.fn = fn;
		memcpy( job.payload, payload, size );
		res.jobQueue[res.nextQueue.fetch_add( 1 ) % res.numThreads].push_back( job );
		res.sleepingCondition.notify_one();
	}
	void Wait()
	{
		if (!IsBusy()) return;
		res.sleepingCondition.notify_all();
		res.work( ctx, res.nextQueue.fetch_add( 1 ) % res.numThreads );
		while (IsBusy())
		{
			std::unique_lock<std::mutex> lock( res.waitingMutex );
			if (IsBusy()) res.waitingCondition.wait( lock, [this] { return !IsBusy(); } );
		}
	}
	bool IsBusy() { return ctx.counter.load() > 0; }
};

// A build uses two job systems: 'subtree' for recursive fork/join (spawn/barrier)
// and 'binning' for the AVX builder's scoped leaf fan-out (parallel_for). One pair
// per host thread; 'binning' is created on demand (the scalar builders never bin).
struct PoolPair
{
	JobSystem subtree;
	std::unique_ptr<JobSystem> binningJobs;
	std::mutex binningMutex;
	JobSystem& binning()
	{
		std::scoped_lock lock( binningMutex );
		if (!binningJobs) binningJobs = std::make_unique<JobSystem>();
		return *binningJobs;
	}
};

// One pair per host thread, owned by that thread and freed (its threads joined) at thread exit.
static thread_local PoolPair* tinybvh_tl_pair = nullptr;
static PoolPair* tinybvh_pool_pair()
{
	static thread_local std::unique_ptr<PoolPair> owned;
	if (!tinybvh_tl_pair) tinybvh_tl_pair = (owned = std::make_unique<PoolPair>()).get();
	return tinybvh_tl_pair;
}

// Spawned tasks run [pair][fn] ahead of the payload: the worker adopts 'pair' so its
// nested spawns reuse this host's pools. Keeps the pool/TLS logic out of JobSystem.
struct BVHSpawnEnvelope { PoolPair* pair; void (*fn)(void*); };
static void tinybvh_spawn_task( void* blob )
{
	BVHSpawnEnvelope* e = (BVHSpawnEnvelope*)blob;
	tinybvh_tl_pair = e->pair;
	e->fn( (uint8_t*)blob + sizeof( BVHSpawnEnvelope ) );
}

// Default BVHContext hooks (declared in tiny_bvh.h); userdata is unused.
void tinybvh_builtin_spawn( void (*fn)(void*), const void* payload, uint32_t payload_size, void* )
{
	// pack [pair][fn][payload]; Execute copies it inline (the caller's frame may unwind).
	PoolPair* pair = tinybvh_pool_pair();
	alignas( 16 ) uint8_t blob[JobSystem::JOB_PAYLOAD_MAX];
	BVHSpawnEnvelope* e = (BVHSpawnEnvelope*)blob;
	e->pair = pair, e->fn = fn;
	memcpy( blob + sizeof( BVHSpawnEnvelope ), payload, payload_size );
	pair->subtree.Execute( &tinybvh_spawn_task, blob, sizeof( BVHSpawnEnvelope ) + payload_size );
}
void tinybvh_builtin_barrier( void* )
{
	tinybvh_pool_pair()->subtree.Wait();
}
struct BVHParallelForArgs { void (*fn)(uint32_t, void*); uint32_t index; void* payload; };
static void tinybvh_parallel_for_task( void* payload )
{
	BVHParallelForArgs* a = (BVHParallelForArgs*)payload;
	a->fn( a->index, a->payload );
}
void tinybvh_builtin_parallel_for( uint32_t n, void (*fn)(uint32_t, void*), void* payload, void* )
{
	if (n == 0) return;
	JobSystem& jobs = tinybvh_pool_pair()->binning();
	for (uint32_t i = 0; i < n; i++)
	{
		BVHParallelForArgs a = { fn, i, payload };
		jobs.Execute( &tinybvh_parallel_for_task, &a, sizeof( a ) );
	}
	jobs.Wait();
}

} // namespace tinybvh
