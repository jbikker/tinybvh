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
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace tinybvh {

// Wicked job system, condensed. https://github.com/turanszkij/WickedEngine
// Removed: Thread priority, Dispatch, graceful shutdown; not needed in TinyBVH.
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
	struct Job
	{
		std::function<void()> task;
		inline uint32_t execute( context& ctx ) { task(); return ctx.counter.fetch_sub( 1 ); }
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
	void Execute( const std::function<void()>& task )
	{
		ctx.counter.fetch_add( 1 );
		Job job;
		job.task = task;
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
	JobSystem* binningJobs = nullptr;
	std::mutex binningMutex;
	JobSystem& binning()
	{
		std::scoped_lock lock( binningMutex );
		if (!binningJobs) binningJobs = new JobSystem();
		return *binningJobs;
	}
};

// Pools are per host thread and intentionally leaked (they outlive every build),
// matching the thread-local job systems the build kernels used to allocate.
static thread_local PoolPair* tinybvh_tl_pair = nullptr;
static PoolPair* tinybvh_pool_pair()
{
	if (!tinybvh_tl_pair) tinybvh_tl_pair = new PoolPair();
	return tinybvh_tl_pair;
}

// Default BVHContext hooks (declared in tiny_bvh.h); userdata is unused.
void tinybvh_builtin_spawn( void (*fn)(void*), const void* payload, uint32_t payload_size, void* )
{
	PoolPair* pair = tinybvh_pool_pair();
	// copy the payload: the caller's stack frame may unwind before fn runs.
	std::vector<uint8_t> buf( (const uint8_t*)payload, (const uint8_t*)payload + payload_size );
	pair->subtree.Execute( [pair, fn, b = std::move( buf )]() mutable {
		tinybvh_tl_pair = pair; // a worker inherits the spawning host's pools
		fn( b.data() );
	} );
}
void tinybvh_builtin_barrier( void* )
{
	tinybvh_pool_pair()->subtree.Wait();
}
void tinybvh_builtin_parallel_for( uint32_t n, void (*fn)(uint32_t, void*), void* payload, void* )
{
	if (n == 0) return;
	JobSystem& jobs = tinybvh_pool_pair()->binning();
	for (uint32_t i = 0; i < n; i++) jobs.Execute( [fn, i, payload]() { fn( i, payload ); } );
	jobs.Wait();
}

} // namespace tinybvh
