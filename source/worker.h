#pragma once

#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/bind.hpp>

#include "concurrent_queue.tpp"
#include "common_types.h"

namespace indoor_context {

// Represents a single-producer single-consumer work queue.
// Note that due to the "finishing" flag this will NOT work in a
// multiple-producer situation
class Worker;
class Worker {
	// Untemplated base class for jobs
	struct job {
		virtual void Run() = 0;
	};

	// Templated job class
	template <typename Function>
	struct function_job : public job {
		Function f;
		function_job(Function& func) : f(func) { }
		virtual void Run() { f(); }
	};

	// Thread-safe queue (don't use shared_ptr due to threads)
	concurrent_queue<job*> work_queue;
	// The consumer thread
	boost::thread consumer_thread;
	// Used to wait for jobs to be completed
	boost::barrier join_barrier;
	// Flag indicating that Join() has not been called.
	bool alive;
	// Mutex for above
	boost::mutex alive_mutex;

	// The consumer loop. Note that we exit if we pop a NULL, which is a
	// special token sent from the producer thread.
	void ConsumeLoop();
	// Disable copy constructor
	Worker(Worker& other);
	public:
	// Constructor. Starts the work loop in a seperate thread
	Worker();
	// Destructor. Must be called after Join()
	~Worker();
	// Wait for all current jobs to finish. The internal thread remains
	// alive and more jobs can subsequently be added.
	void Wait();
	// Wait for all current jobs to finish and stop the work thread. No
	// further jobs can be added.
	void Join();
	// Add a job to the queue
	template <typename Function>
	void Add(Function f) {
		boost::mutex::scoped_lock lock(alive_mutex);
		assert(alive);
		work_queue.push(new function_job<Function>(f));
	}
	// Get the number of threads to use by default for parallel tasks
	static int DefaultNumThreads();
};



// Allocate N jobs to K threads and run in parallel
template <typename Function>
void ParallelPartition(int num_jobs,
                       int num_threads,
                       Function jobfunc) {
	boost::thread_group threads;
	for (int i = 0; i < num_threads; i++) {
		threads.create_thread(bind(jobfunc,
				i*num_jobs/num_threads,
				(i+1)*num_jobs/num_threads - 1));
	}
	threads.join_all();
}

// Allocate N jobs to K threads and run in parallel where K is
// min(N, Worker::DefaultNumThreads())
template <typename Function>
void ParallelPartition(int num_jobs, Function jobfunc) {
	int num_threads = min(num_jobs, Worker::DefaultNumThreads());
	ParallelPartition(num_jobs, num_threads, jobfunc);
}}
