#pragma once

#include <boost/thread.hpp>
#include "concurrent_queue.tpp"

namespace indoor_context {
	// Implements a producer/consumer thread pool
	// The jobs must all be added from a single thread.
	class thread_pool {
	public:
		typedef boost::function<void()> job;
		// Initialize with num_threads = # cores in CPU
		thread_pool();
		// Intialize with the specified number of threads
		thread_pool(int num_threads);
		// Add a job to the pool. Automatically initiates the threads if not already initiated.
		void add(const job& job);
		// Wait for all jobs to complete. The threads will _not_ terminate until this is called.
		void join();
		// Get the number of threads that will be added to the queue
		int concurrency() const { return concurrency_; }
	private:
		void consume_loop_();
		static void end_token_() {};  // a placeholder used to recognize the end of the jobs
		int concurrency_;
		concurrent_queue<job> jobs_;
		boost::thread_group threads_;
	};
}
