#include <boost/bind.hpp>

#include "common_types.h"
#include "thread_pool.h"

namespace indoor_context {
	thread_pool::thread_pool() : concurrency_(boost::thread::hardware_concurrency()) {
	}

	thread_pool::thread_pool(int n) : concurrency_(n) {
	}

	void thread_pool::add(const job& job) {
		jobs_.push(job);
		if (threads_.size() == 0) {
			CHECK_GT(concurrency_, 0);
			for (int i = 0; i < concurrency_; i++) {
				// thread group owns the memory of the thread
				threads_.create_thread(boost::bind(&thread_pool::consume_loop_, this));
			}
		}
	}

	void thread_pool::join() {
		if (threads_.size() == 0) return;

		// Add an end token for each thread
		for (int i = 0; i < concurrency_; i++) {
			jobs_.push(&end_token_);
		}
		// Wait for the threads to complete processing
		threads_.join_all();
	}
		
	// private
	void thread_pool::consume_loop_() {
		job job;
		while (true) {
			jobs_.wait_and_pop(job);
			if (job == &end_token_) {
				break;
			}
			job();
		}
	}
}
