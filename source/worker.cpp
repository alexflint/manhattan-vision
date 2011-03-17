#include <boost/thread.hpp>

#include "worker.h"
#include "common_types.h"

#include "concurrent_queue.tpp"
#include "functional_types.tpp"

namespace indoor_context {
	using boost::mutex;
	using boost::thread;
	using boost::barrier;

	Worker::Worker()
		: consumer_thread(bind(&Worker::ConsumeLoop, ref(*this))),
			join_barrier(2),
			alive(true) {
	}

	Worker::~Worker() {
		mutex::scoped_lock lock(alive_mutex);
		assert(!alive);
	}

	void Worker::ConsumeLoop() {
		while (job* cur = work_queue.wait_and_pop()) {
			cur->Run();
			delete cur;
		}
	}

	void Worker::Wait() {
		Add(bind(&barrier::wait, ref(join_barrier)));  // will assert(alive)
		join_barrier.wait();
	}

	void Worker::Join() {
		mutex::scoped_lock lock(alive_mutex);
		assert(alive);
		alive = false;
		work_queue.push(NULL);
		lock.unlock();

		consumer_thread.join();
	}

	// static
	int Worker::DefaultNumThreads() {
		return boost::thread::hardware_concurrency();
	}
}
