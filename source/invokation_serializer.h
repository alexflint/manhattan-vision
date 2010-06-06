#pragma once

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include "concurrent_queue.tpp"
#include "common_types.h"

namespace indoor_context {
	// Facilitates serializing a bunch of function calls from different
	// threads into a single thread of execution. Calls can also be made
	// from the thread in which they will be invoked.
	class InvokationSerializer {
		// The type of jobs we will execute
		typedef boost::function<void()> Closure;
		// Thread-safe queue (don't use shared_ptr due to threads)
		concurrent_queue<Closure*> work_queue;
		// The thread in which process is called
		thread::id home_thread;
	public:
		// Return true iff the current thread is the home thread
		bool InHomeThread();
		// Make the specified thread the home thread (in which Process() will be called)
		void RegisterHomeThread(thread::id home_thread);
		// Process any outstanding invokations. This must be called from
		// the home thread.
		void Process();

		// Invoke f in the home thread
		template <typename Function>
		void Invoke(const Function& f) {
			if (InHomeThread()) {
				f();
			} else {
				work_queue.push(new Closure(f));
			}
		}

		// Invoke f in the home thread. Put f onto the work queue even
		// when called from the home thread.
		template <typename Function>
		void InvokeLater(const Function& f) {
			work_queue.push(new Closure(f));
		}
	};
}
