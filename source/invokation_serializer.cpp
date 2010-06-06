#include "invokation_serializer.h"

namespace indoor_context {
	// Return true iff the current thread is the home thread
	bool InvokationSerializer::InHomeThread() {
		return this_thread::get_id() == home_thread;
	}

	// Set the home thread to the current thread
	void InvokationSerializer::RegisterHomeThread(thread::id id) {
		static bool first_time = true;
		CHECK(first_time) << "RegisterHomeThread should only ever be called at most once.";
		first_time = false;
		home_thread = id;
	}

	// Process any outstanding invokations. This must be called from
	// the home thread.
	void InvokationSerializer::Process() {
		CHECK(InHomeThread());
		Closure* pf;
		while (work_queue.try_pop(pf)) {
			(*pf)();
			delete pf;
		}
	}
}
