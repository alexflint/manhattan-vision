#pragma once

#include <queue>
#include <boost/thread.hpp>

namespace indoor_context {

template <typename T>
class concurrent_queue {
private:
	std::queue<T> the_queue;
	mutable boost::mutex the_mutex;
	boost::condition_variable the_condition_variable;
public:
	void push(T const& data) {
		boost::mutex::scoped_lock lock(the_mutex);
		the_queue.push(data);
		lock.unlock();
		the_condition_variable.notify_one();
	}

	bool empty() const {
		boost::mutex::scoped_lock lock(the_mutex);
		return the_queue.empty();
	}

	bool try_pop(T& popped_value) {
		boost::mutex::scoped_lock lock(the_mutex);
		if(the_queue.empty()) {
			return false;
		}
        
		popped_value=the_queue.front();
		the_queue.pop();
		return true;
	}

	void wait_and_pop(T& popped_value) {
		boost::mutex::scoped_lock lock(the_mutex);
		while(the_queue.empty()) {
			the_condition_variable.wait(lock);
		}
        
		popped_value=the_queue.front();
		the_queue.pop();
	}

	T wait_and_pop() {
		T popped_value;
		wait_and_pop(popped_value);
		return popped_value;
	}
};

}
