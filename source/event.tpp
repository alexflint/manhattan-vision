#pragma once

#include <boost/function.hpp>

namespace indoor_context {

	// A generic event holding a set of function pointers.
	template <typename F=void()>
	class Event {
	public:
		typedef boost::function<F> Listener;
		typedef typename vector<Listener>::iterator ListenerHandle;

		// True if there are no listeners_
		inline bool empty() const {
			return listeners_.empty();
		}

		// Add a listener, return a handle with which it can later be
		// removed
		ListenerHandle add(const Listener& listener) {
			listeners_.push_back(listener);
			return listeners_.end()-1;
		}

		// Remove a listener
		void remove(const ListenerHandle& handle) {
			listeners_.erase(handle);
		}

		// Fire this event (invoke all listeners_)
		void fire() {
			for (int i = 0; i < listeners_.size(); i++) {
				listeners_[i]();
			}
		}

		// Fire this event (invoke all listeners_)
		template <typename T1>
		void fire(const T1& x1) {
			for (int i = 0; i < listeners_.size(); i++) {
				listeners_[i](x1);
			}
		}

		// Fire this event (invoke all listeners_)
		template <typename T1, typename T2>
		void fire(const T1& x1, const T2& x2) {
			for (int i = 0; i < listeners_.size(); i++) {
				listeners_[i](x1, x2);
			}
		}

		// Fire this event (invoke all listeners_)
		template <typename T1, typename T2, typename T3>
		void fire(const T1& x1, const T2& x2, const T3& x3) {
			for (int i = 0; i < listeners_.size(); i++) {
				listeners_[i](x1, x2, x3);
			}
		}

		// Fire this event (invoke all listeners_)
		template <typename T1, typename T2, typename T3, typename T4>
		void fire(const T1& x1, const T2& x2, const T3& x3, const T4& x4) {
			for (int i = 0; i < listeners_.size(); i++) {
				listeners_[i](x1, x2, x3, x4);
			}
		}

	private:
		vector<Listener> listeners_;
	};
}
