#include <boost/ptr_container/ptr_map.hpp>
#include <boost/thread.hpp>

namespace indoor_context {
	template <typename T>
	class ThreadLocal {
	public:
		boost::ptr_map<boost::thread::id, T> instances;
		boost::mutex get_mutex;

		// Get object for the current thread or construct one
		T& Get(const boost::thread::id& id) {
			// The lock is important as the ptr_map will have internal race
			// conditions (it might need to construct a new object if one
			// doesn't already exist for this thread).
			boost::mutex::scoped_lock lock(get_mutex);
			return instances[id];
		}

		// Get object for the current thread or construct one
		const T& GetConst(const boost::thread::id& id) const {
			return const_cast<ThreadLocal<T>*>(this)->Get(id);
		}

		// Get the object for this thread or construct one
		T& Get() { return Get(this_thread::get_id()); }
		const T& GetConst() const { return GetConst(this_thread::get_id()); }

		// Shortcut for Get
		T& operator*() { return Get(); }
		const T& operator*() const { return Get(); }

		// Shortcut for Get
		T* operator->() { return &Get(); }
		const T* operator->() const { return &Get(); }
	};
}
