#pragma once

#include <boost/thread.hpp>

#include "common_types.h"

namespace indoor_context {
	using GVars3::gvar3;
	using GVars3::gvar2;
	using GVars3::GV3;
	using GVars3::FATAL_IF_NOT_DEFINED;

	// Provides a compile-time mapping from a given type to the
	// corresponding default gvar value. These must be functions, not
	// constants, because they are accessed during static initialization.
	template <typename T>
	struct default_value_map {
		static inline T val() { return 0; }  // assume numeric types by default
	};
	template <>
	struct default_value_map<std::string> {
		static inline std::string val() { return ""; }
	};
	template <int N, typename T>
	struct default_value_map<toon::Vector<N,T> > {
		static inline toon::Vector<N,T> val() { return toon::Zeros; }
	};

	// Lazy layer on top of gvars. A little slower since we now have to
	// check whether we're initialized on every access. Not thread safe
	// (there is a race condition in EnsureInitialized).
	template <typename T>
	class lazyvar : public gvar3<T> {
		const string name;
		const T default_val;
		const int flags;
		mutable boost::mutex initialize_mutex;
	public:
		lazyvar(const string& name_)
			: name(name_),
				default_val(default_value_map<T>::val()),
				flags(FATAL_IF_NOT_DEFINED) { }

		lazyvar(const string& name_, const T& default_val_)
			: name(name_),
				default_val(default_val_),
				flags(FATAL_IF_NOT_DEFINED) { }

		inline T& operator*() {
			EnsureInitialized();
			return gvar2<T>::data->get();
		}
  
		inline const T & operator*() const {
			EnsureInitialized();
			return gvar2<T>::data->get();
		}
  
		inline T* operator->() {
			EnsureInitialized();
			return gvar2<T>::data->ptr();
		}

		inline const T * operator->() const {
			EnsureInitialized();
			return gvar2<T>::data->ptr();
		}

		inline void EnsureInitialized() const {
			boost::mutex::scoped_lock lock(initialize_mutex);
			if (gvar2<T>::data == NULL) {
				GV3::Register(*(gvar3<T>*)this, name, default_val, flags);
			}
		}
	};
}
