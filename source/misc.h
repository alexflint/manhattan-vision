#pragma once

#include <string>
#include <iterator>

#include "common_types.h"

// Like BOOST_FOREACH but also keeps track of the index within the
// array. Use it like this:
// COUNTED_FOREACH(int i, Widget& w, my_widget_container) {
//   w.setSomething(i);
///  if (i == bar) ...
//   ...
// }
#define COUNTED_FOREACH(counter, var, container)		\
		if (int _i = 0 && false) {} else				\
		BOOST_FOREACH(var, container)					\
		if (bool _flag = false) {} else					\
		for (counter = _i++; !_flag; _flag = true)

namespace indoor_context {

// Represents an output iterator that discards its argument
struct null_output_iterator : std::iterator<std::output_iterator_tag,
        null_output_iterator> {
	template<typename T> void operator=(T const&) {
	}
	null_output_iterator& operator*() {
		return *this;
	}
	null_output_iterator& operator++() {
		return *this;
	}
	null_output_iterator& operator++(int) {
		return *this;
	}
};

// Returns true if the string NEEDLE is present in HAYSTACK
bool StringContains(const string& haystack, const string& needle);

}
