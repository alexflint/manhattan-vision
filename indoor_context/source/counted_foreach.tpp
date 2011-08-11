#pragma once

#include "common_types.h"

#include <boost/foreach.hpp>

// Like BOOST_FOREACH but also keeps track of the index within the
// array. Use it like this:
// COUNTED_FOREACH(int i, Widget& w, my_widget_container) {
//   w.setSomething(i);
//   cout << "At iteration " << i << endl;
//   ...
// }
#define COUNTED_FOREACH(counter, var, container)		\
		if (int _i = 0 && false) {} else				\
		BOOST_FOREACH(var, container)					\
		if (bool _flag = false) {} else					\
		for (counter = _i++; !_flag; _flag = true)
