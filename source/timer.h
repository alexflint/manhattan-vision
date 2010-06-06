#pragma once

#include "common_types.h"

#include <VW/Utils/timer.h>

// This macro is used like this:
//
// code: TIMED("my function") foo()
// output: my function time: XX.XXms
//
// code:
// TIMED("some block") {
//   foo();
//   bar();
//   baz();
// }
// output:
// some block: XX.XXms
//
// code:
// TIMED("the loop") for(int i = 0; i < n; i++) {
//   ...
// }
// output:
// the loop: XX.XXms

#define TIMED(str) if (scoped_timer __t = str)

namespace indoor_context {
	// Starts a timer on construction and reports its value on destruction
	struct scoped_timer {
		const string str;
		VW::Timer timer;
		inline scoped_timer(const char* s = "") : str(s) { }
		inline ~scoped_timer() {
			DLOG << str << ": " << timer << endl;
		}
		inline operator bool() {
			return true;
		}
	};
}
