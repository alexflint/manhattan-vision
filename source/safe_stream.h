#pragma once

#include <fstream>

namespace indoor_context {

	// sifstream and sofstream are identical to ifstream and ofstream
	// except that exceptions are turned on automatically. If exceptions
	// are off then it is possible, for example, for an ofstream to fail
	// and then silently discard everything from then on.
	class sifstream : public ifstream {
	public:
		inline sifstream(const char* filename) : ifstream(filename) {
			exceptions(badbit);
		}
		inline sifstream(const string& filename) : ifstream(filename.c_str()) {
			exceptions(badbit);
		}
	};

	class sofstream : public ofstream {
	public:
		inline sofstream(const char* filename) : ofstream(filename) {
			exceptions(failbit | badbit);
		}
		inline sofstream(const string& filename) : ofstream(filename.c_str()) {
			exceptions(failbit | badbit);
		}
	};

}
