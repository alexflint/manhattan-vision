#pragma once

#include <cstdio>
#include <fstream>

namespace indoor_context {
	// sifstream and sofstream are identical to ifstream and ofstream
	// except that exceptions are turned on automatically. If exceptions
	// are off then it is possible, for example, for an ofstream to fail
	// and then silently discard everything from then on.
	class sifstream : public std::ifstream {
	public:
		inline sifstream(const char* filename) : std::ifstream(filename) {
			exceptions(badbit);
		}
		inline sifstream(const string& filename) : std::ifstream(filename.c_str()) {
			exceptions(badbit);
		}
	};

	class sofstream : public std::ofstream {
	public:
		inline sofstream(const char* filename) : std::ofstream(filename) {
			exceptions(failbit | badbit);
		}
		inline sofstream(const string& filename) : std::ofstream(filename.c_str()) {
			exceptions(failbit | badbit);
		}
	};
}
