#include "filesystem_utils.h"

#include <wordexp.h>
#include <boost/filesystem.hpp>

namespace indoor_context {
	// Convenience function to create a range representing the entries in a directory.
	// Use with BOOST_FOREACH:
	// 
	// fs::path someDir = "/path/to/foo"
	// BOOST_FOREACH(const fs::path& entry, directory_contents(someDir)) {
	//    ... do something with entry ...
	// }
	dir_it directory_contents(const fs::path& path) {
		return dir_it(fs::directory_iterator(path),
									fs::directory_iterator());
	}

	string expand_path(const string& path) {
		wordexp_t exp_result;
		wordexp(path.c_str(), &exp_result, 0);

		CHECK_GT(exp_result.we_wordc, 0);
		if (exp_result.we_wordc > 1) {
			DLOG << "WARNING: path expansion of " << path << " yielded multiple paths, "
					 << "ignoring all but the first.";
		}

		string expanded = exp_result.we_wordv[0];
		wordfree(&exp_result);
		return expanded;
	}
}
