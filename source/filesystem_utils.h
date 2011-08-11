#include "common_types.h"

#include <wordexp.h>

#include <boost/range/iterator_range.hpp>
#include <boost/filesystem.hpp>

namespace indoor_context {
	typedef boost::iterator_range<fs::directory_iterator> dir_it;

	// Convenience function to create a range representing the entries in a directory.
	// Use with BOOST_FOREACH:
	// 
	// fs::path someDir = "/path/to/foo"
	// BOOST_FOREACH(const fs::path& entry, directory_contents(someDir)) {
	//    ... do something with entry ...
	// }
	dir_it directory_contents(const fs::path& path);

	// Expand environment variables and leading ~ in paths.
	string expand_path(const string& path);
}
