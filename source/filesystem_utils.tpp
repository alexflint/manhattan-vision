#include <boost/filesystem.hpp>

namespace indoor_context {
	// Convenience function to create a range representing the entries in a directory.
	// Use with BOOST_FOREACH:
	// 
	// fs::path someDir = "/path/to/foo"
	// BOOST_FOREACH(const fs::path& entry, directory_contents(someDir)) {
	//    ... do something with entry ...
	// }
	boost::iterator_range<fs::directory_iterator> directory_contents(const fs::path& path) {
		return boost::iterator_range<fs::directory_iterator>(fs::directory_iterator(path),
																												 fs::directory_iterator());
	}
}
