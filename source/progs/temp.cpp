#include <boost/filesystem.hpp>
#include "entrypoint_types.h"
#include "format_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	CHECK_EQ(argc, 2);
	fs::path dir = argv[1];

	BOOST_FOREACH(const fs::path& path, directory_contents(dir)) {
		string name = path.filename();
		if (name[8] == '_') {
			string new_name = fmt("frame0%s_orients.png", name.substr(5,3));
			fs::path new_path = path.parent_path() / new_name;
			fs::rename(path, new_path);
		}
	}

	return 0;
}
