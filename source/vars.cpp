#include "vars.h"
#include "common_types.h"

namespace indoor_context {
	string config_file;
	string kDefaultConfigFile = "config/common.cfg";

	void InitVars() {
		config_file = kDefaultConfigFile;
		ReloadVars();
	}

	void InitVars(int argc, char **argv) {
		// Parse the arguments first
		GVars3::GUI.parseArguments(argc, argv);
		// Setup a var so that we can specify the config file with -config
		GVars3::gvar3<string> gvConfigFile("config", kDefaultConfigFile, GVars3::SILENT);
		config_file = *gvConfigFile;
		// Now read the config file
		ReloadVars();
	}

	void ReloadVars() {
		GVars3::GUI.LoadFile(config_file);	
	}
}
