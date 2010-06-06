#include "vars.h"
#include "common_types.h"

namespace indoor_context {
	string config_file;

	void InitVars(int argc,
								char **argv,
								const char* default_config_file) {
		// Parse the arguments first
		GVars3::GUI.parseArguments(argc, argv);
		// Setup a var so that we can specify the config file with -config
		GVars3::gvar3<string> gvConfigFile("config", default_config_file, GVars3::SILENT);
		config_file = *gvConfigFile;
		// Now read the config file
		GVars3::GUI.LoadFile(config_file);
	}

	void ReloadVars() {
		GVars3::GUI.LoadFile(config_file);	
	}
}
