#pragma once

namespace indoor_context {
	// Initialize Gvars
	void InitVars(int argc,
								char **argv,
								const char* default_config_file = "config/common.cfg");
	// Reload the current config file
	void ReloadVars();
}
