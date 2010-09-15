#pragma once

#include "common_types.h"

namespace indoor_context {

// Initialize Gvars
void InitVars();
// Initialize Gvars
void InitVars(const string& config_file);
// Initialize Gvars
void InitVars(int argc, char **argv);
// Reload the current config file
void ReloadVars();

}
