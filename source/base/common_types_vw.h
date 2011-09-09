#pragma once
// Provides the same includes and typedefs as common_types.h but also
// inserts all of VW and VNL into our namespace

#include "common_types.h"

namespace indoor_context {
	using namespace VNL;
	using namespace VW;
	using namespace GVars3;
}
