// Provides a convenient set of includes for files with a main()
// function (i.e. entry points for binaries). We avoid including all of these features in .cpp
// files because they contain complicated templates that increase
// compilation time, but including them in entry-point files is okay because they
// will only be included once per binary.

#pragma once

#include <fstream>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "common_types.h"
#include "vars.h"

#include "math_utils.tpp"
#include "vector_utils.tpp"

using namespace indoor_context;
using namespace toon;

