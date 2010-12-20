/**
 * This file is included automatically by a -prefix directive in
 * CMakeLists.txt. It includes all the boost files that are included
 * by *any* header file in the project. Since it will be included
 * first, the #ifnde... directives in the boost headers will cause
 * them to be skipped. This means that compiling boost is done
 * once only.
 */

#include <string>
#include <vector>
#include <utility>
#include <exception>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <boost/lexical_cast.hpp>

#include <boost/array.hpp>

#include <boost/format.hpp>
#include <boost/foreach.hpp>

#include <boost/ptr_container/ptr_vector.hpp>

#include <boost/thread.hpp>

#include <boost/function.hpp>
#include <boost/bind.hpp>

