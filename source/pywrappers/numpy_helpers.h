#include <boost/python/numeric.hpp>
#include <numpy/noprefix.h>

#include "common_types.h"

namespace indoor_context {
	namespace bpnum = boost::python::numeric;

	void InitializeNumpy();
	bpnum::array NewArray(int n, PyArray_TYPES t=PyArray_DOUBLE);
	bpnum::array NewArray(int n, int m, PyArray_TYPES t=PyArray_DOUBLE);
	bool IsArray(const PyObject* obj);
	bool IsNumericArray(const PyObject* obj);
	int GetArrayDimensions(const PyObject* obj);
	int GetArrayLength(const PyObject* obj, int dim);
	VecI GetArrayShape(const PyObject* obj);
}
