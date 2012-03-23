#include "numpy_helpers.h"

#define NO_IMPORT_ARRAY
#include <numpy/noprefix.h>

#include <boost/python/numeric.hpp>
#include <boost/python/extract.hpp>

#include "common_types.h"

namespace indoor_context {
	using namespace boost::python;

	void InitializeNumpy() {
		numeric::array::set_module_and_type("numpy", "ndarray");
		import_array();
	}

	bool IsArray(const PyObject* obj) {
		return PyArray_Check(obj);
	}

	bool IsNumericArray(const PyObject* obj) {
		return IsArray(obj) && PyArray_ISNUMBER(obj);
	}

	numeric::array NewArray(int n, PyArray_TYPES t) {
		PyObject* array = PyArray_FromDims(1, &n, t);
		handle<PyObject> foo(array);
		object obj(foo);
		return extract<numeric::array>(obj);
	}

	numeric::array NewArray(int n, int m, PyArray_TYPES t) {
		int dims[] = {n, m};
		PyObject* array = PyArray_FromDims(2, dims, t);
		handle<PyObject> foo(array);
		object obj(foo);
		return extract<numeric::array>(obj);
	}

	int GetArrayDimensions(const PyObject* obj) {
		return PyArray_NDIM(obj);
	}

	VecI GetArrayShape(const PyObject* obj) {
		int ndim = GetArrayDimensions(obj);
		npy_intp* shape = PyArray_DIMS(obj);
		VecI s(ndim);
		for (int i = 0; i < ndim; i++) {
			s[i] = shape[i];
		}
		return s;
	}

	int GetArrayLength(const PyObject* obj, int dim) {
		CHECK_LT(dim, GetArrayDimensions(obj));
		return PyArray_DIMS(obj)[dim];
	}
}
