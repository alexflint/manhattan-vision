// Testing interop between numpy and VNL,TooN

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/numeric.hpp>

#define PY_ARRAY_UNIQUE_SYMBOL foobarpy
#include <numpy/noprefix.h>

#include "entrypoint_types.h"
#include "python/numpy_conversions.h"
#include "python/numpy_helpers.h"

using namespace toon;
using namespace boost::python;
namespace bp=boost::python;

numeric::array makevec(int n) {
	return NewArray(n);
}

numeric::array makearray(int n, int m) {
	return NewArray(n, m);
}

MatD foo() {
	MatD v(2,3);
	v[0][0] = 0;
	v[0][1] = 10;
	v[0][2] = 20;
	v[1][0] = 30;
	v[1][1] = 40;
	v[1][2] = 50;
	return v;
}

Mat3 foob() {
	Mat3 m = Identity;
	return m;
}

VecI bar() {
	VecI v(7);
	v[0] = 0;
	v[1] = 118;
	return v;
}

Vector<-1> baz() {
	Vector<-1> v(2);
	v[0] = 5;
	v[1] = 6;
	return v;
}

VecD accept(const VecD& x) {
	VecD v = x;
	v[0] = x[0] * 2;
	return v;
}

struct VNLVectorFromPython {
	VNLVectorFromPython() {
		converter::registry::push_back(&convertible,
																	 &construct,
																	 type_id<VecD>());
	}

	static void* convertible(PyObject* obj) {
		if (!IsArray(obj)) {
			return NULL;
		} else if (GetArrayDimensions(obj) != 1) {
			return NULL; 
		} else {
			return obj;
		}
	}

	static void construct(PyObject* obj,
												converter::rvalue_from_python_stage1_data* data) {
		numeric::array a = bp::extract<numeric::array>(obj);
		int len = GetArrayLength(obj, 0);
		VecD* storage = (VecD*)((converter::rvalue_from_python_storage<VecD>*)data)->storage.bytes;
		new (storage) VecD(len);
		for (int i = 0; i < len; i++) {
			(*storage)[i] = extract<double>(a[i]);
		}
		data->convertible = storage;
	}
};

BOOST_PYTHON_MODULE(py_indoor_context) {
	numeric::array::set_module_and_type("numpy", "ndarray");

	InitializeNumpy();
	RegisterNumpyConversions();
	VNLVectorFromPython();

	def("makearray", makearray);
	def("makevec", makevec);
	def("foo", foo);
	def("foob", foob);
	def("bar", bar);
	def("baz", baz);
	def("accept", accept);
}
