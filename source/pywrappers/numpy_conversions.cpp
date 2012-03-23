#include "numpy_conversions.h"

#include <boost/python.hpp>

#include <numpy/noprefix.h>

#include "entrypoint_types.h"
#include "pywrappers/numpy_helpers.h"

#include "matrix_traits.tpp"

#define MAP_NUMPY_TYPE(T,S)																							\
	template <> struct NumpyType<T> { static const PyArray_TYPES id = S; };

namespace indoor_context {
	using namespace boost::python;

	template <typename T> struct NumpyType {};
	MAP_NUMPY_TYPE(int, PyArray_INT)
	MAP_NUMPY_TYPE(float, PyArray_FLOAT)
	MAP_NUMPY_TYPE(double, PyArray_DOUBLE)

	template <typename Matrix>
	numeric::array NewPythonArrayFromMatrix(const Matrix& m) {
		// TODO: use PyArray_SimpleNewFromData to avoid copying
		typedef typename matrix_traits<Matrix>::value_type T;
		numeric::array A = NewArray(matrix_height(m), matrix_width(m), NumpyType<T>::id);
		for (int i = 0; i < matrix_height(m); i++) {
			for (int j = 0; j < matrix_width(m); j++) {
				A[i][j] = m[i][j];
			}
		}
		return A;
	}

	template <typename Vector>
	numeric::array NewPythonArrayFromVector(const Vector& v) {
		// TODO: use PyArray_SimpleNewFromData to avoid copying
		typedef typename vector_traits<Vector>::value_type T;
		numeric::array A = NewArray(vector_length(v), NumpyType<T>::id);
		for (int i = 0; i < vector_length(v); i++) {
			A[i] = v[i];
		}
		return A;
	}


	template <typename Vector>
	bool IsConvertibleToVector(PyObject* obj) {
		if (IsNumericArray(obj) && GetArrayDimensions(obj) == 1) {
			if (vector_traits<Vector>::is_fixed_size) {
				return GetArrayLength(obj, 0) == vector_traits<Vector>::fixed_length;
			} else {
				return true;
			}
		} else {
			return false;
		}
	}

	template <typename Matrix>
	bool IsConvertibleToMatrix(PyObject* obj) {
		if (IsArray(obj) && GetArrayDimensions(obj) == 2) {
			if (matrix_traits<Matrix>::is_fixed_size) {
				return
					GetArrayLength(obj, 0) == matrix_traits<Matrix>::fixed_height &&
					GetArrayLength(obj, 1) == matrix_traits<Matrix>::fixed_width;
			} else {
				return true;
			}
		} else {
			return false;
		}
	}

	// vector allocation
	template <typename T>
	void vector_alloc(int length, VNL::Vector<T>* storage) {
		new (storage) VNL::Vector<T>(length);
	}

	template <int N, typename T>
	void vector_alloc(int length, toon::Vector<N,T>* storage) {
		new (storage) toon::Vector<N,T>();
	}

	template <typename T>
	void vector_alloc(int length, toon::Vector<Dynamic,T>* storage) {
		new (storage) toon::Vector<toon::Dynamic,T>(length);
	}

	template <typename T>
	void vector_alloc(int length, std::vector<T>* storage) {
		new (storage) std::vector<T>(length);
	}

	// Matrix allocation
	template <typename T>
	void matrix_alloc(int nr, int nc, VNL::Matrix<T>* storage) {
		new (storage) VNL::Matrix<T>(nr, nc);
	}

	template <int M, int N, typename T>
	void matrix_alloc(int nr, int nc, toon::Matrix<M,N,T>* storage) {
		new (storage) toon::Matrix<M,N,T>();
	}

	template <typename T>
	void matrix_alloc(int nr, int nc, toon::Matrix<Dynamic,Dynamic,T>* storage) {
		new (storage) toon::Matrix<Dynamic,Dynamic,T>(nr, nc);
	}


	template <typename Vector>
	void CopyPythonArrayToVector(const numeric::array& a, Vector& out) {
		// Use PyArray_GETCONTIGUOUS then some PyArray_GetData-like method
		typedef typename vector_traits<Vector>::value_type T;
		int n = vector_length(out);     // size is already set for us at this point
		for (int i = 0; i < n; i++) {
			out[i] = extract<T>(a[i]);
		}
	}	

	template <typename Matrix>
	void CopyPythonArrayToMatrix(const numeric::array& a, Matrix& out) {
		typedef typename matrix_traits<Matrix>::value_type T;
		int m = matrix_height(out);     // size is already set for us at this point
		int n = matrix_width(out);     // size is already set for us at this point
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				out[i][j] = extract<T>(a[i][j]);
			}
		}
	}	

	//////////////////////////////////////////////////////////////////////
	template <typename Vector>
	struct VectorFromPythonCopyConverter {
		static void* convertible(PyObject* obj) {
			return IsConvertibleToVector<Vector>(obj) ? obj : NULL;
		}

		static void construct(PyObject* obj,
													converter::rvalue_from_python_stage1_data* data) {
			numeric::array a = extract<numeric::array>(obj);
			int len = GetArrayLength(obj, 0);
			Vector* storage = (Vector*)((converter::rvalue_from_python_storage<Vector>*)data)->storage.bytes;
			vector_alloc(len, storage);
			// TODO: avoid copying data around
			CopyPythonArrayToVector(a, *storage);
			data->convertible = storage;
		}
	};

	//////////////////////////////////////////////////////////////////////
	template <typename Matrix>
	struct MatrixFromPythonCopyConverter {
		static void* convertible(PyObject* obj) {
			return IsConvertibleToMatrix<Matrix>(obj) ? obj : NULL;
		}

		static void construct(PyObject* obj,
													converter::rvalue_from_python_stage1_data* data) {
			numeric::array a = extract<numeric::array>(obj);
			int nr = GetArrayLength(obj, 0);
			int nc = GetArrayLength(obj, 1);
			Matrix* storage = (Matrix*)((converter::rvalue_from_python_storage<Matrix>*)data)->storage.bytes;
			matrix_alloc(nr, nc, storage);
			// TODO: avoid copying data around
			CopyPythonArrayToMatrix(a, *storage);
			data->convertible = storage;
		}
	};

	//////////////////////////////////////////////////////////////////////
	template <typename Vector>
	struct VectorToPythonCopyConverter {
		static PyObject* convert(const Vector& v) {
			// TODO: avoid copying data around
			return incref(NewPythonArrayFromVector(v).ptr());
		}
	};

	//////////////////////////////////////////////////////////////////////
	template <typename Matrix>
	struct MatrixToPythonCopyConverter {
		static PyObject* convert(const Matrix& m) {
			// TODO: avoid copying data around
			return incref(NewPythonArrayFromMatrix(m).ptr());
		}
	};



	template <typename Vector>
	void RegisterVectorConverter() {
		to_python_converter<Vector, VectorToPythonCopyConverter<Vector> >();
		converter::registry::push_back(&VectorFromPythonCopyConverter<Vector>::convertible,
																	 &VectorFromPythonCopyConverter<Vector>::construct,
																	 type_id<Vector>());
	}

	template <typename Matrix>
	void RegisterMatrixConverter() {
		to_python_converter<Matrix, MatrixToPythonCopyConverter<Matrix> >();
		converter::registry::push_back(&MatrixFromPythonCopyConverter<Matrix>::convertible,
																	 &MatrixFromPythonCopyConverter<Matrix>::construct,
																	 type_id<Matrix>());
	}

	template <typename T>
	void RegisterConverters() {
		// VNL
		RegisterVectorConverter<VNL::Vector<T> >();
		RegisterMatrixConverter<VNL::Matrix<T> >();

		// TooN Dynamic
		RegisterVectorConverter<toon::Vector<Dynamic,T> >();
		RegisterMatrixConverter<toon::Matrix<Dynamic,Dynamic,T> >();

		// TooN Static vectors
		RegisterVectorConverter<toon::Vector<1,T> >();
		RegisterVectorConverter<toon::Vector<2,T> >();
		RegisterVectorConverter<toon::Vector<3,T> >();
		RegisterVectorConverter<toon::Vector<4,T> >();
		RegisterVectorConverter<toon::Vector<5,T> >();
		RegisterVectorConverter<toon::Vector<6,T> >();

		// TooN Static matrices
		RegisterMatrixConverter<toon::Matrix<1,1,T> >();
		RegisterMatrixConverter<toon::Matrix<2,2,T> >();
		RegisterMatrixConverter<toon::Matrix<3,3,T> >();
		RegisterMatrixConverter<toon::Matrix<4,4,T> >();
		RegisterMatrixConverter<toon::Matrix<5,5,T> >();
		RegisterMatrixConverter<toon::Matrix<6,6,T> >();

		// STL vectors
		RegisterVectorConverter<std::vector<T> >();
	}	

	void RegisterNumpyConversions() {
		RegisterConverters<int>();
		RegisterConverters<float>();
		RegisterConverters<double>();
	}
}
