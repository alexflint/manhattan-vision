#pragma once

#include <GL/gl.h>
#include <boost/function.hpp>
#include <boost/static_assert.hpp>

#include <TooN/TooN.h>

#include "common_types.h"
#include "image_utils.h"
#include "event.tpp"

#define WITH(cap) if (indoor_context::scoped_enabler __x ## __LINE__ = cap)
#define WITHOUT(cap) if (indoor_context::scoped_disabler __x ## __LINE__ = cap)

#define WITH2(cap1,cap2) WITH(cap1) WITH(cap2)
#define WITH3(cap1,cap2,cap3) WITH(cap1) WITH(cap2) WITH(cap3)

#define WITHOUT2(cap1,cap2) WITHOUT(cap1) WITHOUT(cap2)
#define WITHOUT3(cap1,cap2,cap3) WITHOUT(cap1) WITHOUT(cap2) WITHOUT(cap3)

#define GL_MATRIX_SCOPE if (indoor_context::scoped_matrix_pusher __x ## __LINE__ = 0)
#define GL_ATTRIB_SCOPE(attrib) if (indoor_context::scoped_attrib_pusher __x ## __LINE__ = attrib)
#define GL_PRIMITIVE(mode) if (indoor_context::scoped_command __x ## __LINE__ = mode)

#define glError() indoor_context::glErrorInternal(__FILE__, __LINE__);

namespace indoor_context {

	// Report GL errors
	void glErrorInternal(const char* file, int line);

	// Reports errors on construction and destruction. Convertable to a
	// bool in order to let it be used in an IF statement to create a
	// scope.
	struct scope_creator {
		inline scope_creator() { glError(); }
		inline ~scope_creator() { glError(); }
		operator bool() { return true; }
	};

	// Pushes/pops the current matrix during construction/destruction
	struct scoped_matrix_pusher : scope_creator {
		inline scoped_matrix_pusher(int flag) { glPushMatrix(); }
		inline ~scoped_matrix_pusher() { glPopMatrix(); }
	};

	// Pushes/pops a specified attribute during construction/destruction
	struct scoped_attrib_pusher : scope_creator {
		inline scoped_attrib_pusher(GLenum attrib) { glPushAttrib(attrib); glError(); }
		inline ~scoped_attrib_pusher() { glPopAttrib(); glError(); }
	};

	// Enables a specified capacity on construction and restores its old
	// value on destruction
	struct scoped_enabler : scoped_attrib_pusher {
		inline scoped_enabler(GLenum cap) : scoped_attrib_pusher(GL_ENABLE_BIT) {
			glEnable(cap);
		}
	};

	// Disables a specified capacity on construction and restores its
	// old value on destruction
	struct scoped_disabler : scoped_attrib_pusher {
		inline scoped_disabler(GLenum cap) : scoped_attrib_pusher(GL_ENABLE_BIT) {
			glDisable(cap);
		}
	};

	// Issues glBegin(mode) on construction and glEnd(mode) on destruction
	struct scoped_command : scope_creator {
		inline scoped_command(GLenum mode) { glBegin(mode);	}
		inline ~scoped_command() { glEnd();	}
	};




	template <typename T>
	shared_array<T> UnpackMatrix(const toon::Matrix<4,4,T>& m) {
		shared_array<T> arr(new T[16]);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				arr[j*4+i] = m[i][j];
			}
		}
		return arr;
	}

	template <typename T>
	shared_array<T> UnpackMatrix(const toon::Matrix<3,3,T>& m) {
		toon::Matrix<4,4,T> m4 = toon::Identity;
		m4.template slice<0,0,3,3>() = m;
		return UnpackMatrix(m4);
	}

	template <typename Matrix>
	void TransformGL(const Matrix& mat) {
		shared_array<double> arr(UnpackMatrix(mat));
		glMultMatrixd(arr.get());
	}


	// Send Toon vectors down the opengl pipeline
#define TOON_GL_MAP(T, C, N)																\
	inline void glVertexV(const toon::Vector< N, T >& v) {		\
		glVertex ## N ## C ## v(&v[0]);													\
	}

TOON_GL_MAP(float, f, 2);
TOON_GL_MAP(float, f, 3);
TOON_GL_MAP(float, f, 4);
TOON_GL_MAP(double, d, 2);
TOON_GL_MAP(double, d, 3);
TOON_GL_MAP(double, d, 4);
TOON_GL_MAP(int, i, 2);
TOON_GL_MAP(int, i, 3);
TOON_GL_MAP(int, i, 4);

#define VNL_GL_MAP(T, C, N)																				\
	inline void glVertexV(const VNL::VectorFixed< N, T >& v) {			\
		glVertex ## N ## C ## v(&v[0]);																\
	}

VNL_GL_MAP(float, f, 2);
VNL_GL_MAP(float, f, 3);
VNL_GL_MAP(float, f, 4);
VNL_GL_MAP(double, d, 2);
VNL_GL_MAP(double, d, 3);
VNL_GL_MAP(double, d, 4);
VNL_GL_MAP(int, i, 2);
VNL_GL_MAP(int, i, 3);
VNL_GL_MAP(int, i, 4);


	// Set the current GL color from a PixelRGB object. This ignores alpha.
	inline void glColorP(const PixelRGB<byte>& p) {
		glColor3ub(p.r, p.g, p.b);
	}

	// Set the current GL color from a PixelRGB object. This ignores alpha.
	inline void glColorP(const PixelRGB<float>& p) {
		glColor3f(p.r, p.g, p.b);
	}

	// Load a "bright color" into GL
	inline void glBrightColor(int i) {
		glColorP(BrightColors::Get(i));
	}
		
	// Get current GL modelview matrix
	toon::Matrix<4,4> GetGLModelView();
	// Get current GL projection matrix
	toon::Matrix<4,4> GetGLProjection();
	// Get current viewport dimensions
	toon::Vector<4> GetGLViewport();
}
