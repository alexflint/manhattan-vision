
//#define EIGEN_DEFAULT_TO_ROW_MAJOR
//#include <Eigen/Core>

#include <VNL/vector.h>
#include <VNL/vectorfixed.h>
#include <VNL/matrix.h>
#include <VNL/matrixfixed.h>

#include <TooN/TooN.h>

namespace indoor_context {
	/*
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;
	typedef Eigen::Vector4d Vec4;

	typedef Eigen::Vector2i Vec2I;
	typedef Eigen::Vector3i Vec3I;
	typedef Eigen::Vector4i Vec4I;

	typedef Eigen::Matrix2d Mat2;
	typedef Eigen::Matrix3d Mat3;
	typedef Eigen::Matrix4d Mat4;

	typedef Eigen::MatrixXi MatI;
	typedef Eigen::MatrixXf MatF;
	typedef Eigen::MatrixXd MatD;

	typedef Eigen::VectorXi VecI;
	typedef Eigen::VectorXf VecF;
	typedef Eigen::VectorXd VecD;*/

	typedef toon::Vector<2> Vec2;
	typedef toon::Vector<3> Vec3;
	typedef toon::Vector<4> Vec4;
	typedef toon::Vector<5> Vec5;
	typedef toon::Vector<6> Vec6;

	typedef toon::Vector<2,int> Vec2I;
	typedef toon::Vector<3,int> Vec3I;
	typedef toon::Vector<4,int> Vec4I;
	typedef toon::Vector<5,int> Vec5I;
	typedef toon::Vector<6,int> Vec6I;

	typedef toon::Matrix<2> Mat2;
	typedef toon::Matrix<3> Mat3;
	typedef toon::Matrix<4> Mat4;

	typedef VNL::Matrix<int> MatI;
	typedef VNL::Matrix<float> MatF;
	typedef VNL::Matrix<double> MatD;

	typedef VNL::Vector<int> VecI;
	typedef VNL::Vector<float> VecF;
	typedef VNL::Vector<double> VecD;
}
