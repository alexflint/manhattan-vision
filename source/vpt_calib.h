#pragma once

#include "common_types.h"

namespace indoor_context {

	class VanishingPoints;

	struct VanPointPair {
		const VanishingPoints* vpts;
		const int first_ind;
		const int second_ind;
		const Vec3D& first();
		const Vec3D& second();
	};

	// Pack a constraint of the form u^T * S * v = 0 into a matrix row
	void PackFullConstraint(const Vec3D& u, const Vec3D& v, double* row);
	// Unpack a 6dof solution vector into a symmetric 3x3 matrix
	void UnpackFullSolution(const VNL::VectorFixed<6,double>& soln,
													VNL::MatrixFixed<3,3,double>& model);
	// Fit a least-squares model given pairs of vanishing points
	void FitFullModel(const vector<pair<Vec3D,Vec3D> >& pairs,
										VNL::MatrixFixed<3,3,double>& model);


	// Pack a constraint of the form D*u . D*v = 0 where D is diagonal
	void PackScaleConstraint(const Vec3D& u, const Vec3D& v, double* row);
	// Unpack a 3dof solution vector into a diagonal 3x3 matrix
	void UnpackScaleSolution(const VNL::VectorFixed<3,double>& soln,
													 VNL::MatrixFixed<3,3,double>& model);
	// Fit a least-squares scale model given pairs of vanishing points
	void FitScaleModel(const vector<pair<Vec3D,Vec3D> >& pairs,
										 VNL::MatrixFixed<3,3,double>& model);

}
