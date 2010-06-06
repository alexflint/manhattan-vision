#include <VNL/Algo/svd.h>

#include "vpt_calib.h"
#include "vanishing_points.h"

namespace indoor_context {

	namespace {
		// Include these namespaces for this file only
		using namespace VNL;
		using namespace VW;
	}

	const Vec3D& VanPointPair::first() {
		return vpts->detector.vanpts[first_ind].pt_cond;
	}
	const Vec3D& VanPointPair::second() {
		return vpts->detector.vanpts[second_ind].pt_cond;
	}

	// Pack a constraint of the form u^T * S * v = 0 into a matrix row
	void PackFullConstraint(const Vec3D& u, const Vec3D& v, double* row) {
		row[0] = u[0]*v[0];
		row[1] = u[1]*v[1];
		row[2] = u[2]*v[2];
		row[3] = u[0]*v[1] + u[1]*v[0];
		row[4] = u[1]*v[2] + u[2]*v[1];
		row[5] = u[0]*v[2] + u[2]*v[0];
	}

	// Unpack a 6dof solution vector into a symmetric 3x3 matrix
	void UnpackFullSolution(const VectorFixed<6,double>& soln,
													MatrixFixed<3,3,double>& model) {
		model[0][0] = soln[0];
		model[1][1] = soln[1];
		model[2][2] = soln[2];
		model[0][1] = model[1][0] = soln[3];
		model[1][2] = model[2][1] = soln[4];
		model[0][2] = model[2][0] = soln[5];
	}

	// Fit a least-squares model given pairs of vanishing points
	void FitFullModel(const vector<pair<Vec3D,Vec3D> >& pairs,
										MatrixFixed<3,3,double>& model) {
		// Build the Nx6 matrix
		MatD data(pairs.size(), 6);
		for (int i = 0; i < pairs.size(); i++) {
			PackFullConstraint(pairs[i].first, pairs[i].second, data[i]);
		}
		// Solve least squares problem with SVD
		VectorFixed<6,double> soln = SVD<double>(data).Nullvector();
		// Unpack into matrix form
		UnpackFullSolution(soln, model);
	}



	// Pack a constraint of the form D*u . D*v = 0 where D is diagonal
	void PackScaleConstraint(const Vec3D& u, const Vec3D& v, double* row) {
		row[0] = u[0] * v[0];
		row[1] = u[1] * v[1];
		row[2] = u[2] * v[2];
	}

	// Unpack a 3dof solution vector into a diagonal 3x3 matrix
	void UnpackScaleSolution(const VectorFixed<3,double>& soln,
													 MatrixFixed<3,3,double>& model) {
		model.Fill(0.0);
		model[0][0] = soln[0];
		model[1][1] = soln[1];
		model[2][2] = soln[2];
	}

	// Fit a least-squares scale model given pairs of vanishing points
	void FitScaleModel(const vector<pair<Vec3D,Vec3D> >& pairs,
										 MatrixFixed<3,3,double>& model) {
		// Build the Nx3 matrix
		MatD data(pairs.size(), 3);
		for (int i = 0; i < pairs.size(); i++) {
			PackScaleConstraint(pairs[i].first, pairs[i].second, data[i]);
		}
		// Solve least squares problem with SVD
		VectorFixed<3,double> soln = SVD<double>(data).Nullvector();
		// Multiplying by a constant has no net effect but we do want
		// positive solutions since we will take square roots
		soln /= soln[2];
		// Unpack into matrix form
		UnpackScaleSolution(soln, model);
	}

}
