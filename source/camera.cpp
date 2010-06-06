#include <LU.h>

#include "camera.h"
#include "lazyvar.h"
#include "math_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	lazyvar<string> gvDefaultCamera("Map.DefaultCamera");
	lazyvar<Vector<2> > gvImageSize("Camera.ImageSize");

	///// CameraBase

	void CameraBase::SetImageSize(const ImageRef& im_size) {
		im_size_ = im_size;
		im_bounds_ = Bounds2D<double>::FromSize(im_size);
		ret_bounds_ = Bounds2D<double>::FromCorners(ImToRet(im_bounds_.tl()),
																								ImToRet(im_bounds_.br()));
		
	}

	Vector<2> CameraBase::GetRetinaPixelSize() const {
		// don't use toon::Zeros below, it causes overload ambiguity
		return ImToRet(RetToIm(makeVector(0.0, 0.0))+makeVector(1.0, 1.0));
	}

	double CameraBase::GetRetinaPixelDiameter() const {
		return norm(GetRetinaPixelSize());
	}

	// static
	double CameraBase::GetMaxDeviation(const CameraBase& cam1,
																		 const CameraBase& cam2) {
		const int kDeviationSamples = 100;
		double maxerr = 0.0;

		double x1 = cam1.ret_bounds().left;
		double x2 = cam1.ret_bounds().right;
		double y1 = cam1.ret_bounds().top;
		double y2 = cam1.ret_bounds().bottom;

		for (int y = 0; y < kDeviationSamples; y++) {
			for (int x = 0; x < kDeviationSamples; x++) {
				Vector<2> ret_pt = makeVector(x1 + (x2-x1) * x / (kDeviationSamples-1),
																			y1 + (y2-y1) * y / (kDeviationSamples-1));
				Vector<2> cam1_pt = cam1.RetToIm(ret_pt);
				Vector<2> cam2_pt = cam2.RetToIm(ret_pt);
				double err = norm(cam1_pt - cam2_pt);
				if (err > maxerr) {
					maxerr = err;
				}
			}
		}

		return maxerr;
	}


	///// Camera

	Camera::Camera() : cam_(new PTAMM::ATANCamera(*gvDefaultCamera)) {
		SetImageSize(asIR(*gvImageSize));
	}

	Camera::Camera(const ImageRef& im_size)
		: cam_(new PTAMM::ATANCamera(*gvDefaultCamera)) {
		SetImageSize(im_size);
	}

	Camera::Camera(const ImageRef& im_size, const string& cam_name)
		: cam_(new PTAMM::ATANCamera(cam_name)) {
		SetImageSize(im_size);
	}

	Vector<2> Camera::RetToIm(const Vector<2>& v) const {
		return cam_->Project(v);
	}

	Vector<3> Camera::RetToIm(const Vector<3>& v) const {
		return unproject(RetToIm(project(v)));
	}

	Vector<2> Camera::ImToRet(const Vector<2>& v) const {
		return cam_->UnProject(v);
	}

	Vector<3> Camera::ImToRet(const Vector<3>& v) const {
		return unproject(ImToRet(project(v)));
	}



	///// LinearCamera

	LinearCamera::LinearCamera() {
	}

	LinearCamera::LinearCamera(const Matrix<3>& mat, const ImageRef& sz) {
		// Set matrix _must_ come first because the matrix is used in
		// ImToRet, which is called from within SetImageSize
		SetMatrix(mat);
		SetImageSize(sz);
	}

	void LinearCamera::SetMatrix(const Matrix<3>& mat) {
		m = mat;
		m_inv = LU<3>(m).get_inverse();
	}

	Vector<2> LinearCamera::RetToIm(const Vector<2>& v) const {
		return project(RetToIm(unproject(v)));
	}

	Vector<3> LinearCamera::RetToIm(const Vector<3>& v) const {
		return m*v;
	}

	Vector<2> LinearCamera::ImToRet(const Vector<2>& v) const {
		return project(ImToRet(unproject(v)));
	}

	Vector<3> LinearCamera::ImToRet(const Vector<3>& v) const {
		return m_inv * v;
	}
	
	//static
	LinearCamera* LinearCamera::Approximate(const CameraBase& cam) {
		Vector<3> v0 = makeVector(0,0,1);
		Vector<3> v1 = makeVector(1,0,1);
		Vector<3> v2 = makeVector(0,1,1);

		Matrix<3> m;
		Vector<3> c = cam.RetToIm(v0);
		m.T()[0] = cam.RetToIm(v1) - c;
		m.T()[1] = cam.RetToIm(v2) - c;
		m.T()[2] = c;

		return new LinearCamera(m, cam.im_size());
	}


	///// PosedCamera
	PosedCamera::PosedCamera(const toon::SE3<>& pose, const CameraBase& cam)
		: camera(cam) {
		SetPose(pose);
	}

	void PosedCamera::SetPose(const toon::SE3<>& p) {
		pose = p;
		invpose = p.inverse();
	}

	Vector<3> PosedCamera::GetRetinaVpt(int i) const {
		CHECK_GE(i,0);
		CHECK_LT(i,3);
		return col(pose.get_rotation(), i);
	}
	
	Vector<3> PosedCamera::GetImageVpt(int i) const {
		return RetToIm(GetRetinaVpt(i));
	}
}
