#include <LU.h>

#include "camera.h"
#include "lazyvar.h"
#include "math_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
using namespace toon;

lazyvar<string> gvDefaultCamera("Map.DefaultCamera");
lazyvar<Vec2 > gvImageSize("Camera.ImageSize");

///// CameraBase

void CameraBase::SetImageSize(const ImageRef& im_size) {
	im_size_ = im_size;
	im_bounds_ = Bounds2D<double>::FromSize(im_size);
	ret_bounds_ = Bounds2D<double>::FromCorners(
			ImToRet(im_bounds_.tl()), ImToRet(im_bounds_.br()));
}

Vec2 CameraBase::GetRetinaPixelSize() const {
	// don't use toon::Zeros below, it causes overload ambiguity
	return ImToRet(RetToIm(makeVector(0.0, 0.0))+makeVector(1.0, 1.0));
}

double CameraBase::GetRetinaPixelDiameter() const {
	return norm(GetRetinaPixelSize());
}

// static
double CameraBase::GetMaxDeviation(const CameraBase& cam1, const CameraBase& cam2) {
	const int kDeviationSamples = 100;
	double maxerr = 0.0;

	double x1 = cam1.ret_bounds().left;
	double x2 = cam1.ret_bounds().right;
	double y1 = cam1.ret_bounds().top;
	double y2 = cam1.ret_bounds().bottom;

	for (int y = 0; y < kDeviationSamples; y++) {
		for (int x = 0; x < kDeviationSamples; x++) {
			Vec2 ret_pt = makeVector(x1 + (x2-x1) * x / (kDeviationSamples-1),
					y1 + (y2-y1) * y / (kDeviationSamples-1));
			Vec2 cam1_pt = cam1.RetToIm(ret_pt);
			Vec2 cam2_pt = cam2.RetToIm(ret_pt);
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

Vec2 Camera::RetToIm(const Vec2& v) const {
	return cam_->Project(v);
}

Vec3 Camera::RetToIm(const Vec3& v) const {
	return unproject(RetToIm(project(v)));
}

Vec2 Camera::ImToRet(const Vec2& v) const {
	return cam_->UnProject(v);
}

Vec3 Camera::ImToRet(const Vec3& v) const {
	return unproject(ImToRet(project(v)));
}



///// LinearCamera

LinearCamera::LinearCamera() {
}

LinearCamera::LinearCamera(const Mat3& mat, const ImageRef& sz) {
	// Set matrix _must_ come first because the matrix is used in
	// ImToRet, which is called from within SetImageSize
	SetIntrinsics(mat);
	SetImageSize(sz);
}

void LinearCamera::SetIntrinsics(const Mat3& mat) {
	m = mat;
	m_inv = LU<3>(m).get_inverse();
}

Vec2 LinearCamera::RetToIm(const Vec2& v) const {
	return project(RetToIm(unproject(v)));
}

Vec3 LinearCamera::RetToIm(const Vec3& v) const {
	return m*v;
}

Vec2 LinearCamera::ImToRet(const Vec2& v) const {
	return project(ImToRet(unproject(v)));
}

Vec3 LinearCamera::ImToRet(const Vec3& v) const {
	return m_inv * v;
}

//static
LinearCamera* LinearCamera::Approximate(const CameraBase& cam) {
	Mat3 m;
	Linearize(cam, m);
	return new LinearCamera(m, cam.im_size());
}

//static
void LinearCamera::Linearize(const CameraBase& cam, Mat3& m) {
	Vec3 c = cam.RetToIm(makeVector(0.0, 0, 1));
	m.T()[0] = cam.RetToIm(makeVector(1.0, 0, 1)) - c;
	m.T()[1] = cam.RetToIm(makeVector(0.0, 1, 1)) - c;
	m.T()[2] = c;
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

Vec3 PosedCamera::GetRetinaVpt(int i) const {
	CHECK_GE(i,0);
	CHECK_LT(i,3);
	return col(pose.get_rotation(), i);
}

Vec3 PosedCamera::GetImageVpt(int i) const {
	return RetToIm(GetRetinaVpt(i));
}

void PosedCamera::Transform(const toon::SE3<>& m) {
	pose *= m;
	invpose = pose.inverse();
}

Matrix<3,4> PosedCamera::GetLinearApproximation() const {
	Mat3 linear_intr;
	LinearCamera::Linearize(camera, linear_intr);
	return linear_intr * as_matrix(pose);
}

}
