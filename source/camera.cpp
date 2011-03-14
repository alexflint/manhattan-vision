#include <LU.h>

#include "camera.h"
#include "lazyvar.h"
#include "ATANCamera.h"
#include "vector_utils.tpp"

namespace indoor_context {
using namespace toon;

lazyvar<string> gvDefaultCamera("Map.DefaultCameraParams");
lazyvar<Vec2> gvImageSize("Map.DefaultImageSize");

///// CameraBase

void CameraBase::SetImageSize(const ImageRef& image_size) {
	image_size_ = image_size;
	image_bounds_ = Bounds2D<double>::FromSize(image_size);
	retina_bounds_ = Bounds2D<double>::FromCorners(
			ImToRet(image_bounds_.tl()), ImToRet(image_bounds_.br()));
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

	double x1 = cam1.retina_bounds().left();
	double x2 = cam1.retina_bounds().right();
	double y1 = cam1.retina_bounds().top();
	double y2 = cam1.retina_bounds().bottom();

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


///// ATANCamera

ATANCamera::ATANCamera(const Vec5& camera_params, const Vec2& image_size)
	: impl_(new PTAMM::ATANCamera(image_size, camera_params)) {
	SetImageSize(asIR(image_size));
}

ATANCamera::ATANCamera(const Vec5& camera_params, const ImageRef& image_size)
	: impl_(new PTAMM::ATANCamera(asToon(image_size), camera_params)) {
	SetImageSize(image_size);
}

Vec2 ATANCamera::RetToIm(const Vec2& v) const {
	return impl_->Project(v);
}

Vec3 ATANCamera::RetToIm(const Vec3& v) const {
	return unproject(RetToIm(project(v)));
}

Vec2 ATANCamera::ImToRet(const Vec2& v) const {
	return impl_->UnProject(v);
}

Vec3 ATANCamera::ImToRet(const Vec3& v) const {
	return unproject(ImToRet(project(v)));
}

//virtual
const Mat3& ATANCamera::Linearize() const {
	// there may be an error in this function since the linearizations seem
	// to vary from one frame to the next!
	Vec3 c = RetToIm(makeVector(0.0, 0, 1));
	linearized_intrinsics_.T()[0] = RetToIm(makeVector(1.0, 0, 1)) - c;
	linearized_intrinsics_.T()[1] = RetToIm(makeVector(0.0, 1, 1)) - c;
	linearized_intrinsics_.T()[2] = c;
	return linearized_intrinsics_;
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

// virtual
const Mat3& LinearCamera::Linearize() const {
	return intrinsics();
}

///// PosedCamera
PosedCamera::PosedCamera(const toon::SE3<>& pose, const CameraBase* camera)
: camera_(camera) {
	SetPose(pose);
}

void PosedCamera::SetPose(const toon::SE3<>& pose) {
	pose_ = pose;
	invpose_ = pose.inverse();
}

Vec3 PosedCamera::GetRetinaVpt(int i) const {
	CHECK_INTERVAL(i,0,2);
	return col(pose_.get_rotation(), i);
}

Vec3 PosedCamera::GetImageVpt(int i) const {
	return RetToIm(GetRetinaVpt(i));
}

Vec3 PosedCamera::GetRetinaHorizon() const {
	Vec3 v = GetRetinaVpt(kVerticalAxis);
	return v * Sign(v[1]);
}

Vec3 PosedCamera::GetImageHorizon() const {
	Vec3 v = GetImageVpt(0) ^ GetImageVpt(1);  // can't use GetImageVpt(2) because no orthogonality
	return v * Sign(v[1]);
}

void PosedCamera::Transform(const toon::SE3<>& m) {
	pose_ *= m;
	invpose_ = pose_.inverse();
}

Matrix<3,4> PosedCamera::Linearize() const {
	return camera_->Linearize() * as_matrix(pose_);
}


///// CalibratedImage
void CalibratedImage::Allocate() {
	if (!rgb.IsAlloced()) {
		rgb.AllocImageData(camera().image_size().x, camera().image_size().y);
	}
}

}
