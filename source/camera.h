#pragma once

#include <boost/shared_ptr.hpp>

#include <TooN/se3.h>

#include "common_types.h"
#include "image_bundle.h"

#include "polygon.tpp"

// Forward declaration
namespace PTAMM { class ATANCamera; }

namespace indoor_context {

// The vertical axis by convention is fixed as 2
static const int kVerticalAxis = 2;

using boost::shared_ptr;

// Base class for cameras
class CameraBase {
public:
	// Recompute bounds. Calls the pure virtual ImToRet and RetToIm.
	void SetImageSize(const ImageRef& image_size);

	// Get the image size
	inline const ImageRef& image_size() const { return image_size_; }
	// Get the image bounds (same info as image_size() but different format)
	const Bounds2D<>& image_bounds() const { return image_bounds_; }
	// Get the bounds of the image in retina coordinates
	const Bounds2D<>& retina_bounds() const { return retina_bounds_; }

	// Transform retina -> image
	virtual Vec2 RetToIm(const Vec2& v) const = 0;
	// Transform retina -> image in homogeneous coordinates
	virtual Vec3 RetToIm(const Vec3& v) const = 0;
	// Transform image -> retina
	virtual Vec2 ImToRet(const Vec2& v) const = 0;
	// Transform image -> retina in homogeneous coordinates
	virtual Vec3 ImToRet(const Vec3& v) const = 0;

	// Get the dimensions of a pixel in the retina (assumes
	// rectangular pixels). Calls the pure virtual RetToIm and ImToRet.
	Vec2 GetRetinaPixelSize() const;
	// Unproject [1,1] into the retina and return its norm For
	// rectangular pixels, this is the diagonal length of a pixel in
	// the retina at the origin. Calls the pure virtual RetToIm and
	// ImToRet.
	double GetRetinaPixelDiameter() const;

	// Approximate this camera by a linear transform. If this is a
	// linear camera then this simply returns the intrinsics matrix.
	virtual const Mat3& Linearize() const = 0;

	// Project each pixel through two different cameras and return the
	// maximum deviation (euclidean distance). Can be used to compare
	// an ATANCamera to its LinearCamera approximation.
	static double GetMaxDeviation(const CameraBase& cam1,
	                              const CameraBase& cam2);
private:
	ImageRef image_size_;
	Bounds2D<> image_bounds_;
	Bounds2D<> retina_bounds_;
};


// Represents a projection from retina to image coordinates. Simply
// wraps PTAMM::ATANCamera in a standard interface.  Emphatically NOT thread-safe!
class ATANCamera : public CameraBase {
public:
	// Construct a camera for images of a specified size with the given intrinsic parameters.
	ATANCamera(const Vec5& camera_params, const Vec2& image_size);
	ATANCamera(const Vec5& camera_params, const ImageRef& image_size);

	// Transform retina -> image
	Vec2 RetToIm(const Vec2& v) const;
	// Transform retina -> image in homogeneous coordinates
	Vec3 RetToIm(const Vec3& v) const;
	// Transform image -> retina
	Vec2 ImToRet(const Vec2& v) const;
	// Transform image -> retina in homogeneous coordinates
	Vec3 ImToRet(const Vec3& v) const;

	// Approximate this camera by a linear transform.
	virtual const Mat3& Linearize() const;

	// Get the underlying PTAM camera
	inline PTAMM::ATANCamera& impl_camera() const { return *impl_; }
private:
  // lazily constructed on call to Linearize()
	mutable Mat3 linearized_intrinsics_;
	// ATANCamera is mutable because its Project() and UnProject()
	// methods are non-const (but act as if they are const)
	mutable shared_ptr<PTAMM::ATANCamera> impl_;
};

// Represents a camera that consists of a matrix transform in
// homogeneous coordinates (i.e. a homography).
class LinearCamera : public CameraBase {
public:
	// Construct a linear camera for 0 by 0 images
	LinearCamera();
	// Construct a linear camera for images of a specified size
	LinearCamera(const Mat3& m, const ImageRef& image_size);

	// Get the camera matrix
	const Mat3& intrinsics() const { return m; }
	// Get the inverse of the camera matrix
	const Mat3& intrinsics_inverse() const { return m_inv; }
	// Set the camera matrix
	void SetIntrinsics(const Mat3& m);

	// Same as intrinsics()
	virtual const Mat3& Linearize() const;

	// Transform retina -> image
	Vec2 RetToIm(const Vec2& v) const;
	// Transform retina -> image in homogeneous coordinates
	Vec3 RetToIm(const Vec3& v) const;
	// Transform image -> retina
	Vec2 ImToRet(const Vec2& v) const;
	// Transform image -> retina in homogeneous coordinates
	Vec3 ImToRet(const Vec3& v) const;
private:
	Mat3 m;  // the intrinsics matrix (i.e. the transform from retina to image coordinates)
	Mat3 m_inv;  // the transform from image to retina coordinates
};

// Represents a camera, a pose, and an image size.
class PosedCamera {
public:
	PosedCamera() { };
	// Initialize a posed camera
	PosedCamera(const toon::SE3<>& pose, const CameraBase* cam);

	// Get/set pose
	toon::SE3<>& pose() { return pose_; }
	const toon::SE3<>& pose() const { return pose_; }
	const toon::SE3<>& pose_inverse() const { return invpose_; }
	void SetPose(const toon::SE3<>& pose);

	// Get/set camera
	const CameraBase& camera() const { return *camera_; }
	void SetCamera(const CameraBase* camera) { camera_ = camera; }

	// Helpers to get size
	int nx() const { return camera().image_size().x; }
	int ny() const { return camera().image_size().y; }

	// Get the image size
	const ImageRef& image_size() const { return camera().image_size(); }
	// Get the image bounds (same info as image_size() but different format)
	const Bounds2D<>& image_bounds() const { return camera().image_bounds(); }
	// Get the bounds of the image in retina coordinates
	const Bounds2D<>& retina_bounds() const { return camera().retina_bounds(); }
	// Get the camera centre in world coordinates
	const Vec3& world_centre() const { return invpose_.get_translation(); }

	// Transform world coordinate to retina coords
	inline Vec3 WorldToRet(const Vec3& v) const {
		return pose_*v;
	}
	// Transfor world to image coordinates
	Vec3 WorldToIm(const Vec3& v) const {
		return RetToIm(pose_*v);
	}
	// Transform homogeneous retina to image coordinates
	Vec3 RetToIm(const Vec3& v) const {
		return camera().RetToIm(v);
	}
	// Transform retina to image coordinates
	Vec2 RetToIm(const Vec2& v) const {
		return camera().RetToIm(v);
	}
	// Transform homogeneous image to retina coordinates
	Vec3 ImToRet(const Vec3& v) const {
		return camera().ImToRet(v);
	}
	// Transform image to retina coordinates
	Vec2 ImToRet(const Vec2& v) const {
		return camera().ImToRet(v);
	}

	// Transform the camera's pose by M. Note that this is equivalent to transforming
	// the location of things in world by the *inverse* of M
	void Transform(const toon::SE3<>& M);

	// Get a linear approximation of this camera
	// incorporating both intrinsics and extrinisics.
	toon::Matrix<3,4> Linearize() const;

	// Get the i-th vanishing point in retina coords
	Vec3 GetRetinaVpt(int axis) const;
	// Get the i-th vanishing point in image coords
	Vec3 GetImageVpt(int axis) const;

	// Get the horizon line in retina coords (positive side is below horizon)
	Vec3 GetRetinaHorizon() const;
	// Get the horizon line in image coords (positive side is below horizon)
	Vec3 GetImageHorizon() const;
private:
	// camera pose: (world->retina transformation, i.e. extrinsic camera parameters)
	toon::SE3<> pose_;
	// Inverse of above
	toon::SE3<> invpose_;
	// camera model
	const CameraBase* camera_;
};

// Represents an image with the camera that captured it, i.e. an
// image together with its intrinsic camera parameters
class CalibratedImage : public ImageBundle {
public:
	CalibratedImage() { }
	CalibratedImage(const CameraBase* camera) : camera_(camera) { }
	CalibratedImage(const CameraBase* camera, const string& image_file)
	: ImageBundle(image_file), camera_(camera) { }

	// Allocate an image of size camera_->image_size()
	void Allocate();

	// Get/set camera
	const CameraBase& camera() const { return *camera_; }
	void SetCamera(const CameraBase* camera) { camera_ = camera; }
private:
	const CameraBase* camera_;
};

// Represents a calibrated image with a pose in space, i.e. an image
// together with both intrinsic and extrinisic camera parameters
class PosedImage : public CalibratedImage {
public:
	PosedImage() { }
	PosedImage(const toon::SE3<>& pose, const CameraBase* cam)
		: CalibratedImage(cam), pc_(pose, cam) { }
	PosedImage(const toon::SE3<>& pose,
						 const CameraBase* cam,
						 const string& image_file)
		: CalibratedImage(cam, image_file), pc_(pose, cam) { }

	// Get/set posed camera
	PosedCamera& pc() { return pc_; }
	const PosedCamera& pc() const { return pc_; }
private:
	PosedCamera pc_;
};
}
