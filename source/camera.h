#pragma once

#include <TooN/se3.h>

#include "common_types.h"
#include "ATANCamera.h"
#include "image_bundle.h"

#include "polygon.tpp"

namespace indoor_context {
// Base class for cameras
class CameraBase {
public:
	// Recompute bounds. Calls the pure virtual ImToRet and RetToIm.
	void SetImageSize(const ImageRef& im_size);

	// Get the image size
	inline const ImageRef& im_size() const { return im_size_; }
	// Get the image bounds (same info as im_size() but different format)
	const Bounds2D<>& im_bounds() const { return im_bounds_; }
	// Get the bounds of the image in retina coordinates
	const Bounds2D<>& ret_bounds() const { return ret_bounds_; }

	// Transform retina -> image
	virtual Vec2 RetToIm(const Vec2& v) const = 0;
	// Transform retina -> image in homogeneous coordinates
	virtual Vec3 RetToIm(const Vec3& v) const = 0;
	// Transform image -> retina
	virtual Vec2 ImToRet(const Vec2& v) const = 0;
	// Transform image -> retina in homogeneous coordinates
	virtual Vec3 ImToRet(const Vec3& v) const = 0;

	// Approximate this camera by a linear transform
	Mat3 Linearize() const;

	// Get the dimensions of a pixel in the retina (assumes
	// rectangular pixels). Calls the pure virtual RetToIm and ImToRet.
	Vec2 GetRetinaPixelSize() const;
	// Unproject [1,1] into the retina and return its norm For
	// rectangular pixels, this is the diagonal length of a pixel in
	// the retina at the origin. Calls the pure virtual RetToIm and
	// ImToRet.
	double GetRetinaPixelDiameter() const;

	// Project each pixel through two different cameras and return the
	// maximum deviation (euclidean distance). Can be used to compare
	// an ATANCamera to its LinearCamera approximation.
	static double GetMaxDeviation(const CameraBase& cam1,
	                              const CameraBase& cam2);
private:
	ImageRef im_size_;
	Bounds2D<> im_bounds_;
	Bounds2D<> ret_bounds_;
};

// Represents a projection from retina to image coordinates. Simply
// wraps PTAMM::ATANCamera in a standard interface.  Emphatically NOT thread-safe!
//TODO: rename this "ATANCamera"
class Camera : public CameraBase {
public:
	// Construct a camera for 0 by 0 images
	Camera();
	// Construct a camera for images of a specified size
	Camera(const ImageRef& im_size);
	Camera(const ImageRef& im_size, const string& cam_name);

	// Transform retina -> image
	Vec2 RetToIm(const Vec2& v) const;
	// Transform retina -> image in homogeneous coordinates
	Vec3 RetToIm(const Vec3& v) const;
	// Transform image -> retina
	Vec2 ImToRet(const Vec2& v) const;
	// Transform image -> retina in homogeneous coordinates
	Vec3 ImToRet(const Vec3& v) const;

	// Get the underlying PTAM camera
	inline PTAMM::ATANCamera& cam() const { return *cam_; }
private:
	// ATANCamera is mutable because its Project() and UnProject()
	// methods are non-const (but act as if they are const)
	mutable shared_ptr<PTAMM::ATANCamera> cam_;
};

// Represents a camera that consists of a matrix transform in
// homogeneous coordinates (i.e. a homography).
class LinearCamera : public CameraBase {
public:
	// Construct a linear camera for 0 by 0 images
	LinearCamera();
	// Construct a linear camera for images of a specified size
	LinearCamera(const Mat3& m, const ImageRef& im_size);

	// Get the camera matrix
	const Mat3& intrinsics() const { return m; }
	// Set the camera matrix
	void SetIntrinsics(const Mat3& m);

	// Transform retina -> image
	Vec2 RetToIm(const Vec2& v) const;
	// Transform retina -> image in homogeneous coordinates
	Vec3 RetToIm(const Vec3& v) const;
	// Transform image -> retina
	Vec2 ImToRet(const Vec2& v) const;
	// Transform image -> retina in homogeneous coordinates
	Vec3 ImToRet(const Vec3& v) const;

	// Construct a linear camera as an approximation to some other camera.
	static Mat3 Linearize(const CameraBase& cam);
	static void Linearize(const CameraBase& cam, Mat3& m);
private:
	Mat3 m;  // the intrinsics matrix (i.e. the transform from retina to image coordinates)
	Mat3 m_inv;  // the transform from image to retina coordinates
};

// Represents a camera, a pose, and an image size.
class PosedCamera {
public:
	// Initialize a posed camera
	PosedCamera(const toon::SE3<>& pose, const CameraBase& cam);
	// Configure this camera
	void SetPose(const toon::SE3<>& pose);
	// Transform world coordinate to retina coords
	inline Vec3 WorldToRet(const Vec3& v) const {
		return pose*v;
	}
	// Transfor world to image coordinates
	Vec3 WorldToIm(const Vec3& v) const {
		return RetToIm(pose*v);
	}
	// Transform homogeneous retina to image coordinates
	Vec3 RetToIm(const Vec3& v) const {
		return camera.RetToIm(v);
	}
	// Transform retina to image coordinates
	Vec2 RetToIm(const Vec2& v) const {
		return camera.RetToIm(v);
	}
	// Transform homogeneous image to retina coordinates
	Vec3 ImToRet(const Vec3& v) const {
		return camera.ImToRet(v);
	}
	// Transform image to retina coordinates
	Vec2 ImToRet(const Vec2& v) const {
		return camera.ImToRet(v);
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

	// Get the horizon line in the retina (positive side is above horizon)
	Vec3 GetRetinaHorizon() const;
	Vec3 GetImageHorizon() const;

	// Get the image size
	inline const ImageRef& im_size() const { return camera.im_size(); }
	// Get the image bounds (same info as im_size() but different format)
	const Bounds2D<>& im_bounds() const { return camera.im_bounds(); }
	// Get the bounds of the image in retina coordinates
	const Bounds2D<>& ret_bounds() const { return camera.ret_bounds(); }

	// Get the camera centre in world coordinates
	inline Vec3 world_centre() const { return invpose.get_translation(); }

	// camera pose: (world->retina transformation, i.e. extrinsic camera parameters)
	toon::SE3<> pose;
	// inverse of above
	toon::SE3<> invpose;
	// camera model
	const CameraBase& camera;
};

// Represents an image with the camera that captured it, i.e. an
// image together with its intrinsic camera parameters
class CalibratedImage : public ImageBundle {
public:
	CalibratedImage(const CameraBase& cam)
	: camera(cam) { }
	CalibratedImage(const CameraBase& cam, const string& image_file)
	: ImageBundle(image_file), camera(cam) { }
	const CameraBase& camera;
};

// Represents a calibrated image with a pose in space, i.e. an image
// together with both intrinsic and extrinisic camera parameters
class PosedImage : public CalibratedImage {
public:
	PosedImage(const PosedCamera& pcam)
	: CalibratedImage(pcam.camera), pc(pcam) { }
	PosedImage(const PosedCamera& pcam, const string& image_file)
	: CalibratedImage(pcam.camera, image_file), pc(pcam) { }
	const PosedCamera& pc;
};
}
