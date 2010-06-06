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
		virtual toon::Vector<2> RetToIm(const toon::Vector<2>& v) const = 0;
		// Transform retina -> image in homogeneous coordinates
		virtual toon::Vector<3> RetToIm(const toon::Vector<3>& v) const = 0;
		// Transform image -> retina
		virtual toon::Vector<2> ImToRet(const toon::Vector<2>& v) const = 0;
		// Transform image -> retina in homogeneous coordinates
		virtual toon::Vector<3> ImToRet(const toon::Vector<3>& v) const = 0;

		// Get the dimensions of a pixel in the retina (assumes
		// rectangular pixels). Calls the pure virtual RetToIm and ImToRet.
		toon::Vector<2> GetRetinaPixelSize() const;
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
		toon::Vector<2> RetToIm(const toon::Vector<2>& v) const;
		// Transform retina -> image in homogeneous coordinates
		toon::Vector<3> RetToIm(const toon::Vector<3>& v) const;
		// Transform image -> retina
		toon::Vector<2> ImToRet(const toon::Vector<2>& v) const;
		// Transform image -> retina in homogeneous coordinates
		toon::Vector<3> ImToRet(const toon::Vector<3>& v) const;

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
		LinearCamera(const toon::Matrix<3>& m, const ImageRef& im_size);

		// Set the camera matrix
		void SetMatrix(const toon::Matrix<3>& m);

		// Transform retina -> image
		toon::Vector<2> RetToIm(const toon::Vector<2>& v) const;
		// Transform retina -> image in homogeneous coordinates
		toon::Vector<3> RetToIm(const toon::Vector<3>& v) const;
		// Transform image -> retina
		toon::Vector<2> ImToRet(const toon::Vector<2>& v) const;
		// Transform image -> retina in homogeneous coordinates
		toon::Vector<3> ImToRet(const toon::Vector<3>& v) const;

		// Construct a linear camera as an approximation to some other camera.
		static LinearCamera* Approximate(const CameraBase& cam);
	private:
		toon::Matrix<3> m;  // the transform from retina to image coordinates
		toon::Matrix<3> m_inv;  // the transform from image to retina coordinates
	};

	// Represents a camera, a pose, and an image size.
	class PosedCamera {
	public:
		// Initialize a posed camera
		PosedCamera(const toon::SE3<>& pose, const CameraBase& cam);
		// Configure this camera
		void SetPose(const toon::SE3<>& pose);
		// Transform world coordinate to retina coords
		inline toon::Vector<3> WorldToRet(const toon::Vector<3>& v) const {
			return pose*v;
		}
		// Transfor world to image coordinates
		toon::Vector<3> WorldToIm(const toon::Vector<3>& v) const {
			return RetToIm(pose*v);
		}
		// Transform homogeneous retina to image coordinates
		toon::Vector<3> RetToIm(const toon::Vector<3>& v) const {
			return camera.RetToIm(v);
		}
		// Transform retina to image coordinates
		toon::Vector<2> RetToIm(const toon::Vector<2>& v) const {
			return camera.RetToIm(v);
		}
		// Transform homogeneous image to retina coordinates
		toon::Vector<3> ImToRet(const toon::Vector<3>& v) const {
			return camera.ImToRet(v);
		}
		// Transform image to retina coordinates
		toon::Vector<2> ImToRet(const toon::Vector<2>& v) const {
			return camera.ImToRet(v);
		}

		// Transform the camera's pose by M. Note that this is equivalent to transforming
		// the location of things in world by the *inverse* of M
		void Transform(const toon::SE3<>& M) {
			pose *= M;
			invpose = pose.inverse();
		}

		// Get the i-th vanishing point in retina coords
		toon::Vector<3> GetRetinaVpt(int axis) const;
		// Get the i-th vanishing point in image coords
		toon::Vector<3> GetImageVpt(int axis) const;

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
