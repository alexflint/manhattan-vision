#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "camera.h"

namespace indoor_context {
	// Import boost for this file only
	namespace { using namespace boost; }

	// forward decls
	class Map;

	// Represents a measurement of a 3D landmark in a particular frame
	class Measurement {
	public:
		int point_index;  // index of the point (amongst map.pts)
		Vec2 image_pos;  // position in image coordinates
		Vec2 retina_pos;  // position in retina plane
		int pyramid_level; // pyramid level at which the point was detected
	};

	// Represents a frame in a video sequence mapped by PTAM
	class Frame {
	public:
		// The map that owns this keyframe
		Map* map;
		// Index of this frame within the map
		int id;

		// The raw image from the camera together with its pose
		PosedImage image;
		// Path to the image file for this frame (if any)
		string image_file;

		// True iff the camera was lost (or initializing) at this frame
		bool lost;
		// True iff the camera was initializing at this frame
		bool initializing;

		// The landmarks that were observed in this frame
		vector<Measurement> measurements;
		// Output a vector of all points measured in this frame
		void GetMeasuredPoints(vector<Vec3>& out) const;

		Frame() : id(-1) { }
		// Configure this keyframe with specified image file and pose
		void Configure(Map* map,
									 int id,
									 const string& image_file,
									 const toon::SE3<>& pose);
		// Load the image
		void LoadImage();
		// Unload the image
		void UnloadImage();
	};

	// Represents a map from PTAM
	class Map {
	public:
		// The frame information read from XML
		ptr_vector<Frame> frames;
		// The points
		vector<Vec3> points;

		// The camera model for frames in this map
		shared_ptr<ATANCamera> orig_camera;
		shared_ptr<CameraBase> camera;

		// Add a frame to the map. The map takes ownership of the memory.
		void AddFrame(Frame* frame);

		// Get a key frame by its index in the original map. Return NULL
		// if this keyframe is not loaded or does not exist.
		Frame* GetFrameById(int id);
		const Frame* GetFrameById(int id) const;
		// As above but die with an error message if specified key frame not present.
		Frame* GetFrameByIdOrDie(int id);
		const Frame* GetFrameByIdOrDie(int id) const;

		// Load images for every frame
		void LoadAllImages();
		// Get a frame by ID and load its image
		PosedImage& GetImageByIdOrDie(int id);

		// Apply a rigid coordinate transformation to the map. The inverse
		// transformation will be applied to camera poses so that everything
		// remains consistent.
		void Transform(const toon::SE3<>& M);
		void Transform(const toon::SO3<>& R);
		void Translate(const Vec3& delta);
		// Scale the map by a specified factor. The camera positions will
		// be scaled by the reciprocal to keep everything consistent (the
		// rotations will remain perfect rotations).
		void Scale(double s);
		// Translate the map such that the centroid of the points is at
		// the origin.
		void CenterAtOrigin();
		// Translate the map at the origin and scale so that the radius of
		// the map is radius.
		void Normalize(double radius=1.);
	private:
		// Id-to-frame map
		map<int, Frame*> frames_by_id;
	};
}
