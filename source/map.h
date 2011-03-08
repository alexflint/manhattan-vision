#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "camera.h"
#include "map.pb.h"

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
	// The MD5 hash from the map XML
	string image_hash;

	// True iff the camera was lost (or initializing) at this frame
	bool lost;
	// True iff the camera was initializing at this frame
	bool initializing;

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

// Represents a frame that was selected as a key frame by PTAM
class KeyFrame : public Frame {
public:
	// The SLAM landmarks that were observed in this frame
	vector<Measurement> measurements;
	// Output a vector of all points measured in this frame
	void GetMeasuredPoints(vector<Vec3>& out) const;
};

// Represents a map from PTAM
class Map {
public:
	// The dir for this map
	string dir;
	// The number of keyframes in the dir (they might not all be loaded!)
	int num_kfs_available;
	// The frame information read from XML
	ptr_vector<Frame> frames;

	// The keyframes
	ptr_vector<KeyFrame> kfs;
	// Id-to-keyframe map
	map<int, KeyFrame*> kfs_by_id;
	// The points
	vector<Vec3> pts;

	// The camera model for frames in this map
	shared_ptr<ATANCamera> orig_camera;
	shared_ptr<CameraBase> camera;

	// Rotation from SLAM coordinates to canonical scene coordinates
	toon::SO3<> scene_from_slam;

	// Initialize values from gvars
	Map();
	// Load a map, detecting its format automatically from GVars
	void Load(const string& path);
	// Load a map with ground truth. Store the ground truth in gt_map.
	void LoadWithGroundTruth(const string& path, proto::TruthedMap& gt_map);
	// Deprecated: use Load(xml_file) instead
	void LoadXml(const string& path);

	// Load images for each keyframe
	void LoadImages();

	// Apply a rigid transformation to the map. Modifies the keyframe
	// poses and map points.
	void Transform(const toon::SE3<>& M);
	// As above but for pure rotations.
	void Rotate(const toon::SO3<>& R);
	// Rotate to scene frame with a pre-computed rotation. Just calls
	// Rotate() and saves the transform as this->scene_from_slam
	void RotateToSceneFrame(const toon::SO3<>& scene_from_slam);

	// Get a key frame by its index in the original map. Return NULL
	// if this keyframe is not loaded or does not exist.
	KeyFrame* KeyFrameById(int id);
	const KeyFrame* KeyFrameById(int id) const;
	// As above but die with an error message if specified key frame not present.
	KeyFrame* KeyFrameByIdOrDie(int id);
	const KeyFrame* KeyFrameByIdOrDie(int id) const;

	// Get a frame by ID, and load its image
	PosedImage& ImageByIdOrDie(int id);
};
}
