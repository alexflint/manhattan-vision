#pragma once

#include "common_types.h"

#include "image_bundle.h"
#include "unwarped_image.h"
#include "vanishing_points.h"
#include "textons.h"
#include "line_detector.h"
#include "guided_line_detector.h"

namespace indoor_context {

// Import boost for this file only
namespace { using namespace boost; }

// forward decls
class Map;
namespace proto { class TruthedMap; }


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
	// Pose of this keyframe, or null if the camera was lost
	// Equal to &this->image.pc()
	PosedCamera* pc;

	// Path to the image file for this frame (if any)
	string image_file;
	// The MD5 hash from the map XML
	string image_hash;
	// The unwarped image
	UnwarpedImage unwarped;

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
	// Load the image into this->rawimage. If undistort is true, also use the camera
	// model to undistort the image.
	void LoadImage(bool undistort=false);
	void UnloadImage();
	// Undistort the current image using the camera model
	void UndistortImage();
};

// Represents a frame that was selected as a key frame by PTAM
class KeyFrame : public Frame {
public:
	// The SLAM landmarks that were observed in this frame
	vector<Measurement> measurements;

	// The canny line detector
	CannyLineDetector line_detector;
	// Posed image, initialized in RunGuidedLineDetector
	//scoped_ptr<PosedImage> pim;
	// The guided line detector (for after canonical frame is established)
	GuidedLineDetector guided_line_detector;

	// Vanishing points in the retina plane
	toon::Vector<3> retina_vpts[3];
	// Vanishing points in the image plane
	toon::Vector<3> image_vpts[3];

	// Load a keyframe from a file
	//void Load(const string& image_file, const string& pose_file);
	// Compute textons for this keyframe's image
	void ComputeTextonMap(const TextonVocab& vocab);
	// Run the guided line detector: this->pc must be initialized
	void RunGuidedLineDetector();
};

// Represents a map from PTAM
class Map {
public:
	// The dir for this map
	string dir;
	// The number of keyframes in the dir (they might not all be loaded!)
	int num_kfs_available;
	// The frame information read from XML
	ptr_vector<Frame> frame_specs;

	// The keyframes
	ptr_vector<KeyFrame> kfs;
	// Id-to-keyframe map
	map<int, KeyFrame*> kfs_by_id;
	// The points
	vector<toon::Vector<3> > pts;

	// The camera model for frames in this map
	shared_ptr<Camera> orig_camera;
	shared_ptr<CameraBase> camera;

	// The undistort map, cached for performance
	UndistortMap undistorter;

	// Line segments on the plane-at-infinity
	vector<LineDetection> segments;
	// Global vanishing point detector
	ManhattanFrameEstimator manhattan_est;

	// Rotation from SLAM coordinates to canonical scene coordinates
	toon::SO3<> scene_from_slam;

	// Initialize values from gvars
	Map();
	// Load a map, detecting its format automatically from GVars
	void Load(const string& path);
	// Load a map with ground truth. Store the ground truth in tru_map.
	void LoadWithGroundTruth(const string& path, proto::TruthedMap& tru_map);
	// Deprecated: use Load(xml_file) instead
	void LoadXml(const string& path);

	// Load images for each keyframe
	void LoadImages(bool undistort=false);

	// Apply a rigid transformation to the map. Modifies the keyframe
	// poses and map points.
	void Transform(const toon::SE3<>& M);
	// As above but for pure rotations.
	void Rotate(const toon::SO3<>& R);

	// Get a key frame by its index in the original map. Return NULL
	// if this keyframe is not loaded or does not exist.
	KeyFrame* KeyFrameById(int id);
	const KeyFrame* KeyFrameById(int id) const;
	// As above but die with an error message if specified key frame not present.
	KeyFrame* KeyFrameByIdOrDie(int id);
	const KeyFrame* KeyFrameByIdOrDie(int id) const;

	// Get a frame by ID, and load its image
	PosedImage& ImageByIdOrDie(int id);

	// Initialize the undistort map. If no image size is specified
	// then the size of the first keyframe will be used. If no
	// keyframes are loaded then an error will be raised.
	void InitializeUndistorter(const Vec2I& size=toon::Zeros);
	// Detect lines in each keyframe and project to plane at infinity
	void DetectLines();
	// Compute vanishing points globally and project back into frames
	void RunManhattanEstimator();
	// Detect vanishing points and compute the rotation between SLAM
	// coordinates and (axis-aligned) scene coordinates.
	void EstimateSceneRotation();
	// Compute bounds of the map, discarding outliers for robustness
	//void ComputeRobustBounds();
	// Run the guided line detector (Manhattan frame must be established)
	void RunGuidedLineDetectors();

	// Do all of the above. Compute vanishing points, estimate the
	// scene rotation, and transform the map so that 3D surfaces and
	// edges are axis-aligned.
	void RotateToSceneFrame();
	// Rotate to scene frame with a pre-computed rotation. Does
	// Rotate() and ComputeRobustBounds(), etc.
	void RotateToSceneFrame(const toon::SO3<>& scene_from_slam);
};
}
