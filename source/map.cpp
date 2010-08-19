#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <so3.h>
#include <LU.h>
#include <determinant.h>

#include <VW/Image/imagecopy.tpp>

#include "map.h"
#include "safe_stream.h"
#include "common_types.h"
#include "tinyxml.h"
#include "lazyvar.h"
#include "image_utils.h"
#include "io_utils.tpp"

#include "counted_foreach.tpp"
#include "math_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
using namespace toon;

lazyvar<int> gvBoundPercentile("Map.BoundPercentile");
lazyvar<bool> gvLoadOriginalFrames("Map.LoadOriginalFrames");
lazyvar<int> gvLinearizeCamera("Map.LinearizeCamera");
lazyvar<string> gvPtamDir("Map.PtamDir");


void Frame::Configure(Map* m, int i, const string& im_file, const SE3<>& pose) {
	map = m;
	id = i;
	image_file = im_file;
	pc.reset(new PosedCamera(pose, *map->camera));
	image.SetPC(*pc);
}

void Frame::LoadImage(bool undistort) {
	if (!image.loaded()) {
		image.Load(image_file);
		if (undistort) {
			UndistortImage();
		}
	}
}

void Frame::UnloadImage() {
	if (image.loaded()) {
		image.Unload();
	}
}

void Frame::UndistortImage() {
	if (map->undistorter.input_size != image.sz()) {
		map->undistorter.Compute(image.sz());
	}
	unwarped.Compute(image, map->undistorter);
}

void KeyFrame::RunGuidedLineDetector() {
	guided_line_detector.Compute(image);
}




Map::Map() {
	orig_camera.reset(new Camera);
	if (*gvLinearizeCamera) {
		// Use a fast approximation to the camera
		Mat3 cam = orig_camera->Linearize();
		camera.reset(new LinearCamera(cam, orig_camera->image_size()));

		// Report the deviation
		double err = CameraBase::GetMaxDeviation(*orig_camera, *camera);
		DLOG << "Using a linear camera approximation with error=" << err;
	} else {
		camera = orig_camera;
	}
}

void Map::Load(const string& path) {
	// Read the keyframe files one-by-one
	TiXmlDocument doc;
	CHECK(doc.LoadFile(path.c_str()))
	<< "Failed to load " << path;
	fs::path xml_dir(fs::path(path).parent_path());
	fs::path ptam_dir(*gvPtamDir);
	const TiXmlElement* root_elem = doc.RootElement();

	// Read frame poses
	int next_id = 0;
	const TiXmlElement* frames_elem = root_elem->FirstChildElement("FramePoses");
	if (frames_elem != NULL) { // FramePoses is optional
		for (const TiXmlElement* frame_elem = frames_elem->FirstChildElement("Frame");
				frame_elem != NULL;
				frame_elem = frame_elem->NextSiblingElement("Frame")) {
			string image_file = (ptam_dir/frame_elem->Attribute("name")).string();
			Vec6 lnPose = stream_to<Vec6>(frame_elem->Attribute("pose"));
			Frame* f = new Frame;  // will be owned by the ptr_vector
			f->Configure(this, next_id++, image_file, SE3<>::exp(lnPose));
			bool lost = frame_elem->Attribute("lost") == "1";
			f->initializing = norm_sq(lnPose) == 0;
			f->lost = lost || f->initializing;
			frame_specs.push_back(f);
		}
	}

	// Read the points
	map<int, int> points_id_to_index;
	const TiXmlElement* pts_elem = root_elem->FirstChildElement("MapPoints");
	CHECK(pts_elem) << "There was no <MapPoints> element in the map spec.";
	for (const TiXmlElement* pt_elem = pts_elem->FirstChildElement("MapPoint");
			pt_elem != NULL;
			pt_elem = pt_elem->NextSiblingElement("MapPoint")) {
		int id = lexical_cast<int>(pt_elem->Attribute("id"));
		points_id_to_index[id] = pts.size();  // indices start from zero
		pts.push_back(stream_to<Vec3>(pt_elem->Attribute("position")));
	}

	// Read key frames
	const TiXmlElement* kfs_elem = root_elem->FirstChildElement("KeyFrames");
	CHECK(kfs_elem) << "There was no <KeyFrames> element in the map spec.";

	// Create an ID-to-XMLElement map
	for (const TiXmlElement* kf_elem = kfs_elem->FirstChildElement("KeyFrame");
			kf_elem != NULL;
			kf_elem = kf_elem->NextSiblingElement("KeyFrame")) {
		// Get the id, pose, and hash
		int id = lexical_cast<int>(kf_elem->Attribute("id"));
		Vec6 lnPose = stream_to<Vec6>(kf_elem->Attribute("pose"));
		string hash = kf_elem->FirstChildElement("Image")->Attribute("md5");

		// Get the filename
		fs::path image_file;
		if (kf_elem->Attribute("name") != NULL) {
			image_file = kf_elem->Attribute("name");
		}
		if (image_file.empty() || !*gvLoadOriginalFrames) {
			DLOG << "Falling back to monochrome image for keyframe "<<id;
			image_file = xml_dir/kf_elem->FirstChildElement("Image")->Attribute("file");
		} else {
			image_file = ptam_dir/image_file;
		}

		// Add the keyframe
		KeyFrame* kf = new KeyFrame;
		kf->Configure(this, id, image_file.string(), SE3<>::exp(lnPose));
		kf->image_hash = hash;
		kf->lost = false;
		kf->initializing = false;
		kfs.push_back(kf);

		// Add the measurements
		const TiXmlElement* msms_elem = kf_elem->FirstChildElement("Measurements");
		for (const TiXmlElement* msm_elem = msms_elem->FirstChildElement("Measurement");
				msm_elem != NULL;
				msm_elem = msm_elem->NextSiblingElement("Measurement")) {
			Measurement msm;
			int point_id = lexical_cast<int>(msm_elem->Attribute("id"));
			map<int, int>::const_iterator it = points_id_to_index.find(point_id);
			if (it == points_id_to_index.end()) {
				DLOG << "No point with ID="<<point_id;
			} else {
				msm.point_index = it->second;
				msm.image_pos = stream_to<Vec2>(msm_elem->Attribute("v2RootPos"));
				msm.retina_pos = stream_to<Vec2>(msm_elem->Attribute("v2ImplanePos"));
				msm.pyramid_level = lexical_cast<int>(msm_elem->Attribute("nLevel"));
			}
			kf->measurements.push_back(msm);
		}

		CHECK(kfs_by_id.find(id) == kfs_by_id.end())
		<< "A keyframe with ID="<<id<<" already loaded";
		kfs_by_id[id] = kf;
	}

	DLOG << "Loaded " << pts.size() << " map points and " << kfs.size() << " keyframes";
}

void Map::LoadWithGroundTruth(const string& path, proto::TruthedMap& tru_map) {
	ReadProto(path, tru_map);
	Load(tru_map.spec_file());
	RotateToSceneFrame(SO3<>::exp(asToon(tru_map.ln_scene_from_slam())));
}

void Map::LoadXml(const string& xml_file) {
	Load(xml_file);
}

void Map::LoadImages(bool undistort) {
	BOOST_FOREACH(KeyFrame& kf, kfs) {
		kf.LoadImage();
		if (undistort) {
			kf.UndistortImage();
		}
	}
}

KeyFrame* Map::KeyFrameById(int id) {
	map<int, KeyFrame*>::iterator it = kfs_by_id.find(id);
	if (it == kfs_by_id.end()) {
		return NULL;
	} else {
		return it->second;
	}
}

const KeyFrame* Map::KeyFrameById(int id) const {
	map<int, KeyFrame*>::const_iterator it = kfs_by_id.find(id);
	if (it == kfs_by_id.end()) {
		return NULL;
	} else {
		return it->second;
	}
}

KeyFrame* Map::KeyFrameByIdOrDie(int id) {
	KeyFrame* kf = KeyFrameById(id);
	CHECK_NOT_NULL(kf) << "No keyframe with ID=" << id;
	return kf;
}

const KeyFrame* Map::KeyFrameByIdOrDie(int id) const {
	const KeyFrame* kf = KeyFrameById(id);
	CHECK_NOT_NULL(kf) << "No keyframe with ID=" << id;
	return kf;
}

void Map::Transform(const SE3<>& M) {
	// Transform the frames
	SE3<> M_inv = M.inverse();
	BOOST_FOREACH(KeyFrame& kf, kfs) {
		kf.pc->Transform(M_inv);
	}
	BOOST_FOREACH(Frame& f, frame_specs) {
		f.pc->Transform(M_inv);
	}

	// Transform the points
	BOOST_FOREACH(Vec3& v, pts) {
		v = M*v;
	}
}

void Map::Rotate(const SO3<>& R) {
	Vec3 t = Zeros;
	Transform(SE3<>(R,t));
}

void Map::DetectLines() {
	// Detect lines in each keyframe
	segments.clear();
	COUNTED_FOREACH(int i, KeyFrame& kf, kfs) {
		// TODO: check that this still works, or go back to kf.vpt_homog_dual
		Mat3 vpt_homog = kf.pc->pose().get_rotation().inverse() * kf.unwarped.image_to_retina;
		Mat3 vpt_homog_inv = LU<3>(vpt_homog).get_inverse();

		CHECK_GT(kf.unwarped.image.nx(), 0)
		<< "Unwarped image not initialized, perhaps this->auto_undistort=false?";
		kf.line_detector.Compute(kf.unwarped.image);
		BOOST_FOREACH(LineDetection& det, kf.line_detector.detections) {
			det.eqn = vpt_homog_inv.T() * det.eqn;
			segments.push_back(det);
		}
	}
	manhattan_est.Prepare(segments);
	manhattan_est.Bootstrap(segments);
}

void Map::InitializeUndistorter(const Vec2I& imsize) {
	Vec2I sz = imsize;
	if (sz[0] == 0 && sz[1] == 0) {
		CHECK(!kfs.empty())	<< "If no size is passed to Map::InitializeUndistorter then "
				<< "there must be at least one keyframe loaded";
		sz = asToon(kfs[0].image.sz());
	}
	undistorter.Compute(asIR(sz));
}

void Map::RunManhattanEstimator() {
	// Compute scene rotation
	manhattan_est.Compute(segments);

	// Propagate axis info back to original detections
	int src = 0, basei = 0;
	COUNTED_FOREACH(int i, const LineDetection& det, segments) {
		if (i-basei >= kfs[src].line_detector.detections.size()) {
			src++;
			basei = i;
		}
		kfs[src].line_detector.detections[i-basei].axis = det.axis;
	}

	// Propagate vanishing points back to keyframes
	BOOST_FOREACH(KeyFrame& kf, kfs) {
		for (int i = 0; i < 3; i++) {
			kf.retina_vpts[i] = kf.pc->pose().get_rotation() * col(manhattan_est.R, i);
			kf.image_vpts[i] = kf.pc->RetToIm(kf.retina_vpts[i]);
		}
	}
}

void Map::EstimateSceneRotation() {
	// Estimate the scene rotation
	DetectLines();
	RunManhattanEstimator();

	// Count the number of keyframes for which each vanishing point
	// has the largest absolute Y coordinate.
	Vec3I up_votes = Zeros;
	BOOST_FOREACH(const KeyFrame& kf, kfs) {
		double maxy = 0;
		int maxi;
		// Find the vanishing point with largest absolute Y coordinate
		for (int i = 0; i < 3; i++) {
			double y = abs(project(kf.image_vpts[i])[1]);
			if (y > maxy) {
				maxy = y;
				maxi = i;
			}
		}
		up_votes[maxi]++;
	}

	// Re-order the cols of the rotation so that the up direction is the Z axis.
	// Note that this is equivalent to swapping _rows_ in R^-1
	int updir = max_index(&up_votes[0], &up_votes[3]);
	if (updir != 2) {
		Mat3 m = manhattan_est.R.get_matrix().T();
		Vec3 m2 = m[2];
		m[2] = m[updir];
		m[updir] = m2;

		// If the determinint is -1 (i.e. R is a rotoinversion) then
		// SO3<>::exp and SO3<>::ln don't work. We could invert either
		// the X or Y axis; here we arbitrarily choose the X axis.
		if (determinant(m) < 0) {
			m[0] = -m[0];
		}

		manhattan_est.R = m.T();
	}

	// Save the rotation
	scene_from_slam = manhattan_est.R.inverse();
}

void Map::RunGuidedLineDetectors() {
	COUNTED_FOREACH(int i, KeyFrame& kf, kfs) {
		DLOG << "Computing guided lines for keyframe ID=" << kf.id;
		INDENTED kf.RunGuidedLineDetector();
	}
}

void Map::RotateToSceneFrame() {
	EstimateSceneRotation();
	RotateToSceneFrame(scene_from_slam);
}

void Map::RotateToSceneFrame(const SO3<>& R) {
	scene_from_slam = R;
	Rotate(R);
}
}
