#include <so3.h>
#include <LU.h>
#include <determinant.h>

#include <boost/foreach.hpp>

#include "map.h"
#include "common_types.h"
#include "image_utils.h"

#include "vw_image.tpp"
//#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	Frame::Frame() : map(NULL), id(-1), lost(false), initializing(false) {
	}

	Frame::Frame(int id, const string& image_file, const SE3<>& pose)
		: map(NULL), lost(false), initializing(false) {
		Configure(id, image_file, pose);
	}

	void Frame::Configure(int i, const string& im_file, const SE3<>& pose) {
		id = i;
		image_file = im_file;
		image.pc().SetPose(pose);
	}

	void Frame::SetMap(Map* m) {
		CHECK(m != NULL);
		CHECK(m->camera != NULL);
		map = m;
		image.pc().SetCamera(map->camera.get());
	}

	void Frame::LoadImage() {
		if (!image.loaded()) {
			image.Load(image_file);
		}
	}

	void Frame::UnloadImage() {
		if (image.loaded()) {
			image.Unload();
		}
	}

	void Frame::GetMeasuredPoints(vector<Vec3>& out) const {
		CHECK(map != NULL);
		BOOST_FOREACH(const Measurement& msm, measurements) {
			out.push_back(map->points[msm.point_index]);
		}
	}




	void Map::AddFrame(Frame* frame) {
		frames.push_back(frame);
		frame->SetMap(this);

		CHECK(frames_by_id.find(frame->id) == frames_by_id.end())
			<< "A frame with ID=" << frame->id << " already exists";
		frames_by_id[frame->id] = frame;
	}

	void Map::LoadAllImages() {
		if (frames.size() > 100) {
			DLOG << "Warning: loading " << frames.size() << " images simultaneously";
		}
		BOOST_FOREACH(Frame& frame, frames) {
			frame.LoadImage();
		}
	}

	Frame* Map::GetFrameById(int id) {
		map<int, Frame*>::iterator it = frames_by_id.find(id);
		if (it == frames_by_id.end()) {
			return NULL;
		} else {
			return it->second;
		}
	}

	const Frame* Map::GetFrameById(int id) const {
		map<int, Frame*>::const_iterator it = frames_by_id.find(id);
		if (it == frames_by_id.end()) {
			return NULL;
		} else {
			return it->second;
		}
	}

	Frame* Map::GetFrameByIdOrDie(int id) {
		Frame* frame = GetFrameById(id);
		CHECK_NOT_NULL(frame) << "No frame with ID=" << id;
		return frame;
	}

	const Frame* Map::GetFrameByIdOrDie(int id) const {
		const Frame* frame = GetFrameById(id);
		CHECK_NOT_NULL(frame) << "No frame with ID=" << id;
		return frame;
	}

	PosedImage& Map::GetImageByIdOrDie(int id) {
		Frame* f = GetFrameByIdOrDie(id);
		f->LoadImage();
		PosedImage& im = f->image;
		im.BuildMono();
		return im;
	}


	void Map::Transform(const SE3<>& M) {
		// Transform the frames
		SE3<> M_inv = M.inverse();
		/*BOOST_FOREACH(KeyFrame& kf, kfs) {
			kf.image.pc().Transform(M_inv);
			}*/
		BOOST_FOREACH(Frame& f, frames) {
			f.image.pc().Transform(M_inv);
		}

		// Transform the points
		BOOST_FOREACH(Vec3& v, points) {
			v = M*v;
		}
	}

	void Map::Transform(const SO3<>& R) {
		Vec3 t = Zeros;
		Transform(SE3<>(R,t));
	}

	void Map::Translate(const Vec3& delta) {
		Mat3 R = Identity;
		Transform(SE3<>(R,delta));
	}

	void Map::Scale(double s) {
		// For scaling we apply the same scale factor to both the points
		// and the camera centers, and leave the rotations untouched.
		BOOST_FOREACH(Vec3& v, points) {
			v = s*v;
		}
		BOOST_FOREACH(Frame& f, frames) {
			// This is a non-rigid transformation so it looks a bit ugly
			PosedCamera& pc = f.image.pc();
			pc.SetPose(SE3<>(pc.pose().get_rotation(), s*pc.pose().get_translation()));
		}
	}

	void Map::Normalize(double radius) {
		CenterAtOrigin();
		double maxrad;
		for (int i = 0; i < points.size(); i++) {
			maxrad = max(maxrad, norm(points[i]));
		}
		Scale(radius / maxrad);
	}

		

	void Map::CenterAtOrigin() {
		Vec3 sum = Zeros;
		for (int i = 0; i < points.size(); i++) {
			sum += points[i];
		}
		Vec3 centre = sum / points.size();
		Translate(-centre);
	}


	/*
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

	PosedImage& Map::ImageByIdOrDie(int id) {
		Frame* f = KeyFrameByIdOrDie(id);
		f->LoadImage();
		PosedImage& im = f->image;
		im.BuildMono();
		return im;
	}
	*/

	/*void Map::DetectLines() {
	// Detect lines in each keyframe
	segments.clear();
	COUNTED_FOREACH(int i, KeyFrame& kf, kfs) {
		// TODO: check that this still works, or go back to kf.vpt_homog_dual
		Mat3 vpt_homog = kf.image.pc().pose().get_rotation().inverse() * kf.unwarped.image_to_retina;
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
		sz = kfs[0].image.size();
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
			kf.retina_vpts[i] = kf.image.pc().pose().get_rotation() * col(manhattan_est.R, i);
			kf.image_vpts[i] = kf.image.pc().RetToIm(kf.retina_vpts[i]);
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
	}*/

	/*void Map::RotateToSceneFrame(const SO3<>& R) {
	//scene_from_slam = R;
	Transform(R);
	}*/

}
