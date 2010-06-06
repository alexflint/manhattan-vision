// Commandline interface to find vanishing points in PTAM maps

#include <VW/Image/imagecopy.h>
#include <VNL/vector.tpp>

#include <LU.h>

#include "common_types_vw.h"
#include "timer.h"
#include "vanishing_points.h"
#include "image_bundle.h"
#include "unwarped_image.h"
#include "vars.h"

#include "image_utils.tpp"
#include "math_utils.tpp"

using namespace indoor_context;

class KeyFrame {
public:
	ImageBundle rawimage;  // the raw image from the camera (i.e. warped)
	UnwarpedImage unwarped;
	VanishingPoints vpts;
	TooN::SE3<> CfW;
	// homography between image plane and plane-at-infinity
	TooN::Matrix<3> vpt_homog;
	// inverse of above
	TooN::Matrix<3> vpt_homog_inv;
	// as above but for lines rather than points
	TooN::Matrix<3> vpt_homog_dual;
	// inverse of above
	TooN::Matrix<3> vpt_homog_dual_inv;
	// the LU decomposition

	// Compute the homography between the image plane and the plane-at-infinity
	void ComputeInfPlaneHomography() {
		vpt_homog = CfW.get_rotation().inverse() * unwarped.image_to_retina;
		TooN::LU<3> lu(vpt_homog);
		vpt_homog_inv = lu.get_inverse();
		vpt_homog_dual = vpt_homog_inv.T();
		vpt_homog_dual_inv = vpt_homog.T();
	}

	void Load(const char* imagefile, const char* posefile) {
		// Read image and unwarp
		rawimage.Load(imagefile);
		unwarped.Compute(rawimage);

		// Load world-to-camera xform
		ifstream input(posefile);
		TooN::Vector<6> v;
		input >> v;
		CfW = TooN::SE3<>::exp(v);

		// Compute homography with plane-at-infinity
		ComputeInfPlaneHomography();
	}
};

class Map {
public:
	ptr_vector<KeyFrame> kfs;
	void Load(char*const* kf_files, int nkfs) {
		for (int i = 0; i < nkfs; i++) {
			kfs.push_back(new KeyFrame);
			KeyFrame& kf = kfs.back();
			string imgfile(kf_files[i]);
			string posefile = imgfile.substr(0, imgfile.length()-4)+".info";
			kf.Load(imgfile.c_str(), posefile.c_str());
		}
	}
};


int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc < 2) {
		DLOG << "Usage:"<<argv[0]<<" KEYFRAME1 KEYFRAME2 ...";
		return 0;
	}

	Map map;
	map.Load(argv+1, argc-1);

	// Detect lines and add to global list
	vector<LineSegment> segments;
	for (int i = 0; i < map.kfs.size(); i++) {
		KeyFrame& kf = map.kfs[i];
		kf.ComputeInfPlaneHomography();
		kf.vpts.lines.Compute(kf.unwarped.image);
		kf.vpts.last_input = &kf.unwarped.image;
		BOOST_FOREACH(LineSegment& seg, kf.vpts.lines.segments) {
			//seg.line_cond = fromToon(kf.CfW.get_rotation().inverse() * fromVNL(seg.line_cond));
			seg.line_cond = fromToon(kf.vpt_homog_dual * fromVNL(seg.line));
			segments.push_back(seg);
		}
	}

	// Run joint vanishing point estimation
	VanishingPointDetector vpt_detector;
	vpt_detector.Compute(segments);

	// Save the owners vector
	int src = 0, basei = 0;
	for (int i = 0; i < map.kfs.size(); i++) {
		KeyFrame& kf = map.kfs[i];
		kf.vpts.detector.owners.Resize(kf.vpts.lines.segments.size());
		kf.vpts.detector.vanpts = vpt_detector.vanpts;
	}
	for (int i = 0; i < vpt_detector.owners.Size(); i++) {
		if (i-basei >= map.kfs[src].vpts.lines.segments.size()) {
			src++;
			basei = i;
		}
		map.kfs[src].vpts.detector.owners[i-basei] = vpt_detector.owners[i];
	}

	// Draw the vanishing points
	for (int i = 0; i < map.kfs.size(); i++) {
		BOOST_FOREACH(VanishingPoint& vpt, map.kfs[i].vpts.detector.vanpts) {
			vpt.pt = fromToon(map.kfs[i].vpt_homog_inv * fromVNL(vpt.pt_cond));
		}
		map.kfs[i].vpts.OutputVanPointViz("out/vpts"+PaddedInt(i,1)+".png");
	}

	return 0;
}
