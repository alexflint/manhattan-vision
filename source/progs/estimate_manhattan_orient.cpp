#include <boost/filesystem.hpp>

#include <TooN/determinant.h>

#include "entrypoint_types.h"
#include "timer.h"
#include "safe_stream.h"
#include "map.h"
#include "map_io.h"
#include "bld_helpers.h"
#include "canvas.h"
#include "manhattan_ground_truth.h"

#include "geom_utils.h"
#include "vpt_utils.h"
#include "line_detector.h"
#include "line_detector_bank.h"
#include "vanishing_points.h"
#include "vanishing_point_model.h"

#include "drawing.h"

#include "range_utils.tpp"
#include "counted_foreach.tpp"
#include "format_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"

lazyvar<double> gvNumNeighbours("NormalRotationEstimation.NumNeighbours");
lazyvar<int> gvNumBins("NormalRotationEstimation.NumBins");
lazyvar<double> gvPerpTolerance("NormalRotationEstimation.PerpTolerance");


// Get a list of the point indices observed from any of a set of
// frames (no index is repeated).
void GetObservedIds(const vector<const Frame*>& frames,
										vector<int>& ids) {
	set<int> point_ids;
	BOOST_FOREACH(const Frame* frame, frames) {
		BOOST_FOREACH(const Measurement& msm, frame->measurements) {
			point_ids.insert(msm.point_index);
		}
	}
	copy_all_into(point_ids, ids);
}


// Compares points by distance to a pivot. More distant points are
// prioritised highest.
struct DistCompare {
	Vec3 x;
	DistCompare(const Vec3& xx) : x(xx) { }
	bool operator()(const Vec3& a, const Vec3& b) {
		return norm_sq(a-x) < norm_sq(b-x);
	}
};

void EstimateNormals(const vector<Vec3>& points,
										 vector<Vec3>& normals,
										 int nnbrs) {
	CHECK(normals.empty());
	normals.reserve(points.size());

	// Estimate normals at each point
	BOOST_FOREACH(const Vec3& x, points) {
		DistCompare comp(x);
		priority_queue<Vec3, vector<Vec3>, DistCompare> Q(comp);
		BOOST_FOREACH(const Vec3& y, points) {
			Q.push(y);
			if (Q.size() > nnbrs) {
				Q.pop();
			}
		}
		CHECK(Q.size() >= 3) << "Too few points to estimate plane from";

		// Collect neighbours
		vector<Vec3> nbrs;
		while (!Q.empty()) {
			nbrs.push_back(Q.top());
			Q.pop();
		}

		// Estimate plane
		Vec3 normal;
		if (nbrs.size() == 3) {
			normal = unit((nbrs[1]-nbrs[0]) ^ (nbrs[2]-nbrs[0]));
		} else {
			normal = unit(FitPlane(nbrs).slice<0,3>());
		}
		normals.push_back(normal);
	}
}


/////////////////////////////////////////////////////////////
class AlgebraicRotationEstimator {
public:
	// The "inner" estimator
	ManhattanFrameEstimator manhattan_est;
	// Constructor
	AlgebraicRotationEstimator() { }
	// Compute rotation from line detections in a subset of the frames
	SO3<> Compute(const vector<LineObservation>& observations);
};

SO3<> AlgebraicRotationEstimator::Compute(const vector<LineObservation>& observations) {
	vector<LineDetection> detections;
	BOOST_FOREACH(const LineObservation& obs, observations) {
		// Linearize camera
		Mat3 cam = obs.camera->camera().Linearize();

		// Let R be the camera rotation and C be the camera intrinsics
		// Then the transformation on points at infinity (into the camera) is:
		//   C*R
		// and the transformation "out" of the camera is
		//   (C*R)^-1
		// so the transformation for lines "out" of the camera is:
		//   H = ((C*R)^-1)^-T
		//     = R^T * C^T
		Mat3 H = obs.camera->pose().get_rotation().inverse() * cam.T();

		// Compute line segments on plane-at-infinity
		LineDetection det(obs.segment.start, obs.segment.end);
		det.eqn = H * det.eqn;
		detections.push_back(det);
	}

	// Estimate rotation
	DLOG << "Estimating rotation from " << detections.size() << " observations";
	manhattan_est.Compute(detections);
	return manhattan_est.R;
}


/////////////////////////////////////////////////////////////
class NormalRotationEstimator {
public:
	// The estimated normal vectors
	vector<Vec3> normals_;

	// Initialize empty
	NormalRotationEstimator() { }
	// Initialize with points
	NormalRotationEstimator(const vector<Vec3>& points) {
		Configure(points);
	}

	// Estimates normals
	void Configure(const vector<Vec3>& points);
	// Computes the Manhattan rotation from all points
	SO3<> Compute();
	// Computes the Manhattan rotation from a subset of points
	SO3<> Compute(const vector<int>& ids);
};

void NormalRotationEstimator::Configure(const vector<Vec3>& points) {
	normals_.clear();
	TITLED("Estimating point cloud normals")
		EstimateNormals(points, normals_, *gvNumNeighbours);
}

SO3<> NormalRotationEstimator::Compute() {
	CHECK(!normals_.empty());
	vector<int> ids(normals_.size());
	for (int i = 0; i < normals_.size(); i++) {
		ids[i] = i;
	}
	return Compute(ids);
}

SO3<> NormalRotationEstimator::Compute(const vector<int>& ids) {
	vector<Vec3> ns;
	copy_subset_into(normals_, ids, ns);
	DLOG << "Using "<<ids.size()<<" normals out of "<<normals_.size();

	// Cluster the normals
	const int nbins = *gvNumBins;
	int histogram[nbins][nbins];
	for (int i = 0; i < nbins; i++) {
		for (int j = 0; j < nbins; j++) {
			histogram[i][j] = 0;
		}
	}
	BOOST_FOREACH(const Vec3& n, ns) {
		double theta = acos(n[2]);
		double psi = UpAngle(atan(n[1]/n[0]));
		int tbin = floori(theta * nbins / M_PI);
		int pbin = floori(psi * nbins / M_PI);
		CHECK_INTERVAL(tbin, 0, nbins-1) << "Theta="<<theta<<", Psi="<<psi;
		CHECK_INTERVAL(pbin, 0, nbins-1) << "Theta="<<theta<<", Psi="<<psi;;
		histogram[tbin][pbin]++;
	}

	// Identify the largest bin
	int imax = 0, jmax = 0;
	for (int i = 0; i < nbins; i++) {
		for (int j = 0; j < nbins; j++) {
			if (histogram[i][j] > histogram[imax][jmax]) {
				imax = i;
				jmax = j;
			}
		}
	}
	double thetamax = (imax+.5) * M_PI / nbins;
	double psimax = (jmax+.5) * M_PI / nbins;
	Vec3 nmax = SphericalToEuclidean(thetamax, psimax);

	// Identify the largest bin orthog to the first one
	imax = jmax = -1;
	Vec3 n2max;
	for (int i = 0; i < nbins; i++) {
		for (int j = 0; j < nbins; j++) {
			double theta = (i+.5) * M_PI / nbins;
			double psi = (j+.5) * M_PI / nbins;
			Vec3 n = SphericalToEuclidean(theta, psi);
			double a = acos(abs(n*nmax));
			double degs = 90 - a * 180. / M_PI;
			if (degs < *gvPerpTolerance) {
				if (imax == -1 || histogram[i][j] > histogram[imax][jmax]) {
					imax = i;
					jmax = j;
					n2max = n;
				}
			}
		}
	}
	CHECK_NE(imax, -1) << "Found no bins perpendindicular to the first one!";

	// Create the rotation
	Mat3 R;
	R[0] = nmax;
	R[1] = nmax ^ n2max;
	R[2] = R[0] ^ R[1];

	return SO3<>(NormalizeRotation(R));
}

static const int kNumWindows = 1;
VecI kWindows = asVNL(makeVector(3));

int main(int argc, char **argv) {
	InitVars(argc, argv);
	AssertionManager::SetExceptionMode();

	CHECK_EQ(kNumWindows, kWindows.size());

	if (argc < 2 || argc > 3) {
		DLOG << "Usage: estimate_manhattan_orientation SEQUENCE";
		exit(-1);
	}

	// Read parameters
	string sequence = argv[1];
	vector<int> base_frame_ids;
	if (argc == 3) {
		base_frame_ids = ParseMultiRange<int>(argv[2]);
	}

	// Set up results dir
	fs::path results_dir = fs::initial_path();
	CHECK_PRED1(fs::exists, results_dir);


	// Create vizualization dir
	fs::path viz_dir = results_dir / "out";
	if (fs::create_directory(viz_dir)) {
		DLOG << "Created output directory: " << viz_dir;
	}

	// Open the CSV file
	ofstream stats_out;
	string stats_fname = fmt("performance_%s.csv", basename(results_dir));
	fs::path stats_path = results_dir / stats_fname;
	stats_out.open(stats_path.string().c_str());
	stats_out << "\"Number of frames\",\"Accuracy (Algebraic)\","
						<< "\"Accuracy (Furukawa)\",\"Accuracy (Geometric)\"\n";
	format stats_line("%d,%f,%f,%f");

	// Write the parameters
	string params_fname = fmt("parameters_%s.csv", basename(results_dir));
	fs::path params_path = results_dir / params_fname;
	ofstream params_out(params_path.string().c_str());
	GV3::print_var_list(params_out);
	params_out.close();

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	LoadXmlMapWithGroundTruth(GetMapPath(sequence), map, gt_map, false, false);
	SO3<> R_orig_gt = SO3<>::exp(asToon(gt_map.ln_scene_from_slam()));
	map.LoadAllImages();

	// Set up the line detection manager
	LineDetectorBank line_bank(map);

	// Initialize the normal-based rotation estimator
	NormalRotationEstimator normal_est(map.points);

	// Initialize the algebraic rotation estimator
	AlgebraicRotationEstimator algebraic_est;

	// Initialize the geometric estimator
	VanishingPointModel model(6.);

	// Compute the "ground truth" from all frames together
	vector<int> gt_frame_ids;
	vector<LineObservation> gt_observations;
	int nf = map.frames.size();
	for (int i = 5; i < nf-5; i += (nf-10)/10) {
		gt_frame_ids.push_back(i);
		line_bank.GetObservationsFor(i, gt_observations);
	}
	VanishingPointEstimator gt_est(gt_observations, model);
	SO3<> R_gt = gt_est.Compute(R_orig_gt);

	if (base_frame_ids.empty()) {
		copy_all_into(gt_frame_ids, base_frame_ids);
	}

	// Visualize
	int viz_id = 1;
	vector<LineSegment> viz_segs;
	line_bank.GetDetectionsFor(viz_id, viz_segs);
	OutputVptViz(map.GetFrameByIdOrDie(viz_id)->image,
							 viz_segs,
							 R_gt,
							 "out/R_gt.pdf");

	Vector<kNumWindows> sum_alg_err = Zeros;
	Vector<kNumWindows> sum_nrml_err = Zeros;
	Vector<kNumWindows> sum_geom_err = Zeros;
	Vector<kNumWindows> sum_sin_err = Zeros;
	int num_samples = 0;

	BOOST_FOREACH(int base_id, base_frame_ids) {
		TITLE("Next Experiment");
		for (int j = 0; j < kNumWindows; j++) {
			// Collect frames
			vector<int> frame_ids;
			vector<const Frame*> frames;
			DLOG_N << "Using frames: ";
			for (int id = max(0,base_id-kWindows[j]);
					 id <= min(base_id+kWindows[j], map.frames.size()-1);
					 id++) {
				frame_ids.push_back(id);
				frames.push_back(map.GetFrameById(id));
				DLOG_N << id << " ";
			}
			DLOG_N << endl;


			SO3<> R_alg, R_nrml, R_geom, R_sin;

			// Collect line segments
			vector<LineObservation> observations;
			BOOST_FOREACH(int frame_id, frame_ids) {
				line_bank.GetObservationsFor(frame_id, observations);
			}

			// Estimate rotation from line segments
			TITLED("Estimating algebraically") {
				SO3<> R = algebraic_est.Compute(observations);
				double err = RotationDistance(R, R_gt);
				DLOG << "Error = " << err;
				sum_alg_err[j] += err;
				R_alg = R;
			}

			// Estimate rotation from point normals
			TITLED("Estimating from points") {
				vector<int> ids;
				GetObservedIds(frames, ids);
				SO3<> R = normal_est.Compute(ids);
				double err = RotationDistance(R, R_gt);
				DLOG << " Error = " << err;
				sum_nrml_err[j] += err;
				R_nrml = R;
			}

			// Estimate rotation using a single image only
			TITLED("Estimating from single image") {
				vector<LineObservation> sin_observations;
				line_bank.GetObservationsFor(base_id, sin_observations);
				VanishingPointEstimator vpt_estimator(sin_observations, model);
				SO3<> R = vpt_estimator.Compute(R_orig_gt);
				double err = RotationDistance(R, R_gt);
				DLOG << "Error = " << err;
				sum_sin_err[j] += err;
				R_sin = R;
			}

			// Estimate rotation using geometric error
			TITLED("Estimating geometrically (correctly!)") {
				VanishingPointEstimator vpt_estimator(observations, model);
				SO3<> R = vpt_estimator.Compute(R_orig_gt);
				double err = RotationDistance(R, R_gt);
				DLOG << "Error = " << err;
				sum_geom_err[j] += err;
				R_geom = R;
			}


			// Visualize
			TITLED("Outputting Visualizations") {
				vector<LineSegment> segments;
				line_bank.GetDetectionsFor(base_id, segments);
				format filepat("out/%s_frame%02d_%s_window%02d.pdf");
				// Draw line segment version
				OutputVptViz(map.GetFrameById(base_id)->image,
										 segments,
										 R_alg,
										 str(filepat % sequence % base_id % "vpts_algebraic" % kWindows[j]));
				OutputVptViz(map.GetFrameById(base_id)->image,
										 segments,
										 R_nrml,
										 str(filepat % sequence % base_id % "vpts_normals" % kWindows[j]));
				OutputVptViz(map.GetFrameById(base_id)->image,
										 segments,
										 R_geom,
										 str(filepat % sequence % base_id % "vpts_geometric" % kWindows[j]));
				OutputVptViz(map.GetFrameById(base_id)->image,
										 segments,
										 R_sin,
										 str(filepat % sequence % base_id % "vpts_singleimage" % kWindows[j]));
				OutputVptViz(map.GetFrameById(base_id)->image,
										 segments,
										 R_gt,
										 str(filepat % sequence % base_id % "vpts_true" % kWindows[j]));
			}
		}

		// Increment normalizer
		num_samples++;
	}

	// Compute averages
	for (int i = 0; i < kNumWindows; i++) {
		double av_alg_err = sum_alg_err[i] / num_samples;
		double av_nrml_err = sum_nrml_err[i] / num_samples;
		double av_geom_err = sum_geom_err[i] / num_samples;
		double av_sin_err = sum_sin_err[i] / num_samples;
		TITLED("Window="<<kWindows[i]<< ": ") {
			DLOG << "Mean error (algebraic) = " << av_alg_err;
			DLOG << "Mean error (points) = " << av_nrml_err;
			DLOG << "Mean error (single image) = " << av_sin_err;
			DLOG << "Mean error (geometric) = " << av_geom_err;
		}
		int nf = kWindows[i]*2+1;
		stats_out << stats_line % nf % av_alg_err % av_nrml_err % av_geom_err << endl;
	}

	return 0;
}
