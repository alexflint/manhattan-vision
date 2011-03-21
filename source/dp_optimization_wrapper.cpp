// This file is a wrapper to evaluate parameter values given to it by
// a black box optimzier.

#include <exception>

#include <boost/exception/all.hpp>
#include <boost/filesystem.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/thread.hpp>

#include "entrypoint_types.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"
#include "joint_payoffs.h"
#include "manhattan_dp.h"
#include "map.h"
#include "bld_helpers.h"
#include "canvas.h"
#include "manhattan_ground_truth.h"
#include "safe_stream.h"
#include "timer.h"
#include "payoffs.pb.h"
#include "thread_pool.h"

#include "format_utils.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"

lazyvar<string> gvStereoOffsets("JointDP.Stereo.AuxOffsets");

// Ties together sensor data from several sources
class PayoffFeatures {
public:
	DPPayoffs mono_payoffs;
	ptr_vector<MatF> stereo_payoffs;
	MatF agreement_payoffs, occlusion_payoffs;
};

class Parameters {
public:
	float mono_weight;
	float agreement_weight;
	float occlusion_weight;
	float stereo_weight;

	float corner_penalty;
	float occlusion_penalty;
};

class PerformanceStatistics {
public:
	PerformanceStatistics()
		: num_frames(0), sum_depth_error(0), sum_labelling_error(0) { }
	int num_frames;
	float sum_depth_error;
	float sum_labelling_error;
};

// Pack a VNL matrix into a protocol buffer
void PackMatrix(const MatF& in, proto::MatF& out) {
	out.set_rows(in.Rows());
	out.set_cols(in.Cols());
	out.clear_entries();
	for (int y = 0; y < in.Rows(); y++) {
		const float* row = in[y];
		for (int x = 0; x < in.Cols(); x++) {
			out.add_entries(row[x]);
		}
	}
}

// Pack a VNL matrix into a protocol buffer
void PackMatrix(const MatI& in, proto::MatI& out) {
	out.set_rows(in.Rows());
	out.set_cols(in.Cols());
	out.clear_entries();
	for (int y = 0; y < in.Rows(); y++) {
		const int* row = in[y];
		for (int x = 0; x < in.Cols(); x++) {
			out.add_entries(row[x]);
		}
	}
}

// Unpack a VNL matrix from a protocol buffer
void UnpackMatrix(const proto::MatF& in, MatF& out) {
	CHECK_EQ(in.entries_size(), in.rows()*in.cols());
	out.Resize(in.rows(), in.cols());
	int i = 0;
	for (int y = 0; y < in.rows(); y++) {
		float* row = out[y];
		for (int x = 0; x < in.cols(); x++) {
			row[x] = in.entries(i++);
		}
	}
}

// Unpack a VNL matrix from a protocol buffer
void UnpackMatrix(const proto::MatI& in, MatI& out) {
	CHECK_EQ(in.entries_size(), in.rows()*in.cols());
	out.Resize(in.rows(), in.cols());
	int i = 0;
	for (int y = 0; y < in.rows(); y++) {
		int* row = out[y];
		for (int x = 0; x < in.cols(); x++) {
			row[x] = in.entries(i++);
		}
	}
}

void PackFeatures(const vector<int>& aux_ids,
									const JointPayoffGen& joint,
									proto::PayoffFeatures& precomputed) {
	PackMatrix(joint.mono_gen.payoffs.wall_scores[0], *precomputed.mutable_mono0());
	PackMatrix(joint.mono_gen.payoffs.wall_scores[1], *precomputed.mutable_mono1());

	PackMatrix(joint.point_cloud_gen.agreement_payoffs, *precomputed.mutable_agreement());
	PackMatrix(joint.point_cloud_gen.occlusion_payoffs, *precomputed.mutable_occlusion());
	precomputed.set_agreement_sigma(joint.point_cloud_gen.agreement_sigma);

	precomputed.clear_stereo_aux_ids();
	precomputed.clear_stereo();
	for (int i = 0; i < aux_ids.size(); i++) {
		precomputed.add_stereo_aux_ids(aux_ids[i]);
		PackMatrix(joint.stereo_gens[i].payoffs, *precomputed.add_stereo());
	}
}

void UnpackFeatures(const proto::PayoffFeatures& precomputed,
										PayoffFeatures& features) {
	UnpackMatrix(precomputed.mono0(), features.mono_payoffs.wall_scores[0]);
	UnpackMatrix(precomputed.mono1(), features.mono_payoffs.wall_scores[1]);
	UnpackMatrix(precomputed.agreement(), features.agreement_payoffs);
	UnpackMatrix(precomputed.occlusion(), features.occlusion_payoffs);

	features.stereo_payoffs.clear();
	CHECK_EQ(precomputed.stereo_aux_ids_size(), precomputed.stereo_size());
	for (int i = 0; i < precomputed.stereo_size(); i++) {
		features.stereo_payoffs.push_back(new MatF);
		UnpackMatrix(precomputed.stereo(i), features.stereo_payoffs.back());
	}
}

void LoadFeatures(const string& file, PayoffFeatures& features) {
	proto::PayoffFeatures data;
	ReadProto(file, data);
	UnpackFeatures(data, features);
}

void SaveFeatures(const string& file,
									const vector<int>& aux_ids,
									const JointPayoffGen& joint) {
	proto::PayoffFeatures data;
	PackFeatures(aux_ids, joint, data);
	sofstream proto_out(file);
	data.SerializeToOstream(&proto_out);
}

void LoadAndComputePayoffs(const fs::path& file,
													 const Parameters& params,
													 PayoffFeatures& features,
													 DPPayoffs& payoffs) {
	// Load features from file
	CHECK_PRED1(fs::exists, file);
	LoadFeatures(file.string(), features);

	// Mix together to form final payoffs
	payoffs.Resize(matrix_size(features.mono_payoffs));
	payoffs.Add(features.mono_payoffs, params.mono_weight);
	payoffs.Add(features.agreement_payoffs, params.agreement_weight);
	payoffs.Add(features.occlusion_payoffs, params.occlusion_weight);
	if (!features.stereo_payoffs.empty()) {
		double stereo_weight_per_aux = params.stereo_weight / features.stereo_payoffs.size();
		BOOST_FOREACH(const MatF& po, features.stereo_payoffs) {
			// TODO: pass gradient images to stereo (See stereo_dp.cpp)
			payoffs.Add(po, stereo_weight_per_aux);
		}
	}
}

// Globals shared across frame processors
vector<int> stereo_offsets;

fs::path payoffs_dir;
bool load_payoffs, store_payoffs, evaluate;

bool draw_solutions;
fs::path visualization_dir;

Parameters params;
PerformanceStatistics perf;


// Processes frames
class FrameProcessor {
public:
	// Internal buffers
	PayoffFeatures features;
	ManhattanDPReconstructor recon;
	ManhattanGroundTruth gt;
	JointPayoffGen joint;
	vector<Vec3> point_cloud;
	DPPayoffs loaded_payoffs;

	// Mutexes
	static boost::mutex perf_mutex;
	static boost::mutex log_mutex;

	void Process(KeyFrame& frame,
							 const string& sequence,
							 const proto::TruthedMap& gt_map) {
		// Get path
		DPPayoffs* payoffs = NULL;

		// Setup geometry
		DPGeometryWithScale geom(frame.image.pc(),
														 gt_map.floorplan().zfloor(),
														 gt_map.floorplan().zceil());

		fs::path payoffs_file = payoffs_dir / fmt("%s_%03d_precomputed.protodata", sequence, frame.id);
		if (load_payoffs) {
			LoadAndComputePayoffs(payoffs_file.string(), params, features, loaded_payoffs);
			payoffs = &loaded_payoffs;

		} else {
			// Load image and point cloud
			frame.LoadImage();
			point_cloud.clear();
			frame.GetMeasuredPoints(point_cloud);

			// Load auxiliary frames
			vector<const PosedImage*> aux_images;
			vector<int> aux_ids;
			BOOST_FOREACH(int offset, stereo_offsets) {
				Frame* aux_frame = frame.map->KeyFrameById(frame.id+offset);
				if (aux_frame != NULL) {
					aux_frame->LoadImage();
					aux_images.push_back(&aux_frame->image);
					aux_ids.push_back(frame.id+offset);
				}
			}

			// Build payoffs
			joint.mono_weight = params.mono_weight;
			joint.agreement_weight = params.agreement_weight;
			joint.occlusion_weight = params.occlusion_weight;
			joint.stereo_weight = params.stereo_weight;
			joint.Compute(frame.image, geom, point_cloud, aux_images);
			payoffs = &joint.payoffs;

			// Store payoffs?
			if (store_payoffs) {
				SaveFeatures(payoffs_file.string(), aux_ids, joint);
			}
		}

		if (evaluate) {
			// Reconstruct
			CHECK_NOT_NULL(payoffs);
			payoffs->wall_penalty = params.corner_penalty;
			payoffs->occl_penalty = params.occlusion_penalty;
			recon.Compute(frame.image, geom, *payoffs);

			// Compute performance
			gt.Compute(gt_map.floorplan(), frame.image.pc());
			boost::mutex::scoped_lock perf_lock(perf_mutex);
			boost::mutex::scoped_lock log_lock(log_mutex);
			TITLE("Frame " << frame.id) {
				perf.sum_labelling_error += 1. - recon.ReportAccuracy(gt);
				perf.sum_depth_error += recon.ReportDepthError(gt);
				perf.num_frames++;
			}

			// Visualize?
			if (draw_solutions) {
				if (!frame.image.loaded()) {
					frame.LoadImage();
				}

				boost::format viz_filepat("%s/%s_%03d_%s.png");
				viz_filepat.bind_arg(1, visualization_dir.string());
				viz_filepat.bind_arg(2, sequence);
				viz_filepat.bind_arg(3, frame.id);

				// Draw solution
				recon.OutputSolution(str(viz_filepat % "solution"));
			}
		}
	}
};

boost::mutex FrameProcessor::perf_mutex;
boost::mutex FrameProcessor::log_mutex;

template <typename T>
class ThreadLocal {
public:
	boost::ptr_map<boost::thread::id, T> instances;
	boost::mutex get_mutex;

	// Get object for the current thread or construct one
	T& Get(const boost::thread::id& id) {
		// The lock is important as the ptr_map will have internal race
		// conditions (it might need to construct a new object if one
		// doesn't already exist for this thread).
		boost::mutex::scoped_lock lock(get_mutex);
		return instances[id];
	}

	// Get object for the current thread or construct one
	const T& GetConst(const boost::thread::id& id) const {
		return const_cast<ThreadLocal<T>*>(this)->Get(id);
	}

	// Get the object for this thread or construct one
	T& Get() { return Get(this_thread::get_id()); }
	const T& GetConst() const { return GetConst(this_thread::get_id()); }

	// Shortcut for Get
	T& operator*() { return Get(); }
	const T& operator*() const { return Get(); }

	// Shortcut for Get
	T* operator->() { return &Get(); }
	const T* operator->() const { return &Get(); }
};

int main(int argc, char **argv) {
	InitVars(argc, argv);
	AssertionManager::SetExceptionMode();

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
    ("help", "produce help message")
    ("mono_weight", po::value<float>()->required(), "weight for mono payoffs")
    ("stereo_weight", po::value<float>()->required(), "weight for stereo payoffs")
    ("3d_agreement_weight", po::value<float>()->required(), "weight for 3D agreement payoffs")
    ("3d_occlusion_weight", po::value<float>()->required(), "weight for 3D occlusion payoffs")
    ("corner_penalty", po::value<float>()->required(), "per-corner penalty")
    ("occlusion_penalty", po::value<float>()->required(), "additional penalty for occluding corners")
    ("loss", po::value<string>()->default_value("DepthError"), "Loss function: DepthError or LabellingError")

    ("sequence", po::value<vector<string> >()->required(), "sequences to evaluate")
		("frame_stride", po::value<int>()->default_value(1), "number of frames per evaluation")

		("store_payoffs", po::value<string>(), "write payoff matrices to this dir (skips evaluation)")
		("load_payoffs", po::value<string>(), "load payoff matrices from this dir (as generated by --store_payoffs)")
		("evaluate", "if provided, forces evaluation even when --store_payoffs is specified")

    ("concurrency", po::value<int>()->default_value(1), "number of independent threads -- not working yet")

		("visualization_dir", po::value<string>()->default_value("out"), "directory for visualization output")
		("draw_solutions", "draw solutions as PNGs in output directory")
		;

	// Parse options
	po::variables_map opts;
	try {
		po::store(po::parse_command_line(argc, argv, desc), opts);
		po::notify(opts);
	} catch (const po::required_option& ex) {
		cout << "Missing required option: " << ex.get_option_name() << "\n" << desc << "\n";
		return 1;
	}
	if (opts.count("help")) {
    cout << desc << "\n";
    return 1;
	}

	// Deal with params
	vector<string> sequences = opts["sequence"].as<vector<string> >();
	CHECK(!sequences.empty()) << "You must specify at least one sequence with --sequence=<name>";

	int frame_stride = opts["frame_stride"].as<int>();
	CHECK_GT(frame_stride, 0);

	string loss_name = opts["loss"].as<string>();
	CHECK(loss_name == "DepthError" || loss_name == "LabellingError") << EXPR_STR(loss_name);
	bool use_depth_loss = (loss_name == "DepthError");

	// Output dir
	visualization_dir = opts["visualization_dir"].as<string>();
	draw_solutions = opts.count("draw_solutions") > 0;
	if (draw_solutions) {
		fs::create_directory(visualization_dir);
	}

	// Are we loading or storing payoffs?
	store_payoffs = false;
	load_payoffs = false;
	if (opts.count("store_payoffs")) {
		store_payoffs = true;
		payoffs_dir = opts["store_payoffs"].as<string>();
	} else if (opts.count("load_payoffs")) {
		load_payoffs = true;
		payoffs_dir = opts["load_payoffs"].as<string>();
	}
	evaluate = opts.count("evaluate") || !store_payoffs;

	// Initialize the payoff generator
	stereo_offsets = ParseMultiRange<int>(*gvStereoOffsets);

	// Load weights and penalties
	params.mono_weight = opts["mono_weight"].as<float>();
	params.agreement_weight = opts["3d_agreement_weight"].as<float>();
	params.occlusion_weight = opts["3d_occlusion_weight"].as<float>();
	params.stereo_weight = opts["stereo_weight"].as<float>();
	params.corner_penalty = opts["corner_penalty"].as<float>();
	params.occlusion_penalty = opts["occlusion_penalty"].as<float>();

	// Create payoff dir
	if (!payoffs_dir.empty()) {
		fs::create_directory(payoffs_dir);
		CHECK_PRED1(fs::exists, payoffs_dir) << "Failed to create directory";
		CHECK_PRED1(fs::is_directory, payoffs_dir) << "Exists but is not a directory";
	}

	// Create a thread pool and dispatcher
	int concurrency = opts["concurrency"].as<int>();
	if (concurrency == 0) {
		concurrency = boost::thread::hardware_concurrency();
	}
	thread_pool pool(concurrency);
	ThreadLocal<FrameProcessor> processor;

	// Process each sequence
	Map maps[sequences.size()];
	proto::TruthedMap gt_maps[sequences.size()];
	for (int i = 0; i < sequences.size(); i++) {
		maps[i].LoadWithGroundTruth(GetMapPath(sequences[i]), gt_maps[i]);
		for (int j = 0; j < maps[i].kfs.size(); j += frame_stride) {
			KeyFrame& frame = maps[i].kfs[j];
			if (concurrency == 1) {
				processor->Process(frame, sequences[i], gt_maps[i]);
			} else {
				pool.add(boost::bind(&FrameProcessor::Process,
														 boost::bind(&ThreadLocal<FrameProcessor>::Get, &processor),
														 ref(frame),
														 ref(sequences[i]),
														 ref(gt_maps[i])));
			}
		}
	}

	// Wait for jobs to complete
	pool.join();

	// Print results
	if (evaluate) {
		double mean_depth_error = perf.sum_depth_error / perf.num_frames;
		double mean_labelling_error = perf.sum_labelling_error / perf.num_frames;
		DLOG << fmt("AVERAGE DEPTH ERROR: %|40t|%.1f%%", mean_depth_error*100.);
		DLOG << fmt("AVERAGE LABELLING ERROR: %|40t|%.1f%%", mean_labelling_error*100.);

		if (use_depth_loss) {
			DLOG << mean_depth_error;
		} else {
			DLOG << mean_labelling_error;
		}
	}
	
	return 0;
}
