#include <iomanip>
#include <fstream>
#include <iterator>
#include <set>

#include <boost/filesystem.hpp>

#include <TooN/SVD.h>
#include <VW/Utils/timer.h>

#include "entrypoint_types.h"
#include "map.h"
#include "line_sweeper.h"
#include "timer.h"
#include "geom_utils.h"
#include "guided_line_detector.h"
#include "safe_stream.h"
#include "canvas.h"
#include "line_detector.h"
#include "vanishing_points.h"
#include "manhattan_bnb.h"
#include "bld_helpers.h"
#include "manhattan_ground_truth.h"

#include "counted_foreach.tpp"
#include "vector_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"
#include "format_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc < 3 || argc > 4) {
		DLOG	<< "Usage: " << argv[0] << " SEQUENCE INDICES [RESULTDIR]";
		return 0;
	}

	string sequence = argv[1];
	vector<int> frame_ids = ParseMultiRange<int>(argv[2]);

	// Paths
	fs::path results_dir = fs::initial_path();
	if (argc > 3) {
		results_dir = argv[3];
	}

	// Initialize performance spreadhseet
	fs::path stats_file = results_dir / fmt("performance_%s.csv", results_dir.filename());
	ofstream stats_out(stats_file.string().c_str(), ios::app);
	format stats_fmt("\"%s\",\"%s\",%d,%f,%f,%d\n");

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	// Initialize the reconstructor
	ManhattanBnbReconstructor recon;
	
	// Pattern for output files
	fs::path viz_dir = results_dir / "out";
	if (!fs::create_directory(viz_dir)) {
		DLOG << "Created output directory: " << viz_dir;
	}
	format filepat("%s/%s_frame%03d_%s.%s");
	filepat.bind_arg(1, viz_dir.string());
	filepat.bind_arg(2, sequence);

	// Reconstruct each frame
	double sum_acc = 0;
	double sum_err = 0;
	int num_frames = 0, num_initialized_frames = 0;
	BOOST_FOREACH(int frame_id, frame_ids) {
		TITLE("Processing frame "<<frame_id);
		scoped_timer t(fmt("Frame %d", frame_id));

		filepat.bind_arg(3, frame_id);
		filepat.bind_arg(5, "png");
		
		// Run the reconstruction
		KeyFrame& frame = *map.KeyFrameById(frame_id);
		if (&frame == NULL) continue;
		frame.LoadImage();
		recon.Compute(frame);

		double label_accuracy = 0, depth_error = 0;
		num_frames++;
		if (recon.success) {
			num_initialized_frames++;
			// Compute ground truth
			ManhattanGroundTruth gt(gt_map.floorplan(), frame.image.pc());

			// Compute accuracy
			label_accuracy = recon.GetAccuracy(gt);
			sum_acc += label_accuracy;
			DLOG << format("Labelling accuracy: %|40t|%.1f%%") % (label_accuracy*100);

			// Compute mean depth error
			depth_error = recon.ReportDepthError(gt);
			sum_err += depth_error;

			// Output
			recon.OutputSolutionOrients(str(filepat % "bnb_soln"));

			// Write accuracy
			filepat.bind_arg(5, "txt");
			sofstream acc_out(str(filepat % "bnb_accuracy"));
			acc_out << "Labelling accuracy: " << (label_accuracy*100) << endl;
			acc_out << "Mean depth error: " << (depth_error*100) << endl;
		}

		// Write stats
		time_t now = time(NULL);
		string timestamp = asctime(localtime(&now));
		stats_out << stats_fmt
			% timestamp.substr(0,timestamp.length()-1)
			% sequence
			% frame_id
			% label_accuracy
			% depth_error
			% recon.success ? 1 : 0;

		frame.UnloadImage();
	}

	double av_acc = sum_acc / num_frames;
	DLOG << format("AVERAGE LABEL ACCURACY: %|40t|%.1f%%") % (av_acc * 100);
	double av_acc2 = sum_acc / num_initialized_frames;
	DLOG << format("  excluding initialization failures: %|40t|%.1f%%") % (av_acc2 * 100);

	// Add 100% error for frames that failed to initialize
	double av_err = (sum_err+num_initialized_frames) / num_frames;
	DLOG << format("AVERAGE DEPTH ERROR: %|40t|%.1f%%") % (av_err * 100);
	double av_err2 = sum_err / num_initialized_frames;
	DLOG << format("  excluding initialization failures: %|40t|%.1f%%") % (av_err2 * 100);

	return 0;
}
