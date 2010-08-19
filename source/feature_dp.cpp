#include "entrypoint_types.h"
#include "manhattan_dp.h"
#include "vars.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"
#include "timer.h"
#include "clipping.h"
#include "bld_helpers.h"
#include "safe_stream.h"

#include "io_utils.tpp"
#include "table.tpp"

using namespace indoor_context;
using namespace toon;

// TODO: rename!
class StructLearnDP {
public:
	static const int kFeatureLength = 6;
	typedef toon::Vector<kFeatureLength> Feature;

	const PosedImage* input;
	Mat3 floorToCeil;

	GuidedLineDetector line_detector;
	IsctGeomLabeller line_sweeper;

	Table<2,Feature> features;
	ManhattanDP::ScoreFunction scorefunc;
	ManhattanDP dp;

	// Do the once-per-image operations
	void Configure(const PosedImage& pim, const Mat3& floorToCeil);

	// Do the prediction given the specified weight vector
	void Predict(const Vector<>& w);
};

void StructLearnDP::Configure(const PosedImage& pim, const Mat3& hFloorToCeil) {
	input = &pim;
	floorToCeil = hFloorToCeil;

	line_detector.Compute(pim);
	line_sweeper.Compute(pim, line_detector.detections);

	features.Resize(input->nx(), input->ny());
	for (int y = 0; y < input->ny(); y++) {
		const PixelRGB<byte>* imrow = input->rgb[y];
		const int* sweeprow = line_sweeper.orient_map[y];
		for (int x = 0; x < input->nx(); x++) {
			Vector<kFeatureLength>& ftr = features(x,y);
			ftr[0] = imrow[x].r;
			ftr[1] = imrow[x].g;
			ftr[2] = imrow[x].b;
			ftr[3] = sweeprow[x] == 0 ? 1.0 : 0.0;
			ftr[4] = sweeprow[x] == 1 ? 1.0 : 0.0;
			ftr[5] = sweeprow[x] == 2 ? 1.0 : 0.0;
		}
	}
}

void StructLearnDP::Predict(const Vector<>& w) {
	// Compute the score function
	scorefunc.Resize(input->nx(), input->ny());
	for (int i = 0; i < 3; i++) {
		Vector<kFeatureLength> w_i = w.slice(i*kFeatureLength, kFeatureLength);
		for (int y = 0; y < input->ny(); y++) {
			float* scorerow = scorefunc.pixel_scores[i][y];
			for (int x = 0; x < input->nx(); x++) {
				scorerow[x] = features(x,y) * w_i;
			}
		}
	}

	// Run the DP
	dp.Compute(scorefunc, input->pc, floorToCeil);
}

// Pass the ground truth orientations as the initial orientations
// Use this for testing the algorithm
bool pass_gt_orients = false;

// To be moved elsewhere?
scoped_ptr<ManhattanML> recon;
double sum_accuracy;
int num_frames;

void ProcessFrame(Map& map, const proto::TruthedMap& gt_map, int index) {
	KeyFrame& kf = *map.KeyFrameByIdOrDie(index);
	kf.LoadImage();

	PosedImage pim(*kf.pc);
	ImageCopy(kf.image.rgb, pim.rgb);

	Mat3 floorToCeil = GetFloorCeilHomology(*kf.pc, gt_map.floorplan());

	ManhattanML recon;
	recon.Configure(pim, floorToCeil);

	Vector<> w(18);
	w = Zeros;
	w[3] = w[10] = w[17] = 1.0;  // temp hack
	DREPORT(w);
	recon.Predict(w);

	ImageRGB<byte> soln_canvas;
	ImageCopy(pim.rgb, soln_canvas);
	DrawOrientations(recon.dp.soln_orients, soln_canvas, 0.35);
	WriteImage("out/feature_soln.png", soln_canvas);

	kf.UnloadImage();
}

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc < 2 || argc > 4) {
		DLOG << "Usage: " << argv[0] << " truthed_map.pro [INDEX] [--from_gt]";
		return 0;
	}

	vector<int> indices;
	if (argc > 2) indices = ParseMultiRange<int>(argv[2]);

	if (argc >= 4 && string(argv[3]) == "--from_gt") {
		pass_gt_orients = true;
	} else if (argc >= 4) {
		DLOG << "Usage: " << argv[0] << " truthed_map.pro [INDEX] [--from_gt]";
		return 0;
	}

	// Load the map
	Map map;
	proto::TruthedMap tru_map;
	map.LoadWithGroundTruth(argv[1], tru_map);

	sum_accuracy = 0;
	num_frames = 0;
	if (indices.empty()) {
		iota_n(back_inserter(indices), 0, map.kfs.size());
	}
	BOOST_FOREACH(int index, indices) {
		DLOG << "Processing frame " << index;
		TIMED("Processing time") ProcessFrame(map, tru_map, index);
	}

	double average_acc = sum_accuracy / num_frames;
	DLOG << format("Overall Accuracy: %.2f%%") % (average_acc * 100);

	ofstream log_out("accuracies.txt", ios::out | ios::app);  // Output for appending
	log_out << "DP " << (average_acc * 100) << "% " << argv[1] << endl;

	return 0;
}
