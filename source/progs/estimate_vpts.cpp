#include <stdlib.h>

#include <fstream>

#include <boost/format.hpp>

#include <TooN/so3.h>

#include "entrypoint_types.h"
#include "canvas.h"
#include "colors.h"
#include "line_segment.h"
#include "io_utils.tpp"
#include "numeric_utils.h"
#include "geom_utils.h"
#include "vpt_utils.h"
#include "bld_helpers.h"
#include "camera.h"
#include "vanishing_point_model.h"
#include "vanishing_points.h"
#include "line_detector.h"
#include "line_detector_bank.h"
#include "map.h"
#include "map_io.h"
#include "map.pb.h"

#include "drawing.h"

#include "counted_foreach.tpp"
#include "vector_utils.tpp"
#include "format_utils.tpp"
#include "numerical_jacobian.tpp"

using namespace indoor_context;
using namespace toon;


int main(int argc, char **argv) {
	InitVars();

	if (argc != 3) {
		DLOG << "Usage: estimate_vpts SEQUENCE FRAMES";
		exit(-1);
	}

	// Read parameters
	string sequence = argv[1];
	vector<int> frame_ids = ParseMultiRange<int>(argv[2]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	LoadXmlMapWithGroundTruth(GetMapPath(sequence), map, gt_map, false, false);
	SO3<> Rw_true = SO3<>::exp(asToon(gt_map.ln_scene_from_slam()));
	map.LoadAllImages();

	LineDetectorBank line_bank(map);

	// Detect lines
	vector<LineObservation> observations;
	COUNTED_FOREACH(int i, int frame_id, frame_ids) {
		line_bank.GetObservationsFor(frame_id, observations);
	}

	// Pick a start point
	double perturbation = -.1;
	SO3<> Rw_init = Rw_true * RandomRotation(perturbation);

	// Create the model
	VanishingPointModel model(6.);

	// Estimate vanishing points
	VanishingPointEstimator estimator(observations, model);
	SO3<> Rw_basin = estimator.Compute(Rw_init);

	Vec3 true_G_basin = model.ComputeExpectedLogLik_NumericGradient(observations,
																																	estimator.responsibilities,
																																	Rw_basin,
																																	1e-8);
	Vec3 ana_G_basin = model.ComputeExpectedLogLikGradient(observations,
																												 estimator.responsibilities,
																												 Rw_basin);

	DREPORT(true_G_basin);
	DREPORT(ana_G_basin);
	DREPORT(norm(ana_G_basin - true_G_basin));

	DREPORT(RotationDistance(Rw_init, Rw_true));
	DREPORT(RotationDistance(Rw_basin, Rw_true));

	// Vizualize
	vector<LineSeg> segments0;
	line_bank.GetDetectionsFor(frame_ids[0], segments0);
	const PosedImage& image = map.GetFrameById(frame_ids[0])->image;

	OutputVptViz(image, segments0, Rw_true, "out/Rw_true.pdf");
	OutputVptViz(image, segments0, Rw_init, "out/Rw_init.pdf");
	OutputVptViz(image, segments0, Rw_basin, "out/Rw_basin.pdf");
	/*for (int i = 0; i <= 10; i++) {
		OutputVptViz(image, segments, axes, estimator.optimizer.Rs[i],
								 fmt("out/Rw_iter%03d.pdf", i));
								 }*/

	return 0;
}
