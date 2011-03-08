#include <boost/filesystem.hpp>

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

#include "counted_foreach.tpp"
#include "format_utils.tpp"
#include "io_utils.tpp"

#include "model.pb.h"

lazyvar<string> gvStereoOffsets("JointDP.Stereo.AuxOffsets");
lazyvar<int> gvDrawPayoffs("JointDP.Output.DrawPayoffs");

void GetModel(const DPSolution& solution,
							const DPGeometryWithScale& geometry,
							proto::Model& model) {
	model.set_zfloor(geometry.zfloor);
	model.set_zceil(geometry.zceil);
	BOOST_FOREACH(const LineSeg& seg, solution.wall_segments) {
		proto::Polyline* polyline = model.add_segments();
		Vec2 u = geometry.BackProject(seg.start).slice<0,2>();
		Vec2 v = geometry.BackProject(seg.end).slice<0,2>();
		*polyline->add_vertices() = asProto(u);
		*polyline->add_vertices() = asProto(v);
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);

	if (argc < 3 || argc > 4) {
		DLOG << "Usage: estimate_floorplan SEQUENCE FRAMES [OUT_DIR]";
		exit(-1);
	}

	// Read parameters
	string sequence = argv[1];
	vector<int> frame_ids = ParseMultiRange<int>(argv[2]);

	// Setup output dir
	fs::path model_dir;
	if (argc < 4) {
		model_dir = fs::initial_path() / "models";
	} else {
		model_dir = argv[3];
	}

	if (!fs::exists(model_dir)) {
		fs::create_directory(model_dir);
	}
	CHECK_PRED1(fs::is_directory, model_dir);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();

	// Initialize payoff generators
	vector<int> stereo_offsets = ParseMultiRange<int>(*gvStereoOffsets);
	JointPayoffGen payoff_gen;
	ManhattanDPReconstructor recon;

	format filepat("out/frame%03d_%s.png");

	// Process each frame
	BOOST_FOREACH(int frame_id, frame_ids) {
		TITLE("Frame "<<frame_id);
		scoped_timer t("Process frame");

		// Get the frame
		KeyFrame& frame = *map.KeyFrameById(frame_id);
		if (&frame == NULL) continue;
		frame.LoadImage();

		// Compute geometry
		DPGeometryWithScale geom(frame.image.pc(), zfloor, zceil);

		// Get point cloud
		vector<Vec3> point_cloud;
		frame.GetMeasuredPoints(point_cloud);

		// Get auxiliary frames
		vector<const PosedImage*> aux_images;
		COUNTED_FOREACH(int i, int offset, stereo_offsets) {
			KeyFrame* aux_frame = map.KeyFrameById(frame_id+offset);
			if (aux_frame != NULL) {
				aux_frame->LoadImage();
				aux_images.push_back(&aux_frame->image);
			}
		}

		// Reconstruct
		payoff_gen.Compute(frame.image, geom, point_cloud, aux_images);
		recon.Compute(frame.image, geom, payoff_gen.payoffs);
		const DPSolution& soln = recon.dp.solution;

		// Compute ground truth
		ManhattanGroundTruth gt(gt_map.floorplan(), frame.image.pc());
		recon.ReportAccuracy(gt);
		recon.ReportDepthError(gt);

		proto::Model model;
		GetModel(soln, geom, model);
		fs::path out_file = model_dir/fmt("frame%03d_model.pro", frame_id);
		ofstream out(out_file.string().c_str());
		model.SerializeToOstream(&out);

		FloorPlanRenderer re;
		re.Configure(frame.image.pc());
		re.Render(model);
		DREPORT(model.zfloor());
		DREPORT(model.zceil());
		DREPORT(gt_map.floorplan().zfloor(), gt_map.floorplan().zceil());
		WriteOrientationImage("out/orients.png", re.orientations());
		
		// Visualize
		filepat.bind_arg(1, frame_id);
		recon.OutputSolution(str(filepat % "dp"));
	}
	
	return 0;
}
