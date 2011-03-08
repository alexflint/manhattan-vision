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
#include "image_utils.tpp"

lazyvar<string> gvStereoOffsets("JointDP.Stereo.AuxOffsets");
lazyvar<int> gvDrawPayoffs("JointDP.Output.DrawPayoffs");

void OutputPayoffsViz(const MatF& payoffs,
											const DPGeometry& geometry,
											const ManhattanDPReconstructor& recon,
											const ImageRGB<byte>& grid_image,
											const string& file) {
	// Draw payoffs
	ImageRGB<byte> payoff_image(geometry.grid_size[0], geometry.grid_size[1]);
	DrawMatrixRescaled(payoffs, payoff_image);
	recon.dp.DrawWireframeGridSolution(payoff_image);

	// Blend together
	FileCanvas canvas(file, grid_image);
	canvas.DrawImage(payoff_image, 0.6);
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	AssertionManager::SetExceptionMode();

	if (argc < 3) {
		DLOG << "Usage: joint_dp SEQUENCE FRAMES [-q|RESULTS_DIR]";
		DLOG << "           if specified, results will be appended to results/RESULTS_DIR/";
		exit(-1);
	}

	// Read parameters
	string sequence = argv[1];
	vector<int> frame_ids = ParseMultiRange<int>(argv[2]);
	bool quiet = argc > 3 && string(argv[3]) == "-q";

		// Set up results dir
	fs::path results_dir;
	if (!quiet) {
		if (argc < 4) {
			results_dir = fs::initial_path();
		} else {
			results_dir = argv[3];
		}
		CHECK_PRED1(fs::exists, results_dir);
	}

	ofstream stats_out;
	format stats_fmt("\"%s\",\"%s\",%d,%f,%f\n");
	format filepat;

	if (!quiet) {
		// Open the CSV file
		fs::path stats_file = results_dir / fmt("performance_%s.csv", results_dir.filename());
		stats_out.open(stats_file.string().c_str(), ios::app);

		// Write the parameters
		fs::path params_file = results_dir / fmt("parameters_%s.csv", results_dir.filename());
		ofstream params_out(params_file.string().c_str());
		params_out << "\"3D.OcclusionWeight\"," << *gvOcclusionWeight << endl;
		params_out << "\"3D.AgreementWeight\"," << *gvAgreementWeight << endl;
		params_out << "\"Mono.Weight\"," << *gvMonoWeight << endl;
		params_out << "\"Stereo.Weight\"," << *gvStereoWeight << endl;
		params_out << "\"Stereo.AuxOffsets\",\"" << *gvStereoOffsets << "\"" << endl;
		params_out << "\"ManhattanDP.DefaultWallPenalty\"," << *gvDefaultWallPenalty << endl;
		params_out << "\"ManhattanDP.DefaultOcclusionPenalty\"," << *gvDefaultOcclusionPenalty << endl;
		params_out << "\"ManhattanDP.GridSize\",\"" << *gvGridSize << "\"" << endl;
		params_out.close();

		// Create vizualization dir
		fs::path viz_dir = results_dir / "out";
		if (!fs::create_directory(viz_dir)) {
			DLOG << "Created output directory: " << viz_dir;
		}

		// Pattern for output filenames
		filepat = format("%s/%s_frame%03d_%s.%s");
		filepat.bind_arg(1, viz_dir.string());
		filepat.bind_arg(2, sequence);
	}

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();

	// Initialize the payoff generator
	JointPayoffGen joint;
	vector<int> stereo_offsets = ParseMultiRange<int>(*gvStereoOffsets);

	double sum_acc = 0;
	double sum_err = 0;
	int num_frames = 0;

	// Process each frame
	BOOST_FOREACH(int frame_id, frame_ids) {
		TITLE("Frame "<<frame_id);
		scoped_timer t("Process frame");

		// Get the frame
		KeyFrame& frame = *map.KeyFrameById(frame_id);
		if (&frame == NULL) continue;
		frame.LoadImage();
		num_frames++;  // for computing average performance, in case one or more of the frames weren't found

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

		// Compute joint payoffs
		joint.Compute(frame.image, geom, point_cloud, aux_images);

		// Reconstruct
		ManhattanDPReconstructor recon;
		recon.Compute(frame.image, geom, joint.payoffs);
		const DPSolution& soln = recon.dp.solution;

		// Compute ground truth
		ManhattanGroundTruth gt(gt_map.floorplan(), frame.image.pc());

		// Compute payoff without penalties
		double gross_payoffs = soln.GetTotalPayoff(joint.payoffs, false);
		double penalties = gross_payoffs - soln.score;
		double mono_payoffs = soln.GetTotalPayoff(joint.mono_gen.payoffs, false);
		double pt_agree_payoffs = soln.GetPathSum(joint.point_cloud_gen.agreement_payoffs);
		double pt_occl_payoffs = soln.GetPathSum(joint.point_cloud_gen.occlusion_payoffs);
		double stereo_payoffs = 0.0;
		for (int i = 0; i < joint.stereo_gens.size(); i++) {
			stereo_payoffs += soln.GetPathSum(joint.stereo_gens[i].payoffs);
		}
		mono_payoffs *= joint.mono_weight;
		pt_agree_payoffs *= joint.agreement_weight;
		pt_occl_payoffs *= joint.occlusion_weight;
		stereo_payoffs *= (joint.stereo_weight / aux_images.size());  // yes this is correct

		// Compute performance
		double pixel_acc = recon.ReportAccuracy(gt) * 100;
		sum_acc += pixel_acc;
		double mean_err = recon.ReportDepthError(gt) * 100;
		sum_err += mean_err;

		if (!quiet) {
			ImageRGB<byte> canvas2(geom.grid_size[0], geom.grid_size[1]);
			recon.dp.DrawWireframeGridSolution(canvas2);
			WriteImage("out/grid_wires.png", canvas2);

			ImageRGB<byte> canvas(frame.image.sz());
			recon.dp.DrawWireframeSolution(canvas);
			WriteImage("out/image_wires.png", canvas);

			recon.OutputSolution("out/orients.png");

			// Visualize
			filepat.bind_arg(3, frame_id);
			filepat.bind_arg(5, "png");

			// Copy original
			string dest = str(filepat % "orig");
			if (!fs::exists(dest)) {
				fs::copy_file(frame.image_file, dest);
			}

			recon.OutputSolution(str(filepat % "dp"));

			// Draw payoffs
			if (*gvDrawPayoffs) {
				ImageRGB<byte> grid_image;
				geom.TransformToGrid(frame.image.rgb, grid_image);
				for (int i = 0; i < 2; i++) {
					OutputPayoffsViz(joint.payoffs.wall_scores[i],
													 geom, recon, grid_image,
													 str(filepat % fmt("payoffs%d", i)));
				}

				for (int i = 0; i < 2; i++) {
					OutputPayoffsViz(joint.mono_gen.payoffs.wall_scores[i],
													 geom, recon, grid_image,
													 str(filepat % fmt("monopayoffs%d", i)));
				}			

				for (int i = 0; i < joint.stereo_gens.size(); i++) {
					OutputPayoffsViz(joint.stereo_gens[i].payoffs,
													 geom, recon, grid_image,
													 str(filepat % fmt("stereopayoffs_aux%d", i)));
				}
			}

			// Write results to CSV file
			time_t now = time(NULL);
			string timestamp = asctime(localtime(&now));
			stats_out << stats_fmt
				% timestamp.substr(0,timestamp.length()-1)
				% sequence
				% frame_id
				% pixel_acc
				% mean_err;

			// Write results to individual file
			// this must come last because of call to bind_arg()
			filepat.bind_arg(5, "txt");
			sofstream info_out(str(filepat % "stats"));
			info_out << format("Labelling accuracy: %|40t|%f%%\n") % pixel_acc;
			info_out << format("Mean depth error: %|40t|%f%%\n") % mean_err;
			info_out << format("Net score: %|40t|%f\n") % soln.score;
			info_out << format("  Penalties: %|40t|%f%%\n") % (100.0*penalties/gross_payoffs);
			info_out << format("  Gross payoffs: %|40t|%f\n") % gross_payoffs;
			info_out << format("    Mono payoffs: %|40t|%.1f%%\n") % (100*mono_payoffs/gross_payoffs);
			info_out << format("    Stereo payoffs: %|40t|%.1f%%\n") % (100*stereo_payoffs/gross_payoffs);
			info_out << format("    3D (agreement) payoffs: %|40t|%.1f%%\n") % (100.0*pt_agree_payoffs/gross_payoffs);
			info_out << format("    3D (occlusion) payoffs: %|40t|%.1f%%\n") % (100.0*pt_occl_payoffs/gross_payoffs);
		}
	}

	// Note that if one or more frames weren't found then num_frames
	// will not equal frame_ids.size()
	double av_acc = sum_acc / num_frames;  // these are already multipled by 100
	double av_err = sum_err / num_frames;  // these are already multipled by 100
	if (quiet) {
		DLOG << av_acc;
	} else {
		DLOG << format("AVERAGE LABEL ACCURACY: %|40t|%.1f%%") % av_acc;
		DLOG << format("AVERAGE DEPTH ERROR: %|40t|%.1f%%") % av_err;
	}
	
	return 0;
}
