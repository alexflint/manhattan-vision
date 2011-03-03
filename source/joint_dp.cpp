#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "guided_line_detector.h"
#include "line_sweeper.h"
#include "point_cloud_payoffs.h"
#include "stereo_payoffs.h"
#include "monocular_payoffs.h"
#include "manhattan_dp.h"
#include "map.h"
#include "bld_helpers.h"
#include "line_sweep_features.h"
#include "canvas.h"
#include "manhattan_ground_truth.h"
#include "safe_stream.h"
#include "timer.h"

#include "counted_foreach.tpp"
#include "format_utils.tpp"
#include "io_utils.tpp"

lazyvar<double> gvMonoWeight("JointDP.Mono.Weight");
lazyvar<double> gvOcclusionWeight("JointDP.3D.OcclusionWeight");
lazyvar<double> gvAgreementWeight("JointDP.3D.AgreementWeight");
lazyvar<double> gvStereoWeight("JointDP.Stereo.Weight");
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

	if (argc < 3) {
		DLOG << "Usage: joint_dp SEQUENCE FRAMES [RESULTS_DIR]";
		DLOG << "           if specified, results will be appended to results/RESULTS_DIR/";
		exit(-1);
	}

	// Read parameters
	string sequence = argv[1];
	vector<int> frame_ids = ParseMultiRange<int>(argv[2]);

	fs::path results_dir;
	if (argc < 4) {
		results_dir = fs::initial_path();
	} else {
		results_dir = argv[3];
	}

	// Set up results dir
	CHECK_PRED1(fs::exists, results_dir);

	// Create or append to the CSV file
	fs::path stats_file = results_dir / fmt("performance_%s.csv", results_dir.filename());
	ofstream stats_out(stats_file.string().c_str(), ios::app);
	format stats_fmt("\"%s\",\"%s\",%d,%f,%f,%f\n");
	
	// Pattern for output filenames
	fs::path viz_dir = results_dir / "out";
	if (!fs::create_directory(viz_dir)) {
		DLOG << "Created output directory: " << viz_dir;
	}
	format filepat("%s/%s_frame%03d_%s.%s");
	filepat.bind_arg(1, viz_dir.string());
	filepat.bind_arg(2, sequence);

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

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();

	// Initialize payoffs generators
	LineSweepObjectiveGen mono_objective_gen;
	MonocularPayoffGen mono_payoff_gen;
	PointCloudPayoffs pt_payoff_gen;
	vector<int> stereo_offsets = ParseMultiRange<int>(*gvStereoOffsets);
	ptr_vector<StereoPayoffGen> stereo_payoff_gens;

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
		DPGeometry geom(frame.image.pc(), zfloor, zceil);

		// Compute monocular payoffs
		TIMED("Mono payoffs") {
			mono_objective_gen.Compute(frame.image);
			mono_payoff_gen.Compute(mono_objective_gen.objective, geom);
		}

		// Compute 3D payoffs
		TIMED("3D payoffs") {
			vector<Vec3> point_cloud;
			frame.GetMeasuredPoints(point_cloud);
			pt_payoff_gen.Compute(point_cloud, frame.image.pc(), geom, zfloor, zceil);
		}

		// Compute stereo payoffs
		VecI aux_active(stereo_offsets.size(), 0);
		stereo_payoff_gens.resize(stereo_offsets.size());
		TIMED("Stereo payoffs")
		COUNTED_FOREACH(int i, int offset, stereo_offsets) {
			KeyFrame* aux_frame = map.KeyFrameById(frame_id+stereo_offsets[i]);
			if (aux_frame != NULL) {
				aux_frame->LoadImage();
				stereo_payoff_gens[i].Compute(frame.image, aux_frame->image, geom, zfloor, zceil);
				aux_active[i] = 1;
			}
		}
		int num_aux = aux_active.Sum();

		// Combine payoffs
		DPPayoffs joint_payoffs(geom.grid_size);
		joint_payoffs.Add(mono_payoff_gen.payoffs, *gvMonoWeight);
		joint_payoffs.Add(pt_payoff_gen.agreement_payoffs, *gvAgreementWeight);
		joint_payoffs.Add(pt_payoff_gen.occlusion_payoffs, *gvOcclusionWeight);
		if (num_aux > 0) {
			// Near the beginning/end of sequences, num_aux might be less
			// than stereo_payoff_gens.size()
			double stereo_weight_per_aux = *gvStereoWeight / num_aux;
			for (int i = 0; i < stereo_payoff_gens.size(); i++) {
				// TODO: stereo on gradient image (See stereo_dp.cpp)
				if (aux_active[i]) {
					joint_payoffs.Add(stereo_payoff_gens[i].payoffs, stereo_weight_per_aux);
				}
			}
		}

		// Reconstruct
		ManhattanDPReconstructor recon;
		recon.Compute(frame.image, geom, joint_payoffs);
		const DPSolution& soln = recon.dp.solution;

		// Compute ground truth
		ManhattanGroundTruth gt(gt_map.floorplan(), frame.image.pc());

		// Compute payoff without penalties
		double gross_payoffs = soln.GetTotalPayoff(joint_payoffs, false);
		double mono_payoffs = soln.GetTotalPayoff(mono_payoff_gen.payoffs, false);
		double pt_agree_payoffs = soln.GetPathSum(pt_payoff_gen.agreement_payoffs);
		double pt_occl_payoffs = soln.GetPathSum(pt_payoff_gen.occlusion_payoffs);
		double stereo_payoffs = 0.0;
		double penalties = gross_payoffs - soln.score;
		for (int i = 0; i < stereo_payoff_gens.size(); i++) {
			if (aux_active[i]) {
				stereo_payoffs += soln.GetPathSum(stereo_payoff_gens[i].payoffs);
			}
		}
		mono_payoffs *= *gvMonoWeight;
		pt_agree_payoffs *= *gvAgreementWeight;
		pt_occl_payoffs *= *gvOcclusionWeight;
		stereo_payoffs *= (*gvStereoWeight / num_aux);  // yes this is correct

		// Compute performance
		double pixel_acc = recon.ReportAccuracy(gt) * 100;
		sum_acc += pixel_acc;
		double mean_err = recon.ReportDepthError(gt) * 100;
		sum_err += mean_err;

		// Visualize
		filepat.bind_arg(3, frame_id);
		filepat.bind_arg(5, "png");

		// Copy original
		string dest = str(filepat % "orig");
		if (!fs::exists(dest)) {
			fs::copy_file(frame.image_file, dest);//, fs::copy_option::overwrite_if_exists);
		}

		recon.OutputSolution(str(filepat % "dp"));
		//WriteMatrixImageRescaled(str(filepat % "solndepth"), soln_depth);
		//WriteMatrixImageRescaled(str(filepat % "gtdepth"), gt.depth_map());
		//WriteMatrixImageRescaled(str(filepat % "error"), depth_errors);

		if (*gvDrawPayoffs) {
			// Draw image in grid coords
			ImageRGB<byte> grid_image;
			geom.TransformToGrid(frame.image.rgb, grid_image);
			for (int i = 0; i < 2; i++) {
				OutputPayoffsViz(joint_payoffs.wall_scores[i],
												 geom, recon, grid_image,
												 str(filepat % fmt("payoffs%d", i)));
			}

			for (int i = 0; i < 2; i++) {
				OutputPayoffsViz(mono_payoff_gen.payoffs.wall_scores[i],
												 geom, recon, grid_image,
												 str(filepat % fmt("monopayoffs%d", i)));
			}			

			for (int i = 0; i < stereo_payoff_gens.size(); i++) {
				if (aux_active[i]) {
					OutputPayoffsViz(stereo_payoff_gens[i].payoffs,
													 geom, recon, grid_image,
													 str(filepat % fmt("stereopayoffs_aux%d", i)));
				}
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
			% mean_err
			% median_err;

		// Write results to individual file
		// this must come last because of call to bind_arg()
		filepat.bind_arg(5, "txt");
		sofstream info_out(str(filepat % "stats"));
		info_out << format("Labelling accuracy: %|40t|%f%%\n") % pixel_acc;
		info_out << format("Mean depth error: %|40t|%f%%\n") % mean_err;
		info_out << format("Median depth error: %|40t|%f%%\n") % median_err;
		info_out << format("Net score: %|40t|%f\n") % soln.score;
		info_out << format("  Penalties: %|40t|%f%%\n") % (100.0*penalties/gross_payoffs);
		info_out << format("  Gross payoffs: %|40t|%f\n") % gross_payoffs;
		info_out << format("    Mono payoffs: %|40t|%.1f%%\n") % (100*mono_payoffs/gross_payoffs);
		info_out << format("    Stereo payoffs: %|40t|%.1f%%\n") % (100*stereo_payoffs/gross_payoffs);
		info_out << format("    3D (agreement) payoffs: %|40t|%.1f%%\n") % (100.0*pt_agree_payoffs/gross_payoffs);
		info_out << format("    3D (occlusion) payoffs: %|40t|%.1f%%\n") % (100.0*pt_occl_payoffs/gross_payoffs);
	}

	// Note that if one or more frames weren't found then num_frames
	// will not equal frame_ids.size()
	double av_acc = sum_acc / num_frames;  // these are already multipled by 100
	double av_err = sum_err / num_frames;  // these are already multipled by 100
	DLOG << format("AVERAGE LABEL ACCURACY: %|40t|%.2f%%") % av_acc;
	DLOG << format("AVERAGE DEPTH ERROR: %|40t|%.2f%%") % av_err;
	
	return 0;
}
