#include "multiview_reconstructor.h"

#include <boost/array.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "camera.h"
#include "colors.h"
#include "bld_helpers.h"
#include "manhattan_dp.h"
#include "canvas.h"
#include "geom_utils.h"

#include "integral_col_image.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	void MultiViewPayoffs::Configure(const DPGeometry& geom,
																	 const DPObjective& obj,
																	 double zf,
																	 double zc) {
		base_geom = geom;
		zfloor = zf;
		zceil = zc;
		base_payoff_gen.Configure(obj, base_geom);
		base_payoff_gen.GetPayoffs(base_payoffs); // for visualization only
		aux_views.clear();
	}

	void MultiViewPayoffs::AddView(const DPGeometry& aux_geom,
																 const DPObjective& obj) {
		CHECK(!base_payoff_gen.empty) << "Configure() must be called before AddView()";

		AuxiliaryView* view = new AuxiliaryView;
		view->geom = aux_geom;
		view->payoff_gen.Configure(obj, aux_geom);
		view->payoff_gen.GetPayoffs(view->payoffs); // for visualization only
	
		view->image_hfloor = GetHomographyVia(*base_geom.camera,
																					*aux_geom.camera,
																					makeVector(0, 0, -1, zfloor));

		view->image_hceil = GetHomographyVia(*base_geom.camera,
																				 *aux_geom.camera,
																				 makeVector(0, 0, -1, zceil));

		// The following homographies transfer *from* the base image *to*
		// the aux image
		view->grid_hfloor = aux_geom.imageToGrid * view->image_hfloor * base_geom.gridToImage;
		view->grid_hceil = aux_geom.imageToGrid * view->image_hceil * base_geom.gridToImage;

		aux_views.push_back(view);
	}

	void MultiViewPayoffs::Compute() {
		CHECK(!base_payoff_gen.empty) << "Configure() must be called before Compute()";

		// Configure the payoff matrices
		payoffs.Resize(base_geom.grid_size, 0);
		BOOST_FOREACH(AuxiliaryView& aux, aux_views) {
			aux.contrib_payoffs.Resize(base_geom.grid_size, -1);  // for visualization only
		}

		for (int orient = 0; orient < 2; orient++) {
			for (int y = 0; y < base_geom.grid_size[1]; y++) {
				for (int x = 0; x < base_geom.grid_size[0]; x++) {
					// initialize to payoffs in base view
					Vec2I p = makeVector(x,y);
					double sum_payoffs = base_payoff_gen.GetWallScore(p, orient);
					double sum_weights = 1.0;  // the base view gets weighted by 1.0

					BOOST_FOREACH(AuxiliaryView& aux, aux_views) {
						// Construct the weighted sum over the base view and all auxiliary views
						const Mat3& transfer = y<base_geom.horizon_row ? aux.grid_hceil : aux.grid_hfloor;
						Vec2I p_aux = RoundVector(project(transfer * unproject(p)));

						// No need to check the y-coordinate (in fact we _must_ not)
						// as this will be dealt with by GetPayoff()
						if (p_aux[0] >= 0 && p_aux[0] < aux.geom.grid_size[0]) {
							// For now, the aux view contribute half weight w.r.t to base view
							double weight = 0.5;
							double payoff = aux.payoff_gen.GetWallScore(p_aux, orient);
							sum_payoffs += weight * payoff;
							sum_weights += weight;

							aux.contrib_payoffs.wall_scores[orient][y][x] = payoff;  // for visualization only
						}
					}

					// Record the final payoffs
					payoffs.wall_scores[orient][y][x] = sum_payoffs / sum_weights;
				}
			}
		}
	}

	void MultiViewPayoffs::OutputCorrespondences(const string& filename,
																							 int index,
																							 const ImageRGB<byte>& base_im,
																							 const ImageRGB<byte>& aux_im) {
		Vec2I t = makeVector(base_im.GetWidth(), 0);
		Vec2I size = asToon(aux_im.GetSize()) + t;
		FileCanvas canvas(filename,	size);

		canvas.DrawImage(aux_im, 1.0, t);
		canvas.DrawImage(base_im, 1.0, Zeros);

		BrightColors bc;
		const AuxiliaryView& aux = aux_views[index];
		for (int y = 0; y < base_geom.grid_size[1]; y += 15) {
			if (abs(y-base_geom.horizon_row) < 25) continue; // skip the section near the horion
			bool on_ceil = (y < base_geom.horizon_row);
			for (int x = 0; x < base_geom.grid_size[0]; x += 15) {
				Vec2I p = makeVector(x, y);
				const Mat3& transfer = on_ceil ? aux.grid_hceil : aux.grid_hfloor;
				Vec2I p_aux = RoundVector(project(transfer * unproject(p)));
				if (p_aux[0] >= 0 && p_aux[0] < aux.geom.grid_size[0] &&
						p_aux[1] >= 0 && p_aux[1] < aux.geom.grid_size[1]) {
					Vec2I q = project(base_geom.GridToImage(p));
					Vec2I q_aux = project(aux.geom.GridToImage(p_aux));

					PixelRGB<byte> color = bc.Next();
					canvas.DrawDot(q, 4.0, Colors::white());
					canvas.DrawDot(q, 3.0, color);
					canvas.DrawDot(t+q_aux, 4.0, Colors::white());
					canvas.DrawDot(t+q_aux, 3.0, color);
					canvas.StrokeLine(q, t+q_aux, color);
				}
			}
		}
	}










	MultiViewReconstructor::MultiViewReconstructor()
		: base_frame(NULL),
			base_objective(NULL) {
	}

	void MultiViewReconstructor::Configure(const PosedImage& frame,
																				 const DPObjective& obj,
																				 double zf,
																				 double zc) {
		base_frame = &frame;
		base_objective = &obj;
		zfloor = zf;
		zceil = zc;

		Mat3 fToC = GetManhattanHomology(frame.pc(), zfloor, zceil);
		DPGeometry base_geom(frame.pc(), fToC);
		joint_payoff_gen.Configure(base_geom, obj, zfloor, zceil);
		aux_frames.clear();
		aux_objectives.clear();
		aux_gen.clear();
	}

	void MultiViewReconstructor::Configure(const PosedImage& frame,
																				 double zf,
																				 double zc) {
		base_gen.Compute(frame);
		Configure(frame, base_gen.objective, zf, zc);
	}

	void MultiViewReconstructor::AddFrame(const PosedImage& frame,
																				const DPObjective& obj) {
		CHECK(base_frame) << "AddFrame() was called before Configure()";
		aux_frames.push_back(&frame);
		aux_objectives.push_back(&obj);
		Mat3 fToC = GetManhattanHomology(frame.pc(), zfloor, zceil);
		DPGeometry geom(frame.pc(), fToC);
		joint_payoff_gen.AddView(geom, obj);
	}

	void MultiViewReconstructor::AddFrame(const PosedImage& frame) {
		CHECK(base_frame) << "AddFrame() was called before Configure()";
		aux_gen.push_back(new LineSweepDPScore(frame));
		AddFrame(frame, aux_gen.back().objective);
	}

	void MultiViewReconstructor::Reconstruct() {
		CHECK(base_frame) << "Reconstruct() was called before Configure()";

		DLOG << "Reconstructing with " << aux_frames.size() << " aux views";

		joint_payoff_gen.Compute();
		recon.Compute(*base_frame,
									joint_payoff_gen.base_geom,
									joint_payoff_gen.payoffs);
	}






	void MultiViewReconstructor::OutputBasePayoffs(const string& filename) {
		OutputPayoffsViz(filename,
										 base_frame->rgb,
										 joint_payoff_gen.base_payoffs,
										 joint_payoff_gen.base_geom);
	}

	void MultiViewReconstructor::OutputAuxPayoffs(const string& basename) {
		if (!aux_objectives.empty()) {
			CHECK_EQ(aux_objectives.size(), aux_frames.size())
				<< "aux_objectives should either be empty or the same size as aux_frames";
		}

		for(int i = 0; i < aux_frames.size(); i++) {
			DLOG << "Visualizing auxiliary frame " << i;
			OutputPayoffsViz(str(format("%saux%02d_payoffs.png") % basename % i),
											 aux_frames[i]->rgb,
											 joint_payoff_gen.aux_views[i].payoffs,
											 joint_payoff_gen.aux_views[i].geom);
			OutputPayoffsViz(str(format("%saux%02d_contrib.png") % basename % i),
											 base_frame->rgb,
											 joint_payoff_gen.aux_views[i].contrib_payoffs,
											 joint_payoff_gen.aux_views[i].geom);
			if (!aux_objectives.empty()) {
				aux_gen[i].line_sweeper.OutputOrientViz
					(str(format("%saux%02d_sweeps.png") % basename % i));
			}
		}
	}

	void MultiViewReconstructor::OutputJointPayoffs(const string& filename) {
		OutputPayoffsViz(filename,
										 base_frame->rgb,
										 joint_payoff_gen.payoffs,
										 joint_payoff_gen.base_geom);
	}
}  // namespace indoor_context
