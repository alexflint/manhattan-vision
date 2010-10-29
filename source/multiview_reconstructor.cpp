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

	void DrawPayoffs(Canvas& canvas,
									 const boost::array<MatF,2>& payoffs,
									 const DPGeometry& geom) {
		double max_payoff = 0;
		for (int i = 0; i < 2; i++) {
			for (int y = 0; y < payoffs[i].Rows(); y++) {
				const float* row = payoffs[i][y];
				for (int x = 0; x < payoffs[i].Cols(); x++) {
					if (row[x] > max_payoff) max_payoff = row[x];
				}
			}
		}

		static const double kDotSize = 1.0;
		for (int i = 0; i < 2; i++) {
			Vec2 tdot = makeVector(kDotSize*i*1.5, 0);
			for (int y = 0; y < payoffs[i].Rows(); y++) {
				const float* row = payoffs[i][y];
				for (int x = 0; x < payoffs[i].Cols(); x++) {
					if (row[x] >= 0) {
						double v = row[x] / max_payoff;
						PixelRGB<byte> color( (i == 1 ? v*255 : 0),
																	(i == 0 ? v*255 : 0),
																	0);
						Vec2 p = project(geom.GridToImage(makeVector(x,y)));
						canvas.DrawDot(p+tdot, kDotSize, color);
					}
				}
			}
		}
	}

	void OutputPayoffsViz(const string& filename,
												const ImageRGB<byte>& orig,
												const boost::array<MatF,2>& payoffs,
												const DPGeometry& geom) {
		FileCanvas canvas(filename, orig);
		DrawPayoffs(canvas, payoffs, geom);
	}






	void MonocularPayoffs::Configure(const DPObjective& obj,
																	 const DPGeometry& geometry) {
		empty = false;
		geom = geometry;

		// Compute per orientation, per pixel affinities
		MatF grid_buffer;
		for (int i = 0; i < 3; i++) {
			// Allocate memory
			grid_buffer.Resize(geom.grid_size[1], geom.grid_size[0], 0);
			CHECK_EQ(obj.pixel_scores[i].Rows(), geom.camera->image_size().y);
			CHECK_EQ(obj.pixel_scores[i].Cols(), geom.camera->image_size().x);

			// Transform the scores according to ImageToGrid()
			for (int y = 0; y < geom.camera->image_size().y; y++) {
				const float* inrow = obj.pixel_scores[i][y];
				for (int x = 0; x < geom.camera->image_size().x; x++) {
					Vec2I grid_pt = RoundVector(geom.ImageToGrid(makeVector(x, y, 1.0)));
					if (grid_pt[0] >= 0 && grid_pt[0] < geom.grid_size[0] &&
							grid_pt[1] >= 0 && grid_pt[1] < geom.grid_size[1]) {
						grid_buffer[ grid_pt[1] ][ grid_pt[0] ] += inrow[x];
					}
				}
			}

			// Compute the integral-col image
			integ_scores[i].Compute(grid_buffer);
		}
	}

	void MonocularPayoffs::GetAllPayoffs(boost::array<MatF,2>& payoffs) const {
		for (int i = 0; i < 2; i++) {
			payoffs[i].Resize(geom.grid_size[1], geom.grid_size[0]);
			for (int y = 0; y < geom.grid_size[1]; y++) {
				float* outrow = payoffs[i][y];
				for (int x = 0; x < geom.grid_size[0]; x++) {
					outrow[x] = GetPayoff(makeVector(x,y), i);
				}
			}
		}
	}

	double MonocularPayoffs::GetPayoff(const Vec2& grid_pt, int axis) const {
		CHECK_GT(integ_scores[0].m_int.Rows(), 0) << "Configure() must be called before GetPayoff()";
		CHECK_INTERVAL(grid_pt[0], 0, geom.grid_size[0]-1);

		// Compute the row of the opposite face (floor <-> ceiling)
		// TODO: go back to using opp_rows here
		Vec2 opp_pt = geom.Transfer(grid_pt);
		int opp_y = Clamp<int>(opp_pt[1], 0, geom.grid_size[1]-1);

		// Rounding and clamping must come after Transfer()
		int x = roundi(grid_pt[0]);
		int y = Clamp<int>(roundi(grid_pt[1]), 0, geom.grid_size[1]-1);

		// Compute the score for this segment
		int wall_orient = 1-axis;  // orients refer to normal direction rather than vpt index
		int y0 = min(y, opp_y);
		int y1 = max(y, opp_y);
		return integ_scores[kVerticalAxis].Sum(x, 0, y0-1)
			+ integ_scores[wall_orient].Sum(x, y0, y1-1)
			+ integ_scores[kVerticalAxis].Sum(x, y1, geom.grid_size[1]-1);
	}











	void MultiViewPayoffs::Configure(const DPGeometry& geom,
																	 const DPObjective& obj,
																	 double zf,
																	 double zc) {
		base_geom = geom;
		zfloor = zf;
		zceil = zc;
		base_payoff_gen.Configure(obj, base_geom);
		base_payoff_gen.GetAllPayoffs(base_payoffs);	// for visualization only
		aux_views.clear();
	}

	void MultiViewPayoffs::AddView(const DPGeometry& aux_geom,
																 const DPObjective& obj) {
		CHECK(!base_payoff_gen.empty) << "Configure() must be called before AddView()";

		AuxiliaryView* view = new AuxiliaryView;
		view->geom = aux_geom;
		view->payoff_gen.Configure(obj, aux_geom);
		view->payoff_gen.GetAllPayoffs(view->payoffs); 	// for visualization only
	
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

		int nx = base_geom.grid_size[0];
		int ny = base_geom.grid_size[1];
		for (int orient = 0; orient < 2; orient++) {
			payoffs[orient].Resize(ny, nx, 0);
			BOOST_FOREACH(AuxiliaryView& aux, aux_views) {
				// we only keep contrib_payoffs around for visualization
				aux.contrib_payoffs[orient].Resize(ny, nx, -1);
			}
			for (int y = 0; y < ny; y++) {
				bool on_ceil = (y < base_geom.horizon_row);
				for (int x = 0; x < nx; x++) {
					// initialize to payoffs in base view
					Vec2I p = makeVector(x,y);
					double sum_payoffs = base_payoff_gen.GetPayoff(p, orient);
					double norm = 1.0;  // the base view gets weighted by 1.0

					//double sum_payoffs = base_payoff_gen[orient][y][x];  // initialize to payoffs in base view

					BOOST_FOREACH(AuxiliaryView& aux, aux_views) {
						// Construct the weighted sum over the base view and all auxiliary views
						const Mat3& transfer = on_ceil ? aux.grid_hceil : aux.grid_hfloor;
						Vec2I p_aux = RoundVector(project(transfer * unproject(p)));

						// No need to check the y-coordinate (in fact we _must_ not)
						// as this will be dealt with by GetPayoff()
						if (p_aux[0] >= 0 && p_aux[0] < aux.geom.grid_size[0]) {
							// For now,  the aux view contribute half weight w.r.t to base view
							double contrib = 0.5;

							//double payoff = (*aux.payoffs)[orient][ p_aux[1] ][ p_aux[0] ];
							double payoff = aux.payoff_gen.GetPayoff(p_aux, orient);
							sum_payoffs += contrib * payoff;
							norm += contrib;

							// we only keep this data for visualization later
							aux.contrib_payoffs[orient][y][x] = payoff;
						}
					}

					// Record the final payoffs
					payoffs[orient][y][x] = sum_payoffs / norm;
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

		int nx = base_geom.grid_size[0];
		int ny = base_geom.grid_size[1];
		BrightColors bc;
		const AuxiliaryView& aux = aux_views[index];
		for (int y = 0; y < ny; y += 15) {
			if (abs(y-base_geom.horizon_row) < 25) continue; // skip the section near the horion
			bool on_ceil = (y < base_geom.horizon_row);
			for (int x = 0; x < nx; x += 15) {
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
		DPGeometry base_geom(&frame.pc(), fToC);
		joint_payoffs.Configure(base_geom, obj, zfloor, zceil);
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
		DPGeometry geom(&frame.pc(), fToC);
		joint_payoffs.AddView(geom, obj);
	}

	void MultiViewReconstructor::AddFrame(const PosedImage& frame) {
		CHECK(base_frame) << "AddFrame() was called before Configure()";
		aux_gen.push_back(new LineSweepDPScore(frame));
		AddFrame(frame, aux_gen.back().objective);
	}

	void MultiViewReconstructor::Reconstruct() {
		CHECK(base_frame) << "Reconstruct() was called before Configure()";

		DLOG << "Reconstructing with " << aux_frames.size() << " aux views";

		joint_payoffs.Compute();
		recon.Compute(*base_frame,
									joint_payoffs.base_geom,
									joint_payoffs.payoffs,
									base_objective->wall_penalty,
									base_objective->occl_penalty);
	}






	void MultiViewReconstructor::OutputBasePayoffs(const string& filename) {
		OutputPayoffsViz(filename,
										 base_frame->rgb,
										 joint_payoffs.base_payoffs,
										 joint_payoffs.base_geom);
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
											 joint_payoffs.aux_views[i].payoffs,
											 joint_payoffs.aux_views[i].geom);
			OutputPayoffsViz(str(format("%saux%02d_contrib.png") % basename % i),
											 base_frame->rgb,
											 joint_payoffs.aux_views[i].contrib_payoffs,
											 joint_payoffs.aux_views[i].geom);
			if (!aux_objectives.empty()) {
				aux_gen[i].line_sweeper.OutputOrientViz
					(str(format("%saux%02d_sweeps.png") % basename % i));
			}
		}
	}

	void MultiViewReconstructor::OutputJointPayoffs(const string& filename) {
		OutputPayoffsViz(filename,
										 base_frame->rgb,
										 joint_payoffs.payoffs,
										 joint_payoffs.base_geom);
	}

}  // namespace indoor_context
