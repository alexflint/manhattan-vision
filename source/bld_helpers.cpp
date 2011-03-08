#include "bld_helpers.h"

#include <math.h>

#include <boost/filesystem.hpp>
#include <LU.h>

#include "common_types.h"
#include "manhattan_dp.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"
#include "clipping.h"
#include "camera.h"
#include "geom_utils.h"
#include "canvas.h"
#include "vw_image_io.h"

#include "vw_image.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	lazyvar<string> gvSequencesDir("Sequences.DataDir");
	lazyvar<string> gvMapPath("Sequences.MapPath");



	string GetMapPath(const string& sequence_name) {
		fs::path file = fs::path(*gvSequencesDir) / sequence_name / *gvMapPath;
		CHECK_PRED1(fs::exists, file)
			<< "Couldn't find map for sequence: "<<sequence_name<<"\nPath: "<<file;
		return file.string();
	}


	void DownsampleOrients(const MatI& in, MatI& out, const Vec2I& res) {
		MatI votes[3];
		out.Resize(res[1], res[0]);
		for (int i = 0; i < 3; i++) {
			votes[i].Resize(res[1], res[0], 0);
		}

		for (int y = 0; y < in.Rows(); y++) {
			int yy = y * res[1] / in.Rows();
			for (int x = 0; x < in.Cols(); x++) {
				int xx = x * res[0] / in.Cols();
				if (in[y][x] >= 0) {
					CHECK_LT(in[y][x], 3);
					votes[ in[y][x] ][yy][xx]++;
				}
			}
		}

		int area = (in.Rows()/res[1]) * (in.Cols()/res[0]);
		for (int y = 0; y < res[1]; y++) {
			for (int x = 0; x < res[0]; x++) {
				out[y][x] = -1;
				int maxv = area/10;  // below this threshold pixels will be "unknown"
				for (int i = 0; i < 3; i++) {
					if (votes[i][y][x] > maxv) {
						maxv = votes[i][y][x];
						out[y][x] = i;
					}
				}
			}
		}
	}

	void DownsampleOrients(const MatI& in, MatI& out, int k) {
		DownsampleOrients(in, out, makeVector(in.Cols()/k, in.Rows()/k));
	}

	double ComputeAgreementPct(const MatI& a, const MatI& b) {
		return 1.0*ComputeAgreement(a, b) / (a.Rows()*a.Cols());
	}

	int ComputeAgreement(const MatI& a, const MatI& b) {
		CHECK_EQ(a.Rows(), b.Rows());
		CHECK_EQ(a.Cols(), a.Cols());
		int n = 0;
		for (int y = 0; y < a.Rows(); y++) {
			const int* arow = a[y];
			const int* brow = b[y];
			for (int x = 0; x < a.Cols(); x++) {
				if (arow[x] == brow[x]) n++;
			}
		}
		return n;
	}

	void InterchangeLabels(MatI& m, int a, int b) {
		for (int y = 0; y < m.Rows(); y++) {
			int* row = m[y];
			for (int x = 0; x < m.Cols(); x++) {
				// note that row[x] is one of -1,0,1,2
				if (row[x] == a) {
					row[x] = b;
				} else if (row[x] == b) {
					row[x] = a;
				}
			}
		}
	}

	Mat3 GetFloorCeilHomology(const PosedCamera& pc, const proto::FloorPlan& fp) {
		double zfloor = fp.zfloor();
		double zceil = fp.zceil();
		Vec3 vup = pc.pose().inverse() * makeVector(0,1,0);
		if (Sign(zceil-zfloor) == Sign(vup[2])) {
			swap(zfloor, zceil);
		}
		return GetManhattanHomology(pc, zfloor, zceil);
	}




	void DrawPayoffs(Canvas& canvas,
									 const DPPayoffs& payoffs,
									 const DPGeometry& geom) {
		double max_payoff = 0;
		for (int i = 0; i < 2; i++) {
			for (int y = 0; y < payoffs.wall_scores[i].Rows(); y++) {
				const float* row = payoffs.wall_scores[i][y];
				for (int x = 0; x < payoffs.wall_scores[i].Cols(); x++) {
					if (row[x] > max_payoff) max_payoff = row[x];
				}
			}
		}

		static const double kDotSize = 1.0;
		for (int i = 0; i < 2; i++) {
			Vec2 tdot = makeVector(kDotSize*i*1.5, 0);
			for (int y = 0; y < payoffs.wall_scores[i].Rows(); y++) {
				const float* row = payoffs.wall_scores[i][y];
				for (int x = 0; x < payoffs.wall_scores[i].Cols(); x++) {
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
												const DPPayoffs& payoffs,
												const DPGeometry& geom) {
		FileCanvas canvas(filename, orig);
		DrawPayoffs(canvas, payoffs, geom);
	}

	void ComputeDepthErrors(const MatD& est_depth,
													const MatD& gt_depth,
													MatF& errors) {
		CHECK_SAME_SIZE(est_depth, gt_depth);
		errors.Resize(est_depth.Rows(), est_depth.Cols());

		// Check ground truth
		double gt_min = gt_depth.MinValue();
		double gt_max = gt_depth.MaxValue();
		if (!isfinite(gt_max)) {
			WriteImageOfNonFinites("nonfinite_depths.png", gt_depth);
			CHECK_PRED1(isfinite, gt_max) << "Wrote visualization to nonfinite_depths.png";
		}
		if (gt_min <= 0) {
			ImageRGB<byte> canvas(gt_depth.Cols(), gt_depth.Rows());
			DrawMatrixRescaled(gt_depth, canvas);
			DrawNegatives(gt_depth, canvas);
			WriteImage("negative_depths.png", canvas);
			CHECK_GT(gt_min, 0) << "Wrote visualization to negative_depths.png";
		}

		// Check estimated depth
		double min_est_depth = est_depth.MinValue();
		double max_est_depth = est_depth.MaxValue();
		if (!isfinite(max_est_depth)) {
			WriteImageOfNonFinites("nonfinite_depths.png", est_depth);
			CHECK_PRED1(isfinite, max_est_depth) << "Wrote visualization to nonfinite_depths.png";
		}
		if (min_est_depth < 0) {
			DLOG << "Warning: est_depth contains negative values (min is "<<min_est_depth<<")";
		}

		// Compute per-pixel errors
		CHECK_SAME_SIZE(gt_depth, est_depth);
		errors.Resize(gt_depth.Rows(), gt_depth.Cols());
		for (int y = 0; y < gt_depth.Rows(); y++) {
			const double* est_row = est_depth[y];
			const double* gt_row = gt_depth[y];
			float* err_row = errors[y];
			for (int x = 0; x < gt_depth.Cols(); x++) {
				// Should we be dividing by the maximum? Or should we always
				// divide by the ground truth depth?
				err_row[x] = abs(est_row[x]-gt_row[x]) / max(est_row[x],gt_row[x]);
			}
		}
	}

	// Compute per--pixel relative depth errors
	double ComputeMeanDepthError(const MatF& errors) {
		double max_error = errors.MaxValue();
		double min_error = errors.MinValue();
		CHECK_PRED1(isfinite, max_error);
		CHECK_GE(min_error, 0);
		return errors.ArrayOneNorm() / (errors.Rows()*errors.Cols());
	}
}
