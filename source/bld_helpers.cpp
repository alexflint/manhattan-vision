#include <boost/filesystem.hpp>

#include "bld_helpers.h"

#include "common_types.h"
#include "guided_line_detector.h"
#include "manhattan_dp.h"
#include "vars.h"
#include "map.pb.h"
#include "map.h"
#include "camera.h"
#include "clipping.h"
#include "camera.h"
#include "floorplan_renderer.h"

namespace indoor_context {
	using namespace toon;

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

	void GetTrueOrients(const proto::FloorPlan& floorplan,
	                    const PosedCamera& pc,
	                    MatI& gt_orients) {
		FloorPlanRenderer re;
		re.RenderOrients(floorplan, pc, gt_orients);
	}

	void DownsampleOrients(const MatI& in, MatI& out, const toon::Vector<2,int>& res) {
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



	void LoadTrueOrientsOld(const proto::TruthedFrame& tru_frame,
	                        MatI& gt_orients,
	                        bool label_by_tangents) {
		CHECK(tru_frame.has_orient_map_file());
		CHECK(fs::exists(tru_frame.orient_map_file()));
		ImageBundle gt_orient_image(tru_frame.orient_map_file());
		gt_orients.Resize(gt_orient_image.ny(), gt_orient_image.nx());
		for (int y = 0; y < gt_orients.Rows(); y++) {
			PixelRGB<byte>* inrow = gt_orient_image.rgb[y];
			int* outrow = gt_orients[y];
			for (int x = 0; x < gt_orients.Cols(); x++) {
				const PixelRGB<byte>& pixel = inrow[x];
				if (pixel.r > 0) {
					outrow[x] = label_by_tangents ? 0 : 1;
				} else if (pixel.g > 0) {
					outrow[x] = label_by_tangents ? 1 : 0;
				} else {
					outrow[x] = 2;
				}
			}
		}
	}
}
