#include <boost/ptr_container/ptr_vector.hpp>

#include <VW/Image/imagecopy.h>
#include <VNL/vector.tpp>
#include <VNL/Algo/matrixinverse.h>

#include "common_types_vw.h"
#include "timer.h"
#include "vanishing_points.h"
#include "image_bundle.h"
#include "range_utils.tpp"
#include "vpt_calib.h"

const lazyvar<int> gvNumIters("ConsVanPts.NumIterations");
const lazyvar<double> gvVoteThresh("ConsVanPts.VoteThreshold");

PixelRGB<byte> kElimColor(0, 0, 0);

int main(int argc, char **argv) {
	InitVars(argc, argv, "common.cfg");

	// Read the images and find vanishing points
	ptr_vector<ImageBundle> images;
	ptr_vector<VanishingPointsAlg> vpts;
	for (int i = 1; i < argc; i++) {
		images.push_back(new ImageBundle(argv[i]));
		vpts.push_back(VanishingPointsAlg::Create());
		vpts.back().Compute(images.back());
		vpts.back().OutputVanPointViz("out/vpts"+PaddedInt(i,3)+".png");
	}

	// Find indices of all image with >= 3 van pts
	vector<int> img_indices;
	for (int i = 0; i < vpts.size(); i++) {
		int nvpts = vpts[i].vanpts.size();
		if (nvpts >= 3) {
			img_indices.push_back(i);
		} else {
			DLOG << "Image " << i << " has only " << nvpts << " van pts, ignoring";
		}
	}

	// Iterate ransac
	int max_support = 0;
	MatrixFixed<3,3,double> best_model;
	DREPORT(*gvNumIters);
	for (int i = 0; i < *gvNumIters; i++) {
		DLOG << "Iteration " << i;
		SCOPED_INDENT;

		// Pick a random image and choose three random vanishing points from it
		int focus = img_indices[rand()%img_indices.size()];
		vector<int> vpt_indices(vpts[focus].vanpts.size());
		iota_all(vpt_indices, 0);
		random_shuffle_all(vpt_indices);
		const Vec3D& u = vpts[focus].vanpts[vpt_indices[0]].pt_cond;
		const Vec3D& v = vpts[focus].vanpts[vpt_indices[1]].pt_cond;
		const Vec3D& w = vpts[focus].vanpts[vpt_indices[2]].pt_cond;

		// Fit a model to this minimal dataset (only need two pairs since
		// we only recover up to scale). TODO: Should we include the third
		// pair (v,w) for robustness?
		vector<pair<Vec3D,Vec3D> > data;
		data.push_back(make_pair(u, v));
		data.push_back(make_pair(u, w));
		MatrixFixed<3,3,double> proposal;
		FitScaleModel(data, proposal);
		vpts[focus].OutputHighlightViz("out/proposal_data"+PaddedInt(i,2)+".png",
																	 VecI(&vpt_indices[0], 3));

		// If any entry in the estimated model is negative then no
		// solution exists (which is possible). If any entry is close to
		// zero then the solution is degenerate, usually a vanishing point
		// has been mis-estimated as closer to infinity than it should
		// be. In either case we simply discard the proposal.
		bool invalid = false;
		for (int i = 0; i < 3; i++) {
			if (proposal[i][i] < 0 || abs(proposal[i][i]) < 0.05) {
				invalid = true;
				break;
			}
		}
		if (invalid) continue;

		DLOG << "Proposal model:";
		INDENTED {
			DLOG << "Scale_x = " << sqrt(proposal[0][0]);
			DLOG << "Scale_y = " << sqrt(proposal[1][1]);
		}

		// Look for consensus with other vanishing point pairs
		vector<pair<Vec3D,Vec3D> > support;
		for (int j = 0; j < vpts.size(); j++) {
			for (int a = 0; a < vpts[j].vanpts.size(); a++) {
				for (int b = 0; b < a; b++) {
					const Vec3D& u = vpts[j].vanpts[a].pt_cond;
					const Vec3D& v = vpts[j].vanpts[b].pt_cond;
					const double err = abs(DotProduct(u, proposal*v));
					if (err < *gvVoteThresh) {
						support.push_back(make_pair(u, v));
					}
				}
			}
		}

		DREPORT(support.size());
		if (support.size() > max_support && support.size() >= 2) {
			max_support = support.size();
			FitScaleModel(support, best_model);
		}
	}
	DREPORT(max_support);
	DREPORT(best_model);
	DLOG << "ScaleX = " << sqrt(best_model[0][0]);
	DLOG << "ScaleY = " << sqrt(best_model[1][1]);
	DLOG;

	int outi = 0;
	for (int i = 0; i < vpts.size(); i++) {
		// Find agreeing pairs, set others' colors to black
		VecI agree(vpts[i].vanpts.size(), 0);
		for (int a = 0; a < vpts[i].vanpts.size(); a++) {
			for (int b = 0; b < a; b++) {
				const Vec3D& u = vpts[i].vanpts[a].pt_cond;
				const Vec3D& v = vpts[i].vanpts[b].pt_cond;
				const double err = abs(DotProduct(u, best_model*v));
				string filename;
				if (err < *gvVoteThresh) {
					agree[a] = agree[b] = 1;
					filename = "out/agree"+PaddedInt(outi++,3)+".png";
				} else {
					filename = "out/disagree"+PaddedInt(outi++,3)+".png";
				}
				vpts[i].OutputHighlightViz(filename, Vec2I(a, b));
			}
		}

		const int num_agree = agree.Sum();
		DLOG << "Image " << i << " has " << num_agree
				 << " of " << vpts[i].vanpts.size() << " vanpts in agreement";

	}

	return 0;
}
