#include "common_types.h"
#include "dp_payoffs.h"

namespace indoor_context {
	lazyvar<float> gvDefaultWallPenalty("ManhattanDP.DefaultWallPenalty");
	lazyvar<float> gvDefaultOcclusionPenalty("ManhattanDP.DefaultOcclusionPenalty");

	////////////////////////////////////////////////////////////////////////////////
	DPPayoffs::DPPayoffs()
		: wall_penalty(*gvDefaultWallPenalty), occl_penalty(*gvDefaultOcclusionPenalty) {
	}

	DPPayoffs::DPPayoffs(Vec2I size)
		: wall_penalty(*gvDefaultWallPenalty), occl_penalty(*gvDefaultOcclusionPenalty) {
		Resize(size);
	}

	void DPPayoffs::Clear(float fill) {
		wall_scores[0].Fill(fill);
		wall_scores[1].Fill(fill);
	}

	void DPPayoffs::Resize(Vec2I size) {
		wall_scores[0].Resize(size[1], size[0], 0);
		wall_scores[1].Resize(size[1], size[0], 0);
	}

	double DPPayoffs::ComputeScore(const VecI& path,
																 const VecI& axes,
																 int num_walls,
																 int num_occls) const {
		CHECK_EQ(path.Size(), nx());
		return SumOverPath(path, axes) - wall_penalty*num_walls - occl_penalty*num_occls;
	}

	double DPPayoffs::SumOverPath(const VecI& path, const VecI& axes) const {
		CHECK_EQ(path.Size(), nx());
		CHECK_EQ(axes.Size(), nx());

		double score = 0.;
		for (int x = 0; x < path.Size(); x++) {
			CHECK_INTERVAL(axes[x], 0, 1);
			CHECK_INTERVAL(path[x], 0, ny()-1);
			score += wall_scores[ axes[x] ][ path[x] ][ x ];
		}
		return score;
	}

	void DPPayoffs::Add(const DPPayoffs& other, double weight) {
		CHECK_SAME_SIZE(other.wall_scores[0], wall_scores[0]);
		CHECK_SAME_SIZE(other.wall_scores[1], wall_scores[1]);
		for (int i = 0; i < 2; i++) {
			for (int y = 0; y < wall_scores[i].Rows(); y++) {
				const float* in = other.wall_scores[i][y];
				float* out = wall_scores[i][y];
				for (int x = 0; x < wall_scores[i].Cols(); x++) {
					out[x] += weight*in[x];
				}
			}
		}
	}

	void DPPayoffs::Add(const MatF& delta, double weight) {
		CHECK_SAME_SIZE(delta, wall_scores[0]);
		CHECK_SAME_SIZE(delta, wall_scores[1]);
		for (int i = 0; i < 2; i++) {
			for (int y = 0; y < delta.Rows(); y++) {
				const float* in = delta[y];
				float* out = wall_scores[i][y];
				for (int x = 0; x < delta.Cols(); x++) {
					out[x] += weight*in[x];
				}
			}
		}
	}

	void DPPayoffs::CopyTo(DPPayoffs& other) const {
		other.wall_scores[0] = wall_scores[0];
		other.wall_scores[1] = wall_scores[1];
		other.wall_penalty = wall_penalty;
		other.occl_penalty = occl_penalty;
	}
}
