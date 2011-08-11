#include "common_types.h"
#include "dp_payoffs.h"

namespace indoor_context {
	lazyvar<float> gvDefaultWallPenalty("ManhattanDP.DefaultWallPenalty");
	lazyvar<float> gvDefaultOcclusionPenalty("ManhattanDP.DefaultOcclusionPenalty");

	////////////////////////////////////////////////////////////////////////////////
	DPObjective::DPObjective()
		: wall_penalty(*gvDefaultWallPenalty), occl_penalty(*gvDefaultOcclusionPenalty) {
	}

	DPObjective::DPObjective(const Vec2I& size)
		: wall_penalty(*gvDefaultWallPenalty), occl_penalty(*gvDefaultOcclusionPenalty) {
		Resize(size);
	}

	void DPObjective::Resize(const Vec2I& size) {
		for (int i = 0; i < 3; i++) {
			// Note that the third parameter below forces an overwrite of the
			// entire matrix. This is slightly inefficient but protects
			// against the case that the user forgets to do this manually
			// (e.g. iterating over an image and writing into grid coordinates
			// will only write to the grid section inside image bounds)
			pixel_scores[i].Resize(size[1], size[0], 0);
		}
	}

	void DPObjective::CopyTo(DPObjective& rhs) {
		for (int i = 0; i < 3; i++) {
			rhs.pixel_scores[i] = pixel_scores[i];
		}
		rhs.wall_penalty = wall_penalty;
		rhs.occl_penalty = occl_penalty;
	}

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

	double DPPayoffs::SumOverPath(const VecI& path, const VecI& orients) const {
		CHECK_EQ(path.Size(), nx());

		double score = 0.0;
		for (int x = 0; x < path.Size(); x++) {
			CHECK_INTERVAL(orients[x], 0, 1);
			CHECK_INTERVAL(path[x], 0, ny()-1);
			score += wall_scores[ orients[x] ][ path[x] ][ x ];
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

	void DPPayoffs::CopyTo(DPPayoffs& other) {
		other.wall_scores[0] = wall_scores[0];
		other.wall_scores[1] = wall_scores[1];
		other.wall_penalty = wall_penalty;
		other.occl_penalty = occl_penalty;
	}
}
