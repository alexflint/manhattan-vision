#include "dp_affinities.h"

#include "common_types.h"

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
}
