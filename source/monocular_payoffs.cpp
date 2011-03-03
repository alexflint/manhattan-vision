#include "monocular_payoffs.h"

#include "manhattan_dp.h"
#include "common_types.h"

#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	MonocularPayoffGen::MonocularPayoffGen(const DPObjective& obj,
																				 const DPGeometry& geom) {
		Compute(obj, geom);
	}

	void MonocularPayoffGen::Compute(const DPObjective& obj,
																	 const DPGeometry& geom) {
		Configure(obj, geom);
		GetPayoffs(payoffs);
	}

	void MonocularPayoffGen::Configure(const DPObjective& obj,
																		 const DPGeometry& geometry) {
		geom = geometry;
		wall_penalty = obj.wall_penalty;
		occl_penalty = obj.occl_penalty;

		Vec2I obj_size = matrix_size(obj.pixel_scores[0]);
		bool image_coords = (obj_size == asToon(geom.camera->image_size()));
		if (!image_coords) {
			CHECK_EQ(obj_size, geometry.grid_size)
				<< "Objective must be same size as either image or grid";
		}
	
		// Compute per orientation, per pixel affinities
		MatF grid_buffer;  // don't give this a size yet
		for (int i = 0; i < 3; i++) {
			if (image_coords) {
				// Transform the scores according to ImageToGrid()
				geometry.TransformDataToGrid(obj.pixel_scores[i], grid_buffer);
				integ_scores[i].Compute(grid_buffer);
			} else {
				integ_scores[i].Compute(obj.pixel_scores[i]);
			}
		}
	}

	bool MonocularPayoffGen::Empty() const {
		return integ_scores[0].ny() == 0;
	}

	double MonocularPayoffGen::GetWallScore(const Vec2& grid_pt, int axis) const {
		CHECK_GT(integ_scores[0].m_int.Rows(), 0) << "Configure() must be called before GetPayoff()";
		CHECK_INTERVAL(grid_pt[0], 0, geom.grid_size[0]-1);

		int x = roundi(grid_pt[0]);
		int y0, y1;
		geom.GetWallExtent(grid_pt, axis, y0, y1);
		int wall_orient = 1-axis;  // orients refer to normal direction rather than vpt index

		return integ_scores[kVerticalAxis].Sum(x, 0, y0-1)
			+ integ_scores[wall_orient].Sum(x, y0, y1)
			+ integ_scores[kVerticalAxis].Sum(x, y1+1, geom.grid_size[1]-1);
	}

	void MonocularPayoffGen::GetPayoffs(DPPayoffs& payoffs) const {
		for (int i = 0; i < 3; i++) {
			CHECK_EQ(matrix_size(integ_scores[i]), geom.grid_size+makeVector(0,1));
		}

		payoffs.Resize(geom.grid_size);
		for (int i = 0; i < 2; i++) {
			for (int y = 0; y < geom.grid_size[1]; y++) {
				float* outrow = payoffs.wall_scores[i][y];
				for (int x = 0; x < geom.grid_size[0]; x++) {
					outrow[x] = GetWallScore(makeVector(x,y), i);
				}
			}
		}
		payoffs.wall_penalty = wall_penalty;
		payoffs.occl_penalty = occl_penalty;
	}
}
