#include "monocular_payoffs.h"

#include "common_types.h"
#include "manhattan_dp.h"
#include "dp_affinities.h"

#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	////////////////////////////////////////////////////////////////////////////////
	void FeaturePayoffGen::Compute(const MatF& feature,
																 const DPGeometry& geometry) {
		geom = geometry;

		// Compute integral images
		if (matrix_size(feature) == geom.camera->image_size()) {
			geometry.TransformDataToGrid(feature, grid_buffer);
			integ_feature.Compute(grid_buffer);
		} else {
			CHECK_EQ(matrix_size(feature), geometry.grid_size)
				<< "Feature must be same size as either image or grid";
			integ_feature.Compute(feature);
		}

		// Compute payoffs
		for (int i = 0; i < 2; i++) {
			vpayoffs[i].Resize(geom.grid_size);
			for (int y = 0; y < geom.grid_size[1]; y++) {
				float* outrow = vpayoffs[i].wall_scores[i][y];
				for (int x = 0; x < geom.grid_size[0]; x++) {
					outrow[x] = GetVertSum(x,y);
				}
			}
		}

		hpayoffs.Resize(geom.grid_size);
		for (int y = 0; y < geom.grid_size[1]; y++) {
			float* outrow0 = hpayoffs.wall_scores[0][y];
			float* outrow1 = hpayoffs.wall_scores[1][y];
			for (int x = 0; x < geom.grid_size[0]; x++) {
				outrow0[x] = outrow1[x] = GetHorizSum(x,y);
			}
		}
	}

	bool FeaturePayoffGen::Empty() const {
		return integ_feature.ny() == 0;
	}

	double FeaturePayoffGen::GetVertSum(int x, int y) const {
		CHECK_GT(integ_feature.nx(), 0);
		CHECK_INTERVAL(x, 0, geom.grid_size[0]-1);

		int y0, y1;
		geom.GetWallExtent(makeVector(x,y), y0, y1);
		return integ_feature.Sum(x, y0, y1);
	}

	double FeaturePayoffGen::GetHorizSum(int x, int y) const {
		CHECK_GT(integ_feature.nx(), 0);
		CHECK_INTERVAL(x, 0, geom.grid_size[0]-1);

		int y0, y1;
		geom.GetWallExtent(makeVector(x,y), y0, y1);
		return integ_feature.Sum(x, 0, y0-1)
			+ integ_feature.Sum(x, y1+1, geom.grid_size[1]-1);
	}


	////////////////////////////////////////////////////////////////////////////////
	ObjectivePayoffGen::ObjectivePayoffGen(const DPObjective& obj,
																				 const DPGeometry& geom) {
		Compute(obj, geom);
	}

	void ObjectivePayoffGen::Compute(const DPObjective& obj,
																	 const DPGeometry& geom) {
		Configure(obj, geom);
		ComputePayoffs(payoffs);
	}

	void ObjectivePayoffGen::Configure(const DPObjective& obj,
																		 const DPGeometry& geometry) {
		geom = geometry;
		wall_penalty = obj.wall_penalty;
		occl_penalty = obj.occl_penalty;

		Vec2I obj_size = matrix_size(obj.pixel_scores[0]);
		bool image_coords = (obj_size == geom.camera->image_size());
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

	bool ObjectivePayoffGen::Empty() const {
		return integ_scores[0].ny() == 0;
	}

	double ObjectivePayoffGen::GetWallScore(const Vec2& grid_pt, int axis) const {
		CHECK_GT(integ_scores[0].m_int.Rows(), 0)
			<< "Configure() must be called before GetPayoff()";
		CHECK_INTERVAL(grid_pt[0], 0, geom.grid_size[0]-1);

		int x = roundi(grid_pt[0]);
		int y0, y1;
		geom.GetWallExtent(grid_pt, y0, y1);
		int wall_orient = 1-axis;  // orients refer to normal direction rather than vpt index

		return integ_scores[kVerticalAxis].Sum(x, 0, y0-1)
			+ integ_scores[wall_orient].Sum(x, y0, y1)
			+ integ_scores[kVerticalAxis].Sum(x, y1+1, geom.grid_size[1]-1);
	}

	void ObjectivePayoffGen::ComputePayoffs(DPPayoffs& payoffs) const {
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
