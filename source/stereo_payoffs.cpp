#include "stereo_payoffs.h"

#include <TooN/LU.h>

#include "common_types.h"
#include "manhattan_dp.h"
#include "camera.h"
#include "image_utils.h"
#include "canvas.h"
#include "timer.h"
#include "geom_utils.h"

#include "image_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	void HomographyTransform(const ImageF& input,
													 MatF& output,
													 const Mat3& h) {  // h transforms from input to output coords
		ImageRef p;
		Mat3 hinv = LU<3>(h).get_inverse();
		for (int y = 0; y < output.Rows(); y++) {
			float* outrow = output[y];
			for (int x = 0; x < output.Cols(); x++) {
				toImageRef(project(hinv * makeVector(x,y,1.0)), p);
				if (p.x >= 0 && p.x < input.GetWidth() &&
						p.y >= 0 && p.y < input.GetHeight()) {
					outrow[x] = input[p].y;
				}
			}
		}
	}	



	ostream& operator<<(ostream& o, const NCCStatistics& stats) {
		o << format("{sum_a=%f, sum_b=%f, sum_asqr=%f, sum_bsqr=%f, sum_ab=%f, sum_wts=%f}")
			% stats.sum_a % stats.sum_b % stats.sum_asqr % stats.sum_bsqr % stats.sum_ab % stats.sum_wts;
		return o;
	}

	double NCCStatistics::CalculateNCC() {
		return CalculateNCC(sum_a, sum_b, sum_asqr, sum_bsqr, sum_ab, sum_wts);
	}

	void NCCStatistics::Add(double a, double b, double weight) {
		CHECK_GE(weight, 0.0);
		sum_a += weight*a;
		sum_b += weight*b;
		sum_asqr += weight*a*a;
		sum_bsqr += weight*b*b;
		sum_ab += weight*a*b;
		sum_wts += weight;
	}

	// static
	double NCCStatistics::CalculateNCC(double sum_a,
																		 double sum_b,
																		 double sum_asqr,
																		 double sum_bsqr,
																		 double sum_ab,
																		 double sum_wts) {
		CHECK_GE(sum_wts, 0.0);
		CHECK_GE(sum_asqr, 0.0);
		CHECK_GE(sum_bsqr, 0.0);
		if (sum_wts < 1e-8 || sum_asqr < 1e-8 || sum_bsqr < 1e-8) {
			return 0.0;
		} else {
			double rhs_a = sum_a*sum_a/sum_wts;
			double rhs_b = sum_b*sum_b/sum_wts;
			CHECK_GE(sum_asqr, rhs_a*(1.0-1e-6)) << format("sum_a=%f, sum_asqr=%f, sum_wts=%f") % sum_a % sum_asqr % sum_wts;
			CHECK_GE(sum_bsqr, rhs_b*(1.0-1e-6)) << format("sum_b=%f, sum_asqr=%f, sum_wts=%f") % sum_b % sum_bsqr % sum_wts;
			// Lots of instability checks here. We need to account for both
			// small relative differences and small absolute differences,
			// particularly near zero.
			if (sum_asqr+1e-8 < rhs_a*(1.0+1e-6) || sum_bsqr+1e-8 < rhs_b*(1.0+1e-6)) {
				return 0.0;  // not sure if this is really justified...
			} else {
				double denom = sum_asqr*sum_bsqr + rhs_a*rhs_b - sum_asqr*rhs_b - sum_bsqr*rhs_a;
				CHECK_GE(denom, 1e-8)
					<< format("sum_a=%f, sum_b=%f, sum_asqr=%f, sum_bsqr=%d, sum_ab=%f, sum_wts=%f")
					% sum_a % sum_b % sum_asqr % sum_bsqr % sum_ab % sum_wts;
				return (sum_ab - sum_a*sum_b/sum_wts) / sqrt(denom);
			}
		}
	}







	void FastNCC::Compute(const MatF& a, const MatF& b) {
		ComputeInternal(a, b, NULL);
	}

	void FastNCC::Compute(const MatF& a, const MatF& b, const MatI& mask) {
		ComputeInternal(a, b, &mask);
	}

	void FastNCC::Compute(const MatF& a,
												const MatF& b,
												const MatF& asqr,
												const MatF& bsqr,
												const MatF& ab,
												const MatI& nsamples) {
		intg_a.Compute(a);
		intg_b.Compute(b);
		intg_asqr.Compute(asqr);
		intg_bsqr.Compute(bsqr);
		intg_ab.Compute(ab);
		intg_nsamples.Compute(nsamples);
	}

	void FastNCC::ComputeInternal(const MatF& a, const MatF& b, const MatI* mask) {
		CHECK_SAME_SIZE(a, b);
		asqr.Resize(a.Rows(), a.Cols(), 0.0);
		bsqr.Resize(a.Rows(), a.Cols(), 0.0);
		ab.Resize(a.Rows(), a.Cols(), 0.0);
		nsamples.Resize(a.Rows(), a.Cols(), 0);
		for (int y = 0; y < a.Rows(); y++) {
			const float* a_row = a[y];
			const float* b_row = b[y];
			float* asqr_row = asqr[y];
			float* bsqr_row = bsqr[y];
			float* ab_row = ab[y];
			int* sample_row = nsamples[y];
			const int* mask_row = mask == NULL ? NULL : (*mask)[y];
			for (int x = 0; x < a.Cols(); x++) {
				if (mask == NULL || mask_row[x]) {
					asqr_row[x] = a_row[x] * a_row[x];
					bsqr_row[x] = b_row[x] * b_row[x];
					ab_row[x] = a_row[x] * b_row[x];
					sample_row[x] = 1;
				} else {
					CHECK_EQ(a_row[x], 0) << "inputs must be zero whereever mask is zero";
					CHECK_EQ(b_row[x], 0) << "inputs must be zero whereever mask is zero";
				}
			}
		}
		Compute(a, b, asqr, bsqr, ab, nsamples);
	}

	void FastNCC::AddStats(int col, int row0, int row1, NCCStatistics& stats) {
		CHECK_INTERVAL(col, 0, intg_a.nx()-1);
		CHECK_GT(intg_a.ny(), 0) << "AddStats() must not be called before Compute()";
		stats.sum_a += intg_a.Sum(col, row0, row1);
		stats.sum_b += intg_b.Sum(col, row0, row1);
		stats.sum_asqr += intg_asqr.Sum(col, row0, row1);
		stats.sum_bsqr += intg_bsqr.Sum(col, row0, row1);
		stats.sum_ab += intg_ab.Sum(col, row0, row1);

		float delta_wts = intg_nsamples.Sum(col, row0, row1);
		CHECK_GE(delta_wts, 0) << EXPR(col, row0, row1);
		stats.sum_wts += delta_wts;	// This is _not_ necessarily equal to row1-row0 !
	}

	double FastNCC::CalculateNCC(int col, int row0, int row1) {
		CHECK_INTERVAL(col, 0, intg_a.nx()-1);
		NCCStatistics stats;
		AddStats(col, row0, row1, stats);
		return stats.CalculateNCC();
	}




	void HomographyNCC::AddStats(int col, int row0, int row1, NCCStatistics& stats) {
		ncc.AddStats(col, row0, row1, stats);
	}

	double HomographyNCC::CalculateNCC(int col, int row0, int row1) {
		return ncc.CalculateNCC(col, row0, row1);
	}

	void HomographyNCC::Compute(const ImageF& left,
															const ImageF& right,
															const Mat3& left_xfer,
															const Mat3& right_xfer,
															const Vec2I& bounds) {
		CHECK_EQ(left.GetSize(), right.GetSize());

		int nx = left.GetWidth();
		int ny = left.GetHeight();
		left_xfered.Resize(ny, nx, 0);
		right_xfered.Resize(ny, nx, 0);
		mask.Resize(ny, nx, 0);

		ImageRef l, r;
		Vec3I p = makeVector(0,0,1);
		for (p[1] = 0; p[1] < ny; p[1]++) {
			float* left_row = left_xfered[ p[1] ];
			float* right_row = right_xfered[ p[1] ];
			int* mask_row = mask[ p[1] ];
			for (p[0] = 0; p[0] < nx; p[0]++) {
				if (left_row[ p[0] ] >= 0) { // left_row[x] < 0 indicates missing data
					toImageRef(left_xfer*p, l);
					if (l.x >= 0 && l.x < nx &&	l.y >= 0 && l.y < ny) {
						toImageRef(right_xfer*p, r);
						if (r.x >= 0 && r.x < nx &&	r.y >= 0 && r.y < ny) {
							// important to only set these when mask is also being set to 1
							left_row[ p[0] ] = left[l.y][l.x].y;
							right_row[ p[0] ] = right[r.y][r.x].y;
							mask_row[ p[0] ] = 1;
						}
					}
				}
			}
		}

		ncc.Compute(left_xfered, right_xfered, mask);
	}

	void HomographyNCC::OutputTransferredLeft(const string& file) {
		WriteMatrixImageRescaled(file, left_xfered);
	}

	void HomographyNCC::OutputTransferredRight(const string& file) {
		WriteMatrixImageRescaled(file, right_xfered);
	}




	void StereoPayoffGen::Compute(const PosedImage& l_image,
																const PosedImage& r_image,
																const DPGeometryWithScale& geom) {
		CHECK_SAME_SIZE(l_image, r_image);

		l_input = &l_image;
		r_input = &r_image;
		geometry = &geom;
		ImageRef l_sample, r_sample;
		Vec2I l_bounds = l_image.pc().image_size();

		// Ensure the mono images are available
		l_image.BuildMono();
		r_image.BuildMono();

		// Check for NaNs
		for (int y = 0; y < l_image.ny(); y++) {
			for (int x = 0; x < l_image.nx(); x++) {
				CHECK(isfinite(l_image.mono[y][x].y)) << EXPR(x, y, l_image.mono[y][x]);
				CHECK(isfinite(r_image.mono[y][x].y)) << EXPR(x, y, r_image.mono[y][x]);
			}
		}

		// Compute manhattan homologies
		Mat3 l_fToC = GetManhattanHomology(l_image.pc(), geometry->zfloor, geometry->zceil);
		Mat3 l_cToF = LU<3>(l_fToC).get_inverse();

		// Compute vertical rectifiers
		l_vrect = GetVerticalRectifier(l_image.pc());
		l_vrect_inv = LU<3>(l_vrect).get_inverse();

		// Compute manhattan homologies in rectified domain
		Mat3 l_vrect_fToC = l_vrect * l_fToC * l_vrect_inv;
		Mat3 l_vrect_cToF = l_vrect * l_cToF * l_vrect_inv;

		// Compute vertical rectifiers
		Mat3 r_vrect = GetVerticalRectifier(r_image.pc());
		Mat3 r_vrect_inv = LU<3>(r_vrect).get_inverse();

		// Compute horizon
		Vec3 l_horizon = l_image.pc().GetImageHorizon();
		Vec3 l_vrect_horizon = l_horizon * l_vrect_inv;
		double l_vrect_horizon_y = -l_vrect_horizon[2] / l_vrect_horizon[1];

		// Get the camera matrix for the left canera
		const PosedCamera& l_pc = l_image.pc();
		Mat3 l_intr = reinterpret_cast<const LinearCamera&>(l_pc.camera()).intrinsics();
		toon::Matrix<3,4> l_cam = l_image.pc().Linearize();

		// Construct the floor and ceiling planes
		Vec4 floor_plane = makeVector(0, 0, -1, geometry->zfloor);
		Vec4 ceil_plane = makeVector(0, 0, -1, geometry->zceil);

		// Compute transfer homographies
		Mat3 ltr_floor = GetHomographyVia(l_image.pc(), r_image.pc(), floor_plane);
		Mat3 ltr_ceil = GetHomographyVia(l_image.pc(), r_image.pc(), ceil_plane);

		// Compute cross correlations for floor and ceiling surfaces
		floor_ncc.Compute(l_image.mono, r_image.mono, l_vrect_inv, ltr_floor*l_vrect_inv, l_bounds);
		ceil_ncc.Compute(l_image.mono, r_image.mono, l_vrect_inv, ltr_ceil*l_vrect_inv, l_bounds);

		// Rectify and transpose the intensity images
		Mat3 tr = Zeros;
		tr[0][1] = tr[1][0] = tr[2][2] = 1.0;
		l_vrect_im_tr.Resize(l_image.nx(), l_image.ny(), -1);  // *transposed* image
		r_vrect_im_tr.Resize(l_image.nx(), l_image.ny(), -1);  // *transposed* image
		HomographyTransform(l_image.mono, l_vrect_im_tr, tr*l_vrect);
		HomographyTransform(r_image.mono, r_vrect_im_tr, tr*r_vrect);

		// Check that grid_floorToCeil is a pure scale+translation of y coordinates
		CHECK_EQ_TOL(geom.grid_floorToCeil[0][0], 1.0, 1e-8);  // no scaling in x
		CHECK_LE(abs(geom.grid_floorToCeil[0][1]), 1e-8);
		CHECK_LE(abs(geom.grid_floorToCeil[0][2]), 1e-8);  // no translation in x
		CHECK_LE(abs(geom.grid_floorToCeil[1][0]), 1e-8);
		CHECK_LE(abs(geom.grid_floorToCeil[2][0]), 1e-8);
		CHECK_LE(abs(geom.grid_floorToCeil[2][1]), 1e-8);
		CHECK_EQ_TOL(geom.grid_floorToCeil[2][2], 1.0, 1e-8);  // normalised
		double grid_fToC_sy = geom.grid_floorToCeil[1][1];
		double grid_fToC_ty = geom.grid_floorToCeil[1][2];

		toon::Matrix<3,2> curry_x = Zeros;
		curry_x[1][0] = curry_x[2][1] = 1.0;

		// Compute the NCC payoffs
		payoffs.Resize(geom.grid_size[1], geom.grid_size[0]);
		for (int y = 0; y < geom.grid_size[1]; y++) {
			float* payoffs_row = payoffs[y];

			// Compute the vertical transfer function for this image row
			const Vec4& surf_plane = (y < geom.horizon_row) ? ceil_plane : floor_plane;
			Vec3 pt = makeVector(0,y,1.0);  // this can be any point along the current image row
			Vec3 surf_pt = IntersectRay(geom.gridToImage*pt, l_cam, surf_plane);
			const SO3<>& l_rot = l_pc.pose().get_rotation();
			Vec3 line_nrm = l_rot.inverse() * l_intr.T() * geom.imageToGrid.T() * makeVector(0,-1.0,y);
			Vec3 plane_nrm = unit(makeVector(line_nrm[0], line_nrm[1], 0));
			Vec4 plane_eqn = concat(plane_nrm, -plane_nrm*surf_pt);
			Mat3 ltr_wall = GetHomographyVia(l_image.pc(), r_image.pc(), plane_eqn);

			// These transform from grid coordinates to l_vrect and r_vrect
			Mat3 grid_to_l = l_vrect * geom.gridToImage;
			Mat3 grid_to_r = r_vrect * ltr_wall * geom.gridToImage;

			// Compute vrect coords
			int grid_y0, grid_y1;
			if (y < geom.horizon_row) {
				grid_y0 = y;
				grid_y1 = Clamp<int>((y-grid_fToC_ty)/grid_fToC_sy, 0, geom.grid_size[1]-1);
			} else {
				grid_y0 = Clamp<int>(grid_fToC_sy*y + grid_fToC_ty, 0, geom.grid_size[1]-1);
				grid_y1 = y;
			}
			CHECK_LE(grid_y0, grid_y1);

			// Compute NCCs for each column
			for (int x = 0; x < geom.grid_size[0]; x++) {
				//bool special = viz_mask[y][x];
				curry_x[0][1] = x;

				// Compute the remaining transform after x is given
				toon::Matrix<3,2> grid_to_ly = grid_to_l * curry_x;
				grid_to_ly /= grid_to_ly[2][1];
				CHECK_LE(abs(grid_to_ly[0][0]), 1e-8);  // ensure that output X is independent of input Y
				CHECK_LE(abs(grid_to_ly[2][0]), 1e-8); 
				int lx = grid_to_ly[0][1];  // don't clamp LX here, it MUST be outside the image sometimes (see the IF below)
				float l_m = grid_to_ly[1][0];
				float l_c = grid_to_ly[1][1];
				CHECK_GT(l_m, 0);

				toon::Matrix<3,2> grid_to_ry = grid_to_r * curry_x;
				grid_to_ry /= grid_to_ry[2][1];
				CHECK_LE(abs(grid_to_ry[0][0]), 1e-8);  // ensure that output X is independent of input Y
				CHECK_LE(abs(grid_to_ry[2][0]), 1e-8);
				int rx = grid_to_ry[0][1];  // don't clamp RX here, it MUST be outside the image sometimes (see the IF below)
				float r_m = grid_to_ry[1][0];
				float r_c = grid_to_ry[1][1];

				// Compute contributions from the horizontal component
				int vrect_x = Clamp<int>(lx, 0, l_image.nx()-1);  // Clamp here (not above) since we test LX in the IF below
				int vrect_y0 = Clamp<int>(l_m*grid_y0 + l_c, 0, l_image.ny()-1);
				int vrect_y1 = Clamp<int>(l_m*grid_y1 + l_c, 0, l_image.ny()-1);
				NCCStatistics stats;
				ceil_ncc.AddStats(vrect_x, 0, vrect_y0-1, stats);  // portion above the ceiling point
				floor_ncc.AddStats(vrect_x, vrect_y1, l_image.ny()-1, stats);  // portion below the floor point


				// Pull out the rows
				// RY can be outside image bounds because the images may not entirely overlap
				if (lx >= 0 && lx < l_vrect_im_tr.Rows() &&
						rx >= 0 && rx < r_vrect_im_tr.Rows()) {
					const float* l_row = l_vrect_im_tr[lx];  // in transposed image, rows are columns...
					const float* r_row = r_vrect_im_tr[rx];  // in transposed image, rows are columns...  
					int l_len = l_vrect_im_tr.Cols();
					int r_len = r_vrect_im_tr.Cols();

					// Compute contributions from the vertical component
					for (int yy = grid_y0; yy <= grid_y1; yy++) {  // use '<=' because grid_y1 always < geom.grid_size[1]
						int ly = l_m*yy + l_c;
						int ry = r_m*yy + r_c;

						if (ly >= 0 && ly < l_len && l_row[ly] >= 0 &&
								ry >= 0 && ry < r_len && r_row[ry] >= 0) {
							// l_m measures the number of pixels in the vrect
							// domain that each grid pixel corresponds
							// to. Therefore we weight each grid measurement by
							// l_m.
							stats.Add(l_row[ly], r_row[ry], l_m);  // accessing row from transposed image where y=column
						}
					}
				}

				if (stats.sum_wts == 0) {
					payoffs_row[x] = 0;
				} else {
					// NOTE: I think it makes sense to take the absolute NCC since
					// a large negative NCC suggests an anti-correlation, which
					// indicates a good match. But I'm not sure...
					// I'm now adding 1, for convenience and since the DP objective function is additive
					payoffs_row[x] = 1.0 + stats.CalculateNCC();
					CHECK_PRED1(isfinite, payoffs_row[x]) << "[x="<<x<<",y="<<y<<"], stats:"<<stats;
				}
			}
		}
	}



	void StereoPayoffGen::OutputPayoffs(const string& file) {
		FileCanvas payoffs_canvas(file, l_input->rgb);
		for (int y = 0; y < l_input->ny(); y += 5) {
			for (int x = 0; x < l_input->nx(); x += 5) {
				Vec2 vrect_p = project(geometry->imageToGrid * makeVector(x,y,1.0));
				double payoff = payoffs[ roundi(vrect_p[1]) ][ roundi(vrect_p[0]) ];
				if (payoff >= 0) {
					PixelRGB<byte> color(0,payoff*255,0);  // payoffs are in [0,1]
					payoffs_canvas.DrawDot(makeVector(x,y), color);
				}
			}
		}
	}

	void StereoPayoffGen::OutputRawPayoffs(const string& file) {
		WriteMatrixImageRescaled(file, payoffs);
	}
}  // namespace indoor_contex
