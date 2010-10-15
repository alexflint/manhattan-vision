#include <LU.h>

#include "entrypoint_types.h"
#include "map.h"
#include "colors.h"
#include "canvas.h"
#include "timer.h"

#include "integral_col_image.tpp"
#include "io_utils.tpp"
#include "image_utils.tpp"
#include "math_utils.tpp"
#include "vector_utils.tpp"

using namespace indoor_context;

// Computes NCC from five statistics of the two datasets
class NCCStatistics {
public:
	float sum_a, sum_b, sum_asqr, sum_bsqr, sum_ab;
	int nsamples;

	NCCStatistics()
		: sum_a(0), sum_b(0), sum_asqr(0), sum_bsqr(0), sum_ab(0), nsamples(0) { }

	// Add a measurement
	inline void Add(double a, double b) {
		sum_a += a;
		sum_b += b;
		sum_asqr += a*a;
		sum_bsqr += b*b;
		sum_ab += a*b;
		nsamples++;
	}

	// Compute the normalized cross-correlation for the current statistics
	double CalculateNCC();

	// Compute the normalized cross-correlation between two vectors given
	// five statistics:
	//   sum_a =  sum(a_i)
	//   sum_b =  sum(b_i)
	//   sum_asqr = sum(a_i*a_i)
	//   sum_bsqr = sum(b_i*b_i)
	//   sum_ab =  sum(a_i*b_i)
	//   n = length of vectors a and b
	// Returns unit(a-mean(a)) * unit(b-mean(b))
	//   where unit vectors are computed w.r.t. euclidean 2-norm
	// Special case for n=0: return 0
	// Special case for n=1: return 1
	// Special case for sum_xsqr==sum_x*sum_x/n: return 0
	//   {x is a or b, happens only when all x_i in original dataset are equal}
	static double CalculateNCC(double sum_a,
														 double sum_b,
														 double sum_asqr,
														 double sum_bsqr,
														 double sum_ab,
														 int nsamples);
};

ostream& operator<<(ostream& o, const NCCStatistics& stats) {
	o << format("{sum_a=%f, sum_b=%f, sum_asqr=%f, sum_bsqr=%f, sum_ab=%f, n=%d}")
					 % stats.sum_a % stats.sum_b % stats.sum_asqr % stats.sum_bsqr % stats.sum_ab % stats.nsamples;
	return o;
}

double NCCStatistics::CalculateNCC() {
	return CalculateNCC(sum_a, sum_b, sum_asqr, sum_bsqr, sum_ab, nsamples);
}

// static
double NCCStatistics::CalculateNCC(double sum_a,
																	 double sum_b,
																	 double sum_asqr,
																	 double sum_bsqr,
																	 double sum_ab,
																	 int n) {
	if (n == 0) {
		return 0.0;
	} else if (n == 1) {
		return 1.0;
	} else {
		double denom_sqr = (sum_asqr - sum_a*sum_a/n)*(sum_bsqr - sum_b*sum_b/n);
		CHECK_GE(denom_sqr, 0)  // TODO: remove this for speed
			<< format("{sum_a=%f, sum_b=%f, sum_asqr=%f, sum_bsqr=%f, sum_ab=%f, n=%d}")
			% sum_a % sum_b % sum_asqr % sum_bsqr % sum_ab % n;
		if (denom_sqr < 1e-6) {
			return 0;  // not sure if this is justified??
		} else {
			return (sum_ab - sum_a*sum_b/n) / sqrt(denom_sqr);
		}
	}
}







// Pre-computes statistics in order to calculate NCC between two
// images for any segment of any image column in O(1) time.
class FastNCC {
public:
	IntegralColImage<float> intg_a;
	IntegralColImage<float> intg_b;	
	IntegralColImage<float> intg_asqr;
	IntegralColImage<float> intg_bsqr;
	IntegralColImage<float> intg_ab;
	IntegralColImage<int> intg_nsamples;

	// These buffers are used internally; they are member variables so
	// that they can be re-used without re-allocation.
	MatF asqr, bsqr, ab;
	MatI nsamples;

	// Prepare integral images for fast computation of NCC between A and B.
	void Compute(const MatF& a, const MatF& b);

	// Prepare integral images for fast computation of NCC between A and
	// B ignoring any positions for which mask[pos]==0.
	void Compute(const MatF& a, const MatF& b, const MatI& mask);

	// Prepare integral images for fast computation of NCC between A and
	// B. If mask is not null then ignore any positions for which
	// (*mask)[pos]==0
	void ComputeInternal(const MatF& a, const MatF& b, const MatI* mask);

	// Calculate the NCC for the segment [row0,row1) of the specified column
	double CalculateNCC(int col, int row0, int row1);

	// Calculate NCC statistics for the segment [row0,row1) of the specified column
	void AddStats(int col, int row0, int row1, NCCStatistics& stats);
};

void FastNCC::Compute(const MatF& a, const MatF& b) {
	ComputeInternal(a, b, NULL);
}

void FastNCC::Compute(const MatF& a, const MatF& b, const MatI& mask) {
	ComputeInternal(a, b, &mask);
}

void FastNCC::ComputeInternal(const MatF& a, const MatF& b, const MatI* mask) {
	CHECK_EQ(a.Rows(), b.Rows());
	CHECK_EQ(a.Cols(), b.Cols());
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
				CHECK_EQ(a_row[x], 0);
				CHECK_EQ(b_row[x], 0);
			}
		}
	}
	intg_a.Compute(a);
	intg_b.Compute(b);
	intg_asqr.Compute(asqr);
	intg_bsqr.Compute(bsqr);
	intg_ab.Compute(ab);
	intg_nsamples.Compute(nsamples);
}

void FastNCC::AddStats(int col, int row0, int row1, NCCStatistics& stats) {
	CHECK_GT(intg_a.ny(), 0) << "CalculateNCC() was called before Compute()";
	stats.sum_a += intg_a.Sum(col, row0, row1);
	stats.sum_b += intg_b.Sum(col, row0, row1);
	stats.sum_asqr += intg_asqr.Sum(col, row0, row1);
	stats.sum_bsqr += intg_bsqr.Sum(col, row0, row1);
	stats.sum_ab += intg_ab.Sum(col, row0, row1);
	stats.nsamples += intg_nsamples.Sum(col, row0, row1);	// This is _not_ necessarily equal to row1-row0 !
}

double FastNCC::CalculateNCC(int col, int row0, int row1) {
	NCCStatistics stats;
	AddStats(col, row0, row1, stats);
	return stats.CalculateNCC();
}





inline void toImageRef(const Vec2& v, ImageRef& p) {
	p.x = v[0];
	p.y = v[1];
}

inline void toImageRef(const Vec3& v, ImageRef& p) {
	p.x = v[0]/v[2];
	p.y = v[1]/v[2];
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 4) {
		DLOG << "Usage: "<<argv[0]<<" truthed_map.pro FRAME1 FRAME2";
		return -1;
	}

	// Input arguments
	const char* path = argv[1];
	const int l_id = atoi(argv[2]);
	const int r_id = atoi(argv[3]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(path, gt_map);
	Frame* l_frame = map.KeyFrameByIdOrDie(l_id);
	Frame* r_frame = map.KeyFrameByIdOrDie(r_id);
	l_frame->LoadImage();
	r_frame->LoadImage();
	const PosedImage& l_image = l_frame->image;
	const PosedImage& r_image = r_frame->image;
	l_image.BuildMono();
	r_image.BuildMono();
	ImageRef l_sample, r_sample;

	// Get the floor and ceiling positions
	double zfloor = gt_map.floorplan().zfloor();
	double zceil = gt_map.floorplan().zceil();
	Vec3 vup = map.kfs[0].pc->pose_inverse() * makeVector(0,1,0);
	if (Sign(zceil-zfloor) == Sign(vup[2])) {
		swap(zfloor, zceil);
		DLOG << "swapping zfloor and zceil";
	}

	// Compute transfer homographies
	Mat3 ltr_hfloor = GetHomographyVia(l_image.pc(),
																			 r_image.pc(),
																			 makeVector(0, 0, -1, zfloor));
	Mat3 ltr_hceil = GetHomographyVia(l_image.pc(),
																			r_image.pc(),
																			makeVector(0, 0, -1, zceil));

	// Compute manhattan homologies
	Mat3 l_fToC = GetManhattanHomology(l_image.pc(), zfloor, zceil);
	Mat3 l_cToF = LU<3>(l_fToC).get_inverse();

	// Compute vertical rectifiers
	Mat3 l_vrect = GetVerticalRectifier(l_image.pc());
	Mat3 l_vrect_inv = LU<3>(l_vrect).get_inverse();

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
	DREPORT(l_vrect_horizon_y);

	// Setup visualization canvases
	FileCanvas lsamples_canvas("out/samples_l.png", l_image.rgb);
	FileCanvas rsamples_canvas("out/samples_r.png", r_image.rgb);
	BrightColors bc;

	// Specify some points to visualize
	MatI viz_mask(l_image.ny(), l_image.nx(), 0);
	int vizp[] = {44,282,  99,214,  210,195,  260,234};
	for (int i = 0; i < 8; i += 2) {
		ImageRef p = asIR(project(l_vrect * makeVector(vizp[i],vizp[i+1],1.0)));
		viz_mask[p.y][p.x] = 1;
	}

	// Rectify the intensity images
	MatF l_vrect_vals(l_image.ny(), l_image.nx(), 0.0);  // important to initialize to zero here
	MatF l_vrect_xfered_vals(l_image.ny(), l_image.nx(), 0.0);  // important to initialize to zero here
	MatI mask(l_image.ny(), l_image.nx(), 0.0);  // important to initialize to zero here
	for (int y = 0; y < l_image.ny(); y++) {
		float* l_row = l_vrect_vals[y];
		float* xfered_row = l_vrect_xfered_vals[y];
		int* mask_row = mask[y];
		const Mat3& horiz_xfer = y < l_vrect_horizon_y ? ltr_hceil : ltr_hfloor;
		Mat3 vrect_xfer = horiz_xfer * l_vrect_inv;  // Don't pre-multiply by r_vrect as we access the orig image
		for (int x = 0; x < l_image.nx(); x++) {
			toImageRef(l_vrect_inv * makeVector(x,y,1.0), l_sample);
			// TODO: bilinear interpolation here
			if (l_image.contains(l_sample)) {
				toImageRef(vrect_xfer * makeVector(x,y,1.0), r_sample);
				if (r_image.contains(r_sample)) {
					// important to only set these when mask is also being set to 1
					l_row[x] = l_image.mono[l_sample].y;
					xfered_row[x] = r_image.mono[r_sample].y;
					mask_row[x] = 1.0;
				}
			}
		}
	}

	// Precompute statics for fast NCC calculation
	// Note that where mask is 0, l_vrect_vals and lvrect_xfered_vals *must* also be 0
	FastNCC ncc_comp;
	ncc_comp.Compute(l_vrect_vals, l_vrect_xfered_vals, mask);

	// Compute the NCC payoffs
	MatF payoffs(l_image.ny(), l_image.nx());
	TIMED("Compute payoffs")
	for (int y = 0; y < l_image.ny(); y++) {
		float* payoffs_row = payoffs[y];

		// Compute the vertical transfer function for this image row
		const PosedCamera& l_pc = l_image.pc();
		const Mat3 l_intr = reinterpret_cast<const LinearCamera&>(l_pc.camera()).intrinsics();
		Vec3 vrect_p = makeVector(0,y,1.0);  // this can be any point along the current image row
		double surf_z = (y < l_vrect_horizon_y) ? zceil : zfloor;
		Vec4 surf_plane = makeVector(0,0,-1.0,surf_z);
		toon::Matrix<3,4> l_cam = l_image.pc().Linearize();
		Vec3 surf_p = IntersectRay(l_vrect_inv * vrect_p, l_cam, surf_plane);
		const SO3<>& l_rot = l_pc.pose().get_rotation();
		Vec3 line_nrm = l_rot.inverse() * l_intr.T() * l_vrect.T() * makeVector(0,-1.0,y);
		Vec3 plane_nrm = unit(makeVector(line_nrm[0], line_nrm[1], 0));
		Vec4 plane_eqn = concat(plane_nrm, -plane_nrm*surf_p);
		Mat3 vert_xfer = GetHomographyVia(l_image.pc(), r_image.pc(), plane_eqn);
		Mat3 l_vrect_to_r = vert_xfer * l_vrect_inv;  // Don't pre-multiply by r_vrect as we access the orig image

		// Compute NCCs for this row
		if (y%25==0) DLOG << "Computing correspondences for row " << y;
		for (int x = 0; x < l_image.nx(); x++) {

			// Calculate floor and ceiling points
			Vec3 p = makeVector(x,y,1.0);
			int y0, y1;
			if (y < l_vrect_horizon_y) {  // on ceiling, TODO: use cached opp_rows
				y0 = y;
				y1 = Clamp<int>(project(l_vrect_cToF * p)[1], 0, l_image.ny()-1);
			} else {
				y1 = y;
				y0 = Clamp<int>(project(l_vrect_fToC * p)[1], 0, l_image.ny()-1);
			}
			CHECK_LE(y0, y1);

			// Visualize
			bool special = viz_mask[y][x];
			PixelRGB<byte> color;
			if (special) {
				color = bc.Next();
				lsamples_canvas.DrawDot(project(l_vrect_inv * p), 3.0, color);
				rsamples_canvas.DrawDot(project(ltr_hfloor * l_vrect_inv * p), 3.0, color);
			}

			// Compute contributions from the horizontal component
			NCCStatistics stats;
			ncc_comp.AddStats(x, 0, y0, stats);  // portion above the ceiling point
			ncc_comp.AddStats(x, y1, l_image.ny()-1, stats);  // portion below the floor point

			// Compute contributions from the vertical component
			for (int yy = y0; yy < y1; yy++) {
				double l_val = l_vrect_vals[yy][x];
				if (l_val > 0) {  // l_val < 0 indicates there was no corresponding pixel in the original image
					toImageRef(l_vrect_to_r*makeVector(x,yy,1.0), r_sample);  // implicitly uses floor() for speed
					if (r_image.contains(r_sample)) {
						stats.Add(l_val, r_image.mono[r_sample].y);
					}

					// Visualize
					if (special && yy%4==0) {
						Vec3 pp = makeVector(x,yy,1.0);
						lsamples_canvas.DrawDot(project(l_vrect_inv * pp), 1.0, color);
						rsamples_canvas.DrawDot(project(l_vrect_to_r * pp), 1.0, color);
					}
				}
			}

			if (stats.nsamples == 0) {
				payoffs_row[x] = -1;
			} else {
				// NOTE: I think it makes sense to take the absolute NCC since
				// a large negative NCC suggests an anti-correlation, which
				// indicates a good match. But I'm not sure...
				payoffs_row[x] = abs(stats.CalculateNCC());
				CHECK_PRED1(isfinite, payoffs_row[x]) << stats;
				if (special) {
					DLOG << x << "," << y << ":";
					INDENTED DREPORT(payoffs_row[x], stats.nsamples);
				}
			}
		}
	}

	// Save the payoffs matrix
	WriteMatrix("payoffs.dat", payoffs);
	WriteMatrixImageRescaled("out/vrect_l.png", l_vrect_vals);
	WriteMatrixImageRescaled("out/xfered.png", l_vrect_xfered_vals);
	WriteMatrixImageRescaled("out/mask.png", mask);

	WriteImage("out/orig_l.png", l_image.mono);
	WriteImage("out/orig_r.png", r_image.mono);

	// Visualize vertical cross-correlations
	FileCanvas payoffs_canvas("out/payoffs.png", l_image.rgb);
	for (int y = 0; y < l_image.ny(); y += 3) {
		for (int x = 0; x < l_image.nx(); x += 3) {
			Vec2 vrect_p = project(l_vrect * makeVector(x,y,1.0));
			double payoff = payoffs[ roundi(vrect_p[1]) ][ roundi(vrect_p[0]) ];
			if (payoff >= 0) {
				PixelRGB<byte> color(0,payoff*255,0);  // payoffs are in [0,1]
				payoffs_canvas.DrawDot(makeVector(x,y), color);
			}
		}
	}
	WriteMatrixImageRescaled("out/payoffs_raw.png", payoffs);

	return 0;
}
