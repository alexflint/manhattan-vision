#include <iostream>
#include <iomanip>
#include <queue>

#include <VW/Image/imagecopy.tpp>
#include <VW/Utils/timer.h>
#include <VW/Graphics/drawimage.h>
#include <VW/GeomObjects/lineseg2d.h>

#include <so3.h>
#include <determinant.h>
#include <SVD.h>

#include "common_types.h"
#include "image_bundle.h"
#include "kmeans.h"
#include "canny.h"
#include "vanishing_points.h"
#include "rotation_estimator.h"
#include "camera.h"

#include "eigensystem2d.tpp"
#include "range_utils.tpp"
#include "numeric_utils.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "counted_foreach.tpp"

namespace indoor_context {
using namespace toon;

// These are documented in vanishing_points.cfg
const lazyvar<string> gvDefaultStrategy("VanPts.DefaultStrategy");
const lazyvar<int> gvNumVanPts("VanPts.NumVanishingPts");
const lazyvar<int> gvNumBootstrapClusters("VanPts.NumBootstrapClusters");
const lazyvar<int> gvMinSupport("VanPts.MinBootstrapSupport");

const lazyvar<double> gvErrorModelSigma("EMVanPts.ErrorModelSigma");
const lazyvar<double> gvMergeThresh("EMVanPts.MergeThreshold");
const lazyvar<double> gvExitThresh("EMVanPts.ExitThreshold");
const lazyvar<double> gvSpuriousLogLik("EMVanPts.SpuriousLogLik");
const lazyvar<float> gvAdoptThresh("EMVanPts.AdoptThreshold");

const lazyvar<int> gvNumIters("RansacVanPts.NumIterations");
const lazyvar<float> gvVoteThresh("RansacVanPts.VoteThreshold");

const lazyvar<int> gvVizPadding("VanPtsViz.ImagePadding");
const lazyvar<float> gvVanLineAlpha("VanPtsViz.VanLineAlpha");

// Color for line segments not belonging to a vanishing point
static const PixelRGB<byte> kSpuriousColor(255, 255, 255);
static const PixelRGB<byte> kElimColor(0, 0, 0);



// Compute sum over a vector, not very efficient
template <int N, typename T>
T sum(const Vector<N,T>& v) {
	T x = 0;
	for (int i = 0; i < N; i++) {
		x += v[i];
	}
	return x;
}

// Compute sum over a vector, not very efficient
template <typename T>
T sum(const Vector<-1,T>& v) {
	T x = 0;
	for (int i = 0; i < v.size(); i++) {
		x += v[i];
	}
	return x;
}

void ManhattanFrameEstimator::Compute(vector<LineDetection>& segments,
                                      bool use_prev) {
	detections = &segments;
	DLOG << "Computing vanishing points";
	SCOPED_INDENT;

	// Get ready
	Prepare(segments);
	if (use_prev) {
		DLOG << "Using previous vanishing points";
	} else {
		Bootstrap(segments);
	}

	// Run EM
	for (num_iters = 0; !converged && num_iters < 100; num_iters++) {
		EStep();
		MStep();
	}
	Finish();

	// Report the final results
	if (converged) {
		DLOG << "EM converged after " << num_iters << " iterations";
	} else {
		DLOG << "EM failed to converge after " << num_iters << " iterations";
	}
	DLOG << "Final vanishing points:";
	INDENTED {
		for (int i = 0; i < 3; i++) {
			DLOG << project(vpts[i]) << " (Support: " << support[i] << ")";
		}
		DLOG << "plus " << num_spurious << " unassigned line segments";
	}
}

void ManhattanFrameEstimator::Prepare(const vector<LineDetection>& segments) {
	if (resps.Rows() != segments.size() || resps.Cols() != 4) {
		resps.Resize(segments.size(), 4, 0);
	}

	line_eqns.clear();
	line_eqns.reserve(segments.size());
	BOOST_FOREACH(const LineDetection& det, segments) {
		line_eqns.push_back(det.eqn);
	}
}

void ManhattanFrameEstimator::Bootstrap(const vector<LineDetection>& segments) {
	CHECK_GE(segments.size(), *gvNumBootstrapClusters)
		<< "Too few segments were provided to bootstrap the vanishing point detector.";

	// Run K-means on the line segment orientations
	kmeans_owners.Resize(segments.size());
	vector<VecD> thetas, clusters;
	BOOST_FOREACH(const LineDetection& det, segments) {
		Vec2 tangent = unit(project(det.seg.end) - project(det.seg.start));
		double theta = atan2(tangent[1], tangent[0]);
		thetas.push_back(asVNL(makeVector(theta)));
	}
	KMeans::Estimate(thetas, *gvNumBootstrapClusters, clusters, kmeans_owners);

	// Estimate vanishing points for each cluster
	vector<Vec3> candidate_vpts;
	for (int i = 0; i < clusters.size(); i++) {
		int support = 0;
		Vector<> weights(segments.size());
		weights = Zeros;
		for (int j = 0; j < segments.size(); j++) {
			if (kmeans_owners[j] == i) {
				support++;
				weights[j] = 1.0;
			}
		}
		if (support >= *gvMinSupport) {
			candidate_vpts.push_back(FitIsctRansac(segments, weights));
		}
	}

	// Check that we have enough candidates to proceed
	CHECK_GE(candidate_vpts.size(), 2)
		<< "After clustering vanishing points and eliminating clusters with less than "
		<< *gvMinSupport << " supporting lines, too few clusters were left for bootstrap.";

	// Find the pair of vpts with minimal dot product
	double mindp = INFINITY;
	Vec3 u = Zeros, v = Zeros;
	const int num_cands = candidate_vpts.size();
	for (int i = 0; i < num_cands; i++) {
		for (int j = i+1; j < num_cands; j++) {
			const double dp = abs(candidate_vpts[i] * candidate_vpts[j]);
			if (dp < mindp) {
				mindp = dp;
				u = candidate_vpts[i];
				v = candidate_vpts[j];
			}
		}
	}

	// Check that the vanishing points were linearly independent
	CHECK_GT(mindp, 1e-16 * max(norm(u), norm(v)))
					 << "Error: Unable to bootstrap the vanishing point detector:"
					 " There is no pair of linearly independent candidates";

	// Create the initial rotation matrix
	Mat3 R_init;
	R_init.T()[0] = u;
	R_init.T()[1] = (u^v);
	R_init.T()[2] = u^(u^v);  // not simplifiable since u*v != 0 in general

	// Initialize the rotation estimator
	R = SO3<>(R_init);
	converged = false;
	num_iters = 0;
}

void ManhattanFrameEstimator::EStep() {
	CHECK_EQ(resps.Rows(), detections->size());
	CHECK_EQ(resps.Cols(), 4);
	// Update responsibilities given current vanishing points
	for (int i = 0; i < detections->size(); i++) {
		double log_resps[] = {0,0,0,*gvSpuriousLogLik};
		for (int j = 0; j < 3; j++) {
			log_resps[j] = GetLogLik(col(R,j), (*detections)[i].eqn);
		}

		// Last component represents "not an axis-aligned edge"
		const double denom = LogSumExp(log_resps, 4);
		for (int j = 0; j < 4; j++) {
			resps[i][j] = exp(log_resps[j] - denom);
		}
	}
}

void ManhattanFrameEstimator::MStep() {
	// Estimate vanishing points given current responsibilities
	rot_est.Compute(line_eqns, asToon(resps), R);

	// Check for convergence, which is when
	//  (1) no vanishing point is changed by more than gvExitThresh, and
	//  (2) either of
	//      (a) the rotation estimator converged
	//      (b) there has been more than 10 iterations
	converged = true;//(rot_est.converged || num_iters >= 10);
	for (int i = 0; i < 3; i++) {
		const double change = 1.0 - abs(vpts[i] * col(rot_est.R,i));
		if (change > *gvExitThresh) {
			converged = false;
		}
	}

	// Copy rotation
	R = rot_est.R;
	for (int i = 0; i < 3; i++) {
		vpts[i] = col(R,i);
	}
}

void ManhattanFrameEstimator::Finish() {
	for (int i = 0; i < 3; i++) {
		support[i] = 0;
	}
	num_spurious = 0;

	COUNTED_FOREACH(int i, LineDetection& det, *detections) {
		// Assign each segment to its best matching vanishing point
		double max_resp = 0;
		for (int j = 0; j < 4; j++) {
			if (resps[i][j] >= max_resp) {
				max_resp = resps[i][j];
				det.axis = j;
			}
		}

		// Update support counts
		if (det.axis == 3) {
			det.axis = -1;
			num_spurious++;
		} else {
			support[det.axis]++;
		}
	}
}


// static
double ManhattanFrameEstimator::GetLogLik(const Vec3& vpt, const Vec3& line) {
	// TODO: make this more reasonable. At least add some normalization etc...
	// Ideally do projected error at line end-points
	const double& sigma = *gvErrorModelSigma;
	// TODO: use PointLineDist here
	const double dp = vpt * line;
	return -dp*dp / (2.0 * sigma * sigma);
}

Vec3 ManhattanFrameEstimator::FitIsct(const vector<LineDetection>& segments,
																			const Vector<>& weights) const {
	// Set up the linear system
	Matrix<> m(segments.size(), 3);
	for (int i = 0; i < segments.size(); i++) {
		const float wt = (weights.size() == segments.size()) ? weights[i] : 1.0;
		for (int j = 0; j < 3; j++) {
			m[i][j] = segments[i].eqn[j] * wt;
		}
	}

	// Solve it
	SVD<> svd(m);
	return svd.get_VT()[2];
}

Vec3 ManhattanFrameEstimator::FitIsctRansac(const vector<LineDetection>& segments,
																						const Vector<>& weights) const {
	// Generate cumulative weight vector
	const double wsum = sum(weights);
	Vector<> wcum(weights.size());
	wcum[0] = weights[0];
	for (int i = 1; i < weights.size(); i++) {
		wcum[i] = wcum[i-1] + weights[i];
	}

	// Iterate ransac
	const int num_iters = min(100, max(wsum, wsum*wsum / 5));
	Vec3 max_isct;
	float max_score = -1;
	for (int i = 0; i < num_iters; i++) {
		int a, b;
		do {
			const double ra = rand() * wsum / RAND_MAX;
			a = lower_bound(begin(wcum), end(wcum), ra) - begin(wcum);
			const double rb = rand() * wsum / RAND_MAX;
			b = lower_bound(begin(wcum), end(wcum), rb) - begin(wcum);
		} while (a == b);
		Vec3 isct = unit(segments[a].eqn ^ segments[b].eqn);
		float score = 0.0;
		for (int j = 0; j < segments.size(); j++) {
			// TODO: use the propper error here as per PointLineDist
			const double err = abs(isct * segments[j].eqn);
			if (err < *gvVoteThresh) {
				score += weights[j];
			}
		}
		if (score > max_score) {
			max_score = score;
			max_isct = isct;
		}
	}
	return max_isct;
}









void VanishingPoints::Compute(const CalibratedImage& image, bool use_prev) {
	input = &image;
	line_detector.Compute(image);
	DREPORT(line_detector.detections.size());
	if (line_detector.detections.size() < 3) {
		DLOG << "Error: too few lines detected to proceed with vanishing point estimation";
	} else {
		// Transfer to calibrated domain
		// TODO: implement this when we don't have a linear camera

		// If M is a 3x3 matrix mapping points from one space to another,
		// then M^-T is the corresponding mapping for lines. Since points
		// are mapped through the _inverse_ of the camera intrinsics, we
		// should map lines through the transpose of the (non-inverted)
		// camera intrinsics.
		Mat3 line_calib = ((LinearCamera&)image.camera()).intrinsics().T();
		BOOST_FOREACH(LineDetection& det, line_detector.detections) {
			det.eqn = line_calib * det.eqn;
		}

		// Do EM in the calibrated domain
		manhattan_est.Compute(line_detector.detections, use_prev);

		// Convert vanishing points back to the image domain
		for (int i = 0; i < 3; i++) {
			image_vpts[i] = image.camera().RetToIm(manhattan_est.vpts[i]);
		}
	}
}

void VanishingPoints::OutputLineViz(const string& filename) const {
	ImageRGB<byte> viz(input->sz());
	ImageCopy(input->rgb, viz);
	ResetAlpha(viz);
	line_detector.Draw(viz);
	WriteImage(filename, viz);
};

void VanishingPoints::OutputVanPointViz(const string& filename) const {
	ImageRGB<byte> canvas;
	DrawVanPointViz(canvas);
	WriteImage(filename, canvas);
}

void VanishingPoints::OutputHighlightViz(const string& filename,
                                         const VecI& inds) {
	ImageRGB<byte> canvas;
	DrawHighlightViz(canvas, inds);
	WriteImage(filename, canvas);
}

void VanishingPoints::DrawVanPointViz(ImageRGB<byte>& canvas) const {
	vector<PixelRGB<byte> > vpt_colors(3);
	for (int i = 0; i < 3; i++) {
		vpt_colors[i] = Colors::primary(i);
	}
	DrawVanPointViz(canvas, vpt_colors);
}

void VanishingPoints::DrawHighlightViz(ImageRGB<byte>& canvas,
                                       const VecI& inds) {
	vector<PixelRGB<byte> > vpt_colors;
	for (int i = 0; i < 3; i++) {
		const int* it = find(inds.begin(), inds.end(), i);
		vpt_colors.push_back(it == inds.end() ? kElimColor : BrightColors::Get(i));
	}
	DrawVanPointViz(canvas, vpt_colors);
}


void VanishingPoints::DrawVanPointViz(ImageRGB<byte>& canvas,
                                      vector<PixelRGB<byte> > vpt_colors) const {
	// Copy the original image into the vizualization
	if (!canvas.IsAlloced()) {
		const int pad = *gvVizPadding;
		canvas.AllocImageData(input->nx()+pad*2,
				input->ny()+pad*2);
	}
	Vec2 offs = makeVector((canvas.GetWidth() - input->nx()) / 2,
			(canvas.GetHeight() - input->ny()) / 2);
	canvas.Clear(PixelRGB<byte>(255, 255, 255));
	CopyImageInto(input->rgb, offs[1], offs[0], canvas);

	// Draw the extensions
	vector<PixelRGB<byte> > line_colors(line_detector.detections.size());;
	COUNTED_FOREACH (int i, const LineDetection& det, line_detector.detections) {
		if (det.axis == -1) {
			line_colors[i] = kSpuriousColor;
		} else {
			line_colors[i] = vpt_colors[det.axis];
			// Set the set the third component of any vanishing point at
			// infinity to a small number; i.e. approximate a point infinitely
			// far away by a point very far away. This produces near-identical
			// vizualization. TODO: deal with vanishing points at infinity
			// properly
			Vec3 vpt = image_vpts[det.axis];
			// Crude hack for line drawing:
			if (abs(vpt[2]) < 1e-8) {
				vpt[2] = 1e-8;
			}
			Vec3 hstart = unit(det.seg.start);
			Vec3 hend = unit(det.seg.end);
			const Vec3& endpt = hstart*vpt > hend*vpt ? hstart : hend;
			const Vec2 a = project(endpt) + offs;
			const Vec2 b = project(vpt) + offs;
			DrawLineClipped(canvas, a, b, line_colors[i], *gvVanLineAlpha);
		}
	}

	// Draw the line segments over the extension lines
	line_detector.Draw(canvas, line_colors, offs);

	// Draw the vanishing points
	PixelRGB<byte> white(255, 255, 255);
	for (int i = 0; i < 3; i++) {
		const Vec3& vpt = image_vpts[i];
		TITLED(i) DREPORT(vpt);
		if (abs(vpt[2]) > 1e-8) {
			const Vec2 drawpos = project(vpt) + offs;
			DrawSpot(canvas, drawpos, white, 4);
			DrawSpot(canvas, drawpos, vpt_colors[i], 3);
		}
	}
}
}
