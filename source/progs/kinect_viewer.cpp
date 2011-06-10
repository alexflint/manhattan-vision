#include "kinect_device.h"

#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <cmath>
#include <vector>

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <Eigen/Core>
#include <Eigen/Array> 

#include <TooN/so3.h>
#include <TooN/determinant.h>
#include <TooN/LU.h>

#include "kinect_transform.hpp"
#include "colors.h"
#include "viewer3d.h"
#include "map_widgets.h"
#include "lazyvar.h"
#include "entrypoint_types.h"
#include "line_sweep_features.h"
#include "geom_utils.h"
#include "camera.h"
#include "landmark_payoffs.h"
#include "canvas.h"
#include "timer.h"

#include "vector_utils.tpp"
#include "image_utils.tpp"
#include "gl_utils.tpp"
#include "io_utils.tpp"

using namespace toon;

lazyvar<string> gvKinectCalibrationDir("Kinect.CalibrationDir");

void DrawPoints(const vector<Vec3>& points,
								const vector<PixelRGB<byte> >& colors) {
	CHECK_EQ(points.size(), colors.size());

	glPointSize(2.0);
	GL_PRIMITIVE(GL_POINTS) {
		for (int i = 0; i < points.size(); i++) {
			glColorP(colors[i]);
			glVertexV(points[i]);
		}
	}
}

void HomographyTransform(const ImageRGB<byte>& input,
												 ImageRGB<byte>& output,
												 const Mat3& h) {  // h transforms from input to output coords
	ImageRef p;
	Mat3 hinv = LU<3>(h).get_inverse();
	for (int y = 0; y < output.GetHeight(); y++) {
		PixelRGB<byte>* outrow = output[y];
		for (int x = 0; x < output.GetWidth(); x++) {
			toImageRef(project(hinv * makeVector(x,y,1.0)), p);
			if (p.x >= 0 && p.x < output.GetWidth() &&
					p.y >= 0 && p.y < output.GetHeight()) {
				outrow[x] = input[p];
			}
		}
	}
}

// Eigen matrix -> Toon Matrix
template <int M, int N, typename T>
toon::Matrix<M,N,T> asToon(const Eigen::Matrix<T,M,N>& in) {
	toon::Matrix<M,N,T> out;
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			out[i][j] = in(i,j);
		}
	}
	return out;
}

// Eigen column vector -> Toon Vector
template <int N, typename T>
toon::Vector<N,T> asToon(const Eigen::Matrix<T,N,1>& in) {
	toon::Vector<N,T> out;
	for (int i = 0; i < N; i++) {
		out[i] = in(i);
	}
	return out;
}

// Eigen row vector -> Toon Vector
template <int N, typename T>
toon::Vector<N,T> asToon(const Eigen::Matrix<T,1,N>& in) {
	toon::Vector<N,T> out;
	for (int i = 0; i < N; i++) {
		out[i] = in(i);
	}
	return out;
}

Vec3 atdepth(const Vec3& ret_pt, double depth) {
	return depth * atretina(ret_pt);
}

Vec3 atdepth(const Vec2& im_pt, const CameraBase& cam, double depth) {
	return atdepth(cam.ImToRet(unproject(im_pt)), depth);
}

int main(int argc, char **argv) {
	InitVars();
	AssertionManager::SetExceptionMode();

	// Load the calibration data
	KinectTransform calib;
	calib.load_calibration(*gvKinectCalibrationDir);

	ImageRGB<byte> rgb_input;
	MatF kinect_depth;

	// This must be outside as the destructor seems to hang
	KinectDriver kinect;

	// Acquire images
	if (argc > 3) {
		cout << "Usage: kinect_viewer\n";
		cout << "       kinect_viewer RGB_FILE DEPTH_FILE\n";
		cout << "       kinect_viewer FRAME_INDEX\n";
		exit(-1);

	} else if (argc == 1 || argc == 2) {

		string rgb_file, depth_file;
		if (argc == 2) {
			int n = atoi(argv[1]);
			rgb_file = "kinectdata/lab_corridor/frame-" + itoa(n,3) + ".png";
			depth_file = "kinectdata/lab_corridor/frame-" + itoa(n,3) + "-depth.dat";
		} else {
			CHECK_EQ(argc, 3);
			rgb_file = argv[1];
			depth_file = argv[2];
		}

		ReadImage(rgb_file, rgb_input);

		int npix = rgb_input.GetWidth()*rgb_input.GetHeight();
		vector<uint16_t> depth_input(npix);
		FILE* fp = fopen(depth_file.c_str(), "r");
		if (!fp) {
			perror("Failed to open depth file for reading");
			exit(-1);
		}
		size_t n = fread(depth_input.data(), sizeof(uint16_t), npix, fp);

		kinect_depth.Resize(rgb_input.GetHeight(), rgb_input.GetWidth());
		const uint16_t* pdepth = depth_input.data();
		for (int y = 0; y < kinect_depth.Rows(); y++) {
			float* outrow = kinect_depth[y];
			for (int x = 0; x < kinect_depth.Cols(); x++) {
				outrow[x] = *pdepth++;
			}
		}

	} else {
		// Capture depth and RGB
		KinectDevice& device = kinect.device();
		device.start();
		device.waitForRGB(rgb_input);
		device.waitForDepth(kinect_depth);
		device.stop();
	}

	ImageRef input_size = rgb_input.GetSize();
	CHECK_EQ(kinect_depth.Rows(), input_size.y);
	CHECK_EQ(kinect_depth.Cols(), input_size.x);

	// Construct calibrated cameras
	LinearCamera rgb_camera(asToon(calib.rgb_calibration().camera_matrix), input_size);
	LinearCamera depth_camera(asToon(calib.depth_calibration().camera_matrix), input_size);
	Mat4 depth_to_rgb = asToon(calib.depth_to_rgb());
	toon::SE3<> ident;

	// Set up the calibrated image
	PosedImage kinect_image(ident, &rgb_camera);  // initialize with identity pose, update later
	const PosedCamera& pc = kinect_image.pc();
	ImageCopy(rgb_input, kinect_image.rgb);

	// Homography transform
	// TODO: enable this if these are not Identity
	/*Mat3 rgb_rect = asToon(calib.rgb_calibration().rectification_matrix);
	Mat3 depth_rect = asToon(calib.depth_calibration().rectification_matrix);
	DREPORT(rgb_rect, depth_rect);*/

	vector<Vec3> points;
	vector<PixelRGB<byte> > colors;
	int kSampleRate = 2;

	// Re-project depth into RGB coords
	MatF depth_in_rgb_coords(input_size.y, input_size.x);
	depth_in_rgb_coords.Fill(-1);

	bool has_first = false;
	Vec2 first_pt_proj;

	Vec3 manual_zfloor_pt;

	// Construct depth for each point in the image
	for (int i = 0, n = kinect_image.nx()*kinect_image.ny(); i < n; i++) {
		int x = i%kinect_image.nx();
		int y = i/kinect_image.nx();
		double depthv = calib.get_depth(kinect_depth[y][x]);
		if (depthv <= 0.) continue;

		// Back-project into 3D space
		Vec3 depth_im_pt = makeVector(x,y,1.);
		Vec3 depth_world_pt = depthv * atretina(depth_camera.ImToRet(depth_im_pt));
		// Project into RGB coords
		Vec3 rgb_world_pt = project(depth_to_rgb * unproject(depth_world_pt));
		Vec2 rgb_im_pt = project(rgb_camera.RetToIm(rgb_world_pt));
		int rgb_x = floori(rgb_im_pt[0]);
		int rgb_y = floori(rgb_im_pt[1]);
		if (rgb_x >= 0 && rgb_x < depth_in_rgb_coords.Cols() &&
				rgb_y >= 0 && rgb_y < depth_in_rgb_coords.Rows()) {
			depth_in_rgb_coords[rgb_y][rgb_x] = depthv;

			if (x > 95 && x < 105 && y > 400 && y < 410) {
				manual_zfloor_pt = rgb_world_pt;
			}

			// Add to viewer
			if (x%kSampleRate==0 && y%kSampleRate==0) {
				points.push_back(rgb_world_pt);
				colors.push_back(kinect_image.rgb[rgb_y][rgb_x]);				

				if (!has_first) {
					first_pt_proj = rgb_im_pt;
					has_first = true;
				}
			}
		}
	}

	// Detect lines (for visualization)
	CannyLineDetector line_detector;
	line_detector.Compute(kinect_image);

	// Sample surface normals
	const int kNormalWindow = 6;
	DLOG << "Sampling surface normals...";
	vector<Vec3> nrm_samples;
	vector<Vec2I> pixel_samples;
	vector<Vec3> pt_samples;
	while (nrm_samples.size() < 250) {
			// don't sample from row 0 or column 0
		int x = kNormalWindow + rand()%(depth_in_rgb_coords.Cols()-kNormalWindow*2);
		int y = kNormalWindow + rand()%(depth_in_rgb_coords.Rows()-kNormalWindow*2);

		double dc = depth_in_rgb_coords[y][x];
		if (dc == -1) continue;  // No depth here?
		Vec3 center_pt = atdepth(makeVector(x, y), pc.camera(), dc);

		// Collect points from the window
		vector<Vec3> points;
		for (int dy = -kNormalWindow; dy <= kNormalWindow; dy++) {
			for (int dx = -kNormalWindow; dx <= kNormalWindow; dx++) {
				double depth = depth_in_rgb_coords[y+dy][x+dx];
				if (depth != -1) {
					points.push_back(atdepth(makeVector(x+dx,y+dy), pc.camera(), depth));
				}
			}
		}

		// Too few points with valid depth?
		if (points.size() < 6) continue;

		// Form matrix
		Matrix<> m(points.size(), 4);
		for (int i = 0; i < points.size(); i++) {
			m.slice(i,0,1,4) = unproject(points[i]).as_row();
		}

		// Fit plane
		Vec4 b = Zeros;
		SVD<> svd(points.size(), 4);
		svd.compute(m);
		Vector<> v = svd.get_VT()[3];
		Vec4 plane = v;
		Vec3 n = unit(plane.slice<0,3>());

		pixel_samples.push_back(makeVector(x,y));
		nrm_samples.push_back(n);
		pt_samples.push_back(center_pt);
	}

	// RANSAC over rotations
	const double kInlierThreshold = 0.999;
	DLOG << "Sampling rotations...";
	int best_inliers = 0;
	Mat3 best_R;
	INDENTED
	for (int iter = 0; iter < 100; iter++) {
		// Get two normal directions
		Vec3 n1 = nrm_samples[rand() % nrm_samples.size()];
		Vec3 n2;
		int trys = 0;
		do {
			n2 = nrm_samples[rand() % nrm_samples.size()];
		} while (trys++ < 20 && abs(n1*n2) > 0.1);
		
		if (trys == 20) continue;

		// Construct the rotation to be applied to world points
		Mat3 R;
		R[0] = n1;
		R[1] = n2^n1;
		R[2] = n1^(n2^n1);

		// Count inliers
		int num_inliers = 0;
		BOOST_FOREACH(const Vec3& auxnrm, nrm_samples) {
			for (int i = 0; i < 3; i++) {
				if (abs(auxnrm * R[i]) > kInlierThreshold) {
					num_inliers++;
					break;
				}
			}
		}

		// If better then save
		if (num_inliers > best_inliers) {
			best_inliers = num_inliers;
			best_R = R;
		}
	}
	CHECK_GT(best_inliers, 0) << "No inliers found for any hypothesis!";

	// Re-estimate
	/*vector<Vec3> nrms;
	vector<int> axes;
	BOOST_FOREACH(const Vec3& auxnrm, nrm_samples) {
		for (int i = 0; i < 3; i++) {
			if (abs(auxnrm * best_R[i]) > 0.999) {
				nrms.push_back(auxnrm);
				axes.push_back(i);
				break;
			}
		}
	}
	Matrix<> one_minus_assocs(nrms.size(), 3);
	one_minus_assocs = Ones;
	for (int i = 0; i < nrms.size(); i++) {
		one_minus_assocs[i][axes[i]] = 0.0;
	}
	RotationEstimator estimator;
	estimator.max_steps = 200;
	SO3<> start(best_R.T());
	estimator.Compute(nrms, one_minus_assocs, start);
	DREPORT(start, estimator.R);
	best_R = estimator.R.get_matrix().T();
	*/

	// Compute vanishing points in image
	Vec3 image_vpts[3];
	for (int i = 0; i < 3; i++) {
		Vec3 ret_vpt = pc.pose().get_rotation() * best_R[i];
		image_vpts[i] = pc.RetToIm(ret_vpt);
	}

	// Find the vanishing point with largest absolute Y coordinate in the image domain
	int up_axis = 0;
	for (int i = 1; i < 3; i++) {
		// Cross multiply to avoid infinities
		// The use of abs() here is valid, I've checked it
		if (abs(image_vpts[i][1]*image_vpts[up_axis][2])
				> abs(image_vpts[up_axis][1]*image_vpts[i][2])) {
			up_axis = i;
		}
	}

	// Ensure that the "up" axis is the 3rd axis
	if (up_axis != 2) {
		Vec3 temp = best_R[2];
		best_R[2] = best_R[up_axis];
		best_R[up_axis] = temp;
	}

	// Ensure that the "up" in the image is "up" in the world
	// best_R*[0 0 1] is the projection of the z axis into the image
	// if it is negative then the rotation flips the z axis -- bad!
	if (best_R[2][1]*best_R[2][2] < 0) {
		best_R[2] *= -1;
	}

	// Ensure that we still have a pure rotation (not a roto-inversion)
	if (determinant(best_R) < 0) {
		best_R[0] *= -1;
	}
	CHECK_EQ_TOL(determinant(best_R), 1.0, 1e-3);

	// Apply the rotation to the points
	SO3<> R(best_R);
	BOOST_FOREACH(Vec3& v, points) {
		v = R*v;
	}

	// Compute a pose that includes the inverse of the rotation applied to the points
	SE3<> pose(R.inverse(), makeVector(0,0,0));
	kinect_image.pc().SetPose(pose);

	// Check that points still project into the image as before
	Vec2 new_proj = project(pc.WorldToIm(points[0]));
	CHECK_EQ_TOL(new_proj, first_pt_proj, 1e-8);

	// Find the floor and ceiling plane
	// Note that here we assume that "up" in the world corresponds to the negative Z axis
	Vec3 vfloor, vceil;
	double zfloor = -INFINITY, zceil = INFINITY;
	CHECK_GT(points.size(), 0);
	BOOST_FOREACH(const Vec3& v, points) {
		if (v[2] < zceil) {
			vceil = v;
			zceil = v[2];
		}
		if (v[2] > zfloor) {
			vfloor = v;
			zfloor = v[2];
		}
	}

	/*double manual_zfloor = (R*manual_zfloor_pt)[2];
	DREPORT(zfloor, manual_zfloor);
	zfloor = manual_zfloor;*/

	// Compute floor-ceiling homology
	DLOG << "Computing manhattan homology...";
	Mat3 manhattan_h = GetManhattanHomology(pc, zfloor, zceil);
	CHECK(isfinite(manhattan_h[0]));
	DPGeometry geom(pc, manhattan_h);

	// Compute features from depth data
	LandmarkPayoffs payoff_gen;
	TIMED("Computing payoffs") 
		payoff_gen.Compute(points, pc, geom, zfloor, zceil);
	DPPayoffs payoffs(geom.grid_size);
	payoffs.wall_scores[0] = payoff_gen.agreement_payoffs + payoff_gen.occlusion_payoffs*0.02;
	payoffs.wall_scores[1] = payoffs.wall_scores[0];

	// Reconstruct
	ManhattanDPReconstructor recon;
	recon.Compute(kinect_image, geom, payoffs); // manhattan_h, objective_gen.objective);
	recon.OutputSolution("out/manhattan.png");
	recon.OutputGridViz("out/grid_manhattan.png");
	recon.OutputManhattanHomologyViz("out/homo.png");
	recon.OutputPayoffsViz("out/payoffs.png");

	payoff_gen.OutputProjectionViz(kinect_image, "out/projections.png");

	/*{
		LandmarkPayoffs slow_payoff_gen;
		TITLED("Slow payoffs") TIMED("Computing payoffs (slow)") 
			slow_payoff_gen.ComputeSlow(points, pc, geom, zfloor, zceil);
		DPPayoffs slow_payoffs(geom.grid_size);
		slow_payoffs.wall_scores[0] = slow_payoff_gen.agreement_payoffs*100.0 + slow_payoff_gen.occlusion_payoffs;
		slow_payoffs.wall_scores[1] = slow_payoffs.wall_scores[0];

		ManhattanDPReconstructor check_recon;
		check_recon.Compute(kinect_image, geom, slow_payoffs); // manhattan_h, objective_gen.objective);
		check_recon.OutputPayoffsViz("out/slow_payoffs.png");
		}*/

	// Visualize vanishing points
	{
		FileCanvas vpt_canvas("out/vpts.png", kinect_image.rgb);
		for (int i = 0; i < 3; i++) {
			Vec3 vpt = image_vpts[i];
			if (abs(vpt[2]) < 1e-8) vpt[2] = 1e-8;
			Vec2 v = project(vpt);
			BOOST_FOREACH(const LineDetection& det, line_detector.detections) {
				Vec2 mid = 0.5 * (project(det.seg.start) + project(det.seg.end));
				vpt_canvas.StrokeLine(mid, v, Colors::primary(i));
			}
		}
	}

	DREPORT(recon.dp.solution.score, pose.ln());
	DLOG << "Done.";

	Viewer3D vw;
	vw.Add(bind(&DrawPoints, ref(points), ref(colors)));
	vw.AddOwned(new GroundPlaneWidget());
	vw.AddOwned(new PointWidget(vfloor, 5, Colors::red()));
	vw.AddOwned(new PointWidget(vceil, 5, Colors::blue()));

	for (int i = 0; i < pt_samples.size(); i++) {
		bool inlier = false;
		for (int j = 0; j < 3; j++) {
			if (abs(nrm_samples[i] * best_R[j]) > kInlierThreshold) {
				inlier = true;
				break;
			}
		}

		PixelRGB<byte> color = inlier ? Colors::green() : Colors::red();
		vw.AddOwned(new PointWidget(R*pt_samples[i], 2.0, color));
		vw.AddOwned(new LineWidget(R*pt_samples[i],
															 R*(pt_samples[i]+nrm_samples[i]*0.1),
															 1.0,
															 color));
	}
	vw.Run();

	return 0;
}
