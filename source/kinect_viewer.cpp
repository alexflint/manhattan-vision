/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


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

#include <determinant.h>
#include <LU.h>

#include "kinect_transform.hpp"
#include "colors.h"
#include "viewer3d.h"
#include "map_widgets.h"
#include "lazyvar.h"
#include "entrypoint_types.h"
#include "line_sweep_features.h"
#include "geom_utils.h"
#include "landmark_payoffs.h"

#include "vector_utils.tpp"
#include "image_utils.tpp"
#include "gl_utils.tpp"

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


int main(int argc, char **argv) {
	InitVars();
	AssertionManager::SetExceptionMode();

	// Initialize kinect
	KinectDriver kinect;
	KinectDevice& device = kinect.device();
	ImageRef im_size(device.image_width(), device.image_height());

	// Load the calibration data
	KinectTransform calib;
	calib.load_calibration(*gvKinectCalibrationDir);

	// Construct calibrated cameras
	LinearCamera rgb_camera(asToon(calib.rgb_calibration().camera_matrix), im_size);
	LinearCamera depth_camera(asToon(calib.depth_calibration().camera_matrix), im_size);
	Mat4 depth_to_rgb = asToon(calib.depth_to_rgb());
	toon::SE3<> ident;
	PosedImage kinect_image(ident, &rgb_camera);  // initialize with identity pose, update later
	MatF kinect_depth;

	// Capture depth and RGB
	device.start();
	device.waitForRGB(kinect_image.rgb);
	device.waitForDepth(kinect_depth);
	device.stop();

	// Homography transform
	// TODO: enable this if these are not Identity
	/*Mat3 rgb_rect = asToon(calib.rgb_calibration().rectification_matrix);
	Mat3 depth_rect = asToon(calib.depth_calibration().rectification_matrix);
	DREPORT(rgb_rect, depth_rect);*/

	// Set up 3D viewer
	vector<Vec3> points;
	vector<PixelRGB<byte> > colors;
	int kSampleRate = 2;

	// Re-project depth into RGB coords
	MatF depth_in_rgb_coords(im_size.y, im_size.x);
	depth_in_rgb_coords.Fill(-1);

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

			// Add to viewer
			if (x%kSampleRate==0 && y%kSampleRate==0) {
				points.push_back(rgb_world_pt);
				colors.push_back(kinect_image.rgb[rgb_y][rgb_x]);
			}
		}
	}


	// Find vanishing points
	VanishingPoints vpt_detector;
	try {
		vpt_detector.Compute(kinect_image);
	} catch (const std::exception& ex) {
		DLOG << "Vanishing point detector threw an exception. Continuing...";
	}
	vpt_detector.OutputVanPointViz("out/vpts.png");

	// Find the vanishing point with largest absolute Y coordinate in the image domain
	int up_axis = -1;
	double max_y = -1;
	for (int i = 0; i < 3; i++) {
		double y = abs(project(vpt_detector.image_vpts[i])[1]);
		if (y > max_y) {
			max_y = y;
			up_axis = i;
		}
	}
	CHECK_GE(up_axis, 0);

	Mat3 rot = vpt_detector.manhattan_est.R.get_matrix();
	DREPORT(rot);
	if (up_axis != 2) {
		Vec3 temp = rot.T()[2];
		rot.T()[2] = rot.T()[up_axis];
		rot.T()[up_axis] = temp;

		// Ensure that the Z axis is positive w.r.t. up direction in image
		if (rot[2][2] < 0) {
			rot.T()[2] *= -1;
		}

		// Ensure that we still have a pure rotation (not a roto-inversion)
		if (determinant(rot) < 0) {
			rot.T()[0] *= -1;
		}
	}
	DREPORT(rot);

	// Compute a pose from the vanishing points
	SO3<> pose_rot(rot);
	SE3<> pose(pose_rot, makeVector(0,0,0));

	//SE3<> pose = SE3<>::exp(makeVector(0, 0, 0, 0.154556, -2.15036, 2.15554));
	kinect_image.pc().SetPose(pose);

	// Transform points to Manhattan frame and find floor-ceiling mapping
	Vec3 vfloor, vceil;
	double zfloor = INFINITY, zceil = -INFINITY;
	SE3<> pose_inv = pose.inverse();
	CHECK_GT(points.size(), 0);
	BOOST_FOREACH(Vec3& v, points) {
		v = pose_inv * v;
		if (v[2] > zceil) {
			vceil = v;
			zceil = v[2];
		}
		if (v[2] < zfloor) {
			vfloor = v;
			zfloor = v[2];
		}
	}

	// Compute floor-ceiling homology
	Mat3 manhattan_h = GetManhattanHomology(kinect_image.pc(), zfloor, zceil);
	DREPORT(kinect_image.pc().pose(), zfloor, zceil, manhattan_h);
	CHECK(!isnan(manhattan_h[0]));

	// Compute photometric features
	//LineSweepDPScore objective_gen(kinect_image);
	//objective_gen.OutputOrientViz("out/sweeps.png");

	// Compute features from depth data
	DPGeometry geom(kinect_image.pc(), manhattan_h);
	MatF isct_payoffs, occl_payoffs;
	ComputeLandmarkPayoffs(points, kinect_image.pc(), geom,
												 zfloor, zceil,
												 isct_payoffs, occl_payoffs);
	DPPayoffs payoffs(geom.grid_size);
	payoffs.wall_scores[0] = isct_payoffs*100.0 + occl_payoffs;
	payoffs.wall_scores[1] = payoffs.wall_scores[0];
	WriteMatrixImageRescaled("out/payoffs.png", payoffs.wall_scores[0]);

	// Reconstruct
	ManhattanDPReconstructor recon;
	recon.Compute(kinect_image, geom, payoffs); // manhattan_h, objective_gen.objective);
	recon.OutputSolution("out/manhattan.png");
	recon.OutputGridViz("out/grid_manhattan.png");
	recon.OutputManhattanHomologyViz("out/homo.png");

	DREPORT(pose.ln());
	DLOG << "Done.";

	Viewer3D viewer;
	viewer.Add(bind(&DrawPoints, ref(points), ref(colors)));
	viewer.AddOwned(new GroundPlaneWidget());
	viewer.AddOwned(new PointWidget(vfloor, 5, Colors::red()));
	viewer.AddOwned(new PointWidget(vceil, 5, Colors::blue()));
	viewer.Run();

	return 0;
}
