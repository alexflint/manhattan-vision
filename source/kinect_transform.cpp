#include "kinect_transform.hpp"
#include "common_types.h"

#include <fstream>
#include <Eigen/SVD>
#include <Eigen/Array> 
#include <cmath>

using namespace Eigen;
using namespace std;

inline float round(float a) {
  return std::floor(a + .5);
}

bool ROSCameraCalibration::load_calibration(std::string const & filename) {
  ifstream fin(filename.c_str());
  if(!fin) {
    cout << "ERROR: Could not open '" << filename << "' for parsing." << endl;
    return false;
  }
  YAML::Parser parser(fin);

  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc["image_width"] >> image_width;
    doc["image_height"] >> image_height;
    doc["camera_name"] >> camera_name;
    
    load_matrix(doc["camera_matrix"], camera_matrix);
    load_matrix(doc["rectification_matrix"], rectification_matrix);
    load_matrix(doc["distortion_coefficients"], distortion_coefficients);
    load_matrix(doc["projection_matrix"], projection_matrix);
  }
  fin.close();

  // inverse_projection_matrix = pseudo_inverse(projection_matrix);
  // implemented through SVD
  SVD<Matrix<float, 4, 3> > svd(projection_matrix.transpose());
  MatrixXf diag = svd.singularValues().asDiagonal();
  for(int i = 0; i < diag.rows(); i++) {
    if(diag(i, i) <= 1e-6) diag(i, i) = 0;
    else diag(i, i) = 1./diag(i, i);
  }
  inverse_projection_matrix = svd.matrixU()*diag*svd.matrixV().transpose();

  return true;
}

bool KinectTransform::load_calibration(std::string const & calibration_dir) {
  bool st1 = calibration_rgb.load_calibration(calibration_dir + "/calibration_rgb.yaml");
  bool st2 = calibration_depth.load_calibration(calibration_dir + "/calibration_depth.yaml");
  bool st3 = load_kinect_params(calibration_dir + "/kinect_params.yaml");
  if(st1 && st2 && st3) {  
    compute_rays();
    calibration_loaded = true;
    return true;
  }
  else {
    calibration_loaded = false;
    return false;
  }
  
}

bool KinectTransform::load_kinect_params(std::string const & filename) {
  ifstream fin(filename.c_str());
  if(!fin) {
    cout << "ERROR: Could not open '" << filename << "' for parsing." << endl;
    return false;
  }
  
  YAML::Parser parser(fin);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    doc["shift_offset"] >> shift_offset;
    doc["projector_depth_baseline"] >> projector_depth_baseline;
    
    YAML::Node const & rotation = doc["depth_rgb_rotation"];
    YAML::Node const & translation = doc["depth_rgb_translation"];

    // this is assuming the rotation matrix is row-major, but it doesn't matter for rotations if we're wrong.
    int i = 0;
    for(int r = 0; r < 3; r++) {
      translation[r] >> depth_to_rgb_translation(r);
      for(int c = 0; c < 3; c++) {
	rotation[i] >> depth_to_rgb_rotation(r, c);
	i++;
      }
    }

    // Compose into a convenient 4x4 transform
    depth_to_rgb_transform.setIdentity();  // this is important as it initializes the 4th row
    depth_to_rgb_transform.corner<3,3>(TopLeft) = depth_to_rgb_rotation;
    depth_to_rgb_transform.corner<3,1>(TopRight) = depth_to_rgb_translation;
  }
  
  //cout << "rot: " << endl << depth_rgb_rotation << endl;
  //cout << "trans: " << endl << depth_rgb_translation << endl;
  
  fin.close();

  return true;
}

double KinectTransform::get_depth(double v) {
  float focal_length_x = calibration_depth.camera_matrix(0, 0);
  float disparity = (shift_offset - v)/8.;
  return projector_depth_baseline*focal_length_x/disparity;
}

bool KinectTransform::uvd_to_xyz(Eigen::Vector3f const& in, Eigen::Vector3f & out) {
  float x_center = calibration_depth.camera_matrix(0, 2);
  float y_center = calibration_depth.camera_matrix(1, 2);
  float focal_length_x = calibration_depth.camera_matrix(0, 0);
  float focal_length_y = calibration_depth.camera_matrix(1, 1);
  
  // compute the depth from the disparity
  float disparity = (shift_offset - in.z())/8.;
  float depth = projector_depth_baseline*focal_length_x/disparity;

  if(depth <= 0.) {
    return false;
  }

  // cast the depth along the cached ray at the given u,v coordinates
  out.x() = depth*(in.x() - x_center)/focal_length_x;
  out.y() = depth*(in.y() - y_center)/focal_length_y;
  out.z() = depth;
	
  return true;
}

void KinectTransform::compute_rays() {
  
}
