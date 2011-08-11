#ifndef KINECT_TRANSFORM_HPP
#define KINECT_TRANSFORM_HPP

#include <stdint.h>
#include <string>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>


template<typename Derived>
void load_matrix(YAML::Node const & node, Eigen::MatrixBase<Derived> & matrix) {
  int rows;
  int cols;
  node["rows"] >> rows;
  node["cols"] >> cols;
    
  // the following matrix loading assumes row-major storage in the yaml file
  matrix.derived().resize(rows, cols);
  YAML::Iterator iter = node["data"].begin();
  for(int r = 0; r < rows; r++) {
    for(int c = 0; c < cols; c++) {
      *iter >> matrix(r, c);
      iter++;
    }
  }
}


struct ROSCameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  bool load_calibration(std::string const & filename);
  
  int image_width;
  int image_height;
  std::string camera_name;
  Eigen::Matrix<float, 3, 3> camera_matrix;
  Eigen::Matrix<float, 1, 5> distortion_coefficients;
  Eigen::Matrix<float, 3, 3> rectification_matrix;
  Eigen::Matrix<float, 3, 4> projection_matrix;
  Eigen::Matrix<float, 4, 3> inverse_projection_matrix;
};

class KinectTransform {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  KinectTransform() : calibration_loaded(false) { }
  
  bool load_calibration(std::string const & calibration_dir);
  
  bool load_kinect_params(std::string const & filename);

  void compute_rays();

  // maps a point/pixel (x, y, kinect_disparity) in the disparity image
  // to a 3D point (X, Y, Z) in the camera frame of the Kinect IR camera.
  bool uvd_to_xyz(Eigen::Vector3f const& in, Eigen::Vector3f & out);

	// Computes depth from the values provided in the Kinect depth buffer
	double get_depth(double v);

	const ROSCameraCalibration& rgb_calibration() const { return calibration_rgb; }
	const ROSCameraCalibration& depth_calibration() const { return calibration_depth; }

	// The transform in calibrated camera coordinates from the depth camera to RGB camera
	const Eigen::Matrix4f& depth_to_rgb() const { return depth_to_rgb_transform; }
private:
  // generic camera calibration parameters for IR and RGB cameras
  ROSCameraCalibration calibration_rgb;
  ROSCameraCalibration calibration_depth;

  // Kinect specific parameters
  float shift_offset;
  float projector_depth_baseline;
  Eigen::Matrix<float, 3, 3> depth_to_rgb_rotation;
  Eigen::Matrix<float, 3, 1> depth_to_rgb_translation;
  Eigen::Matrix4f depth_to_rgb_transform;
  Eigen::Matrix<float, 3, Eigen::Dynamic> rays;

  bool calibration_loaded;
};



#endif
