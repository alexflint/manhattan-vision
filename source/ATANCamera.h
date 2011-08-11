// [alex] Forked from release-ready PTAM v2 April 2nd 2010

// N-th implementation of a camera model
// GK 2007
// Evolved a half dozen times from the CVD-like model I was given by
// TWD in 2000
// 
// This one uses the ``FOV'' distortion model of
// Deverneay and Faugeras, Straight lines have to be straight, 2001
//
// BEWARE: This camera model caches intermediate results in member variables
// Some functions therefore depend on being called in order: i.e.
// GetProjectionDerivs() uses data stored from the last Project() or UnProject()
// THIS MEANS YOU MUST BE CAREFUL WITH MULTIPLE THREADS
// Best bet is to give each thread its own version of the camera!
//
// Camera parameters are stored in a GVar, but changing the gvar has no effect
// until the next call to RefreshParams() or SetImageSize().
//
// Pixel conventions are as follows:
// For Project() and Unproject(),
// round pixel values - i.e. (0.0, 0.0) - refer to pixel centers
// I.e. the top left pixel in the image covers is centered on (0,0)
// and covers the area (-.5, -.5) to (.5, .5)
//
// Be aware that this is not the same as what opengl uses but makes sense
// for acessing pixels using ImageRef, especially ir_rounded.
//
// What is the UFB?
// This is for projecting the visible image area
// to a unit square coordinate system, with the top-left at 0,0,
// and the bottom-right at 1,1
// This is useful for rendering into textures! The top-left pixel is NOT
// centered at 0,0, rather the top-left corner of the top-left pixel is at 
// 0,0!!! This is the way OpenGL thinks of pixel coords.
// There's the Linear and the Distorting version - 
// For the linear version, can use 
// glMatrixMode(GL_PROJECTION); glLoadIdentity();
// glMultMatrix(Camera.MakeUFBLinearFrustumMatrix(near,far));
// To render un-distorted geometry with full frame coverage.
//

#ifndef __ATAN_CAMERA_H
#define __ATAN_CAMERA_H

#include <TooN/TooN.h>
#include <cmath>

namespace PTAMM {
  
	using namespace TooN;

	static const int kNumCameraParameters = 5;
	typedef TooN::Vector<kNumCameraParameters> CameraParameters;
	typedef Vector<2> Vec2;
	typedef Matrix<2,2> Mat2;

	// The parameters are:
	// 0 - normalized x focal length
	// 1 - normalized y focal length
	// 2 - normalized x offset
	// 3 - normalized y offset
	// 4 - w (distortion parameter)

	class ATANCamera
	{
  public:
    //ATANCamera( std::string sName );
    ATANCamera(Vec2 irSize, CameraParameters vParams);
    
    // Image size get/set: updates the internal projection params to that target image size.
    void SetImageSize(Vec2 v2ImageSize);
    inline Vec2 GetImageSize() {return mvImageSize;};
    inline CameraParameters GetParams()  { return mvCameraParams; }
    void RefreshParams();
    
    // Various projection functions
    Vec2 Project(const Vec2& camframe); // Projects from camera z=1 plane to pixel coordinates, with radial distortion
    Vec2 UnProject(const Vec2& imframe); // Inverse operation
    
    Vec2 UFBProject(const Vec2& camframe);
    Vec2 UFBUnProject(const Vec2& camframe);
    inline Vec2 UFBLinearProject(const Vec2& camframe);
    inline Vec2 UFBLinearUnProject(const Vec2& fbframe);
    
		Mat2 GetProjectionDerivs(); // Projection jacobian
    
    inline bool Invalid() {  return mbInvalid;}
    inline double LargestRadiusInImage() {  return mdLargestRadius; }
    inline double OnePixelDist() { return mdOnePixelDist; }
    
    // The z=1 plane bounding box of what the camera can see
    inline Vec2 ImplaneTL(); 
    inline Vec2 ImplaneBR(); 

    // OpenGL helper function
    TooN::Matrix<4> MakeUFBLinearFrustumMatrix(double near, double far);

    // Feedback for Camera Calibrator
    double PixelAspectRatio() { return mvFocal[1] / mvFocal[0];}
    
    Vec2 ImageSize() { return mvImageSize; }

    // Useful for gvar-related reasons (in case some external func tries to read the camera params gvar, and needs some defaults.)
    static const CameraParameters mvDefaultParams;

  protected:
		CameraParameters mvCameraParams;
		//GVars3::gvar3<CameraParameters > mgvvCameraParams; // The actual camera parameters
  
    TooN::Matrix<2, kNumCameraParameters> GetCameraParameterDerivs();
    void UpdateParams(CameraParameters vUpdate);
		void DisableRadialDistortion();
    
    // Cached from the last project/unproject:
    Vec2 mvLastCam;      // Last z=1 coord
    Vec2 mvLastIm;       // Last image/UFB coord
    Vec2 mvLastDistCam;  // Last distorted z=1 coord
    double mdLastR;           // Last z=1 radius
    double mdLastDistR;       // Last z=1 distorted radius
    double mdLastFactor;      // Last ratio of z=1 radii
    bool mbInvalid;           // Was the last projection invalid?
    
    // Cached from last RefreshParams:
    double mdLargestRadius; // Largest R in the image
    double mdMaxR;          // Largest R for which we consider projection valid
    double mdOnePixelDist;  // z=1 distance covered by a single pixel offset (a rough estimate!)
    double md2Tan;          // distortion model coeff
    double mdOneOver2Tan;   // distortion model coeff
    double mdW;             // distortion model coeff
    double mdWinv;          // distortion model coeff
		double mdDistortionEnabled; // One or zero depending on if distortion is on or off.
    Vec2 mvCenter;     // Pixel projection center
    Vec2 mvFocal;      // Pixel focal length
    Vec2 mvInvFocal;   // Inverse pixel focal length
    Vec2 mvImageSize;  
    Vec2 mvUFBLinearFocal;
    Vec2 mvUFBLinearInvFocal;
    Vec2 mvUFBLinearCenter;
    Vec2 mvImplaneTL;   
    Vec2 mvImplaneBR;

		// Radial distortion transformation factor: returns ration of distorted / undistorted radius.
		inline double rtrans_factor(double r)
		{
			if(r < 0.001 || mdW == 0.0)
				return 1.0;
			else 
				return (mdWinv* atan(r * md2Tan) / r);
		};

		// Inverse radial distortion: returns un-distorted radius from distorted.
		inline double invrtrans(double r)
		{
			if(mdW == 0.0)
				return r;
			return(tan(r * mdW) * mdOneOver2Tan);
		};
	};

	// Some inline projection functions:
	inline Vec2 ATANCamera::UFBLinearProject(const Vec2& camframe)
	{
		Vec2 v2Res;
		v2Res[0] = camframe[0] * mvUFBLinearFocal[0] + mvUFBLinearCenter[0];
		v2Res[1] = camframe[1] * mvUFBLinearFocal[1] + mvUFBLinearCenter[1];
		return v2Res;
	}

	inline Vec2 ATANCamera::UFBLinearUnProject(const Vec2& fbframe)
	{
		Vec2 v2Res;
		v2Res[0] = (fbframe[0] - mvUFBLinearCenter[0]) * mvUFBLinearInvFocal[0];
		v2Res[1] = (fbframe[1] - mvUFBLinearCenter[1]) * mvUFBLinearInvFocal[1];
		return v2Res;
	}


}

#endif

