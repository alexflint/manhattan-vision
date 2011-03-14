// Copyright 2008 Isis Innovation Limited
#include "ATANCamera.h"
#include <helpers.h>
#include <iostream>

namespace PTAMM {

using namespace std;

	/*ATANCamera::ATANCamera(string sName)
{
  // The camera name is used to find the camera's parameters in a GVar.
  msName = sName;
  mvCameraParams = GV2.Get(sName+".Parameters", mvDefaultParams, HIDDEN | FATAL_IF_NOT_DEFINED);
  mvImageSize[0] = 640.0;
  mvImageSize[1] = 480.0;
  RefreshParams();
}
	*/

/**
 * Create a camera using the specified name, image size, and parameters.
 * This will overwrite any existing GVars of the same name (e.g. Camera.Parameters)
 * @param sName camera name (Camera)
 * @param irSize image size
 * @param vParams The camera parameters
 */
ATANCamera::ATANCamera(Vec2 irSize, CameraParameters vParams)
{
	mvCameraParams = vParams;
  mvImageSize = irSize;
  RefreshParams();
}



void ATANCamera::SetImageSize(Vec2 vImageSize)
{
  mvImageSize = vImageSize;
  RefreshParams();
};

void ATANCamera::RefreshParams() 
{
  // This updates internal member variables according to the current camera parameters,
  // and the currently selected target image size.
  //
  
  // First: Focal length and image center in pixel coordinates
  mvFocal[0] = mvImageSize[0] * mvCameraParams[0];
  mvFocal[1] = mvImageSize[1] * mvCameraParams[1];
  mvCenter[0] = mvImageSize[0] * mvCameraParams[2] - 0.5;
  mvCenter[1] = mvImageSize[1] * mvCameraParams[3] - 0.5;
  
  // One over focal length
  mvInvFocal[0] = 1.0 / mvFocal[0];
  mvInvFocal[1] = 1.0 / mvFocal[1];

  // Some radial distortion parameters..
  mdW =  mvCameraParams[4];
  if(mdW != 0.0)
    {
      md2Tan = 2.0 * tan(mdW / 2.0);
      mdOneOver2Tan = 1.0 / md2Tan;
      mdWinv = 1.0 / mdW;
      mdDistortionEnabled = 1.0;
    }
  else
    {
      mdWinv = 0.0;
      md2Tan = 0.0;
      mdDistortionEnabled = 0.0;
    }
  
  // work out biggest radius in image
  Vec2 v2;
  v2[0]= max(mvCameraParams[2], 1.0 - mvCameraParams[2]) / mvCameraParams[0];
  v2[1]= max(mvCameraParams[3], 1.0 - mvCameraParams[3]) / mvCameraParams[1];
  mdLargestRadius = invrtrans(sqrt(v2*v2));
  
  // At what stage does the model become invalid?
  mdMaxR = 1.5 * mdLargestRadius; // (pretty arbitrary)

  // work out world radius of one pixel
  // (This only really makes sense for square-ish pixels)
  {
    Vec2 v2Center = UnProject(mvImageSize / 2);
    Vec2 v2RootTwoAway = UnProject(mvImageSize / 2 + makeVector(1,1));
    Vec2 v2Diff = v2Center - v2RootTwoAway;
    mdOnePixelDist = sqrt(v2Diff * v2Diff) / sqrt(2.0);
  }
  
  // Work out the linear projection values for the UFB
  {
    // First: Find out how big the linear bounding rectangle must be
    vector<Vec2 > vv2Verts;
    vv2Verts.push_back(UnProject(makeVector( -0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, mvImageSize[1]-0.5)));
    vv2Verts.push_back(UnProject(makeVector( -0.5, mvImageSize[1]-0.5)));
    Vec2 v2Min = vv2Verts[0];
    Vec2 v2Max = vv2Verts[0];
    for(int i=0; i<4; i++)
      for(int j=0; j<2; j++)
	{
	  if(vv2Verts[i][j] < v2Min[j]) v2Min[j] = vv2Verts[i][j];
	  if(vv2Verts[i][j] > v2Max[j]) v2Max[j] = vv2Verts[i][j];
	}
    mvImplaneTL = v2Min;
    mvImplaneBR = v2Max;
    
    // Store projection parameters to fill this bounding box
    Vec2 v2Range = v2Max - v2Min;
    mvUFBLinearInvFocal = v2Range;
    mvUFBLinearFocal[0] = 1.0 / mvUFBLinearInvFocal[0];
    mvUFBLinearFocal[1] = 1.0 / mvUFBLinearInvFocal[1];
    mvUFBLinearCenter[0] = -1.0 * v2Min[0] * mvUFBLinearFocal[0];
    mvUFBLinearCenter[1] = -1.0 * v2Min[1] * mvUFBLinearFocal[1];
  }
  
}

// Project from the camera z=1 plane to image pixels,
// while storing intermediate calculation results in member variables
Vec2 ATANCamera::Project(const Vec2& vCam){
  mvLastCam = vCam;
  mdLastR = sqrt(vCam * vCam);
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mdLastFactor * mvLastCam;
  
  mvLastIm[0] = mvCenter[0] + mvFocal[0] * mvLastDistCam[0];
  mvLastIm[1] = mvCenter[1] + mvFocal[1] * mvLastDistCam[1];
  
  return mvLastIm;
}

// Un-project from image pixel coords to the camera z=1 plane
// while storing intermediate calculation results in member variables
Vec2 ATANCamera::UnProject(const Vec2& v2Im)
{
  mvLastIm = v2Im;
  mvLastDistCam[0] = (mvLastIm[0] - mvCenter[0]) * mvInvFocal[0];
  mvLastDistCam[1] = (mvLastIm[1] - mvCenter[1]) * mvInvFocal[1];
  mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
  mdLastR = invrtrans(mdLastDistR);
  double dFactor;
  if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
  else
    dFactor = 1.0;
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = dFactor * mvLastDistCam;
  return mvLastCam;
}

// Utility function for easy drawing with OpenGL
// C.f. comment in top of ATANCamera.h
Matrix<4> ATANCamera::MakeUFBLinearFrustumMatrix(double near, double far)
{
  Matrix<4> m4 = Zeros;
  

  double left = mvImplaneTL[0] * near;
  double right = mvImplaneBR[0] * near;
  double top = mvImplaneTL[1] * near;
  double bottom = mvImplaneBR[1] * near;
  
  // The openGhelL frustum manpage is A PACK OF LIES!!
  // Two of the elements are NOT what the manpage says they should be.
  // Anyway, below code makes a frustum projection matrix
  // Which projects a RHS-coord frame with +z in front of the camera
  // Which is what I usually want, instead of glFrustum's LHS, -z idea.
  m4[0][0] = (2 * near) / (right - left);
  m4[1][1] = (2 * near) / (top - bottom);
  
  m4[0][2] = (right + left) / (left - right);
  m4[1][2] = (top + bottom) / (bottom - top);
  m4[2][2] = (far + near) / (far - near);
  m4[3][2] = 1;
  
  m4[2][3] = 2*near*far / (near - far);

  return m4;
};

Matrix<2,2> ATANCamera::GetProjectionDerivs()
{
  // get the derivative of image frame wrt camera z=1 frame at the last computed projection
  // in the form (d im1/d cam1, d im1/d cam2)
  //             (d im2/d cam1, d im2/d cam2)
  
  double dFracBydx;
  double dFracBydy;
  
  double &k = md2Tan;
  double &x = mvLastCam[0];
  double &y = mvLastCam[1];
  double r = mdLastR * mdDistortionEnabled;
  
  if(r < 0.01)
    {
      dFracBydx = 0.0;
      dFracBydy = 0.0;
    }
  else
    {
      dFracBydx = 
	mdWinv * (k * x) / (r*r*(1 + k*k*r*r)) - x * mdLastFactor / (r*r); 
      dFracBydy = 
	mdWinv * (k * y) / (r*r*(1 + k*k*r*r)) - y * mdLastFactor / (r*r); 
    }
  
  Matrix<2> m2Derivs;
  
  m2Derivs[0][0] = mvFocal[0] * (dFracBydx * x + mdLastFactor);  
  m2Derivs[1][0] = mvFocal[1] * (dFracBydx * y);  
  m2Derivs[0][1] = mvFocal[0] * (dFracBydy * x);  
  m2Derivs[1][1] = mvFocal[1] * (dFracBydy * y + mdLastFactor);  
  return m2Derivs;
}

Matrix<2,kNumCameraParameters> ATANCamera::GetCameraParameterDerivs()
{
  // Differentials wrt to the camera parameters
  // Use these to calibrate the camera
  // No need for this to be quick, so do them numerically
  
  Matrix<2, kNumCameraParameters> m2NNumDerivs;
  CameraParameters vNNormal = mvCameraParams;
  Vec2 v2Cam = mvLastCam;
  Vec2 v2Out = Project(v2Cam);
  for(int i=0; i<kNumCameraParameters; i++)
    {
      if(i == kNumCameraParameters-1 && mdW == 0.0)
	continue;
      CameraParameters vNUpdate;
      vNUpdate = Zeros;
      vNUpdate[i] += 0.001;
      UpdateParams(vNUpdate); 
      Vec2 v2Out_B = Project(v2Cam);
      m2NNumDerivs.T()[i] = (v2Out_B - v2Out) / 0.001;
      mvCameraParams = vNNormal;
      RefreshParams();
    }
  if(mdW == 0.0)
    m2NNumDerivs.T()[kNumCameraParameters-1] = Zeros;
  return m2NNumDerivs;
}

void ATANCamera::UpdateParams(Vector<5> vUpdate)
{
  // Update the camera parameters; use this as part of camera calibration.
  mvCameraParams = mvCameraParams + vUpdate;
  RefreshParams();
}

void ATANCamera::DisableRadialDistortion()
{
  // Set the radial distortion parameter to zero
  // This disables radial distortion and also disables its differentials
  mvCameraParams[kNumCameraParameters-1] = 0.0;
  RefreshParams();
}

Vec2 ATANCamera::UFBProject(const Vec2& vCam)
{
  // Project from camera z=1 plane to UFB, storing intermediate calc results.
  mvLastCam = vCam;
  mdLastR = sqrt(vCam * vCam);
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mdLastFactor * mvLastCam;
  
  mvLastIm[0] = mvCameraParams[2]  + mvCameraParams[0] * mvLastDistCam[0];
  mvLastIm[1] = mvCameraParams[3]  + mvCameraParams[1] * mvLastDistCam[1];
  return mvLastIm;
}

Vec2 ATANCamera::UFBUnProject(const Vec2& v2Im)
{
  mvLastIm = v2Im;
  mvLastDistCam[0] = (mvLastIm[0] - mvCameraParams[2]) / mvCameraParams[0];
  mvLastDistCam[1] = (mvLastIm[1] - mvCameraParams[3]) / mvCameraParams[1];
  mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
  mdLastR = invrtrans(mdLastDistR);
  double dFactor;
  if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
  else
    dFactor = 1.0;
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = dFactor * mvLastDistCam;
  return mvLastCam;
}

const CameraParameters ATANCamera::mvDefaultParams = makeVector(0.5, 0.75, 0.5, 0.5, 0.1);

}
