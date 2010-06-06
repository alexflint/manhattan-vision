#pragma once

#include <vector>

#include "line_detector.h"
#include "common_types.h"
#include "line_sweeper.h"
#include "canny.h"
#include "camera.h"

namespace indoor_context {
	// Represents a pixel that voted for a line
	struct LinePixel {
		float d0;
		float effmag;
		toon::Vector<2> pos;
		LinePixel() { }
		LinePixel(float dd0, int xx, int yy) : d0(dd0), pos(toon::makeVector(xx,yy)) { }
		inline bool operator<(const LinePixel& other) const {
			return d0 < other.d0;
		}
	};

	// Represents a histogram bin
	struct LineBin {
		double support;
		float start_d0, end_d0;
		toon::Vector<2> start, end;
		vector<LinePixel> pixels;
	};

	// Represents the vpt-targetted line detection algorithm
	class GuidedLineDetector {
	public:
		// Inputs
		const PosedImage* input;

		// Image gradients
		Gradients gradients;

		// Projected vanishing points
		toon::Vector<3> image_vpts[3];
		toon::Vector<3> retina_vpts[3];

		// Undistort map
		MatF undist[3];

		// Pixel-wise values
		MatF responses[3];  // the agreement of each pixel with each vpt
		MatI assocs;  // the vpt each pixel is associated with
		MatF thetas;  // the angle each pixel makes about its associated vpt
		MatF d0s;  // the distance of each pixel from its associated vpt
		MatI bins[3];  // the bin index for each pixel

		// Bounds
		float min_d0[3], max_d0[3];

		// histograms
		int num_t_bins;
		double theta_offsets[3], theta_spans[3];
		vector<LineBin> histogram[3];

		// Histogram peaks as (theta, bin_index) pairs, listed by vanishing point
		vector<pair<float, int> > peaks[3];
		// Segments, listed by vanishing point
		vector<LineDetection> detections[3];

		// Windows around vanishing points
		vector<toon::Vector<3> > vpt_windows[3];
		vector<double> vpt_votes[3];

		// The samples we selected to localise the vanishing points
		vector<ImageRef> vpt_samples[3];

		// Vizualization settings
		bool draw_thick_lines;  // default is false

		// Initialize empty
		GuidedLineDetector() : draw_thick_lines(false) { } 
		// Initialize and detect lines
		GuidedLineDetector(const PosedImage& image) : draw_thick_lines(false) {
			Compute(image);
		}

		// Detect line segments (does all of the below)
		void Compute(const PosedImage& image);

		// Compute the map from image to retina coordinates
		void ComputeUndist(const ImageRef& sz, const CameraBase& cam);
		// Compute image gradients
		void ComputeGradients();
		// Compute vanishing point locations in the image and retina
		void ComputeVptProjs();
		// Compute pixelwise vanishing point associations
		void ComputeAssocs();
		// Compute histograms over angle
		void ComputeHistograms();
		// Identify peaks in the histogram
		void ComputePeaks();
		// Identify individual line segments corresponding to histogram peaks
		void ComputeSegments();
		// Alternative way to identify peaks. Not currently used.
		void ComputeSegmentsWalking();
		// Sample some pixels and vote on "true" vanishing point locations
		void ComputeVptWindows();

		// Various utility functions
		int GetBin(int x, int y, int vpt_index); // only for the outside world, not used internally
		int GetBinForTheta(double theta, int assoc);

		// Draw functions
		void DrawSceneAxes(ImageRGB<byte>& canvas);
		void DrawBins(ImageRGB<byte>& canvas);
		void DrawAssociations(ImageRGB<byte>& canvas);
		void DrawDists(ImageRGB<byte>& canvas);
		void DrawRays(ImageRGB<byte>& canvas, int axis=-1);
		void DrawSegments(ImageRGB<byte>& canvas, int axis=-1);
		void DrawSegPixels(ImageRGB<byte>& canvas);
		void DrawVptWindows(ImageRGB<byte>& canvas);
		void DrawHistogram(int i, ImageRGB<byte>& canvas);

		// Vizualizations functions
		void OutputSceneAxesViz(const string& filename);
		void OutputBinViz(const string& filename);
		void OutputAssociationViz(const string& filename);
		void OutputDistsViz(const string& filename);
		void OutputThetaViz(const string& basename);
		void OutputRayViz(const string& filename, int axis=-1);
		void OutputSegmentsViz(const string& filename, int axis=-1);
		void OutputSegPixelsViz(const string& filename);
		void OutputVptWindowViz(const string& filename);
	};
}
