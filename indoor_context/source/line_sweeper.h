#pragma once

#include "common_types.h"
#include "image_bundle.h"
#include "line_detector.h"
#include "camera.h"

namespace indoor_context {
// Sweeps lines towards vanishing points
class LineSweeper {
public:
	// Support maps for each orientation
	MatI support_maps[3];
	// Last input image
	const PosedImage* input;
	// The number of pixels set to 1 in each support map
	int num_pixels[3];

	// Initialize empty
	LineSweeper();
	// Initialize and compute
	LineSweeper(const PosedImage& pim,
	            const vector<LineDetection> lines[3]);
	// Compute support regions for the specified image
	void Compute(const PosedImage& pim,
	             const vector<LineDetection> lines[3]);
	// Produce a set of support vizualizations
	void OutputSupportViz(const string& basename) const;
};


// Base class for algorithms that label pixel orientations from a single image
class GeomLabellerBase {
public:
	// Map of orientation labels, -1 represents no label
	MatI orient_map;
	// Last input image
	const PosedImage* input;

	// Constructor
	GeomLabellerBase() : input(NULL) { }

	// Initialize orient_map and input
	void Init(const PosedImage& input);
	// Produce a vizualization of the orientations
	void DrawOrientViz(ImageRGB<byte>& canvas) const;
};


// Labels pixels by sweeping lines and identifying consensus regions
class IsctGeomLabeller : public GeomLabellerBase {
public:
	// The line sweeper
	LineSweeper sweeper;
	// Initialize empty
	IsctGeomLabeller();
	// Initialize and compute, use GuidedLineDetector
	IsctGeomLabeller(const PosedImage& pim);
	// Initialize and compute
	IsctGeomLabeller(const PosedImage& pim,
	                 const vector<LineDetection> lines[3]);
	// Compute plane orientations
	void Compute(const PosedImage& pim,
	             const vector<LineDetection> lines[3]);
	// Produce a vizualization of the orientations
	void OutputOrientViz(const string& filename) const;
};
}
