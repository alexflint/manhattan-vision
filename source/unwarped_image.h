#pragma once

#include "ptam_include.h"

#include "common_types.h"

namespace indoor_context {
	// Represents a precomputed undistort map
	class UndistortMap {
	public:
		ImageF xmap;  // The source X coordinate for each output location
		ImageF ymap;  // The source X coordinate for each output location
		PTAMM::ATANCamera cam;

		// Expected input size
		ImageRef input_size;
		// Transforms from pixel coordinates to retina plane coordinates
		toon::Matrix<3> image_to_retina;
		// Inverse of the above
		toon::Matrix<3> retina_to_image;
		// Bounds of the image in the retina plane
		toon::Vector<2> tl, tr, bl, br;

		// Bounds poly. This consists of four homogenous *line equations*
		// representing the four boundaries in order: top, left, bottom,
		// right.
		vector<toon::Vector<3> > retina_bounds;

		// Initialize empty
		UndistortMap();
		// Initialize and compute
		UndistortMap(const ImageRef& size);
		// Calculate the mapping
		void Compute(const ImageRef& size);
		// Calculate the mapping if the current image size is not already
		// identical
		void Prepare(const ImageRef& size);

		// Apply the mapping to an image
		void Apply(const ImageBundle& in, ImageBundle& out) const;

		// For a given output (undistorted) pixel Y, get the input (raw,
		// distorted) pixel X such that ATANCamera::Project(X)=Y.
		toon::Vector<2> operator()(const ImageRef& ir) const;

		// Save xmap and ymap to "{basename}-xmap.png"
		void Save(const string& basename);
		// Load xmap and ymap from "{basename}-ymap.png"
		void Load(const string& basename);

		// Get the jacobian of the undistort function at the location P
		// This version inverts the Jacobian from ATANCamera
		toon::Matrix<2> ComputeDerivsAt(const toon::Vector<2>& pos);
		// Project a gradient at a given position into the unwarped domain
		toon::Vector<2> ProjectGradient(const toon::Vector<2>& gradient,
																		const toon::Vector<2>& position);
	};

	// Represents an undistorted image
	class UnwarpedImage {
	public:
		ImageBundle image;  // the resultant unwarped image
		toon::Matrix<3> image_to_retina;
		toon::Matrix<3> retina_to_image;
		const UndistortMap* map;
		const ImageBundle* prev_input;

		// From last invokation of ComputeDerivs
		UnwarpedImage();
		UnwarpedImage(const ImageBundle& image, const UndistortMap& map);
		void Compute(const ImageBundle& image, const UndistortMap& map);

		// Get the corners of this image in the retina plane
		toon::Vector<2> TLinRetina() const;
		toon::Vector<2> TRinRetina() const;
		toon::Vector<2> BRinRetina() const;
		toon::Vector<2> BLinRetina() const;
	};
}
