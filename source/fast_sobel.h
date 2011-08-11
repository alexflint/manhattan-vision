#pragma once

#include "common_types.h"
#include "vw_image-fwd.h"

namespace indoor_context {

// Provides a high-speed parallelized implementation of sobel filters
class FastSobel {
public:
	// The two types of sobel filters
	enum SobelType {
		kSobelX,
		kSobelY,
	};

	// Run a tweaked x-sobel convolution
	static void ConvolveX(const ImageF& input, ImageF& output) {
		Convolve(input, output, kSobelX);
	}

	// Run a tweaked y-sobel convolution
	static void ConvolveY(const ImageF& input, ImageF& output) {
		Convolve(input, output, kSobelY);
	}

	// Run a tweaked sobel convolution
	static void Convolve(const ImageF& input,
	                     ImageF& output,
	                     const int direction);

	// Convolve rows in the interval [r0,r1]
	static void ConvolveRowRange(const ImageF& input,
	                             ImageF& output,
	                             const int direction,
	                             const int r0,
	                             const int r1);

	// Convolve a row with the X sobel filter
	static void ConvolveRowX(const int w,
	                         const PixelF* r1,
	                         const PixelF* r2,
	                         const PixelF* r3,
	                         PixelF* out);

	// Convolve a row with the Y sobel filter
	static void ConvolveRowY(const int w,
	                         const PixelF* r1,
	                         const PixelF* r2,
	                         const PixelF* r3,
	                         PixelF* out);
};

}
