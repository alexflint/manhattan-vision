#pragma once

#include <boost/utility.hpp>
#include "common_types.h"
#include "cuda_conv.h"

namespace indoor_context {

// Runs convolutions using special purpose graphics hardware.
class HwConvolver : public boost::noncopyable {
	device_image input;
	device_image temp;
	device_image output;
	static void EnsureInitialized();  // Initialize hardware
public:
	// Allocates memory in the device (expensive)
	HwConvolver(int width, int height);
	// Free device memory (expensive)
	~HwConvolver();

	// The device is known to take a long time to run the first
	// convolution (~200ms extra). This method "warms up" the device by
	// running a dummy convolution.
	static void Warmup();
	// Load an image into device memory
	void LoadImage(const MatF& image);
	// Convolve an image in the device and retrieve the result.
	void Convolve(const VecF& kernelx,
								const VecF& kernely,
								MatF& output);
	// Take the different between two convolutions. Result is
	// image*kernel1 - image*kernel2. Note that this _cannot_ be
	// decomposed into image*(kernel1-kernel2)
	void ConvolveDiff(const VecF& kernelx1,
										const VecF& kernely1,
										const VecF& kernelx2,
										const VecF& kernely2,
										MatF& output);
private:
	static void CheckKernels(const VecF& kx, const VecF& ky,
													 float** px, float** py, int* rad);
};

}
