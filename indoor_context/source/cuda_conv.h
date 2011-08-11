#pragma once

namespace indoor_context {

struct device_image {
	inline device_image() : width(0), height(0), size(0), data(NULL) { }
	int width;
	int height;
	int size; // = sizeof(float) * width * height
	float *data;  // *in device memory*
};

// Initialize the device
void cudaconv_Init();

// Allocate an in-device image
void cudaconv_AllocImage(int w, int h, device_image* out);

// Load an image into device memory
void cudaconv_LoadImage(float *src, device_image* dest);

// Convolve an in-device image. If subtract is true then the result
// will be subtracted from whatever is currently in the destination
// buffer.
void cudaconv_Convolve(device_image* src,
											 int kernel_radius,
											 float *kernel_x,
											 float *kernel_y,
											 device_image* temp,
											 device_image* dest,
											 bool subtract);

// Get an image from the device
void cudaconv_RetrieveImage(device_image* src, float* dest);

// Free an in-device image
void cudaconv_FreeImage(device_image* image);

// Shut down the device
void cudaconv_Cleanup();

}

