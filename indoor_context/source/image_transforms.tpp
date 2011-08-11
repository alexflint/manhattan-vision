#pragma once

#include <TooN/LU.h>

#include "common_types.h"

#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "range_utils.tpp"

namespace indoor_context {
	template <typename T>
	void TransformImage(const ImageRGB<T>& input,
											ImageRGB<T>& output,
											const Mat3& h) {  // h transforms from input to output coords
		ImageRef p;
		Mat3 hinv = toon::LU<3>(h).get_inverse();
		for (int y = 0; y < output.GetHeight(); y++) {
			PixelRGB<T>* outrow = output[y];
			for (int x = 0; x < output.GetWidth(); x++) {
				toImageRef(project(hinv * toon::makeVector(x,y,1.0)), p);
				if (p.x >= 0 && p.x < input.GetWidth() &&
						p.y >= 0 && p.y < input.GetHeight()) {
					outrow[x] = input[p];
				}
			}
		}
	}	

	template <typename T>
	void FlipVertical(ImageRGB<T>& image) {
		int nx = image.GetWidth();
		int ny = image.GetHeight();
		for (int y = 0; y < ny/2; y++) {
			PixelRGB<T>* r1 = image[y];
			PixelRGB<T>* r2 = image[ny-y-1];
			for (int x = 0; x < nx; x++) {
				swap(r1[x], r2[x]);
			}
		}
	}
}
