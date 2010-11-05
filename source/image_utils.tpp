#pragma once

#include "image_utils.h"

#include <cmath>
#include <utility>

#include <boost/bind.hpp>
#include <cvd/image.h>
#include <cvd/rgba.h>
#include <VW/Image/imagecopy.h>

#include "common_types.h"

#include "polygon.tpp"

namespace indoor_context {
// Get the width of a matrix or image
template<typename Array>
int Width(const Array& x) {
	return x.GetWidth();
}

template<typename T>
int Width(const VNL::Matrix<T>& x) {
	return x.Cols();
}

// Get the height of a matrix or image
template<typename Array>
int Height(const Array& x) {
	return x.GetHeight();
}

template<typename T>
int Height(const VNL::Matrix<T>& x) {
	return x.Rows();
}

// Get the maximum absolute value of a pixel in an image
template<typename T>
T GetMaxAbsPixel(const VW::ImageMono<T>& image) {
	T maxabs = 0.0;
	for (int r = 0; r < image.GetHeight(); r++) {
		for (int c = 0; c < image.GetWidth(); c++) {
			T v = fabs(image[r][c].y);
			if (v > maxabs) {
				maxabs = v;
			}
		}
	}
	return maxabs;
}

// Apply a function to an image
template<typename T, typename Function>
void ImApply(VW::ImageMono<T>& image, Function f) {
	for (int r = 0; r < image.GetHeight(); r++) {
		VW::PixelMono<T>* row = image[r];
		for (int c = 0; c < image.GetWidth(); c++) {
			row[c].y = f(row[c].y);
		}
	}
}

// Apply a linear transform to all pixels so that 0 becomes mid-grey
// and the highest/lowest pixel is white/black respectively.
template<typename T, typename S>
void Recentre(VW::ImageMono<T>& image, const S& max) {
	T offset = max / 2.0;
	T scale = offset / GetMaxAbsPixel(image);
	for (int r = 0; r < image.GetHeight(); r++) {
		for (int c = 0; c < image.GetWidth(); c++) {
			image[r][c] = offset + scale * image[r][c].y;
		}
	}
}

// Transform pixels so that 0 remains at 0 and the brighest pixel is
// white.
template<typename T, typename S>
void Rescale(VW::ImageMono<T>& image, const S& max) {
	T scale = max / GetMaxAbsPixel(image);
	ImApply(image, bind(multiplies<T>(), scale, _1));
}

// Transform pixels so that 0 remains at 0 and the brighest pixel is
// white.
template<typename T, typename S>
void RescaleAbs(VW::ImageMono<T>& image, const S& max) {
	for (int y = 0; y < Width(image); y++) {
		VW::PixelMono<T>* row = image[y];
		for (int x = 0; x < Height(image); x++) {
			row[x].y = abs(row[x].y);
		}
	}
	Rescale(image, max);
}

// Transform pixels so that 0 becomes mid-grey and the
// highest/lowest pixel is white/black respectively.
inline void Recentre(ImageF& image) {
	Recentre(image, (1 << 16) - 1);
}
inline void Recentre(ImageD& image) {
	Recentre(image, (1 << 8) - 1);
}

// Transform pixels so that 0 remains at 0 and the brighest pixel is
// white.
inline void Rescale(ImageF& image) {
	Rescale(image, (1 << 16) - 1);
}
inline void Rescale(ImageD& image) {
	Rescale(image, (1 << 8) - 1);
}

// Take absolute and then apply Rescale()
inline void RescaleAbs(ImageF& image) {
	RescaleAbs(image, (1 << 16) - 1);
}
inline void RescaleAbs(ImageD& image) {
	RescaleAbs(image, (1 << 8) - 1);
}

// Copy one image into another with translation and scale changes
void CopyImageScaled(const ImageRGB<unsigned char>& image, int destx,
                     int desty, int destsize, ImageRGB<unsigned char>& canvas);

// Copy one image into another with translation and scale changes
void CopyImageInto(const ImageRGB<unsigned char>& image, int destr, int destc,
                   ImageRGB<unsigned char>& canvas);

// Downsample an image by a factor K
template<typename Image>
void Downsample(const Image& input, int k, Image& output) {
	assert(output.GetWidth() == input.GetWidth()/k);
	assert(output.GetHeight() == input.GetHeight()/k);
	for (int r = 0; r < output.GetHeight(); r++) {
		typename Image::PixelPtr in = input.GetRowStart(r * k);
		typename Image::PixelPtr out = output.GetRowStart(r);
		for (int c = 0; c < output.GetWidth(); c++) {
			out[c] = in[c * k];
		}
	}
}

// Downsample an image by a factor K
template<typename Image>
Image* Downsample(const Image& input, int k) {
	Image* output = new Image(input.GetWidth() / k, input.GetHeight() / k);
	Downsample(input, k, *output);
	return output;
}

template<typename T, typename S>
void MatrixToImage(const VNL::Matrix<T>& mat, ImageMono<S>& image) {
	if (image.IsAlloced()) {
		CHECK_EQ(image.GetWidth(), mat.Cols());
		CHECK_EQ(image.GetHeight(), mat.Rows());
	} else {
		image.AllocImageData(mat.Cols(), mat.Rows());
	}
	for (int r = 0; r < image.GetHeight(); r++) {
		const T* matrow = mat[r];
		PixelMono<S>* imrow = image[r]; 
		for (int c = 0; c < image.GetWidth(); c++) {
			imrow[c].y = matrow[c];
		}
	}
}


template<typename T, typename S>
void DrawMatrixRescaled(const VNL::Matrix<T>& mat, ImageRGB<S>& image) {
	T scale = 255.0 / mat.MaxValue();DREPORT(scale);
	if (image.IsAlloced()) {
		CHECK_EQ(image.GetWidth(), mat.Cols());
		CHECK_EQ(image.GetHeight(), mat.Rows());
	} else {
		image.AllocImageData(mat.Cols(), mat.Rows());
	}
	for (int r = 0; r < image.GetHeight(); r++) {
		const T* matrow = mat[r];
		PixelRGB<S>* imrow = image[r]; 
		for (int c = 0; c < image.GetWidth(); c++) {
			imrow[c].r = imrow[c].g = imrow[c].b = matrow[c]*scale;
		}
	}
}

template<typename T, typename String>
void WriteMatrixImage(String filename, const VNL::Matrix<T>& mat) {
	ImageF image;
	MatrixToImage(mat, image);
	WriteImage(filename, image);
}

template<typename T, typename String>
void WriteMatrixImageRescaled(String filename, const VNL::Matrix<T>& mat) {
	ImageF image;
	MatrixToImage(mat, image);
	WriteImageRescaledInPlace(filename, image);
}

template<typename T, typename String>
void WriteMatrixImageRecentred(String filename, const VNL::Matrix<T>& mat) {
	ImageF image;
	MatrixToImage(mat, image);
	WriteImageRecentredInPlace(filename, image);
}

template<typename T, typename String>
void WriteImageRescaledInPlace(String filename, ImageMono<T>& image) {
	Rescale(image);
	WriteImage(filename, image);
}

template<typename T, typename String>
void WriteImageRescaled(String filename, const ImageMono<T>& image) {
	ImageMono<T> clone;
	ImageCopy(image, clone);
	WriteImageRescaledInPlace(filename, clone);
}

template<typename T, typename String>
void WriteImageRecentredInPlace(String filename, ImageMono<T>& image) {
	Recentre(image);
	WriteImage(filename, image);
}

template<typename T, typename String>
void WriteImageRecentred(String filename, const ImageMono<T>& image) {
	ImageMono<T> clone;
	ImageCopy(image, clone);
	WriteImageRecentredInPlace(filename, clone);
}

// Recovers a segmentation from an RGB image assuming that pixels in
// the image are the same colour if and only if they are part of the
// same region. The values written to SEG increase sequentially
// starting from FIRST.
int ImageToSegmentation(const ImageRGB<byte>& image,
                        MatI& seg,
                        const int first = 0);

// Draw a polygon
template<unsigned N, typename T>
void DrawPolygon(ImageRGB<byte>& canvas,
                 const Polygon<N, T>& poly,
                 const PixelRGB<byte>& color) {
	for (int i = 0; i < N; i++) {
		DrawLine(project(poly.verts[i]), project(poly.verts[(i + 1) % N]),
		        color);
	}
}

// Draw a polygon
template<unsigned N, typename T>
void DrawPolygonClipped(ImageRGB<byte>& canvas,
                        const Polygon<N, T>& poly,
                        const PixelRGB<byte>& color) {
	for (int i = 0; i < N; i++) {
		DrawLineClipped(project(poly.verts[i]),
		        project(poly.verts[(i + 1) % N]), color);
	}
}

// Add two HSV pixels
template<typename T, typename S>
PixelHSV<T> operator+(const PixelHSV<T>& a, const PixelHSV<S>& b) {
	return PixelHSV<T> (a.h + b.h, a.s + b.s, a.v + b.v);
}

// Add two HSV pixels
template<typename T, typename S>
PixelHSV<T>& operator+=(PixelHSV<T>& a, const PixelHSV<S>& b) {
	a.h += b.h;
	a.s += b.s;
	a.v += b.v;
	return a;
}

// Divide an HSV pixel by a scalar
template<typename T, typename S>
PixelHSV<T> operator/(const PixelHSV<T>& a, const S& k) {
	return PixelHSV<T> (a.h / k, a.s / k, a.v / k);
}

// Divide an HSV pixel by a scalar
template<typename T, typename S>
PixelHSV<T>& operator/=(PixelHSV<T>& a, const S& k) {
	a.h /= k;
	a.s /= k;
	a.v /= k;
	return a;
}

// Re-allocate an image if it is not already the right size. Return
// true if an allocation was performed
template<class Image>
bool ResizeImage(Image& image, const int nx, const int ny) {
	if (image.IsAlloced() && image.GetSize() != ImageRef(nx, ny)) {
		image.FreeImageData();
	}
	if (!image.IsAlloced()) {
		image.AllocImageData(nx, ny);
		return true;
	}
	return false;
}

// Re-allocate an image if it is not already the right size. Return
// true if an allocation was performed
template<typename Image>
bool ResizeImage(Image& image, const ImageRef& ref) {
	return ResizeImage(image, ref.x, ref.y);
}

// Choose a random color
inline PixelRGB<byte> RandomColor() {
	return PixelRGB<byte> (rand() % 255, rand() % 255, rand() % 255);
}

// Convert a pixel from CVD to VW
template<typename T>
PixelRGB<T> fromCVD(const CVD::Rgba<T>& p) {
	return PixelRGB<T> (p.red, p.green, p.blue, p.alpha);
}

// Convert a pixel from VW to CVD
template<typename T>
CVD::Rgba<T> toToon(const PixelRGB<T>& p) {
	return CVD::Rgba<T>(p.r, p.g, p.b, p.alpha);
}

// Output conversions for pixels
template<typename T>
ostream& operator<<(ostream& o, const PixelRGB<T>& p) {
	o << "{" << static_cast<int> (p.r) << "," << static_cast<int> (p.g) << ","
	        << static_cast<int> (p.b) << "}";
	return o;
}

// Output conversions for pixels
template<typename T>
ostream& operator<<(ostream& o, const PixelMono<T>& p) {
	o << "{" << static_cast<int> (p.y) << "}";
	return o;
}
}
