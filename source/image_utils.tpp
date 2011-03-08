#pragma once

#include "image_utils.h"

#include <cmath>
#include <utility>

#include <boost/bind.hpp>
#include <cvd/image.h>
#include <cvd/rgba.h>

#include "common_types.h"
#include "vw_image_io.h"
#include "image_bundle.h"

#include "matrix_traits.tpp"
#include "polygon.tpp"
#include "vw_image.tpp"

namespace indoor_context {

// Get the maximum absolute value of a pixel in an image
template<typename T>
T GetMaxAbsPixel(const ImageMono<T>& image) {
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
void ImApply(ImageMono<T>& image, Function f) {
	for (int r = 0; r < image.GetHeight(); r++) {
		PixelMono<T>* row = image[r];
		for (int c = 0; c < image.GetWidth(); c++) {
			row[c].y = f(row[c].y);
		}
	}
}

// Apply a linear transform to all pixels so that 0 becomes mid-grey
// and the highest/lowest pixel is white/black respectively.
template<typename T, typename S>
void Recentre(ImageMono<T>& image, const S& max) {
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
void Rescale(ImageMono<T>& image, const S& max) {
	T scale = max / GetMaxAbsPixel(image);
	ImApply(image, bind(multiplies<T>(), scale, _1));
}

// Transform pixels so that 0 remains at 0 and the brighest pixel is
// white.
template<typename T, typename S>
void RescaleAbs(ImageMono<T>& image, const S& max) {
	for (int y = 0; y < image.GetWidth(); y++) {
		PixelMono<T>* row = image[y];
		for (int x = 0; x < image.GetHeight(); x++) {
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

// Transform pixels so that 0 remains at 0 and the brighest pixel is
// white.
inline void Rescale(ImageF& image) {
	Rescale(image, (1 << 16) - 1);
}

// Take absolute and then apply Rescale()
inline void RescaleAbs(ImageF& image) {
	RescaleAbs(image, (1 << 16) - 1);
}

// Copy one image into another with translation and scale changes
void CopyImageScaled(const ImageRGB<unsigned char>& image, int destx,
                     int desty, int destsize, ImageRGB<unsigned char>& canvas);

// Copy one image into another with translation and scale changes
void CopyImageInto(const ImageRGB<unsigned char>& image, int destr, int destc,
                   ImageRGB<unsigned char>& canvas);

// Downsample an image by a factor K
template<typename Array>
void Downsample(const Array& input, int k, Array& output) {
	typedef typename matrix_traits<Array>::value_type Element;
	CHECK_EQ(matrix_size(output), matrix_size(input)/k);
	for (int r = 0; r < matrix_height(output); r++) {
		const Element* in = input[r*k];
		Element* out = output[r];
		for (int c = 0; c < matrix_width(output); c++) {
			out[c] = in[c*k];
		}
	}
}

// Upsample by a factor K
template <typename Array>
void Upsample(const Array& input, int k, Array& output) {
	typedef typename matrix_traits<Array>::value_type Element;

	CHECK_EQ(matrix_size(input)*k, matrix_size(output));
	for (int r = 0; r < matrix_height(output); r++) {
		const Element* inrow1 = input[r/k];
		const Element* inrow2 = (r >= matrix_height(output)-k) ? inrow1 : input[r/k+1];
		float* outrow = output[r];
		for (int c = 0; c < matrix_width(output)-k; c++) {
			const int t = c%k;
			const int u = r%k;
			const float A = (k-t) * (k-u) * inrow1[c/k];
			const float B = t * (k-u) * inrow1[c/k+1];
			const float C = (k-t) * u * inrow2[c/k];
			const float D = t * u * inrow2[c/k+1];
			outrow[c] = (A+B+C+D) / (k*k);
		}
		// Deal with the last K columns seperately to avoid reading past
		// the end of rows
		for (int c = matrix_width(output)-k; c < matrix_width(output); c++) {
			const int u = r%k;
			const float A = (k-u) * inrow1[c/k];
			const float C = u * inrow2[c/k];
			outrow[c] = (A+C) / k;
		}
	}
}



template<typename T>
void ImageToMatrix(const ImageMono<T>& image, VNL::Matrix<T>& mat) {
	mat.Resize(image.GetHeight(), image.GetWidth());
	for (int y = 0; y < image.GetHeight(); y++) {
		const PixelF* in = image[y];
		float* out = mat[y];
		for (int x = 0; x < image.GetWidth(); x++) {
			out[x] = in[x].y;
		}
	}
}

template<typename T>
void ImageToMatrix(const ImageBundle& image, VNL::Matrix<T>& mat) {
	image.BuildMono();
	ImageToMatrix(image.mono, mat);
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
	T scale = 255.0 / mat.MaxValue();
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
			imrow[c].alpha = 0;
		}
	}
}

template<typename T, typename S>
void DrawMatrixRecentred(const VNL::Matrix<T>& mat, ImageRGB<S>& image) {
	T min = mat.MinValue();
	T max = mat.MaxValue();
	T scale = 255. / (max-min);
	T offset = -scale * min;
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
			imrow[c].r = imrow[c].g = imrow[c].b = offset + matrow[c]*scale;
			imrow[c].alpha = 0;
		}
	}
}

template<typename T>
void WriteMatrixImage(const string& filename, const VNL::Matrix<T>& mat) {
	ImageF image;
	MatrixToImage(mat, image);
	WriteImage(filename, image);
}

template<typename T>
void WriteImageRescaledInPlace(const string& filename, ImageMono<T>& image) {
	Rescale(image);
	WriteImage(filename, image);
}

template<typename T>
void WriteImageRescaled(const string& filename, const ImageMono<T>& image) {
	ImageMono<T> clone;
	ImageCopy(image, clone);
	WriteImageRescaledInPlace(filename, clone);
}

template<typename T>
void WriteImageRecentredInPlace(const string& filename, ImageMono<T>& image) {
	Recentre(image);
	WriteImage(filename, image);
}

template<typename T>
void WriteImageRecentred(const string& filename, const ImageMono<T>& image) {
	ImageMono<T> clone;
	ImageCopy(image, clone);
	WriteImageRecentredInPlace(filename, clone);
}

template<typename T>
void WriteMatrixImageRescaled(const string& filename, const VNL::Matrix<T>& mat) {
	ImageF image;
	MatrixToImage(mat, image);
	WriteImageRescaledInPlace(filename, image);
}

template<typename T>
void WriteMatrixImageRecentred(const string& filename, const VNL::Matrix<T>& mat) {
	ImageF image;
	MatrixToImage(mat, image);
	WriteImageRecentredInPlace(filename, image);
}

template<typename T>
void WriteImageOfNonFinites(const string& filename,
														const VNL::Matrix<T>& mat) {
	MatI m(mat.Rows(), mat.Cols());
	for (int y = 0; y < mat.Rows(); y++) {
		for (int x = 0; x < mat.Cols(); x++) {
			m[y][x] = (isfinite(mat[y][x]) ? 0.0 : 1.0);
		}
	}
	WriteMatrixImageRescaled(filename, m);
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
/*template<typename T, typename S>
PixelHSV<T> operator+(const PixelHSV<T>& a, const PixelHSV<S>& b) {
	return PixelHSV<T> (a.h + b.h, a.s + b.s, a.v + b.v);
	}*/

/*
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
*/

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
