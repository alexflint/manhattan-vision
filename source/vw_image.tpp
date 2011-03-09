#pragma once

#include <iostream>

#include "common_types.h"
#include "vw_image-fwd.h"

#include "matrix_traits.tpp"

namespace indoor_context {
	using boost::scoped_array;

	// Minimal mock of VW::ImageRGB, VW::ImageMono, etc
	class ImageRef {
	public:
		int x, y;
		inline ImageRef() : x(0), y(0) { }
		inline ImageRef(int xx, int yy) : x(xx), y(yy) { }
		inline bool operator==(const ImageRef& other) const {
			return x==other.x && y==other.y;
		}
		inline bool operator!=(const ImageRef& other) const {
			return x!=other.x || y!=other.y;
		}
		inline ImageRef operator+(const ImageRef& other) const {
			return ImageRef(x+other.x, y+other.y);
		}
	};

	template <typename T> class PixelMono {
	public:
		T y;
		inline PixelMono() { }
		inline PixelMono(const T& yy) : y(yy) { }
	};

	template <typename T> class PixelRGB {
	public:
		T r, g, b, alpha;
		inline PixelRGB() { }
		inline PixelRGB(const T& rr, const T& gg, const T& bb) 
			: r(rr), g(gg), b(bb), alpha(0.) { }
		inline PixelRGB(const T& rr, const T& gg, const T& bb, const T& aa)
			: r(rr), g(gg), b(bb), alpha(aa) { }
		inline void Set(const T& rr, const T& gg, const T& bb) {
			r = rr;
			g = gg;
			b = bb;
		}
	};

	template <typename T>
	class ImageBase {
	public:
		ImageBase() : size_(0,0), data_(NULL) { }
		ImageBase(const ImageRef& p) : data_(NULL) {
			AllocImageData(p.x, p.y);
		}
		ImageBase(int width, int height) : data_(NULL) {
			AllocImageData(width, height);
		}
		~ImageBase() {
			if (IsAlloced()) {
				delete[] data_;
				delete[] rows_;
			}
		}
		// Get image dimensions
		inline int GetWidth() const { return size_.x; }
		inline int GetHeight() const { return size_.y; }
		inline const ImageRef& GetSize() const { return size_; }
		// Get image data
		inline T* GetData() { return data_; }
		inline const T* GetData() const { return data_; }
		// Get image data -- identical to above but matches VW::ImageBase
		inline T* GetImageBuffer() { return data_; }
		inline const T* GetImageBuffer() const { return data_; }
		// Allocate image data
		inline bool IsAlloced() const { return data_ != NULL; }
		int AllocImageData(int width, int height) {
			size_.x = width;
			size_.y = height;

			if (IsAlloced()) {
				FreeImageData();
			}

			data_ = new T[width*height];
			rows_ = new T*[height];
			for (int i = 0; i < size_.y; i++) {
				rows_[i] = &data_[i*size_.x];
			}
		}
		int FreeImageData() {
			delete[] data_;
			data_ = NULL;
			delete[] rows_;
			rows_ = NULL;
		}
		// Fill image with a specified value
		void Clear(const T& v) {
			for (int i = 0, n = size_.x*size_.y; i < n; i++) {
				data_[i] = v;
			}
		}
		inline T* operator[](int r) { return rows_[r]; }
		inline const T* operator[](int r) const { return rows_[r]; }
		inline T& operator[](const ImageRef& p) { return (*this)[p.y][p.x]; }
		inline const T& operator[](const ImageRef& p) const { return (*this)[p.y][p.x]; }
	private:
		ImageRef size_;
		T* data_;  // made these non-scoped_arrays but performance didn't improve
		T** rows_;
	};

	template <typename T>
	class ImageMono : public ImageBase<PixelMono<T> > {
	public:
		ImageMono() { }
		ImageMono(const ImageRef& p)
			: ImageBase<PixelMono<T> >(p) { }
		ImageMono(int width, int height)
			: ImageBase<PixelMono<T> >(width, height) { }
	};

	template <typename T>
	class ImageRGB : public ImageBase<PixelRGB<T> > {
	public:
		ImageRGB() { }
		ImageRGB(const ImageRef& p)
			: ImageBase<PixelRGB<T> >(p) { }
		ImageRGB(int width, int height)
			: ImageBase<PixelRGB<T> >(width, height) { }
	};

	// Image copying
	template <typename T>
	void ImageCopy(const ImageBase<T>& in,
								 ImageBase<T>& out) {
		if (out.IsAlloced()) {
			CHECK_SAME_SIZE(in, out);
		} else {
			out.AllocImageData(in.GetWidth(), in.GetHeight());
		}
		for (int y = 0; y < in.GetHeight(); y++) {
			const T* inrow = in[y];
			T* outrow = out[y];
			for (int x = 0; x < in.GetWidth(); x++) {
				outrow[x] = inrow[x];
			}
		}
	}

	// matrix_traits implementation for images
	template<typename T>
	inline int matrix_width(const ImageBase<T>& image) {
		return image.GetWidth();
	}
	template<typename T>
	inline int matrix_height(const ImageBase<T>& image) {
		return image.GetHeight();
	}
	template<typename T>
	inline int matrix_width(const ImageMono<T>& image) {
		return image.GetWidth();
	}
	template<typename T>
	inline int matrix_height(const ImageMono<T>& image) {
		return image.GetHeight();
	}
	template<typename T>
	inline int matrix_width(const ImageRGB<T>& image) {
		return image.GetWidth();
	}
	template<typename T>
	inline int matrix_height(const ImageRGB<T>& image) {
		return image.GetHeight();
	}

	// Hack because the above weren't being picked up for some bizarre reason...
	template<>
	inline int matrix_width(const ImageMono<float>& image) {
		return image.GetWidth();
	}
	template<>
	inline int matrix_height(const ImageMono<float>& image) {
		return image.GetHeight();
	}
	template<>
	inline int matrix_width(const ImageRGB<byte>& image) {
		return image.GetWidth();
	}
	template<>
	inline int matrix_height(const ImageRGB<byte>& image) {
		return image.GetHeight();
	}


  // ugly hack for CHECK_SAME_SIZE
	inline int matrix_width(const ImageRef& p) {
		return p.x;
	}
	inline int matrix_height(const ImageRef& p) {
		return p.y;
	}

}
