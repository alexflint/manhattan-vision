#include "vw_image.tpp"

namespace indoor_context {
	/*ostream& operator<<(ostream& o, const ImageRef& p) {
		return o << "["<<p.x<<","<<p.y<<"]";
		}*/

	void ImageConvert(const ImageRGB<byte>& in, ImageMono<float>& out) {
		if (out.IsAlloced()) {
			CHECK_SAME_SIZE(in, out);
		} else {
			out.AllocImageData(in.GetWidth(), in.GetHeight());
		}
		for (int y = 0; y < in.GetHeight(); y++) {
			const PixelRGB<byte>* inrow = in[y];
			PixelMono<float>* outrow = out[y];
			for (int x = 0; x < in.GetWidth(); x++) {
				outrow[x].y = (inrow[x].r + inrow[x].g + inrow[x].b) / 3.;
			}
		}
	}

	void ImageConvert(const ImageMono<float>& in, ImageRGB<byte>& out) {
		if (out.IsAlloced()) {
			CHECK_SAME_SIZE(in, out);
		} else {
			out.AllocImageData(in.GetWidth(), in.GetHeight());
		}
		for (int y = 0; y < in.GetHeight(); y++) {
			const PixelMono<float>* inrow = in[y];
			PixelRGB<byte>* outrow = out[y];
			for (int x = 0; x < in.GetWidth(); x++) {
				outrow[x].r = outrow[x].g = outrow[x].b = static_cast<byte>(inrow[x].y);
				outrow[x].alpha = 0;
			}
		}
	}
}
