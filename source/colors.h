#pragma once

#include "common_types.h"

#include "vw_image.tpp"  // eek

namespace indoor_context {
	// Utility class of colours
	class Colors {
	public:
		static inline const PixelRGB<byte>& black() {
			static PixelRGB<byte> v(0, 0, 0);
			return v;
		}
		static inline const PixelRGB<byte>& white() {
			static PixelRGB<byte> v(255, 255, 255);
			return v;
		}
		static inline const PixelRGB<byte>& grey() {
			static PixelRGB<byte> v(128, 128, 128);
			return v;
		}
		static inline const PixelRGB<byte>& grey(int a) {
			static PixelRGB<byte> v(a, a, a);
			return v;
		}
		static inline PixelRGB<byte> grey(double a) {
			return PixelRGB<byte>(a*255, a*255, a*255);
		}
		static inline const PixelRGB<byte>& red() {
			static PixelRGB<byte> v(255, 0, 0);
			return v;
		}
		static inline const PixelRGB<byte>& green() {
			static PixelRGB<byte> v(0, 255, 0);
			return v;
		}
		static inline const PixelRGB<byte>& blue() {
			static PixelRGB<byte> v(0, 0, 255);
			return v;
		}
		static inline const PixelRGB<byte>& yellow() {
			static PixelRGB<byte> v(255, 255, 0);
			return v;
		}
		static inline const PixelRGB<byte>& fuchsia() {
			static PixelRGB<byte> v(255, 0, 255);
			return v;
		}
		static inline const PixelRGB<byte>& aqua() {
			static PixelRGB<byte> v(0, 255, 255);
			return v;
		}
		static inline const PixelRGB<byte>& primary(int i) {
			return i == 0 ? red() : (i == 1 ? green() : blue());
		}
		static inline const PixelRGB<byte>& comp(int i) {
			return i == 0 ? aqua() : (i == 1 ? fuchsia() : yellow());
		}
		static inline PixelRGB<byte> alpha(double a, const PixelRGB<byte>& in) {
			return PixelRGB<byte>(in.r, in.g, in.b, (1-a)*255);
		}

		// RGB/HSV conversions
		static void RGB2HSV(byte r, byte g, byte b, byte& h, byte& s, byte& v);
		static void HSV2RGB(byte h, byte s, byte v, byte& r, byte& g, byte& b);
	};

	// Iterate through colors with full Saturation and Value --
	// i.e. colors that show up well and look nice in vizualizations.
	class BrightColors {
		byte next;
	public:
		// Initialize a bright color iterator
		BrightColors() : next(0) { }
		// Initialize a bright color iterator
		BrightColors(int a) : next(a) { }
		// Get the n-th bright color (there are 256 of them)
		static PixelRGB<byte> Get(const byte i, const float alpha=0);
		// Get the next bright color
		PixelRGB<byte> Next();
	};
}
