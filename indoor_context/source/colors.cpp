#include "colors.h"

#include "common_types.h"

#include "numeric_utils.tpp"

#define RETURN_RGB(R,G,B)				        \
	r=static_cast<byte>(255. * R);				\
	g=static_cast<byte>(255. * G);				\
	b=static_cast<byte>(255. * B);

namespace indoor_context {
	PixelRGB<byte> BrightColors::Get(const byte i, const float alpha) {
		// Reversing the bits of i gives nicely distributed colours where if
		// N colors are used then they will reaonably far from each other,
		// but the sequence is independent of the number of colors actually
		// used.
		PixelRGB<byte> rgb;
		Colors::HSV2RGB(ReverseBits(i), 255, 255, rgb.r, rgb.g, rgb.b);
		rgb.alpha = alpha*255;
		return rgb;
	}

	PixelRGB<byte> BrightColors::Next() {
		// If this overflows then we just go back to the first color
		return Get(next++);
	}

	// static
	void Colors::RGB2HSV(byte r, byte g, byte b, byte& h, byte& s, byte& v) {
		// from http://www.alvyray.com/Papers/hsv2rgb.htm
		// RGB are each on [0, 1]. S and V are returned on [0, 1] and H is
		// returned on [0, 6]. Exception: H is returned UNDEFINED if S==0.
		float R = 1. * r / 255.;
		float G = 1. * g / 255.;
		float B = 1. * b / 255.;
		float x = min(R, min(G, B));
		float vv = max(R, min(G, B));
		if(vv == x) {
			h = 0;
			s = 0;
			v = static_cast<byte>(255 * vv);
		} else {
			float f = (R == x) ? G - B : ((G == x) ? B - R : R - G);
			int i = (R == x) ? 3 : ((G == x) ? 5 : 1);
			h = static_cast<byte>(255. * (i - f /(vv - x)) / 6.);
			s = static_cast<byte>(255. * (vv - x)/vv);
			v = static_cast<byte>(255. * vv);
		}
	}

	// static
	void Colors::HSV2RGB(byte H, byte S, byte V, byte& r, byte& g, byte& b) {
		// H is given on [0, 6] or UNDEFINED. S and V are given on [0, 1].
		// RGB are each returned on [0, 1].
		float h = 6. * H / 255.;
		float s = 1. * S / 255.;
		float v = 1. * V / 255.;
		//if (h == UNDEFINED) RETURN_RGB(v, v, v);
		int i = floor(h);
		float f = h - i;
		if ( !(i&1) ) f = 1 - f; // if i is even
		float m = v * (1 - s);
		float n = v * (1 - s * f);
		switch (i) {
		case 6:
		case 0: RETURN_RGB(v, n, m); break;
		case 1: RETURN_RGB(n, v, m); break;
		case 2: RETURN_RGB(m, v, n); break;
		case 3: RETURN_RGB(m, n, v); break;
		case 4: RETURN_RGB(n, m, v); break;
		case 5: RETURN_RGB(v, m, n); break;
		}
	}
}
