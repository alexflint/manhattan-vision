#include "colors.h"
#include "common_types.h"

#include "numeric_utils.tpp"

namespace indoor_context {
	PixelRGB<byte> BrightColors::Get(const byte i, const float alpha) {
		// Reversing the bits of i gives nicely distributed colours where if
		// N colors are used then they will reaonably far from each other,
		// but the sequence is independent of the number of colors actually
		// used.
		PixelRGB<byte> rgb;
		VW::ImageConversions::HSV2RGB(ReverseBits(i), 255, 255, rgb.r, rgb.g, rgb.b);
		rgb.alpha = alpha*255;
		return rgb;
	}

	PixelRGB<byte> BrightColors::Next() {
		// If this overflows then we just go back to the first color
		return Get(next++);
	}
}
