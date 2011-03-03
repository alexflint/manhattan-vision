#include "svm_helpers.h"

#include "common_types.h"
#include "manhattan_dp.h"
#include "camera.h"

namespace indoor_context {
	/*void OutputLabelViz(const ImageRGB<byte>& bg,
											const MatI& clas,
											const string& file) {
		// Output classifications
		MatI orients(bg.GetHeight(), bg.GetWidth());
		int s = *gvFeatureStride;
		for (int y = 0; y < bg.GetHeight(); y++) {
			for (int x = 0; x < bg.GetWidth(); x++) {
				switch(clas[s*(y/s)]
							 [s*(x/s)]) {
				case -1: orients[y][x] = 0; break;
				case 0: orients[y][x] = -1; break;
				case 1: orients[y][x] = 2; break;
				}
			}
		}

		ImageRGB<byte> canvas;
		ImageCopy(bg, canvas);
		DrawOrientations(orients, canvas, 0.35);
		WriteImage(file, canvas);
		}*/

	/*void OutputResponseViz(const ImageRGB<byte>& bg,
												 const MatF& responses,
												 const string& file) {
		int s = *gvFeatureStride;
		double m = responses.AbsoluteValueMax();

		// Generate visualization
		ImageRGB<byte> canvas;
		ImageCopy(bg, canvas);
		for (int y = 0; y < bg.GetHeight(); y++) {
			for (int x = 0; x < bg.GetWidth(); x++) {
				float response = responses[s*(y/s)][s*(x/s)];
				byte v = 255 * abs(response) / m;
				PixelRGB<byte> color(response < 0 ? v : 0,
														 response > 0 ? v : 0,
														 0);
				BlendWith(canvas[y][x], color, 0.6);
			}
		}
		WriteImage(file, canvas);
		}*/
}
