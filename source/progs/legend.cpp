#include "common_types_vw.h"
#include "misc.h"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	int n = atoi(argv[1]);
	int w = sqrt(n);
	ImageRGB<byte> image(w, n);
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < w; j++) {
			image[i][j] = BrightColors::Get(i);
		}
	}
	WriteImage("out/legend.png", image);
	return 0;
}
