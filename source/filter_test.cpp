#include <iostream>
#include <VW/Image/imageio.tpp>

#include "entrypoint_types.h"
#include "filters.h"
#include "gaussian_pyramid.h"
#include "building_features.h"
#include "timer.h"

#include "matrix_traits.tpp"
#include "image_utils.tpp"
#include "format_utils.tpp"

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		cout << "Usage: filter_test INPUT RADIUS\n";
		exit(-1);
	}

	// Load image
	string input = argv[1];
	float radius = atof(argv[2]);

	ImageBundle image(input);
	MatF mat;
	ImageToMatrix(image, mat);

	// Accumulate
	AccumulatedFeatures acc;
	TIMED_SECTION("Generate features") acc.Compute(mat, radius);

	// Output
	TITLED("Write images") {
		WriteMatrixImageRescaled("out/acc.png", acc.results[0]);	
		for (int i = 0; i < 4; i++) {
			WriteMatrixImageRescaled(fmt("out/nbr%d.png", i), acc.results[i+1]);
		}
		DLOG << "foo " << makeVector(3,3);
	}

	return 0;
}
