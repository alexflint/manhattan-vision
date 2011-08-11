#include "common_types_vw.h"

#include <VW/Image/imageio.tpp>

#include "common_types.h"
#include "vanishing_points.h"
#include "timer.h"
#include "image_bundle.h"
#include "unwarped_image.h"
#include "map.h"
#include "vars.h"

#include "image_utils.tpp"
//#include "numeric_utils.tpp"

#include "unwarped_image.h"
#include "vars.h"

using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG << "Usage: "<<argv[0]<<" INPUT OUTPUT";
		return -1;
	}

	ImageBundle image(argv[1]);
	UnwarpedImage unwarped(image);
	WriteImage(string(argv[2]), unwarped.image.rgb);

	return 0;
}
