#include <boost/filesystem.hpp>

#include "common_types_entry.h"
#include "map.h"
#include "vars.h"
#include "canvas.h"
#include "colors.h"
#include "image_utils.h"

#include "math_utils.tpp"
#include "io_utils.tpp"
#include "quaternion.tpp"

using namespace toon;
using namespace indoor_context;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		cerr << "Usage: ./draw_pts pts.txt\n";
		exit(-1);
	}

	int last_camera = -1;
	fs::path pts_file(argv[1]);
	fs::path image_dir = pts_file.parent_path()/"visualize";

	// Read the images
	ptr_vector<ImageBundle> images;
	int index = 0;
	while (true) {
		fs::path image_file = image_dir / (PaddedInt(index,8)+".jpg");
		if (!fs::exists(image_file)) break;
		images.push_back(new ImageBundle(image_file.string()));
		index++;
		DLOG << "read " << image_file;
	}
	DREPORT(images.size());

	// Read the points
	vector<Vec3> points;
	vector<vector<pair<int, Vec2> > > projs;
	ifstream pts_input(pts_file.string().c_str());
	BrightColors bc;
	int drew=0, total=0;
	while (!pts_input.eof()) {
		Vec3 pt;
		int n;
		pts_input >> pt >> n;
		PixelRGB<byte> color = bc.Next();
		total += n;
		for (int i = 0; i < n; i++) {
			int frame;
			Vec2 p;
			pts_input >> frame >> p;
			frame;
			if (frame >= 0 && frame < images.size()) {
				DrawSpot(images[frame].rgb, p, color, 1);
				drew++;
			}
		}
	}

	DLOG << "Rendered " << drew << " of " << total << " points";

	// Write the images
	for (int i = 0; i < images.size(); i++) {
		WriteImage(str(format("out/%08d.png")%i), images[i].rgb);
	}

	return 0;
}
