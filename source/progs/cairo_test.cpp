#include "common_types.h"
#include "colors.h"
#include "image_bundle.h"

#include "canvas.tpp"
#include "vector_utils.tpp"

using namespace indoor_context;

int main(int argc, char **argv) {
	ImageBundle im("misc-imgs/tablescene.png");

	CHECK_EQ(argc, 2);
	FileCanvas canvas(argv[1], asToon(im.sz()));
	canvas.DrawImage(im.rgb);


	PixelRGB<byte> red = Colors::red();
	canvas.StrokeLine(makeVector(0,0), makeVector(50,50), red);

	PixelRGB<byte> blue = Colors::blue();
	blue.alpha = 127;
	Polygon<4> poly;
	poly.verts[0] = makeVector(50,0,1);
	poly.verts[1] = makeVector(0,50,1);
	poly.verts[2] = makeVector(20,80,1);
	poly.verts[3] = makeVector(100,10,1);
	canvas.FillPolygon(poly, blue);

	return 0;
}
