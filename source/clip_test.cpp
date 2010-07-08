// This code tests FillPolygonFast in clipping.h
// Usage: ./clip_test
//
// Look at shapes.png, it should consist of a grid of squares,
// diamonds, and octagons

#include "entrypoint_types.h"
#include "clipping.h"
#include "image_utils.h"

using namespace indoor_context;
using namespace toon;

vector<Vector<3> > offset(const vector<Vector<3> >& a, double dx, double dy) {
	vector<Vector<3> > b;
	BOOST_FOREACH(const Vector<3>& x, a) {
		b.push_back(unproject(project(x) + makeVector(dx,dy)));
	}
	return b;
}

vector<Vector<3> > rotate(const vector<Vector<3> >& a, int r) {
	vector<Vector<3> > b;
	int n = a.size();
	for (int i = 0; i < a.size(); i++) {
		b.push_back(a[(i+r)%a.size()]);
	}
	return b;
}

int main(int argc, char **argv) {
	vector<Vector<3> > square;
	square.push_back(makeVector(0,0,1));
	square.push_back(makeVector(0,20,1));
	square.push_back(makeVector(20,20,1));
	square.push_back(makeVector(20,0,1));

	vector<Vector<3> > diamond;
	diamond.push_back(makeVector(10,0,1));
	diamond.push_back(makeVector(0,10,1));
	diamond.push_back(makeVector(10,20,1));
	diamond.push_back(makeVector(20,10,1));

	vector<Vector<3> > oct;
	oct.push_back(makeVector(5,0,1));
	oct.push_back(makeVector(15,0,1));
	oct.push_back(makeVector(20,5,1));
	oct.push_back(makeVector(20,15,1));
	oct.push_back(makeVector(15,20,1));
	oct.push_back(makeVector(5,20,1));
	oct.push_back(makeVector(0,15,1));
	oct.push_back(makeVector(0,5,1));

	ImageRGB<byte> canvas(180, 180);
	FillPolygonFast(offset(square,20,20), canvas, Colors::red());
	
	vector<Vector<3> >* shapes[] = {&square, &diamond, &oct};
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 8; j++) {
			int index = (i*5+j)%3;
			int dx = -40 + i*30 + (i-4)*1e-8;  // the small pertubation tests horizontal line code
			int dy = -40 + j*30 + (i-4)*1e-8;

			vector<Vector<3> > poly = rotate(offset(*shapes[index], dx, dy), i);
			FillPolygonFast(poly, canvas, Colors::primary(index));
		}
	}
	
	WriteImage("shapes.png", canvas);
	return 0;
}

