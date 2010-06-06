#include <stdlib.h>

#include "common_types.h"
#include "map_widgets.h"
#include "map.h"
#include "vars.h"

using namespace std;
using namespace TooN;
using namespace VW;
using namespace indoor_context;

void DrawStuff() {
	glColor3f(1,1,1);
	glPointSize(5);
	glBegin(GL_POINTS);
	glVertex3f(1,1,1);
	glEnd();
}


void DrawSomePoints(const vector<Vector<3> >& pts) {
	glColor3f(1,1,1);
	glPointSize(5);
	glBegin(GL_POINTS);
	BOOST_FOREACH(const Vector<3>& v, pts) glVertexV(v);
	glEnd();
}



int main(int argc, char **argv) {
	InitVars(argc, argv);
	VizTool3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return -1;
	}

	vector<Vector<3> > magicPoints;
	magicPoints.push_back(makeVector(2.0,2,2));
	magicPoints.push_back(makeVector(6.0,2,3));
	// ....

	VizTool3D viewer;
	viewer.Create();
	viewer.Add(&DrawStuff);

	viewer.Add(bind(&DrawSomePoints, magicPoints));

	viewer.Run();

	return 0;
}
