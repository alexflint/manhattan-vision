#include <cmath>

#include <iostream>
#include <sstream>

#include <gvars3/GUI.h>

#include <se3.h>
#include <LU.h>
#include <Cholesky.h>
#include <helpers.h>

#include "misc.h"
#include "common_types.h"
#include "vanishing_points.h"
#include "timer.h"
#include "image_bundle.h"
#include "unwarped_image.h"
#include "map_widgets.h"
#include "vars.h"
#include "clipping.h"
#include "progress_reporter.h"
#include "hotspot.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"
#include "range_utils.tpp"

using namespace indoor_context;

int axis_map[3];
Vector<3> vmin, vmax, center;


Vector<6> bounds;  // [xmin,ymin,zmin,xmax,ymax,zmax]
int selected_axis;

void SelectAxis(int i) {
	selected_axis = i;
}

void OnPointSelected(const MapViz& mapviz) {
	int i = mapviz.pts_widget->selected_point;
	if (i != -1) {
		Vector<3> v = mapviz.map.pts[i];
		switch (selected_axis) {
		case 0: bounds[ v[0] < center[0] ? 0 : 3 ] = v[0]; break;
		case 1: bounds[ v[1] < center[1] ? 1 : 4 ] = v[1]; break;
		case 2: bounds[ v[2] < center[2] ? 2 : 5 ] = v[2]; break;
		}
	}
}

void RenderBounds() {
	double x = bounds[0], X = bounds[3];
	double y = bounds[1], Y = bounds[4];
	double z = bounds[2], Z = bounds[5];

	glColor3f(1,1,1);

	GL_PRIMITIVE(GL_LINE_LOOP) {
		glVertex3f(x, y, z);
		glVertex3f(x, Y, z);
		glVertex3f(x, Y, Z);
		glVertex3f(x, y, Z);
	}

	GL_PRIMITIVE(GL_LINE_LOOP) {
		glVertex3f(X, y, z);
		glVertex3f(X, Y, z);
		glVertex3f(X, Y, Z);
		glVertex3f(X, y, Z);
	}

	GL_PRIMITIVE(GL_LINES) {
			glVertex3f(x, y, z);
			glVertex3f(X, y, z);

			glVertex3f(X, y, Z);
			glVertex3f(x, y, Z);

			glVertex3f(x, Y, z);
			glVertex3f(X, Y, z);

			glVertex3f(X, Y, Z);
			glVertex3f(x, Y, Z);
	}

	//glDepthMask(GL_FALSE);
	WITH(GL_BLEND) {
		GL_PRIMITIVE(GL_QUADS) {
			selected_axis == 0 ? glColor4f(1,1,0,0.25) : glColor4f(1,1,1,0.25);
			glVertex3f(x, y, z);
			glVertex3f(x, Y, z);
			glVertex3f(x, Y, Z);
			glVertex3f(x, y, Z);

			glVertex3f(X, y, z);
			glVertex3f(X, Y, z);
			glVertex3f(X, Y, Z);
			glVertex3f(X, y, Z);
			
			selected_axis == 1 ? glColor4f(1,1,0,0.25) : glColor4f(1,1,1,0.25);
			glVertex3f(x, y, z);
			glVertex3f(X, y, z);
			glVertex3f(X, y, Z);
			glVertex3f(x, y, Z);

			glVertex3f(x, Y, z);
			glVertex3f(X, Y, z);
			glVertex3f(X, Y, Z);
			glVertex3f(x, Y, Z);

			selected_axis == 2 ? glColor4f(1,1,0,0.25) : glColor4f(1,1,1,0.25);
			glVertex3f(x, y, z);
			glVertex3f(x, Y, z);
			glVertex3f(X, Y, z);
			glVertex3f(X, y, z);

			glVertex3f(x, y, Z);
			glVertex3f(x, Y, Z);
			glVertex3f(X, Y, Z);
			glVertex3f(X, y, Z);
		}
	}
	//glDepthMask(GL_TRUE);
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map
	MapViz mapviz;

	// Compute vanishing points
	{
		DISABLE_DLOG;
		mapviz.map.ComputeVanPts();
	}

	mapviz.pts_widget->SetSelectable(true);

	// Get the rotation
	SO3<> R = mapviz.map.vpt_detector.rot_est.R;

	// Find the vanishing point with largest absolute y coordinate
	Vector<3,int> upcounts = Zeros;
	BOOST_FOREACH (const KeyFrame& kf, mapviz.map.kfs) {
		double maxy = 0;
		int maxi;
		for (int i = 0; i < 3; i++) {
			double y = abs(project(fromVNL(kf.vpts.detector.vanpts[i].pt))[1]);
			if (y > maxy) {
				maxy = y;
				maxi = i;
			}
		}
		upcounts[maxi]++;
	}

	// Hack to keep track of the reordered axes
	axis_map[0] = 0;
	axis_map[1] = 1;
	axis_map[2] = 2;

	// Re-order the cols of the rotation so that the up direction is the Z axis.
	// Note that this is equivalent to swapping rows in R^-1
	int updir = max_index(&upcounts[0], &upcounts[3]);
	if (updir != 2) {
		Matrix<3> m = R.get_matrix().T();
		Vector<3> m2 = m[2];
		m[2] = m[updir];
		m[updir] = m2;
		R = m.T();
		mapviz.map.vpt_detector.rot_est.R = R;
	}
	swap(axis_map[2], axis_map[updir]);

	// Invert the rotation
	SO3<> Rinv = R.inverse();

	// Transform the points
	vector<double> ps[3];
	vmin = vmax = mapviz.map.pts[0];
	int i = 0;
	BOOST_FOREACH(Vector<3>& v, mapviz.map.pts) {
		v = Rinv * v;
		for (int i = 0; i < 3; i++) {
			if (v[i] < vmin[i]) vmin[i] = v[i];
			if (v[i] > vmax[i]) vmax[i] = v[i];
			ps[i].push_back(v[i]);
		}
	}
	center = makeVector(0,0,0);

	// Transform the keyframes
	SE3<> Rse;
	Rse.get_rotation() = R;
	BOOST_FOREACH(KeyFrame& kf, mapviz.map.kfs) {
		kf.CfW *= Rse;
		kf.WfC = kf.CfW.inverse();
	}

	// Initialize the bounds to anything (just so we can see what's going on)
	bounds.slice<0,3>() = center - makeVector(1, 1, 1);
	bounds.slice<3,3>() = center + makeVector(1, 1, 1);

	// Setup callbacks
	mapviz.viewer.BindKey('x', bind(&SelectAxis, 0));
	mapviz.viewer.BindKey('y', bind(&SelectAxis, 1));
	mapviz.viewer.BindKey('z', bind(&SelectAxis, 2));
	mapviz.pts_widget->SelectedPointChanged.add(bind(&OnPointSelected, ref(mapviz)));
	mapviz.viewer.Add(&RenderBounds);

	// Run the vizualization
	mapviz.Run();

	return 0;
}
