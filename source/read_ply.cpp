/*
 * read_ply.cpp
 *
 *  Created on: 14 May 2010
 *      Author: alexf
 */

#include <fstream>
#include "read_ply.h"

#include "vw_image.tpp"

namespace indoor_context {

void ReadPly(const string& file,
             vector<pair<Vec3,PixelRGB<byte> > >& points) {
	ifstream input(file.c_str());

	string magic, line;
	getline(input, magic);
	CHECK(magic == "ply") << EXPR(magic) << "Wrong file format";

	getline(input, line); // trash

	// Get the number of vertices
	int num_verts;
	input >> line >> line >> num_verts;
	DREPORT(num_verts);

	while (line != "end_header") {
		getline(input, line);
	}

	Vec3 v, n;
	int r, g, b;
	for (int i = 0; i < num_verts; i++) {
		// streaming directly into PixelRGB<byte>::r etc seems to fail
		input >> v[0] >> v[1] >> v[2] >> n[0] >> n[1] >> n[2] >> r >> g >> b;
		points.push_back(make_pair(v, PixelRGB<byte>(r, g, b)));
	}
}

}
