/*
 * read_ply.cpp
 *
 *  Created on: 14 May 2010
 *      Author: alexf
 */

#include <fstream>

#include "common_types.h"
#include "read_ply.h"
#include "vw_image.tpp"

namespace indoor_context {
	void ReadPly(const string& file,
							 vector<pair<Vec3,PixelRGB<byte> > >& points,
							 bool read_normals) {
		ifstream input(file.c_str());

		string magic, line;
		getline(input, magic);
		CHECK(magic == "ply") << EXPR(magic) << "Wrong file format";

		int num_verts = -1;
		do {
			getline(input, line);
			if (line.substr(0,14) == "element vertex") {
				num_verts = boost::lexical_cast<int>(line.substr(15, line.size()-15));
			}
		} while (line != "end_header");
		CHECK(num_verts != -1) << "Failed to parse PLY: there was no line starting with 'element vertex'";

		Vec3 v, n;
		int r, g, b;
		for (int i = 0; i < num_verts; i++) {
			// streaming directly into PixelRGB<byte>::r etc seems to fail
			input >> v[0] >> v[1] >> v[2];
			if (read_normals) {
				input >> n[0] >> n[1] >> n[2];
			}
			input >> r >> g >> b;
			points.push_back(make_pair(v, PixelRGB<byte>(r, g, b)));
		}
	}
}
