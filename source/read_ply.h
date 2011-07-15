/*
 * read_ply.h
 *
 *  Created on: 14 May 2010
 *      Author: alexf
 */
#pragma once

#include "common_types.h"
#include "vw_image-fwd.h"

namespace indoor_context {
	// Read a .ply file as produced by PMVS etc.
	// If read_normals is false then this assumes lines of the form
	//    X Y Z R G B
	// If read_normals is true then this assumes lines of the form
	//    X Y Z NX NY NZ R G B
	void ReadPly(const string& file,
							 vector<pair<Vec3,PixelRGB<byte> > >& points,
							 bool read_normals=false);
}
