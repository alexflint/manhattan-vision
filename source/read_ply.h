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

// Read a .ply file as produced by PMVS etc
void ReadPly(const string& file, vector<pair<Vec3,PixelRGB<byte> > >& points);

}
