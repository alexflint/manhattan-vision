#include <fstream>

#include <LU.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "entrypoint_types.h"
#include "camera.h"
#include "canvas.tpp"
#include "map.h"
#include "map.pb.h"
#include "bld_helpers.h"
#include "floorplan_renderer.h"
#include "geom_utils.h"
#include "manhattan_ground_truth.h"

#include "clipping3d.tpp"
#include "image_utils.tpp"
#include "vector_utils.tpp"
#include "range_utils.tpp"
#include "io_utils.tpp"
#include "format_utils.tpp"
#include "counted_foreach.tpp"

Mat3 HomographyTransform(const ImageRGB<byte>& image,
												 ImageRGB<byte>& canvas,
												 const Mat3& Hinv) {
	for (int y = 0; y < image.GetHeight(); y++) {
		for (int x = 0; x < image.GetWidth(); x++) {
			ImageRef p = asIR(project(Hinv*makeVector(x, y, 1.)));
			if (p.x >= 0 && p.x < image.GetWidth() &&
					p.y >= 0 && p.y < image.GetHeight()) {
				canvas[y][x] = image[p];
			}
		}
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 3) {
		DLOG << "Usage: "<<argv[0]<<" SEQUENCE FRAMES";
		exit(-1);
	}

	string sequence = argv[1];
	vector<int> frame_ids = ParseMultiRange<int>(argv[2]);

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	map.LoadWithGroundTruth(GetMapPath(sequence), gt_map);

	// Draw the ground truth
	BOOST_FOREACH(int frame_id, frame_ids) {
		TITLE("Processing frame " << frame_id);
		
		// Get the frame
		KeyFrame& frame = *map.KeyFrameByIdOrDie(frame_id);
		frame.LoadImage();

		//WriteImage(fmt("out/frame%03d_original.png", frame_id), frame.image.rgb);

		// Get ground truth
		ManhattanGroundTruth gt(gt_map.floorplan(), frame.image.pc());

		// Rectify the image
		Mat3 H = GetVerticalRectifier(frame.image.pc());
		Mat3 Hinv = LU<3>(H).get_inverse();
		ImageRGB<byte> rectified(frame.image.sz());
		rectified.Clear(Colors::white());
		HomographyTransform(frame.image.rgb, rectified, Hinv);
		WriteImage(fmt("out/frame%03d_rectified.png", frame_id), rectified);

		// Draw orientations
		ImageRGB<byte> orients(frame.image.sz());
		for (int y = 0; y < orients.GetHeight(); y++) {
			for (int x = 0; x < orients.GetWidth(); x++) {
				if (gt.orientations()[y][x] < 2) {
					orients[y][x] = Colors::primary(gt.orientations()[y][x]);
				} else {
					orients[y][x] = Colors::white();
				}
			}
		}
		HomographyTransform(orients, rectified, Hinv);
		WriteImage(fmt("out/frame%03d_gt.png", frame_id), rectified);

		/*
		// Use floorplan renderer to get the wall vertices
		Matrix<3,4> camera = H * frame.image.pc().Linearize();
		Vec2I viewport = frame.image.size();
		FloorPlanRenderer re;
		re.Render(gt_map.floorplan(), camera, viewport);

		// Get clipped polygons
		ptr_vector<vector<Vec2> > polygons;
		BOOST_FOREACH(const Polygon<4>& wall, re.walls()) {
			vector<Vec3> clipped;
			ClipToFrustrum(array_range(wall.verts, 4), camera, viewport, back_inserter(clipped));
			polygons.push_back(new vector<Vec2>);
			BOOST_FOREACH(const Vec3& v, clipped) {
				polygons.back().push_back(project(camera * unproject(v)));
			}
		}

		// Initialize canvas
		FileCanvas canvas(fmt("out/frame%03d_gt.png", frame_id), rectified);

		// Draw interiors
		COUNTED_FOREACH(int i, const vector<Vec2>& polygon, polygons) {
			DREPORT(polygon.size());
			if (polygon.size() > 2) {
				canvas.FillPolygon(polygon, Colors::primary(re.wall_labels()[i]));
			}
		}

		// Draw outlines
		canvas.SetLineWidth(8.0);
		BOOST_FOREACH(const vector<Vec2>& polygon, polygons) {
			for (int i = 0; i < polygon.size(); i++) {
				const Vec2& u = polygon[i];
				const Vec2& v = polygon[(i+1)%polygon.size()];
				canvas.StrokeLine(u, v, Colors::black());
			}
			}*/
		
		frame.UnloadImage();
	}

	return 0;
}
