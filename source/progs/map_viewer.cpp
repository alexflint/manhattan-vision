#include <iomanip>

#include <LU.h>
#include <so3.h>

#include "entrypoint_types.h"
#include "map_widgets.h"
#include "map.h"
#include "map_io.h"
#include "map.pb.h"
#include "textons.h"
#include "vars.h"
#include "bld_helpers.h"

#include "widget3d.h"
#include "viewer3d.h"

#include "gl_utils.tpp"
#include "image_utils.tpp"
#include "io_utils.tpp"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);

	po::options_description desc("Allowed options");
	desc.add_options()
    ("help", "produce help message")
		("sequence", po::value<string>(), "Map by sequence name")
		("xml", po::value<string>(), "Map in PTAM XML format")
		("bundler", po::value<string>(), "Map in bundler format")
		("voodoo", po::value<string>(), "Map in voodoo text format")
		("images", po::value<string>(), "Pattern for voodoo images")
		("load_images", "Load all images (slow for large maps)")
		("selectable", "Make frames and points selectable (slow for large maps)")
		;

	// Parse options
	po::variables_map opts;
	try {
		po::store(po::parse_command_line(argc, argv, desc), opts);
		po::notify(opts);
	} catch (const po::required_option& ex) {
		cout << "Missing required option: "
				 << ex.get_option_name() << "\n" << desc << "\n";
		return 1;
	}
	if (opts.count("help")) {
    cout << desc << "\n";
    return -1;
	}

	// Load the map
	Map map;
	proto::TruthedMap gt_map;
	if (opts.count("sequence")) {
		string path = GetMapPath(opts["sequence"].as<string>());
		LoadXmlMapWithGroundTruth(path, map, gt_map);
	} else if (opts.count("xml")) {
		LoadXmlMap(opts["sequence"].as<string>(), map);
	} else if (opts.count("bundler")) {
		LoadBundlerMap(opts["bundler"].as<string>(), map);
	} else if (opts.count("voodoo")) {
		if (opts.count("images") == 0) {
			cout << "--images must be specified whenever --voodoo is\n";
			return -1;
		}
		LoadMapFromVoodooTextFile(opts["voodoo"].as<string>(),
															opts["images"].as<string>(),
															map);
	} else {
		cout << "You must specify a map\n" << desc << endl;
		return -1;
	}
		
	DLOG << boost::format("Map contains %d frames and %d points")
		% map.frames.size() % map.points.size();

	if (opts.count("load_images")) {
		map.LoadAllImages();
	}

	// Configure the map widget
	MapWidget map_widget(&map);
	if (opts.count("selectable") == 0) {
		map_widget.point_cloud_widget->SetSelectable(false);
		BOOST_FOREACH(FrameWidget* w, map_widget.frame_widgets) {
			w->SetSelectable(false);
		}
	}

	// Enter the event loop
	Viewer3D v;
	v.Add(map_widget);
	v.Run();

	return 0;
}
