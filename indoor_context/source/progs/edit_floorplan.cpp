#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <fstream>

#include "entrypoint_types.h"
#include "floorplan_editor.h"
#include "map_widgets.h"
#include "map.h"
#include "map_io.h"
#include "vars.h"
#include "bld_helpers.h"

using namespace indoor_context;
using namespace toon;

proto::TruthedMap gt_map;
scoped_ptr<Map> slam_map;
scoped_ptr<FloorPlanEditor> editor;
string filename;

int lastCapturedIndex;
int curIndex;

void WriteToFile() {
	ofstream s(filename.c_str(), ios::binary);
	if (gt_map.SerializeToOstream(&s)) {
		cout << "Wrote TruthedMap to " << filename << endl;
	} else {
		cerr << "Error: Failed to serialize TruthedMap to " << filename << endl;
	}
}

proto::TruthedFrame* GetOrCreateTFrame(proto::TruthedMap& gt_map, int id) {
	for (int i = 0; i < gt_map.frame_size(); i++) {
		if (gt_map.frame(i).id() == id) {
			return gt_map.mutable_frame(i);
		}
	}
	// No truthed frame with this id so create one
	proto::TruthedFrame* tru_frame = gt_map.add_frame();
	tru_frame->set_id(id);
	return tru_frame;
}

void GenerateOrientMaps() {
	curIndex = 0;
	lastCapturedIndex = -1;
	editor->camViewState = ViewType::FILLED;
	editor->SelectFrameByIndex(curIndex);
}

// Create a directory if it does not already exist. Return true if
// successful.
bool CreateOrKeepDir(const fs::path& dir) {
	if (fs::exists(dir)) {
		return fs::is_directory(dir);
	} else {
		return fs::create_directory(dir);
	}
}

// Return the current working dir as a boost path
fs::path GetWorkingDir() {
	char cwd[2048];
	CHECK_NOT_NULL(getcwd(cwd, 2048));
	return fs::path(cwd);
}

void editor_AfterDisplay() {
	if (lastCapturedIndex != curIndex) {
		fs::path base_dir = fs::path(gt_map.spec_file()).parent_path().parent_path();
		CHECK_PRED1(fs::exists, base_dir);

		fs::path gt_dir = base_dir/"ground_truth";
		CreateOrKeepDir(gt_dir);
		fs::path orients_dir = gt_dir/"orient_maps";
		CreateOrKeepDir(orients_dir);

		string orients_file = str(format("frame%d_orients.png") % slam_map->frames[curIndex].id);
		fs::path orients_path = orients_dir/orients_file;
		fs::path complete_path = complete(orients_path, GetWorkingDir());

		editor->camView.OutputFrameBuffer(complete_path.string());
		lastCapturedIndex = curIndex;
		DLOG << "Captured to " << complete_path;

		int id = slam_map->frames[curIndex].id;
		proto::TruthedFrame* tf = GetOrCreateTFrame(gt_map, id);
		tf->set_id(id);
		tf->set_orient_map_file(complete_path.string());

		if (curIndex+1 < slam_map->frames.size()) {
			curIndex++;
			editor->SelectFrameByIndex(curIndex);
		} else {
			WriteToFile();
		}
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2 && argc != 4) {
		DLOG << "Usage: "<<argv[0]<<" SEQUENCE [ -c map.xml ]";
		exit(-1);
	}

	// Create the map
	slam_map.reset(new Map);

	if (argc == 2) {
		// Load an existing floorplan
		LoadXmlMapWithGroundTruth(GetMapPath(argv[1]), *slam_map, gt_map);

	} else {
		// Check args
		if (strcmp(argv[2], "-c")) {
			DLOG << "Unrecognised option: "<<argv[2];
			DLOG << "Usage: "<<argv[0]<<" FILE.pro [ -c map.xml ]";
			exit(-1);
		}
		fs::path spec_file(argv[3]);
		CHECK_PRED1(fs::exists, spec_file) << "You must specify an XML map file";
		DLOG << "Creating ground truth for " << spec_file.string();
			
		// Load the map and estimate a Manhattan frame
		LoadXmlMap(spec_file.string(), *slam_map);
		cout << "***** TODO: re-wire Map::RotateToSceneFrame!";
		cout << "Not estimating any Manhattan frame, outputting identity";		
		//slam_map->RotateToSceneFrame();

		// Create a new truthed map
		gt_map.set_spec_file(spec_file.string());
		SO3<> scene_from_slam; // TODO: estimate this!!
		gt_map.mutable_ln_scene_from_slam()->CopyFrom(asProto(scene_from_slam.ln()));
		gt_map.mutable_floorplan()->set_zfloor(-1);
		gt_map.mutable_floorplan()->set_zceil(1);
	}

	curIndex = 0;
	lastCapturedIndex = 0;

	// Configure and launch the editor
	CHECK(gt_map.has_floorplan());
	editor.reset(new FloorPlanEditor(*slam_map));
	editor->Attach(gt_map.mutable_floorplan());
	editor->camView.AfterDisplay.add(&editor_AfterDisplay);
	editor->KeyStroke('g').add(&GenerateOrientMaps);
	editor->KeyStroke('w').add(&WriteToFile);
	editor->Run();

	return 0;
}
