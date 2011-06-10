#include "common_types.h"
#include "textons.h"
#include "map.h"
#include "vars.h"
#include "progress_reporter.h"
#include "kmeans.h"
//#include "numeric_utils.tpp"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);

	// Load the map
	Map map;
	map.LoadXml(GV3::get<string>("Map.SpecFile"));
	vector<ImageBundle*> images;
	BOOST_FOREACH(KeyFrame& kf, map.kfs) {
		images.push_back(&kf.image);
	}

	// Learn the textons
	TextonVocab vocab;
	vocab.Compute(images);
	vocab.Save(GV3::get<string>("Textons.VocabFile");

	return 0;
}

