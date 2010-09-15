#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "map.h"
#include "vars.h"
#include "canvas.h"
#include "colors.h"

#include "math_utils.tpp"
#include "io_utils.tpp"
#include "quaternion.tpp"
#include "counted_foreach.tpp"

using namespace toon;
using namespace indoor_context;

// Convert PTAM maps to PMVS inputs
class PmvsFormat {
public:
	static void WriteMap(const Map& map, const fs::path& base_dir);
	static void CleanDir(const fs::path& dir);
	static void EnsureDirExists(const fs::path& dir);
	static void PrepareDir(const fs::path& dir);
};

// Remove contents of a given dir
void PmvsFormat::CleanDir(const fs::path& dir) {
	CHECK_PRED(fs::exists, dir);
	fs::directory_iterator end_it; // default construction yields past-the-end
  for (fs::directory_iterator it(dir); it != end_it; it++) {
    if (fs::is_regular_file(it->status())) {
			fs::remove(it->path());
    }
  }
}

// Create a dir if it doesn't exist already
void PmvsFormat::EnsureDirExists(const fs::path& dir) {
	if (fs::exists(dir)) {
		CHECK_PRED(fs::is_directory, dir) << "The path exists but is not a directory.";
	} else {
		CHECK_PRED(fs::create_directory, dir);
	}
}

// Create a dir if it doesn't exist, or delete its contents if it does exist
void PmvsFormat::PrepareDir(const fs::path& dir) {
	EnsureDirExists(dir);
	CleanDir(dir);
}

void PmvsFormat::WriteMap(const Map& map, const fs::path& base_dir) {
	fs::path image_dir = base_dir/"visualize";
	fs::path calib_dir = base_dir/"txt";
	//fs::path mask_dir = base_dir/"mask";
	fs::path model_dir = base_dir/"models";
	fs::path sba_dir = base_dir/"sba";
	fs::path debug_dir = base_dir/"debug";
	fs::path masks_dir = base_dir/"masks";

	EnsureDirExists(base_dir);
	PrepareDir(image_dir);
	PrepareDir(calib_dir);
	//PrepareDir(mask_dir);
	PrepareDir(sba_dir);
	PrepareDir(masks_dir);
	EnsureDirExists(model_dir);  // PMVS-2 will produce no output if this dir doesn't exist
	//PrepareDir(debug_dir);

	// Write the config file
	fs::path config_path = base_dir/"option.txt";
	ofstream config_out(config_path.string().c_str());
	config_out << "timages -1 0 " << map.kfs.size() << "\n";
	config_out << "oimages 0\nlevel 1\ncsize 2\nthreshold 0.7\n";
	config_out << "minImageNum 2\nCPU 4\nsequence 1\nmaxAngle 5\n";
	config_out.close();

	// Write the per-frame data
	int index = 0;
	BOOST_FOREACH(const KeyFrame& kf, map.kfs) {
		string id_string = PaddedInt(index, 8);
		// Write the camera matrix
		const Mat3& intr = kf.unwarped.retina_to_image;  // intrinsic
		Matrix<3,4> extr = as_matrix(kf.pc->pose()); // extrinsic
		Matrix<3,4> cam = intr * extr;
		fs::path calib_path = calib_dir/(id_string+".txt");
		ofstream calib_out(calib_path.string().c_str());

		calib_out << "CONTOUR\n" << cam;
		calib_out.close();

		// Write the images
		fs::path image_path = image_dir/(id_string+".jpg");
		WriteImage(image_path.string(), kf.unwarped.image.rgb);

		// Write the masks (always fully visible at the moment)
		/*ImageRGB<byte> mask(kf.rawimage.sz());
		mask.Fill(Colors::black());
		fs::path mask_path = mask_dir/(short_id_string+".pgm");
		WriteImage(mask_path.string(), mask);*/

		// Draw reprojected points for checking
		/*fs::path debug_path = debug_dir/(id_string+".png");
		FileCanvas canvas(debug_path.string(), kf.unwarped.image.rgb);
		BrightColors bc;
		BOOST_FOREACH(const Vec3& v, map.pts) {
			canvas.DrawDot(project(cam*unproject(v)), 2.0, bc.Next());
			}*/
		index++;
	}

	// Write the intrinsics in SBA format (apparently distortion is assumed zero in this case)
	// TODO: SBA might fail if the lower triangle in this matrix is not zero
	fs::path calib_path = sba_dir/"calib.txt";
	ofstream calib_out(calib_path.string().c_str());
	calib_out << map.kfs[0].unwarped.retina_to_image << endl;

	// Write the camera poses in SBA format
	fs::path cams_path = sba_dir/"ptam_cams.txt";
	ofstream cams_out(cams_path.string().c_str());
	BOOST_FOREACH(const KeyFrame& kf, map.kfs) {
		double error;
		Mat3 R = kf.pc->pose().get_rotation().get_matrix();
		Vec4 q = FitQtrn(R, error);
		cams_out << q << " " << kf.pc->pose().get_translation() << "\n";
	}

	// Write the observations so we can check them with reprerr.pl in sba
	fs::path pts_path = sba_dir/"ptam_pts.txt";
	ofstream pts_out(pts_path.string().c_str());
	vector<pair<int, Measurement> > observations[map.pts.size()];
	COUNTED_FOREACH(int i, const KeyFrame& kf, map.kfs) {
		BOOST_FOREACH(const Measurement& msm, kf.measurements) {
			observations[msm.point_index].push_back(make_pair(i, msm));
		}
	}
	for (int i = 0; i < map.pts.size(); i++) {
		// Skip points with no observations among the current set since this can confuse SBA
		if (!observations[i].empty()) {
			pts_out << map.pts[i] << " " << observations[i].size();
			for (int j = 0; j < observations[i].size(); j++) {
				const pair<int, Measurement>& msm = observations[i][j];
				pts_out << " " << msm.first << " " << msm.second.image_pos;
			}
			pts_out << "\n";
		}
	}
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 4 && argc != 3) {
		cerr << "Usage: ./map2pmvs map.xml [FRAMES] OUTDIR\n";
		exit(-1);
	}

	fs::path spec_file(argv[1]);
	fs::path out_dir(argv[argc-1]);

	vector<int> kf_ids;
	if (argc == 4) {
		ParseMultiRange(string(argv[2]), kf_ids);
	}

	Map map;
	map.LoadXml(spec_file.string());

	PmvsFormat::WriteMap(map, out_dir);
	return 0;
}
