#include <map>

#include <boost/filesystem.hpp>

#include "common_types.h"
#include "safe_stream.h"
#include "map_io.h"
#include "tinyxml.h"
#include "vw_image_io.h"

#include "io_utils.tpp"
#include "protobuf_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	lazyvar<Vec5> gvDefaultCameraParams("Map.DefaultCameraParameters");
	lazyvar<Vec2> gvDefaultImageSize("Map.DefaultImageSize");
	lazyvar<int> gvLinearizeCamera("Map.LinearizeCamera");

	lazyvar<bool> gvLoadOriginalFrames("Map.LoadOriginalFrames");
	lazyvar<string> gvOrigFramesDir("Map.OrigFramesDir");

	void LoadBundlerMap(const string& bundle_dir, Map& map) {
		fs::path dir = fs::path(bundle_dir);
		string bundle_file = (dir / "bundle/bundle.out").string();
		string list_file = (dir / "list.txt").string();
		LoadBundlerMap(bundle_file, list_file, map);
	}

	void LoadBundlerMap(const string& structure_file, const string& image_list_file, Map& map) {
		CHECK_PRED1(fs::exists, structure_file);
		CHECK_PRED1(fs::exists, image_list_file);

		// Get the base path to help find images
		fs::path basedir = fs::path(image_list_file).parent_path();

		// Read list of images
		sifstream list_in(image_list_file);
		vector<fs::path> image_paths;
		int nonexistent = 0;
		while (true) {
			string image_file;
			getline(list_in, image_file);
			if (list_in.eof()) break;
			fs::path path = basedir / image_file;
			if (!fs::exists(path)) {
				DLOG << "Warning: image file not found: " << path;
			}
			image_paths.push_back(path);
		}

		// Load structure
		sifstream in(structure_file);

		// Ignore comment
		string comment;
		getline(in, comment);

		// Load number of cameras and points
		int num_cameras, num_points;
		in >> num_cameras >> num_points;

		CHECK(num_cameras > 0);
		CHECK_EQ(image_paths.size(), num_cameras)
			<< "Number of cameras should equal number of image files in " << image_list_file;

		// Load poses
		double focal_length;
		double f;
		Mat3 R;
		Vec3 t;
		Vec2 distortion;
		for (int i = 0; i < num_cameras; i++) {
			in >> f >> distortion >> R >> t;
			CHECK_EQ(distortion, makeVector(0,0)) << "Can't handle distorted cameras yet";
			if (i == 0) {
				focal_length = f;
				// Construct a linear camera
				Mat3 cam = Identity;
				cam[0][0] = cam[1][1] = focal_length;
				map.camera.reset(new LinearCamera(cam, asIR(*gvDefaultImageSize)));
			} else {
				CHECK_EQ(f, focal_length) << EXPR(i) << "Can't handle multiple focal lengths yet";
			}

			// Bundler considers cameras looking down the negative Z axis so
			// invert (this only really matters for visualizations and
			// visibility testing).
			R = -R;
			t = -t;

			Frame* frame = new Frame;
			string image_file = i < image_paths.size() ? image_paths[i].string() : "";
			frame->Configure(&map, i, image_file, SE3<>(SO3<>(R),t));
			frame->initializing = false;
			frame->lost = false;
			map.frames.push_back(frame);
		}


		// Load points
		// TODO: store observations
		Vec3 v, color;
		Vec2 obs_pt;
		int num_obs, obs_camera, obs_key;
		for (int i = 0; i < num_points; i++) {
			in >> v >> color >> num_obs;
			for (int j = 0; j < num_obs; j++) {
				in >> obs_camera >> obs_key >> obs_pt;
			}
			map.points.push_back(v);
		}
	}

	void LoadXmlMap(const string& path,
									Map& map,
									bool all_frames) {
		// Clear any old state
		map.frames.clear();
		map.points.clear();

		// Configure the camera
		map.orig_camera.reset(new ATANCamera(*gvDefaultCameraParams, *gvDefaultImageSize));
		if (*gvLinearizeCamera) {
			Mat3 cam = map.orig_camera->Linearize();
			map.camera.reset(new LinearCamera(cam, map.orig_camera->image_size()));
		} else {
			map.camera = map.orig_camera;
		}

		// Parse the XML file
		TiXmlDocument doc;
		CHECK_PRED1(doc.LoadFile, path.c_str()) << "Failed to load " << path;
		fs::path xml_dir(fs::path(path).parent_path());
		fs::path frames_dir(*gvOrigFramesDir);
		const TiXmlElement* root_elem = doc.RootElement();

		// Read the points
		std::map<int, int> points_id_to_index;
		const TiXmlElement* points_elem = root_elem->FirstChildElement("MapPoints");
		CHECK(points_elem) << "There was no <MapPoints> element in the map spec.";
		for (const TiXmlElement* pt_elem = points_elem->FirstChildElement("MapPoint");
				 pt_elem != NULL;
				 pt_elem = pt_elem->NextSiblingElement("MapPoint")) {
			int id = lexical_cast<int>(pt_elem->Attribute("id"));
			points_id_to_index[id] = map.points.size();  // indices start from zero
			map.points.push_back(stream_to<Vec3>(pt_elem->Attribute("position")));
		}

		// vector of frame IDs for which we fell back to B&W images
		vector<int> fallback_frames;

		// Read frame poses
		int next_id = 0;
		const TiXmlElement* frames_elem = root_elem->FirstChildElement("FramePoses");
		if (all_frames) {
			if (frames_elem != NULL) { // FramePoses is optional
				for (const TiXmlElement* frame_elem = frames_elem->FirstChildElement("Frame");
						 frame_elem != NULL;
						 frame_elem = frame_elem->NextSiblingElement("Frame")) {
					string image_file = (frames_dir/frame_elem->Attribute("name")).string();
					Vec6 lnPose = stream_to<Vec6>(frame_elem->Attribute("pose"));
					Frame* f = new Frame;  // will be owned by the ptr_vector
					f->Configure(&map, next_id++, image_file, SE3<>::exp(lnPose));
					f->initializing = norm_sq(lnPose) == 0;
					f->lost = frame_elem->Attribute("lost") == "1" || f->initializing;
					map.frames.push_back(f);
				}
			}
		} else {

			// Read key frames
			const TiXmlElement* kfs_elem = root_elem->FirstChildElement("KeyFrames");
			CHECK(kfs_elem) << "There was no <KeyFrames> element in the map spec.";

			// Create an ID-to-XMLElement map
			for (const TiXmlElement* kf_elem = kfs_elem->FirstChildElement("KeyFrame");
					 kf_elem != NULL;
					 kf_elem = kf_elem->NextSiblingElement("KeyFrame")) {
				// Get the id, pose, and hash
				int id = lexical_cast<int>(kf_elem->Attribute("id"));
				Vec6 lnPose = stream_to<Vec6>(kf_elem->Attribute("pose"));
				string hash = kf_elem->FirstChildElement("Image")->Attribute("md5");

				// Get the filename
				fs::path image_file;
				if (kf_elem->Attribute("name") != NULL) {
					image_file = kf_elem->Attribute("name");
				}
				if (image_file.empty() || !*gvLoadOriginalFrames) {
					fallback_frames.push_back(id);
					image_file = xml_dir/kf_elem->FirstChildElement("Image")->Attribute("file");
				} else {
					image_file = frames_dir/image_file;
				}

				// Add the keyframe
				Frame* frame = new Frame;
				frame->Configure(&map, id, image_file.string(), SE3<>::exp(lnPose));
				frame->lost = false;
				frame->initializing = false;
				map.AddFrame(frame);

				// Add the measurements
				const TiXmlElement* msms_elem = kf_elem->FirstChildElement("Measurements");
				for (const TiXmlElement* msm_elem = msms_elem->FirstChildElement("Measurement");
						 msm_elem != NULL;
						 msm_elem = msm_elem->NextSiblingElement("Measurement")) {
					Measurement msm;
					int point_id = lexical_cast<int>(msm_elem->Attribute("id"));
					std::map<int, int>::const_iterator it = points_id_to_index.find(point_id);
					if (it == points_id_to_index.end()) {
						DLOG << "No point with ID="<<point_id;
					} else {
						msm.point_index = it->second;
						msm.image_pos = stream_to<Vec2>(msm_elem->Attribute("v2RootPos"));
						msm.retina_pos = stream_to<Vec2>(msm_elem->Attribute("v2ImplanePos"));
						msm.pyramid_level = lexical_cast<int>(msm_elem->Attribute("nLevel"));
					}
					frame->measurements.push_back(msm);
				}
			}
		}

		DLOG << "Loaded " << map.points.size() << " points and " << map.frames.size() << " frames";
		if (fallback_frames.size() == 1) {
			DLOG << "Using grayscale image for frame " << fallback_frames[0];
		} else if (!fallback_frames.empty()) {
			DLOG << "Using grayscale image for frames " << iowrap(fallback_frames, ",");
		}
	}

	void LoadXmlMapWithGroundTruth(const string& path,
																 Map& map,
																 proto::TruthedMap& gt_map,
																 bool include_non_keyframes) {
		ReadProto(path, gt_map);
		LoadXmlMap(gt_map.spec_file(), map, include_non_keyframes);
		map.Transform(SO3<>::exp(asToon(gt_map.ln_scene_from_slam())));

		double zfloor = gt_map.floorplan().zfloor();
		double zceil = gt_map.floorplan().zceil();
		Vec3 vup = map.frames[0].image.pc().pose_inverse() * makeVector(0,1,0);
		if (Sign(zceil-zfloor) == Sign(vup[2])) {
			swap(zfloor, zceil);
			gt_map.mutable_floorplan()->set_zfloor(zfloor);
			gt_map.mutable_floorplan()->set_zceil(zceil);
		}
	}
}
