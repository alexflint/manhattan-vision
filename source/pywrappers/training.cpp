#include "training.h"

#include <stdlib.h>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/filesystem.hpp>

#include "entrypoint_types.h"
#include "timer.h"
#include "map.h"
#include "map_io.h"
#include "map.pb.h"
#include "payoff_helpers.h"
#include "manhattan_dp.h"
#include "manhattan_ground_truth.h"
#include "geom_utils.h"
#include "building_features.h"

#include "io_utils.tpp"
#include "format_utils.tpp"

namespace indoor_context {
	void PredictGridLabels(const ManhattanHypothesis& hyp,
												 const DPGeometry& geometry,
												 MatI& grid_orientations) {
		int nx = geometry.grid_size[0];
		int ny = geometry.grid_size[1];
		grid_orientations.Resize(ny, nx);
		grid_orientations.Fill(kVerticalAxis);
		for (int i = 0; i < nx; i++) {
			float path_y = hyp.path_ys[i];
			float opp_y = geometry.Transfer(makeVector(i, path_y))[1];
			int ceil_y = roundi(min(path_y, opp_y));
			int floor_y = roundi(max(path_y, opp_y));
			CHECK_INTERVAL(hyp.path_axes[i], 0, 2) << "i="<<i;
			for (int y = max(ceil_y,0); y <= min(floor_y,ny-1); y++) {
				grid_orientations[y][i] = hyp.path_axes[i];
			}
		}
	}

	void PredictGridDepths(const ManhattanHypothesis& hyp,
												 const DPGeometryWithScale& geometry,
												 MatF& grid_depths) {
		int nx = geometry.grid_size[0];
		int ny = geometry.grid_size[1];
		grid_depths.Resize(ny, nx);

		// Compute some geometric entities
		Matrix<3,4> cam = geometry.imageToGrid * geometry.camera->Linearize();
		Vec3 princ = unit(geometry.camera->GetPrincipalDirection());
		Vec3 normal = princ ^ GetAxis<3>(2);
		Vec4 ceil_plane = makeVector(0, 0, 1, -geometry.zceil);
		Vec4 floor_plane = makeVector(0, 0, 1, -geometry.zfloor);
		Vec3 ceil_deqn = PlaneToDepthEqn(cam, ceil_plane);
		Vec3 floor_deqn = PlaneToDepthEqn(cam, floor_plane);

		// Compute losses
		for (int x = 0; x < nx; x++) {
			int y = hyp.path_ys[x];
			float opp_y = geometry.Transfer(makeVector<double>(x,y))[1];
			float ceil_y = min(y, opp_y);
			float floor_y = max(y, opp_y);
			Vec3 base = geometry.BackProjectFromGrid(makeVector<double>(x,y));
			Vec4 plane = concat(normal, -normal*base);
			Vec3 wall_deqn = PlaneToDepthEqn(cam, plane);
			for (int yy = 0; yy < ny; yy++) {
				double depth;
				if (yy < ceil_y) {
					depth = EvaluateDepthEqn(ceil_deqn, makeVector<double>(x,yy));
				} else if (yy < floor_y) {
					depth = EvaluateDepthEqn(wall_deqn, makeVector<double>(x,yy));
				} else {
					depth = EvaluateDepthEqn(floor_deqn, makeVector<double>(x,yy));
				}
				grid_depths[yy][x] = depth;
			}
		}
	}

	ManhattanHypothesis::ManhattanHypothesis() : instance(NULL) {
	}

	ManhattanHypothesis::ManhattanHypothesis(const VecI& ys,
																					 const VecI& axes,
																					 int ncorners,
																					 int nocclusions)
		: path_ys(ys),
			path_axes(axes),
			num_corners(ncorners),
			num_occlusions(nocclusions),
			instance(NULL) {
	}

	void TrainingInstance::Configure(const Frame& f, const proto::FloorPlan& fp) {
		frame = &f;
		geometry.Configure(f.image.pc(), fp.zfloor(), fp.zceil());

		// Compute ground truth
		gt.Compute(fp, f.image.pc());
		gt.ComputePathAndAxes(geometry, gt_hypothesis.path_ys, gt_hypothesis.path_axes);
		gt_hypothesis.num_corners = gt.num_walls();
		gt_hypothesis.num_occlusions = gt.num_occlusions();
		gt_hypothesis.instance = this;
		ConfigureL1Loss();
	}

	void TrainingInstance::ConfigureL1Loss(double kcondition) {
		gt.ComputeL1LossTerms(geometry, loss_terms);

		if (kcondition == 0.) {
			kcondition = frame->image.ny()/10;
		}
		loss_terms /= kcondition;
	}

	void TrainingInstance::ConfigureLabellingLoss(double kcondition) {
		gt.ComputeLabellingLossTerms(geometry, loss_terms);

		if (kcondition == 0.) {
			kcondition = frame->image.ny()/10;
		}
		loss_terms /= kcondition;
	}

	void TrainingInstance::ConfigureDepthLoss(double kcondition) {
		TIMED("Computing depth losses")
			gt.ComputeDepthLossTerms(geometry, loss_terms);

		if (kcondition == 0.) {
			kcondition = frame->image.ny()*10;
		}
		loss_terms /= kcondition;
	}

	MatI TrainingInstance::ComputeLabels(const ManhattanHypothesis& hyp) const {
		MatI orients;
		PredictGridLabels(hyp, geometry, orients);
		return orients;
	}

	MatF TrainingInstance::ComputeDepths(const ManhattanHypothesis& hyp) const {
		MatF depths;
		PredictGridDepths(hyp, geometry, depths);
		return depths;
	}

	ManhattanHypothesis TrainingInstance::GetGroundTruth() const {
		return gt_hypothesis;
	}

	MatI TrainingInstance::GetGroundTruthLabels() const {
		return gt.orientations();
	}

	MatD TrainingInstance::GetGroundTruthDepths() const {
		return gt.depthmap();
	}

	MatF TrainingInstance::GetLossTerms() const {
		return loss_terms;
	}

	// Draw ground truth as labels
	void TrainingInstance::OutputGroundTruthViz(const string& path) const {
		gt.OutputOrientations(frame->image.rgb, path);
	}

	double TrainingInstance::ComputeLoss(const ManhattanHypothesis& hyp) const {
		CHECK(loss_terms.Rows() != 0) << "TrainingInstance must be configured";
		CHECK_EQ(hyp.path_ys.Size(), gt_hypothesis.path_ys.Size());
		CHECK_EQ(hyp.path_ys.Size(), loss_terms.Cols());
		double loss = 0.;
		for (int i = 0; i < hyp.path_ys.Size(); i++) {
			CHECK_INTERVAL(hyp.path_ys[i], 0, loss_terms.Rows()-1) << EXPR(hyp.path_ys);
			loss += loss_terms[ hyp.path_ys[i] ][ i ];
		}
		return loss;
	}

	int TrainingManager::NumInstances() const {
		return instances.size();
	}

	TrainingInstance& TrainingManager::GetInstance(int i) {
		CHECK_INDEX(i, instances);
		return instances[i];
	}

	void TrainingManager::LoadSequence(const string& sequence,
																		 vector<int> frame_ids) {
		Map* map = NULL;
		proto::TruthedMap* gt_map = NULL;

		if (maps_by_sequence.find(sequence) == maps_by_sequence.end()) {
			map = new Map;
			gt_map = new proto::TruthedMap;
			LoadXmlMapWithGroundTruth(GetMapPath(sequence), *map, *gt_map);

			maps.push_back(map);
			gt_maps.push_back(gt_map);
			maps_by_sequence[sequence] = map;
			gt_maps_by_sequence[sequence] = gt_map;

		} else {
			map = maps_by_sequence[sequence];
			gt_map = gt_maps_by_sequence[sequence];
		}

		BOOST_FOREACH(int id, frame_ids) {
			Frame* frame = map->GetFrameByIdOrDie(id);
			frame->LoadImage();
			TrainingInstance* instance = new TrainingInstance();
			instance->Configure(*frame, gt_map->floorplan());
			instance->sequence = sequence;
			instances.push_back(instance);
		}
	}

	void TrainingManager::NormalizeFeatures(FeatureManager& fmgr) {
		CHECK(!instances.empty());
		int nf = fmgr.NumFeatures();
		VecD sums(nf, 0.);
		VecD sum_sqrs(nf, 0.);
		VecI norms(nf, 0);

		// Accumulate statistics
		BOOST_FOREACH(TrainingInstance& instance, instances) {
			fmgr.LoadFeaturesFor(instance);
			CHECK_EQ(fmgr.feature_set.features.size(), nf);
			for (int f = 0; f < nf; f++) {
				for (int k = 0; k < 2; k++) {
					const MatF& F = fmgr.feature_set.features[f].wall_scores[k];
					if (F.Rows() > 0 && F.Cols() > 0) {
						sums[f] += F.Mean() * F.Rows() * F.Cols();
						sum_sqrs[f] += F.FroNorm() * F.FroNorm();
						norms[f] += F.Rows() * F.Cols();
					}
				}
			}
		}

		// Normalize features
		BOOST_FOREACH(TrainingInstance& instance, instances) {
			fmgr.LoadFeaturesFor(instance);
			for (int f = 0; f < nf; f++) {
				double mean = sums[f] / norms[f];
				double stddev = sqrt(sum_sqrs[f]/norms[f] - mean*mean);
				for (int k = 0; k < 2; k++) {
					fmgr.feature_set.features[f].wall_scores[k] -= mean;
					fmgr.feature_set.features[f].wall_scores[k] /= stddev;
				}
			}
			fmgr.CommitFeatures();
		}
	}

	ManhattanHypothesis ManhattanInference::Solve(const TrainingInstance& instance,
																								const DPPayoffs& payoffs) {
		last_instance = &instance;
		WITHOUT_DLOG
			reconstructor.Compute(instance.frame->image, instance.geometry, payoffs);
		const DPSolution& soln = reconstructor.dp.solution;
		return ManhattanHypothesis(soln.path_ys,
															 soln.path_axes,
															 soln.num_walls,
															 soln.num_occlusions);
	}

	double ManhattanInference::GetSolutionScore() const {
		CHECK_NOT_NULL(last_instance);
		CHECK(reconstructor.dp.solution.path_ys.Size() > 0);
		return reconstructor.dp.solution.score;
	}

	MatI ManhattanInference::GetSolutionLabels() const {
		CHECK_NOT_NULL(last_instance);
		CHECK(reconstructor.dp.solution.path_ys.Size() > 0);
		return reconstructor.dp.solution.pixel_orients;
	}

	MatD ManhattanInference::GetSolutionDepths() {
		CHECK_NOT_NULL(last_instance);
		CHECK(reconstructor.dp.solution.path_ys.Size() > 0);
		return reconstructor.dp.solution.GetDepthMap(last_instance->geometry);
	}

	void ManhattanInference::OutputSolutionViz(const string& path) const {
		CHECK_NOT_NULL(last_instance);
		CHECK(reconstructor.dp.solution.path_ys.Size() > 0);
		reconstructor.OutputSolutionViz(path);
	}

	FeatureManager::FeatureManager(const string& dir)
		: feature_dir(dir) {
		CHECK(fs::exists(dir));
	}

	string FeatureManager::GetPathFor(const TrainingInstance& instance) {
		boost::format pat("%s_frame%03d_features.protodata");
		string filename = str(pat % instance.sequence % instance.frame->id);
		return (fs::path(feature_dir) / filename).string();
	}

	int FeatureManager::NumFeatures() const {
		return feature_set.features.size();
	}

	void FeatureManager::ComputeMultiViewFeatures(const TrainingInstance& instance,
																								const vector<int>& stereo_offsets) {
		DLOG << "Computing multi--view features for frame "
				 << instance.sequence << ":" << instance.frame->id;

		last_instance = &instance;
		feature_set.Clear();

		const PosedImage& image = instance.frame->image;
			
		// Compute monocular features
		objective_gen.Compute(image);
		mono_gen.Compute(objective_gen.objective, instance.geometry);
		feature_set.AddCopy(mono_gen.payoffs, "Line sweeps");

		// Compute 3D features
		vector<Vec3> point_cloud;
		instance.frame->GetMeasuredPoints(point_cloud);
		point_cloud_gen.Compute(point_cloud, image.pc(), instance.geometry);
		feature_set.AddCopy(point_cloud_gen.agreement_payoffs,
												"Point cloud (ON)");
		feature_set.AddCopy(point_cloud_gen.occlusion_payoffs,
												"Point cloud (IN)");

		// Compute stereo features
		BOOST_FOREACH(int offset, stereo_offsets) {
			int aux_id = instance.frame->id + offset;
			Frame* aux_frame = instance.frame->map->GetFrameById(aux_id);
			while (aux_frame == NULL) {
				if (aux_id < 0) {
					aux_id++;
				} else {
					aux_id--;
				}
				aux_frame = instance.frame->map->GetFrameById(aux_id);
			}
					
			aux_frame->LoadImage();
			stereo_gen.Compute(image, aux_frame->image, instance.geometry);
			feature_set.AddCopy(stereo_gen.payoffs, fmt("Stereo (offset %+d)",offset));
			// the plus sign above forces a sign character to be included
		}
	}

	void FeatureManager::ComputeSweepFeatures(const TrainingInstance& instance) {
		DLOG << "Computing sweep features for frame "
				 << instance.sequence << ":" << instance.frame->id;

		last_instance = &instance;
		feature_set.Clear();

		const PosedImage& image = instance.frame->image;
			
		// Compute monocular features
		objective_gen.Compute(image);
		mono_gen.Compute(objective_gen.objective, instance.geometry);
		feature_set.AddCopy(mono_gen.payoffs, "Line sweeps");
	}

	void FeatureManager::ComputeMonoFeatures(const TrainingInstance& instance,
																					 const string& spec) {
		// This is allocated here because otherwise all features are kept
		// until the end. TODO: implement PhotometricFeatures::Clear()
		PhotometricFeatures photom_ftr_gen;

		DLOG << "Computing single-view features for frame "
				 << instance.sequence << ":" << instance.frame->id;

		feature_set.Clear();
		last_instance = &instance;

		// Compute features
		photom_ftr_gen.Configure(spec);
		photom_ftr_gen.Compute(instance.frame->image);

		// Convert each to payoffs
		for (int i = 0; i < photom_ftr_gen.features.size(); i++) {
			payoff_gen.Compute(*photom_ftr_gen.features[i], instance.geometry);
			feature_set.AddCopy(payoff_gen.vpayoffs[0],
													fmt("%s [left]", photom_ftr_gen.feature_strings[i]));
			feature_set.AddCopy(payoff_gen.vpayoffs[1],
													fmt("%s [right]", photom_ftr_gen.feature_strings[i]));
			feature_set.AddCopy(payoff_gen.hpayoffs,
													fmt("%s [floor/ceil]", photom_ftr_gen.feature_strings[i]));
		}
	}

	void FeatureManager::ComputeMockFeatures(const TrainingInstance& instance) {
		DLOG << "Computing mock features for frame " << instance.frame->id;
		
		feature_set.Clear();
		last_instance = &instance;

		int ny = instance.geometry.ny();
		int nx = instance.geometry.nx();
		MatF pos(ny, nx);
		MatF neg(ny, nx);
		MatF ran(ny, nx);

		pos.Fill(0.);
		neg.Fill(0.);
		CHECK_EQ(instance.gt_hypothesis.path_ys.Size(), nx);
		CHECK_EQ(instance.gt_hypothesis.path_axes.Size(), nx);
		for (int i = 0; i < nx; i++) {
			CHECK_INTERVAL( instance.gt_hypothesis.path_ys[i], 0, ny-1 );
			pos[ instance.gt_hypothesis.path_ys[i] ][ i ] = 1.;
			neg[ instance.gt_hypothesis.path_ys[i] ][ i ] = -2.;
		}
		srand(instance.frame->id);  // repeatable *per frame*
		for (int i = 0; i < ny; i++) {
			for (int j = 0; j < nx; j++) {
				ran[i][j] = 1. * rand() / RAND_MAX;
			}
		}
		feature_set.AddCopy(pos, "positive");
		feature_set.AddCopy(neg, "negative");
		feature_set.AddCopy(ran, "random");
	}

	void FeatureManager::CommitFeatures() {
		WriteFeatures(GetPathFor(*last_instance), feature_set);
	}

	void FeatureManager::LoadFeaturesFor(const TrainingInstance& instance) {
		if (&instance != last_instance) {
			string path = GetPathFor(instance);
			CHECK(fs::exists(path)) << "Looking for features at " << path;
			TIMED("Loading features") ReadFeatures(path, feature_set);
			last_instance = &instance;
		}
	}

	void FeatureManager::Compile(const ManhattanHyperParameters& params,
															 DPPayoffs& payoffs) {
		feature_set.Compile(params, payoffs);
	}

	void FeatureManager::CompileWithLoss(const ManhattanHyperParameters& params,
																			 const TrainingInstance& instance,
																			 DPPayoffs& payoffs) {
		Compile(params, payoffs);
		payoffs.Add(instance.loss_terms);
	}

	MatF FeatureManager::GetFeature(int i, int orient) const {
		CHECK_INDEX(i, feature_set.features);
		return feature_set.features[i].wall_scores[orient];
	}

	const string& FeatureManager::GetFeatureComment(int i) const {
		CHECK_INDEX(i, feature_set.descriptions);
		return feature_set.descriptions[i];
	}

	VecD FeatureManager::ComputeFeatureForHypothesis(const ManhattanHypothesis& hyp) const {
		int nf = feature_set.features.size();
		VecD ftr(nf+2, 0);
		for (int f = 0; f < nf; f++) {
			for (int i = 0; i < hyp.path_ys.size(); i++) {
				int a = hyp.path_axes[i];
				int y = hyp.path_ys[i];
				ftr[f] += feature_set.features[f].wall_scores[a][y][i];
			}
		}
		// These two are negative since extra walls are *penalised*: important!
		ftr[nf] = -hyp.num_corners;
		ftr[nf+1] = -hyp.num_occlusions;
		return ftr;
	}
}
