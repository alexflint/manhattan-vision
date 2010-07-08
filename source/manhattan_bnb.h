#pragma once

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/function.hpp>

#include "common_types.h"
#include "ptam_include.h"
#include "line_detector.h"
#include "camera.h"
#include "progress_reporter.h"
#include "map.h"  // TODO: remove when debugging done

namespace indoor_context {

struct ManhattanEdge {
	toon::Vector<3> start, end, eqn;
	int axis;
	int id; // unique identifier for this edge
	ManhattanEdge();
	ManhattanEdge(const toon::Vector<3>& a, const toon::Vector<3>& b, int axis,
	              int id);
	bool Contains(const toon::Vector<3>& p) const;
};

struct ManhattanCorner {
	toon::Vector<3> div_eqn; // homogeneous line equation of this edge,
	// positive side is always to the left in the
	// image
	toon::Vector<3> left_ceil, left_floor, right_ceil, right_floor;
	int left_axis, right_axis;
	bool leftmost, rightmost;
	int occl_side; // -1 for left, 1 for right, 0 for no occlusion
	ManhattanCorner() :
		div_eqn(toon::Zeros), left_ceil(toon::Zeros), left_floor(toon::Zeros),
		        right_ceil(toon::Zeros), right_floor(toon::Zeros),
		        left_axis(-1), right_axis(-1), occl_side(0), leftmost(false),
		        rightmost(false) {
	}
};

struct ManhattanBuilding {
	typedef list<ManhattanCorner>::iterator CnrIt;
	typedef list<ManhattanCorner>::const_iterator ConstCnrIt;
	set<int> edge_ids; // edge IDs that generated this structure
	list<ManhattanCorner> cnrs; // cnrs of the building

	bool ContainsEdge(int id) const;
};

// Generates predictions from building hypotheses
class ManhattanEvaluator {
public:
	const PosedCamera* pc;	// the complete camera model
	int vert_axis, h1_axis, h2_axis;  // axis indices
	Vec3 horizon;  // the horizon line

	// Get ready for prediction
	void Configure(const PosedCamera& pcam, int vert_axis);

	// Predict the orientation map for a model. Will scale the
	// orientations to fit in the supplied matrix.
	void PredictOrientations(const ManhattanBuilding& bld, MatI& orients) const;
	void PredictGridOrientations(const ManhattanBuilding& bld, MatI& orients) const;
	void PredictImOrientations(const ManhattanBuilding& bld, MatI& orients) const;

	// Toggles between h1_axis and h2_axis
	int OtherHorizAxis(int a) const;

	////////////////////////////////////////////
	// Vizualization
	////////////////////////////////////////////

	void DrawBuilding(const ManhattanBuilding& bld, ImageRGB<byte>& canvas);
	void DrawPrediction(const ManhattanBuilding& bld, ImageRGB<byte>& canvas);
	void DrawOrientations(const MatI& orients, ImageRGB<byte>& canvas);

	// Output
	void OutputBuildingViz(const ImageRGB<byte>& image,
	                          const ManhattanBuilding& bld,
	                          const string& filename);
	void OutputPredictionViz(const ManhattanBuilding& bld,
	                         const string& filename);
	void OutputAllViz(const ImageRGB<byte>& image,
	                  const ManhattanBuilding& bld, const string& basename);

	// Output a text description of a model
	void WriteBuilding(const ManhattanBuilding& bld, const string& filename);
};

// Represents the branch and bound that explores the space of possible Manhattan buildings.
// We only use retina coordinates here
class ManhattanBranchAndBound {
public:
	int max_corners;	// input params, set to gvar values in constructor

	// Camera and pose parameters
	const PosedCamera* pc;	// the complete camera model
	double px_diam;  // diameter of pixels in the retina
	int vert_axis, h1_axis, h2_axis;  // axis indices
	Vec3 vpts[3], vert_vpt;  // the vanishing points
	Vec3 horizon;  // we always have horizon[1] > 0

	// Edges sorted according to their position/direction
	vector<const ManhattanEdge*> vert_edges;
	vector<const ManhattanEdge*> ceil_edges[3];
	vector<const ManhattanEdge*> floor_edges[3];

	// The set of starting hypotheses
	ptr_vector<ManhattanBuilding> init_hypotheses;

	// The current visit function that we call for each node
	function<void(const ManhattanBuilding&)> visitor;

	// Inirialize the reconstructor with the given pose and camera
	ManhattanBranchAndBound();

	// Must be called before Compute
	void Configure(const PosedCamera& pcam, int vert_axis);

	// Enumerate hypotheses, invoking visit(x) on each
	bool Compute(const vector<ManhattanEdge> edges[],
	             function<void(const ManhattanBuilding&)> visitor);

	// Initialize the list of buildings
	void Initialize(const vector<ManhattanEdge> edges[]);

	// Explore the search tree starting from the initial hypotheses
	void Enumerate();

	// Explore the search tree rooted at a given node
	void BranchFrom(const ManhattanBuilding& bld);

	// Generate all building corners reachable by adding a single
	// horizontal edge to the specified building
	int HorizBranchFrom(const ManhattanBuilding& bld);

	// Generate all building corners reachable by adding a single
	// vertical edge to the specified building
	int VertBranchFrom(const ManhattanBuilding& bld);

	// Add a corner to bld. Return true if the corner is valid
	bool AddCorner(ManhattanBuilding& bld, int edge_id,
	               const toon::Vector<3>& div_eqn, int new_axis,
	               bool update_left);

	// Returns either v or -v, whichver is a valid div_eqn (i.e. has v[0] > 0)
	toon::Vector<3> DivVec(const toon::Vector<3>& v);

	// Above/below horizon checks
	bool AboveHorizon(const toon::Vector<3>& v) const;
	bool BelowHorizon(const toon::Vector<3>& v) const;

	// Occlusion tests
	bool OcclusionValid(ManhattanCorner& cnr);
	bool UpdateOcclusion(ManhattanCorner& cnr);
	bool UpdateStripe(ManhattanCorner& left_cnr,
	                  ManhattanCorner& right_cnr,
	                  const toon::Vector<3>& floor_line,
	                  const toon::Vector<3>& ceil_line,
	                  int new_axis);

	// Determine the stripe containing the point p, set l_cnr and r_cnr
	// to the corners to the point's left and right respectively, and
	// return the index of the left corner
	int LocateStripe(const ManhattanBuilding& bld, const toon::Vector<3>& p,
	                 ManhattanBuilding::ConstCnrIt& l_cnr,
	                 ManhattanBuilding::ConstCnrIt& r_cnr) const;
	int LocateStripe(ManhattanBuilding& bld, const toon::Vector<3>& p,
	                 ManhattanBuilding::CnrIt& l_cnr,
	                 ManhattanBuilding::CnrIt& r_cnr);

	// Determine whether p is between left and right corners
	bool StripeContains(const toon::Vector<3>& left_div,
	                    const toon::Vector<3>& right_div,
	                    const toon::Vector<3>& p);
	bool StripeContains(const ManhattanCorner& left,
	                    const ManhattanCorner& right, const toon::Vector<3>& p);

	// Get the minimum distance from a point to the boundary of a stripe
	double GetStripeMargin(const ManhattanCorner& left,
	                       const ManhattanCorner& right,
	                       const toon::Vector<3>& p);
};


// Represents the single-image Manhattan recovery algorithm.
class MonocularManhattanBnb {
public:
	const PosedCamera* pc;	// the complete camera model
	int vert_axis;  // axis indices
	Vec3 horizon;  // the horizon line

	// The best hypothesis
	ManhattanBuilding soln;
	int soln_score;
	MatI soln_orients;

	// The estimated orientations (from line sweep etc)
	MatI est_orients;

	// The number of hypotheses enumerated so far
	int hypothesis_count;

	// The buffer used to compute predictions
	// (beware: this makes Compute() un-parallelizable!)
	mutable MatI predict_buffer;

	// The object that explores the search tree
	ManhattanBranchAndBound enumerator;

	// The object that maps models to their predicted pixel labellings
	ManhattanEvaluator renderer;

	// Do the reconstruction
	bool Compute(const PosedCamera& pcam,
	             const vector<ManhattanEdge> edges[],
	             const MatI& orients);

	// Score a model according to its agreement with an orientation estimate
	void EvaluateHypothesis(const ManhattanBuilding& bld);

	//////////////////////////////////////////////
	// 3D Reconstruction
	//////////////////////////////////////////////

	// Toggles between h1_axis and h2_axis
	int OtherHorizAxis(int a) const;
	// Project points into the world
	toon::Vector<3> FloorPoint(const toon::Vector<3>& p, double z);
	toon::Vector<3> CeilPoint(const toon::Vector<3>& p,
	                          const toon::Vector<3>& floor_pt);

	// Transfer a building between frames. PredictOrientations is a
	// special case of this for orig_pose = new_pose.
	void TransferBuilding(const ManhattanBuilding& bld,
	                      //const toon::SE3<>& orig_pose,
	                      const toon::SE3<>& new_pose,
	                      double floor_z,
	                      MatI& predicted);
};

}
