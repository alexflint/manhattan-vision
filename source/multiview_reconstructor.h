#include <boost/array.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include "common_types.h"
#include "camera.h"
#include "canvas.h"
#include "manhattan_dp.h"

#include "integral_col_image.tpp"

namespace indoor_context {

	// Draw an array of dots to visualize the given payoffs
	void DrawPayoffs(Canvas& canvas,
									 const boost::array<MatF,2>& payoffs,
									 const DPGeometry& geom);

	// Output a visualization of the given payoffs to the file specified
	void OutputPayoffsViz(const string& filename,
												const ImageRGB<byte>& orig,
												const boost::array<MatF,2>& payoffs,
												const DPGeometry& geom);

	////////////////////////////////////////////////////////////////////////////////
	class MonocularPayoffs {
	public:
		bool empty;
		DPGeometry geom;
		IntegralColImage<float> integ_scores[3];

		MonocularPayoffs() : empty(true) { }

		// Compute the integral images in preparation for calls to GetPayoff
		void Configure(const DPObjective& obj,
									 const DPGeometry& geom);

		// Get payoff for building a wall at a point in grid coordinates.
		// grid_pt can be outside the grid bounds, clamping will be applied appropriately
		double GetPayoff(const Vec2& grid_pt, int orient) const;

		// Resize the matrix to the size of the grid and fill it with all payoffs
		void GetAllPayoffs(boost::array<MatF,2>& payoffs) const;
	};


	////////////////////////////////////////////////////////////////////////////////
	class MultiViewPayoffs {
	public:
		struct AuxiliaryView {
			DPGeometry geom;  // DP geometry for base view
			Mat3 grid_hfloor;  // homography from base view to this one, via floor plan, in grid coords
			Mat3 grid_hceil;  // homography from base view to this one, via ceiling plan, in grid coords
			MonocularPayoffs payoff_gen;

			// These are only kept for analysis/visualization:
			boost::array<MatF,2> payoffs;  // Payoffs for this view
			boost::array<MatF,2> contrib_payoffs;  // Payoffs transferred into base view
			Mat3 image_hfloor;  // only for debugging...
			Mat3 image_hceil;
		};

		double zfloor, zceil;

		DPGeometry base_geom;  // DP geometry for base view
		MonocularPayoffs base_payoff_gen;  // The payoff generator
		boost::array<MatF,2> base_payoffs;  // Monocular payoffs for base view (only kept for visualization)

		boost::ptr_vector<AuxiliaryView> aux_views; // DP geometries for auxiliary views
	
		boost::array<MatF,2> payoffs;  // Node payoffs computed for all views jointly
	
		// Set up the base view
		void Configure(const DPGeometry& base_geom,
									 const DPObjective& base_obj,
									 double zfloor,
									 double zceil);
		// Add an auxiliary view
		void AddView(const DPGeometry& geom,
								 const DPObjective& payoffs);
		// Compute the joint payoffs
		void Compute();

		// Output corresponding points between base and aux view
		void OutputCorrespondences(const string& filename,
															 int index,
															 const ImageRGB<byte>& base,
															 const ImageRGB<byte>& aux);

	};





	////////////////////////////////////////////////////////////////////////////////
	class MultiViewReconstructor {
	public:
		double zfloor, zceil;

		// The base frame
		const PosedImage* base_frame;
		const DPObjective* base_objective;
		LineSweepDPScore base_gen;  // only kept for visualisation

		// The auxiliary frames
		vector<const PosedImage*> aux_frames;
		vector<const DPObjective*> aux_objectives;  // only kept for visualisation
		boost::ptr_vector<LineSweepDPScore> aux_gen;  // only kept for visualisation

		// The reconstructor
		MultiViewPayoffs joint_payoffs;
		ManhattanDPReconstructor recon;

		// Constructor
		MultiViewReconstructor();
		// Configure the reconstructor with a base frame (call this first)
		void Configure(const PosedImage& base_frame, double zfloor, double zceil);
		void Configure(const PosedImage& base_frame,
									 const DPObjective& base_obj,
									 double zfloor,
									 double zceil);
		// Add a frame (call this zero or more times between Configure() and Reconstruct())
		void AddFrame(const PosedImage& aux_frame);
		void AddFrame(const PosedImage& aux_frame, const DPObjective& aux_obj);
		// Compute payoffs for all frames and do the reconstructor (call this last)
		void Reconstruct();
		// Visualizations
		void OutputBasePayoffs(const string& filename);
		void OutputAuxPayoffs(const string& basename);
		void OutputJointPayoffs(const string& filename);
	};

}  // namespace indoor_context
