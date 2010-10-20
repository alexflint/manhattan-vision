#include "camera.h"
#include "manhattan_dp.h"

namespace indoor_context {
	// Represents five statistics of two datasets from which NCC can be
	// computed.
	class NCCStatistics {
	public:
		double sum_a, sum_b, sum_asqr, sum_bsqr, sum_ab, sum_wts;

		NCCStatistics()
			: sum_a(0), sum_b(0), sum_asqr(0), sum_bsqr(0), sum_ab(0), sum_wts(0) { }

		// Add a measurement
		// If a weight is provided then this measurement contributes as if
		// multiple copies measurements of this kind were made. That is
		// _not_ the same as multiplying A and B by the weight!
		inline void Add(double a, double b, double weight=1.0) {
			sum_a += weight*a;
			sum_b += weight*b;
			sum_asqr += weight*a*a;
			sum_bsqr += weight*b*b;
			sum_ab += weight*a*b;
			sum_wts += weight;
		}

		// Compute the normalized cross-correlation for the current statistics
		double CalculateNCC();

		// Compute the normalized cross-correlation between two vectors given
		// five statistics:
		//   sum_a =  sum(a_i)
		//   sum_b =  sum(b_i)
		//   sum_asqr = sum(a_i*a_i)
		//   sum_bsqr = sum(b_i*b_i)
		//   sum_ab =  sum(a_i*b_i)
		//   n = length of vectors a and b
		// Returns unit(a-mean(a)) * unit(b-mean(b))
		//   where unit vectors are computed w.r.t. euclidean 2-norm
		// Special case for n=0: return 0
		// Special case for n=1: return 1
		// Special case for sum_xsqr==sum_x*sum_x/n: return 0
		//   {x is a or b, happens only when all x_i in original dataset are equal}
		static double CalculateNCC(double sum_a,
															 double sum_b,
															 double sum_asqr,
															 double sum_bsqr,
															 double sum_ab,
															 double nsamples);
	};
	
	// Output operator for NCCStatistics;
	ostream& operator<<(ostream& o, const NCCStatistics& stats);





	// Pre-computes statistics in order to calculate NCC between two
	// images for any segment of any image column in O(1) time.
	class FastNCC {
	public:
		// These must use doubles not floats because the round-off errors
		// for values above ~1e+7 when using floats can cause the
		// denominator to be *much* less than zero in certain cases. This
		// is to do with the cumulative sums held inside the integral
		// images; the inputs can safely remain as floats.
		IntegralColImage<double> intg_a;
		IntegralColImage<double> intg_b;	
		IntegralColImage<double> intg_asqr;
		IntegralColImage<double> intg_bsqr;
		IntegralColImage<double> intg_ab;
		IntegralColImage<int> intg_nsamples;

		// These buffers are used internally; they are member variables so
		// that they can be re-used without re-allocation.
		MatF asqr, bsqr, ab;
		MatI nsamples;

		// Prepare integral images for fast computation of NCC between A and B.
		void Compute(const MatF& a, const MatF& b);

		// Prepare integral images for fast computation of NCC between A and
		// B ignoring any positions for which mask[pos]=0.
		void Compute(const MatF& a, const MatF& b, const MatI& mask);

		// Prepare integral images for fast computation of NCC between A and
		// B. This overload allows explicit computation of asqr, bsqr, ab,
		// and nsamples, which can be used for efficiency or when there are
		// multiple observations per cell.
		void Compute(const MatF& a,
								 const MatF& b,
								 const MatF& asqr,
								 const MatF& bsqr,
								 const MatF& ab,
								 const MatI& nsamples);

		// Prepare integral images for fast computation of NCC between A and
		// B. If mask is not null then ignore any positions for which
		// (*mask)[pos]==0
		void ComputeInternal(const MatF& a,
												 const MatF& b,
												 const MatI* mask);

		// Calculate the NCC for the segment [row0,row1) of the specified column
		double CalculateNCC(int col, int row0, int row1);

		// Calculate NCC statistics for the segment [row0,row1) of the specified column
		void AddStats(int col, int row0, int row1, NCCStatistics& stats);
	};





	// Provides fast NCC computation between two images that are related
	// via a homography.
	class HomographyNCC {
	public:
		MatF left_xfered;
		MatF right_xfered;
		MatI mask;

		FastNCC ncc;

		// Computes NCC stats in a third coordinate frame related to the two
		// images by seperate homographies (one of which could be the
		// identity). The two homographies transfer from points in the
		// output coordinate frame to points in the respective input images.
		void Compute(const ImageF& left,
								 const ImageF& right,
								 const Mat3& left_xfer,
								 const Mat3& right_xfer,
								 const Vec2I& bounds);

		// Defers to FastNCC::AddStats
		void AddStats(int col, int row0, int row1, NCCStatistics& stats);

		// Defers to FastNCC::CalculateNCC
		double CalculateNCC(int col, int row0, int row1);

		// Visualization
		void OutputTransferredLeft(const string& file);
		void OutputTransferredRight(const string& file);
	};






	// Computes payoffs for DP reconstruction from stereo photoconsistency
	class StereoPayoffs {
	public:
		const PosedImage* l_input;
		const PosedImage* r_input;
		const DPGeometry* geometry;

		// The left vertical-rectification homography
		Mat3 l_vrect;
		Mat3 l_vrect_inv;

		// NCC integral images
		HomographyNCC floor_ncc;
		HomographyNCC ceil_ncc;

		// Transposed and rectified intensity images
		MatF l_vrect_im_tr;
		MatF r_vrect_im_tr;
	
		// The final payoff matrix
		boost::array<MatF,2> payoffs;

		// Compute Payoffs
		void Compute(const PosedImage& left_image,
								 const PosedImage& right_image,
								 const DPGeometry& geom,
								 double zfloor,
								 double zceil);

		// Visualization
		void OutputRectifiedLeft(const string& file);
		void OutputTransferredRight(const string& file);
		void OutputMask(const string& file);
		void OutputPayoffs(const string& file);
		void OutputRawPayoffs(const string& file);
	};




}  // namespace indoor_context
