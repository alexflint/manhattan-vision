#include "vanishing_point_model.h"

#include <TooN/so3.h>

#include "common_types.h"
#include "canvas.h"
#include "colors.h"
#include "clipping.h"
#include "line_segment.h"
#include "numeric_utils.h"
#include "geom_utils.h"
#include "vpt_utils.h"
#include "camera.h"

#include "canvas.h"
#include "drawing.h"

#include "vector_utils.tpp"
#include "format_utils.tpp"
#include "numerical_jacobian.tpp"

namespace indoor_context {
	using namespace toon;

	// The prior probability of a spurious detection. 
	static const double kSpuriousPrior = .2;

	// The probability of a specific line detection given that it is a
	// spurious detection. Technically, this should equal 1/(num possible
	// line detections) since all spurious detections are equally probable
	// and they should sum to 1.
	static const double kSpuriousLik = 1e-6;

	// Variance of the reprojection error, in pixels.
	// Recall that in the normal distribution the variance is sigma *squared*
	static const double kLineSigmaSqr = 6.;

	// Parameters for the rotation optimizer
	const int kMaxIters = 500;
	const double kMinStepNorm = 1e-10;
	const double kMinImprovement = 1e-8;

	// Helper for numerical gradients via boost::bind
	SO3<> multiply_exp(const SO3<>& R, const Vec3& m) {
		return R * SO3<>::exp(m);
	}

	/////////////////////////////////////////////////////////////////////////
	double VanishingPointModel::ComputeSegmentLogLik(const LineSeg& seg,
																									 const Vec3& vpt) const {
		double dist = ComputeReprojError(seg, vpt);
		return LogGauss1D(dist, 0, line_sigma_sqr);
	}

	double VanishingPointModel::ComputeObservationLogLik(const LineObservation& obs,
																											 int label,
																											 const SO3<>& R) const {
		CHECK_INTERVAL(label, -1, 2);
		if (label == -1) {
			return log(kSpuriousPrior) + log(kSpuriousLik);
		} else {
			Vec3 vpt = ProjectVanishingPoint(label, R, *obs.camera);
			double vpt_prior_prob = (1. - kSpuriousPrior) / 3.;
			return log(vpt_prior_prob) + ComputeSegmentLogLik(obs.segment, vpt);
		}
	}

	double VanishingPointModel::ComputeCompleteLogLik(const CompleteData& data,
																										const SO3<>& R) const {
		double loglik = 0.;
		for (int i = 0; i < data.size(); i++) {
			loglik += ComputeObservationLogLik(data[i].first,
																				 data[i].second,
																				 R);
		}
		return loglik;
	}

	double VanishingPointModel::ComputeExpectedLogLik(const vector<LineObservation>& data,
																										const MatD& responsibilities,
																										const SO3<>& R) const {			
		CHECK_EQ(responsibilities.Rows(), data.size());
		CHECK_EQ(responsibilities.Cols(), 4);

		double loglik = 0.;
		for (int i = 0; i < data.size(); i++) {
			for (int k = 0; k < 4; k++) {
				double cll = ComputeObservationLogLik(data[i], (k==3 ? -1 : k), R);
				loglik += responsibilities[i][k] * cll;
			}
		}
		return loglik;		
	}

	////////////////////////////////////////////////////////////////////////
	// Posteriors

	void VanishingPointModel::ComputePosteriorOnLabels(const vector<LineObservation>& data,
																										 const toon::SO3<>& R,
																										 MatD& resps) const {
		CHECK_EQ(resps.Rows(), data.size());
		CHECK_EQ(resps.Cols(), 4);

		for (int i = 0; i < data.size(); i++) {
			//TITLE("Observation " << i);
			double log_resps[4];
			// Compute log probability of each label (there are 4 incl. spurious)
			for (int j = 0; j < 4; j++) {
				int label = j == 3 ? -1 : j;
				log_resps[j] = ComputeObservationLogLik(data[i], label, R);
			}

			// Normalize and compute posterior (in a numerically stable way)
			double denom = LogSumExp(log_resps, 4);
			for (int j = 0; j < 4; j++) {
				resps[i][j] = exp(log_resps[j] - denom);
			}
		}
	}


	////////////////////////////////////////////////////////////////////////
	// Gradients

	Vec3 VanishingPointModel::ComputeDistGradient(const LineSeg& seg,
																								int axis,
																								const PosedCamera& camera,
																								const SO3<>& Rcur) const {
		CHECK_INTERVAL(axis, 0, 2);

		Vec3 vpt = ProjectVanishingPoint(axis, Rcur, camera);
		Mat3 Hcam = camera.camera().Linearize();
		SO3<> Rcam = camera.pose().get_rotation();

		Vec3 a = seg.start;
		Vec3 midp = seg.midpoint();
		Vec3 m = vpt ^ seg.midpoint();

		Vec3 e0 = GetAxis<3>(0);
		Vec3 e1 = GetAxis<3>(1);
		double eta = sqrt(m[0]*m[0] + m[1]*m[1]);
		Vec3 term1 = ((m[0]*e0 + m[1]*e1) / (eta*eta*eta)) * (m*a);
		Vec3 term2 = a/eta;

		Mat3 skew = cross_product_matrix(midp);

		Mat3 G_R;
		for (int i = 0; i < 3; i++) {
			G_R.T()[i] = Hcam * Rcam * Rcur * SO3<>::generator_field(i, GetAxis<3>(axis));
		}

		return (1. / a[2]) * (term1 - term2) * skew * G_R;
	}

	Vec3 VanishingPointModel::ComputeSegmentLogLikGradient(const LineSeg& seg,
																												 int axis,
																												 const PosedCamera& camera,
																												 const SO3<>& Rcur) const {
		double dist = SignedReprojError(seg, ProjectVanishingPoint(axis, Rcur, camera));
		Vec3 J_dist_wrt_R = ComputeDistGradient(seg, axis, camera, Rcur);
		return -dist * J_dist_wrt_R / line_sigma_sqr;
	}

	Vec3 VanishingPointModel::ComputeObservationLogLikGradient(const LineObservation& obs,
																														 int axis,
																														 const SO3<>& Rcur) const {
		CHECK_INTERVAL(axis, 0, 3);
		return ComputeSegmentLogLikGradient(obs.segment, axis, *obs.camera, Rcur);
	}

	Vec3 VanishingPointModel::ComputeCompleteLogLikGradient(const CompleteData& data,
																													const SO3<>& Rcur) const {
		Vec3 G = Zeros;
		for (int i = 0; i < data.size(); i++) {
			G += ComputeObservationLogLikGradient(data[i].first,
																						data[i].second,
																						Rcur);
		}
		return G;
	}

	Vec3 VanishingPointModel::ComputeExpectedLogLikGradient(const vector<LineObservation>& data,
																													const MatD& responsibilities,
																													const SO3<>& Rcur) const {
		CHECK_EQ(responsibilities.Rows(), data.size());
		CHECK_EQ(responsibilities.Cols(), 4);
		Vec3 G = Zeros;
		for (int i = 0; i < data.size(); i++) {
			// We omit k=3 here because those terms all contribute 0 to the gradient
			for (int k = 0; k < 3/*yes, this should be 3 not 4*/; k++) {
				Vec3 grad = ComputeObservationLogLikGradient(data[i], k, Rcur);
				G += responsibilities[i][k] * grad;
			}
		}
		return G;
	}

	Vec3 VanishingPointModel::ComputeCompleteLogLik_NumericGradient(const CompleteData& data,
																																	const SO3<>& Rcur,
																																	double delta) const {
		boost::function<double(const Vec3&)> f = 
			boost::bind(&VanishingPointModel::ComputeCompleteLogLik,
									this,
									ref(data),
									boost::bind(&multiply_exp,
															Rcur,
															_1));
		Vec3 zero = Zeros;
		return NumericJacobian(f, zero, delta);
	}


	Vec3 VanishingPointModel::
	ComputeExpectedLogLik_NumericGradient(const vector<LineObservation>& data,
																				const MatD& responsibilities,
																				const SO3<>& Rcur,
																				double delta) const {
		CHECK_EQ(responsibilities.Rows(), data.size());
		CHECK_EQ(responsibilities.Cols(), 4);

		boost::function<double(const Vec3&)> f = 
			boost::bind(&VanishingPointModel::ComputeExpectedLogLik,
									this,
									ref(data),
									ref(responsibilities),
									boost::bind(&multiply_exp,
															Rcur,
															_1));
		Vec3 zero = Zeros;
		return NumericJacobian(f, zero, delta);
	}



	/////////////////////////////////////////////////////////////////////////
	// Constructor
	RotationOptimizer::RotationOptimizer() :
		max_iters(kMaxIters),
		min_step_norm(kMinStepNorm),
		min_improvement(kMinImprovement),
		initialized(false) {
	}

	toon::SO3<> RotationOptimizer::OptimizeLogLik(const VanishingPointModel& model,
																								const CompleteData& data,
																								const toon::SO3<>& R_init) {
		f = boost::bind(&VanishingPointModel::ComputeCompleteLogLik,
										ref(model),
										ref(data),
										_1);
		Jf = boost::bind(&VanishingPointModel::ComputeCompleteLogLikGradient,
										 ref(model),
										 ref(data),
										 _1);
		Compute(R_init);
	}

	SO3<> RotationOptimizer::Compute(const SO3<>& R_init) {
		Initialize(R_init);
		do {
			Step();
		} while (!Converged());
		return Rcur;
	}

	void RotationOptimizer::Initialize(const SO3<>& R) {
		Rs.clear();
		fs.clear();
		gradients.clear();
		step_sizes.clear();

		initialized = true;
		step_size = 1e-2;
		improvement = 0.;
		num_steps = 0;
		MoveTo(R);
	}

	Vec3 RotationOptimizer::Step() {
		CHECK(initialized) << "Must call Initialize(...) before Step()";
		//TITLE("At f(" << Rcur.ln() << ") = " << fcur);

		// Check against numeric gradient
		/*if (verify_gradients) {
			Vec3 Gcur_numeric = model->ComputeCompleteLogLik_NumericGradient(*data, Rcur, 1e-12);
			double abserr = norm(Gcur - Gcur_numeric);
			double relerr = abserr / norm(Gcur);
			bool error_small_enough;
			if (norm(Gcur) < 1e-6) {
			error_small_enough = abserr < 1e-5;
			} else {
			error_small_enough = relerr < 1e-5;
			}
			CHECK(error_small_enough) << EXPR(abserr, relerr, Gcur, Gcur_numeric);
			DLOG << "[ Gradient Error = " << abserr << " ]";
			}*/

		// Note: gradient already computed

		// Take step and update step size
		Vec3 step;
		SO3<> Rnext;
		double fnext;
		do {
			// note that despite the name of this class, we're actually walking *uphill*
			step = step_size * Gcur;
			step_norm = norm(step);
			Rnext = Rcur * SO3<>::exp(step);
			fnext = f(Rnext);
			if (fnext < fcur) {
				step_size /= 2.;
			} else {
				step_size *= 1.1;
			}
		} while (fnext < fcur && step_norm > min_step_norm);

		improvement = fnext - fcur;
		num_steps++;
		MoveTo(Rnext, fnext);
		last_step = step;
		return step;
	}

	void RotationOptimizer::MoveTo(const SO3<>& R) {
		CHECK(f) << "You must assign to RotationOptimizer::f before optimizing";
		MoveTo(R, f(R));
	}

	void RotationOptimizer::MoveTo(const SO3<>& R, double fR) {
		CHECK(Jf) << "You must assign to RotationOptimizer::Jf before optimizing";
		Rcur = R;
		fcur = fR;
		Gcur = Jf(R);
		RecordState();
	}

	bool RotationOptimizer::Initialized() {
		return initialized;
	}

	bool RotationOptimizer::Converged() {
		return initialized && (step_norm < min_step_norm ||
													 improvement < min_improvement ||
													 num_steps >= max_iters);
	}

	void RotationOptimizer::RecordState() {
		Rs.push_back(Rcur);
		fs.push_back(fcur);
		step_sizes.push_back(step_size);
	}


	///////////////////////////////////////////////////////////////////////////////////
	VanishingPointEstimator::VanishingPointEstimator() {
	}

	VanishingPointEstimator::VanishingPointEstimator(const vector<LineObservation>& obs) {
		Configure(obs);
	}

	VanishingPointEstimator::VanishingPointEstimator(const vector<LineObservation>& obs,
																									 const VanishingPointModel& model) {
		Configure(obs, model);
	}

	void VanishingPointEstimator::Configure(const vector<LineObservation>& obs) {
		owned_model.reset(new VanishingPointModel(kLineSigmaSqr));
		Configure(obs, *owned_model);
	}

	void VanishingPointEstimator::Configure(const vector<LineObservation>& obs,
																					const VanishingPointModel& m) {
		observations = &obs;
		model = &m;
		responsibilities.Resize(observations->size(), 4);

		// Configure the optimizer
		optimizer.f = boost::bind(&VanishingPointModel::ComputeExpectedLogLik,
															ref(*model),
															ref(*observations),
															ref(responsibilities),
															_1);
		optimizer.Jf = boost::bind(&VanishingPointModel::ComputeExpectedLogLikGradient,
															 ref(*model),
															 ref(*observations),
															 ref(responsibilities),
															 _1);
	}

	SO3<> VanishingPointEstimator::Compute(const SO3<>& R_init) {
		// We must *not* call optimizer.Initialize(...) yet because the
		// responsibility matrix contains garbage.
		R_cur = R_init;

		// Iterate EM
		do {
			//TITLE("Iteration "<<optimizer.num_steps);
			//DREPORT(optimizer.Rcur);

			// Compute responsibilities
			EStep();
			//DREPORT(responsibilities);

			// Take a gradient step
			MStep();
			/*DLOG << "Now at " << optimizer.Rcur.ln();
				DLOG << "step was " << optimizer.last_step;
				DLOG << "  norm = " << optimizer.step_norm;
				DLOG << "f(cur) = " << optimizer.fcur;
				DLOG << "gradient(cur) = " << optimizer.Gcur;*/
		} while (!optimizer.Converged());

		DLOG << "Finished after " << optimizer.num_steps << " iterations";

		return CurrentEstimate();
	}

	void VanishingPointEstimator::EStep() {
		// Note that in the very first iteration, R_cur will not equal
		// optimizer.Rcur. This cannot be avoided since we must compute
		// responsibilities before initializing the optimizer.
		model->ComputePosteriorOnLabels(*observations, R_cur, responsibilities);
	}

	void VanishingPointEstimator::MStep() {
		// Note that in the very first iteration, R_cur will not equal
		// optimizer.Rcur. This cannot be avoided since we must compute
		// responsibilities before initializing the optimizer.
		if (optimizer.Initialized()) {
			// Since we've changed the responsibilities we need to re-compute
			// the objective and its gradient. Unfortunately this is a little
			// wasteful and also results in double-counting in the history
			// vectors.
			optimizer.MoveTo(R_cur);
		} else {
			// We must *not* call Initialize until after the first E step
			optimizer.Initialize(R_cur);
		}

		// Take a gradient step
		optimizer.Step();
		R_cur = optimizer.Rcur;
	}

	bool VanishingPointEstimator::Converged() {
		return optimizer.Converged();
	}

	SO3<> VanishingPointEstimator::CurrentEstimate() {
		return R_cur;
	}
}
