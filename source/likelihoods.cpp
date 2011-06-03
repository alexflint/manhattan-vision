#include "likelihoods.h"
#include "payoffs.pb.h"
#include "payoff_helpers.h"
#include "numeric_utils.h"
#include "gaussian.h"

#include "vector_utils.tpp"

namespace indoor_context {

	// TODO: compute these by simple average over training set
	static const double kBgMean = 5;
	static const double kBgVariance = 5;

	using namespace toon;
	////////////////////////////////////////////////////////////////////
	ModelLikelihood::ModelLikelihood() {
		Configure(Zeros);
	}

	ModelLikelihood::ModelLikelihood(const Vec2& lambda) {
		Configure(lambda);
	}

	void ModelLikelihood::Configure(const Vec2& l) {
		CHECK_EQ(l.size(), J_loglik.size());
		lambda = l;
		loglik = 0;
		J_loglik = Zeros;
	}

	void ModelLikelihood::Process(const proto::FrameWithFeatures& instance) {
		CHECK_GT(norm(lambda), 0.) << "Compute called before initializing parameters";

		// Compute model log-likelihood: 
		double n1 = instance.num_walls();
		double n2 = instance.num_occlusions();
		double log_top = -lambda[0]*n1 + -lambda[1]*n2;
		double exp1 = exp(-lambda[0]);
		double exp2 = exp(-lambda[1]);
		double exp12 = exp(-lambda[0]-lambda[1]);
		double bottom = 1. - exp1 - exp2 + exp12;
		CHECK_GT(bottom, 1e-17);
		double model_loglik = log_top - log(bottom);

		// Compute derivative of model log-likelihood
		Vec2 J_model_loglik;
		J_model_loglik[0] = -n1 - (exp1-exp12)/bottom;
		J_model_loglik[1] = -n2 - (exp2-exp12)/bottom;

		// Accumulate
		loglik += model_loglik;
		J_loglik += J_model_loglik;
	}

	/////////////////////////////////////////////////////////////////////////////////
	LogitFeatureLikelihood::LogitFeatureLikelihood(const Vector<>& th)
		: J_loglik(th.size()), theta(th) {
		Configure(th);
	}

	void LogitFeatureLikelihood::Configure(const Vector<>& th) {
		CHECK_EQ(th.size(), J_loglik.size());
		theta = th;
		loglik = 0;
		J_loglik = Zeros;
	}

	void LogitFeatureLikelihood::Process(const proto::FrameWithFeatures& instance) {
		// Unpack path features
		UnpackMatrix(instance.path_features(), path_ftrs);
		const int nx = path_ftrs.Cols();

		// Sum the features over the path. Normalization is important
		// here. It can be shown (though not trivially) that for logistic
		// P(f|m,p), dividing by the norm of the parameter vector means
		// that integrating P(f|m,p) over f results some constant that is
		// independent of p). This means that the log-likelihood for each
		// column needs to have log(norm(p)) subtracted.
		double sum_logit = 0;
		Vector<> J_sum_logit(theta.size());
		J_sum_logit = Zeros;
		for (int x = 0; x < nx; x++) {
			double y = 0; 
			for (int k = 0; k < theta.size(); k++) {
				double fk = path_ftrs[k][x];
				y += fk * theta[k];
				J_sum_logit[k] += fk * exp(-y) / (1. + exp(-y));
			}
			sum_logit += log(1. + exp(-y));
		}

		// The following normalization is important when doing inference on
		// the theta as it renders the integral over the feature space
		// independent of theta.
		double ftr_loglik = -nx*log(norm(theta)) - sum_logit;
		Vector<> J_ftr_loglik = -nx*theta/norm_sq(theta) + J_sum_logit;

		// Accumulate
		loglik += ftr_loglik;
		J_loglik += J_ftr_loglik;
	}

	/////////////////////////////////////////////////////////////////////////////////
	GaussianFeatureLikelihood::GaussianFeatureLikelihood(const Vector<>& th)
		: theta(th), J_loglik(th.size()), enable_jacobian(true) {
		Configure(th);
	}

	void GaussianFeatureLikelihood::Configure(const Vector<>& th) {
		CHECK_EQ(th.size(), J_loglik.size());
		theta = th;
		loglik = 0;
		J_loglik = Zeros;
	}

	void GaussianFeatureLikelihood::Process(const proto::FrameWithFeatures& instance) {
		CHECK(instance.features_size() != 0)
			<< "Features were omitted in this dataset. Re-generate with --store_features.";

		// Unpack path
		MatI mpath, morients;
		UnpackMatrix(instance.path(), mpath);
		UnpackMatrix(instance.orients(), morients);
		Vector<-1,int> path = asToon(mpath).T()[0];
		Vector<-1,int> orients = asToon(morients).T()[0];

		// Unpack features
		UnpackFeatures(instance, features);
		const int nf = features.features.size();
		const int nx = features.features[0].nx();
		const int ny = features.features[0].ny();

		// Upack components of theta
		CHECK_EQ(theta.size(), nf*4);
		Vector<> ideal_fs = theta.slice(0, nf);
		Vector<> fall_offs = theta.slice(nf, nf);
		Vector<> noise_vars = theta.slice(nf*2, nf);
		Vector<> fg_priors = theta.slice(nf*3, nf);

		double log_bg_var = log(kBgVariance);
		double sqr_bg_var = kBgVariance*kBgVariance;

		// Accumulate the log likelihood
		double frame_loglik = 0;
		Vector<> frame_J_loglik(theta.size());
		frame_J_loglik = Zeros;
		for (int k = 0; k < nf; k++) {
			CHECK_INTERVAL(fg_priors[k], 1e-17, 1.);
			double fg_logprior = log(fg_priors[k]);
			double bg_logprior = log(1. - fg_priors[k]);
			double log_noise_var = log(noise_vars[k]);
			double noise_recip = 1. / noise_vars[k];

			const DPPayoffs& po = features.features[k];
			for (int y = 0; y < ny; y++) {
				//if (y != 478) continue;  // debug only
				double py = 1.* y / ny;
				const float* feature_row[] = { po.wall_scores[0][y], po.wall_scores[1][y] };
				for (int x = 0; x < nx; x++) {
					//if (x != 155) continue;
					double f = feature_row[ orients[x] ][ x ];
					//TITLE(k<<":"<<x<<","<<y<<" = "<<f);
					double ppath = 1. * path[x] / ny;
					double f_ampl = Gauss1D(py, ppath, fall_offs[k]);
					double f_mean = ideal_fs[k] * f_ampl;
					double fg_loglik = FastLogGauss1D(f, f_mean, noise_vars[k], log_noise_var);
					double bg_loglik = FastLogGauss1D(f, kBgMean, kBgVariance, log_bg_var);
					double fg_logpost = fg_logprior + fg_loglik;
					double bg_logpost = bg_logprior + bg_loglik;
					double logjoint = LogSumExp(fg_logpost, bg_logpost);
					//DREPORT(fg_logpost, bg_logpost, logjoint);
					frame_loglik += logjoint;

					if (enable_jacobian) {
						// Compute gradient w.r.t. ideal feature
						//double ideal_grad = noise_recip * (f-f_mean) * ampl;
						//DREPORT(f, noise_recip, f-f_mean, ampl, ideal_grad);
						double top = (fg_priors[k] * (f-f_mean) / noise_vars[k]) * exp(fg_logpost) * f_ampl;
						double ideal_grad = top / exp(logjoint);
						frame_J_loglik[k] += ideal_grad;
						//DREPORT(ideal_grad);

						// Compute gradient w.r.t. fall-off
						double y_dev = (py-ppath)*(py-ppath) / (2*fall_offs[k]*fall_offs[k]);
						frame_J_loglik[nf+k] += noise_recip * (f-f_mean) * f_mean * y_dev;

						// Compute gradient w.r.t. noise variance
						double f_dev = (f-f_mean)*(f-f_mean) / (noise_vars[k]*noise_vars[k]*2);
						double term1 = -.5*noise_recip + f_dev;
						double term2_log_top = bg_logprior + bg_loglik - fg_loglik;
						double term2_log_bottom = LogSumExp(fg_logprior, term2_log_top);
						double noise_grad = term1 + exp(term2_log_top - term2_log_bottom);
						frame_J_loglik[nf*2+k] += noise_grad;

						// Compute jacobian w.r.t. fg_priors
						double log_top = 1. - exp(bg_loglik - fg_loglik);
						double log_bottom = LogSumExp(fg_logprior, bg_logprior + bg_loglik - fg_loglik);
						double fg_grad = exp(log_top - log_bottom);
						frame_J_loglik[nf*3+k] += fg_grad;
					}

					//break;  // debug
				}
				//break;  // debug
			}
		}
		loglik += frame_loglik;
		J_loglik += frame_J_loglik;
	}
}
