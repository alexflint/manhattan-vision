#include "gaussian.h"

#include <ext/algorithm>
#include <vector>
#include <fstream>
#include <functional>
#include <iterator>

#include <boost/shared_array.hpp>

#include <VNL/matrix.h>
#include <VNL/vector.h>
#include <VNL/sample.h>
#include <VNL/Algo/matrixinverse.h>
#include <VNL/Algo/determinant.h>
#include <VNL/Algo/cholesky.h>

#include "common_types.h"
#include "kmeans.h"
#include "numeric_utils.h"

namespace indoor_context {
	using boost::shared_array;

	static const double kSqrtTwoPi = sqrt(2*M_PI);
	static const double kLogSqrtTwoPi = log(sqrt(2*M_PI));

	double LogGauss1D(double x, double m, double s) {
		return -(x-m)*(x-m)/(2*s*s) - log(s) - kLogSqrtTwoPi;
	}

	double FastLogGauss1D(double x, double m, double log_s, double s_sqr) {
		return -(x-m)*(x-m)/(2*s_sqr) - log_s - kLogSqrtTwoPi;
	}

	double Gauss1D(double x, double m, double s) {
		return exp(-(x-m)*(x-m)/(2*s*s)) / (s*kSqrtTwoPi);
	}

	double Gauss2D(const Vec2& x, const Vec2& m, double s) {
		return exp(-0.5 * norm_sq(m-x) / s) / (2*M_PI*sqrt(s));
	}

	double Gaussian::Evaluate(const VecD& x) const {
		return GaussianEvaluator(*this).Evaluate(x);
	}

	VecD Gaussian::Sample() const {
		return GaussianSampler(*this).Sample();
	}

	GaussianEvaluator::GaussianEvaluator(const Gaussian& g)
		: gaussian(g),	chol(g.cov) {
		// Det(A) = (product of diagonal entries of cholesky decomp) ^ 2
		// log(Det(A)) = 2 * sum of log of diagonal entries
		logdet = chol.LogDeterminant();
		logdenom = 0.5*g.dim*log(2.0*M_PI) + 0.5*logdet;
		denom = exp(logdenom);
	}

	// Evaluate the log of this gaussian at x
	double GaussianEvaluator::EvaluateLog(const VecD& x) const {
		VecD d = x - gaussian.mean;
		double proj = DotProduct(d, chol.Solve(d));
		return -0.5*proj - logdenom;
	}

	// Evaluate this gaussian at x
	double GaussianEvaluator::Evaluate(const VecD& x) const {
		return exp(EvaluateLog(x));
	}

	GaussianSampler::GaussianSampler(const Gaussian& g)
		: gaussian(g), chol(g.cov) {
	}

	VecD GaussianSampler::GaussianSampler::Sample() const {
		VecD unitrand(gaussian.dim);
		generate(unitrand.begin(), unitrand.end(), &SampleStdNormal);
		return gaussian.mean + chol.LowerTriangle() * unitrand;
	}

	// Static
	double GaussianSampler::SampleStdNormal() {
		return VNL::SampleNormal(0.0, 1.0);
	}

	GaussMixture::GaussMixture(int ncomps_, int dim_)
		: ncomps(ncomps_),
			dim(dim_),
			weights(ncomps),
			comps(new scoped_ptr<Gaussian>[ncomps]) {
		assert(ncomps > 0);
		assert(dim > 0);
		for (int i = 0; i < ncomps; i++) {
			comps[i].reset(new Gaussian(dim));
		}
	}

	GaussMixtureEvaluator* GaussMixture::PrepareEval() const {
		return new GaussMixtureEvaluator(*this);
	}

	double GaussMixture::Evaluate(const VecD& x) const {
		return GaussMixtureEvaluator(*this).Evaluate(x);
	}

	VecD GaussMixture::Sample() const {
		return GaussMixtureSampler(*this).Sample();
	}

	void GaussMixture::Write(const char* filename) const {
		ofstream output(filename);
		output << *this;
		output.close();
	}

	// Static
	GaussMixture* GaussMixture::Read(const char* filename) {
		int ncomps, dim;
		ifstream input(filename);
		input >> ncomps >> dim;

		GaussMixture* model = new GaussMixture(ncomps, dim);
		input >> model->weights;

		for (int i = 0; i < ncomps; i++) {
			input >> model->comps[i]->mean;
			input >> model->comps[i]->cov;
		}
		input.close();

		return model;
	}


	GaussMixtureEvaluator::GaussMixtureEvaluator(const GaussMixture& model)
		: gm(model) {
		Prepare();
	}

	void GaussMixtureEvaluator::Prepare() {
		assert(gm.ncomps > 0);
		logweights = gm.weights.Apply(log);
		for (int i = 0; i < gm.ncomps; i++) {
			GaussianEvaluator* eval = new GaussianEvaluator(*gm.comps[i]);
			evaluators.push_back(shared_ptr<GaussianEvaluator>(eval));
		}
	}

	// Evaluate this model at x
	double GaussMixtureEvaluator::Evaluate(const VecD& x) const {
		double f = 0.0;
		for (int i = 0; i < gm.ncomps; i++) {
			f += evaluators[i]->Evaluate(x) * gm.weights[i];
		}
		return f;
	}

	// Evaluate this model at x
	double GaussMixtureEvaluator::EvaluateLog(const VecD& x) const {
		VecD ys(gm.ncomps);
		for (int i = 0; i < gm.ncomps; i++) {
			ys[i] = evaluators[i]->EvaluateLog(x) + logweights[i];
		}
		return LogSumExp(ys);
	}

	// Initialize a sampler for the given model
	GaussMixtureSampler::GaussMixtureSampler(const GaussMixture& model) :
		gm(model),
		weightsum(model.weights.Sum()) {
		assert(weightsum > 0);
		assert(model.ncomps > 0);
		for (int i = 0; i < model.ncomps; i++) {
			GaussianSampler* sampler = new GaussianSampler(*model.comps[i]);
			samplers.push_back(shared_ptr<GaussianSampler>(sampler));
		}
	}

	// Sample from the model
	VecD GaussMixtureSampler::Sample() const {
		double r = rand() * weightsum / RAND_MAX;
		double wcum = 0.0;
		for (int i = 0; i < gm.ncomps; i++) {
			wcum += gm.weights[i];
			if (wcum > r) {
				return samplers[i]->Sample();
			}
		}
		DLOG << "Failed to choose a component to sample from!" << endl;
	}

	GaussMixtureEstimator::GaussMixtureEstimator(int ncomp)
		: ncomps(ncomp),
			max_iters(100),
			exit_thresh(1e-8),
			axis_aligned(false),
			spherical(false) { }
	
	GaussMixture* GaussMixtureEstimator::Estimate(vector<VecD>& pts) const {
		assert(pts.size() >= ncomps);
		int npts = pts.size();
		int dim = pts[0].size();

		// Compute the number of free variables in the model and in the
		// data. If the former is less than the latter then the system is
		// underspecified and will lead to singularities in the likelihood
		// function.
		int model_params;
		if (spherical) {
			model_params = ncomps * (2 + dim);
		} else if (axis_aligned) {
			model_params = ncomps * (1 + 2*dim);
		} else {
			model_params = ncomps * (1 + dim + dim*(dim+1)/2);
		}
		int data_params = npts * dim;  // number of free variables in the data
		assert(data_params >= model_params);  // is the system under-specified?

		// Initialize the model using k-means clustering
		GaussMixture* model = new GaussMixture(ncomps, dim);
		MatD resps(npts, ncomps);
		vector<VecD> initmeans;
		KMeans::Estimate(pts, ncomps, initmeans, resps);
		for (int i = 0; i < ncomps; i++) {
			model->weights[i] = 1.0 / ncomps;
			model->comps[i]->mean = initmeans[i];
			// set covariances to 1e-10 * Identity so that in the first
			// iteration each point is assigned entirely to the nearest
			// component
			model->comps[i]->cov.SetIdentity(1e-10);
		}

		// Begin iterating
		double loglik = 0.0, prev_loglik = 0.0;
		for (int i = 0; i < max_iters; i++) {
			GaussMixtureEvaluator eval(*model);

			// Check for small determinants (indicates poor support)
			for (int j = 0; j < model->ncomps; j++) {
				double logdetcov = eval.Component(j).GetLogDetCov();
				if (logdetcov < -50*dim) {
					cerr << "Warning: log(det(covariance of component " << j << "))"
							 << " is very small: " << logdetcov << endl;
					cerr << "  its total support is " << resps.GetRow(j).Sum() << endl;
					cerr << "  at iteration " << i << endl;
				}
			}

			// Compute responsibilities (E step)
			//DLOG << "E step\n";
			prev_loglik = loglik;
			loglik = 0.0;
			for (int j = 0; j < npts; j++) {
				double logdenom = eval.EvaluateLog(pts[j]);
				loglik += logdenom;

				// Compute the responsibilities
				for (int k = 0; k < ncomps; k++) {
					const GaussianEvaluator& g = eval.Component(k);
					double logresp = eval.logweights[k] + g.EvaluateLog(pts[j]);
					resps[j][k] = exp(logresp - logdenom);
				}
			}

			// Test for convergence
			//DLOG << "After iteration " << i << " log likelihood = " << loglik << endl;
			double reldiff = fabs((prev_loglik - loglik) / prev_loglik);
			if (reldiff < exit_thresh) {
				cout << "EM converged after " << i << " iterations" << endl;
				return model;
			}
			prev_loglik = loglik;

			// Estimate new parameters (M step)
			//DLOG << "M step\n";
			for (int k = 0; k < ncomps; k++) {
				Gaussian* comp = model->comps[k].get();
				VecD col = resps.GetColumn(k);
				double colsum = resps.GetColumn(k).Sum();

				// Estimate means
				comp->mean.Fill(0.0);
				for (int j = 0; j < npts; j++) {
					comp->mean += col[j] * pts[j];
				}
				comp->mean /= colsum;

				// Estimate covariances

				// Initialize the forward diagonal to a small constant to
				// prevent singularities when components only have a small
				// number of points assigned to them.
				comp->cov.SetIdentity(1e-10);

				for (int j = 0; j < npts; j++) {
					if (spherical) {
						double d = col[j] * VectorSSD(pts[j], comp->mean) / dim;
						for (int k = 0; k < dim; k++) {
							comp->cov[k][k] += d;
						}
					} else if (axis_aligned) {
						VecD d = pts[j] - comp->mean;
						for (int k = 0; k < dim; k++) {
							comp->cov[k][k] += col[j] * d[k]*d[k];
						}
					} else {
						VecD v = pts[j] - comp->mean;
						comp->cov += col[j] * OuterProduct(v, v);
					}
				}
				comp->cov /= colsum;
				if (spherical) {
					for (int k = 0; k < dim; k++) {
						comp->cov[k][k] = sqrt(comp->cov[k][k]);
					}
				}

				// Estimate weights
				model->weights[k] = colsum;
			}
			// Normalize mixing coefficients
			model->weights /= model->weights.Sum();
		}

		DLOG << "EM failed to converge after " << max_iters << " iterations" << endl;
		return model;
	}

	ostream& operator<<(ostream& s, const GaussMixture& model) {
		s << model.ncomps << " " << model.dim << endl;
		s << model.weights << endl << endl;
		for (int i = 0; i < model.ncomps; i++) {
			s << model.comps[i]->mean << endl;
			s << model.comps[i]->cov << endl;
		}
		return s;
	}

}
