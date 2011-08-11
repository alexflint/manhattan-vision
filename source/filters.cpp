#include <VNL/vector.tpp>

#include "filters.h"
#include "common_types.h"
#include "hw_convolver.h"
#include "worker.h"

#include "numeric_utils.tpp"
#include "image_utils.tpp"

namespace indoor_context {
	using boost::bind;
	using boost::ref;

lazyvar<string> gvDefaultParallelism("FilterBank.DefaultParallelism");

template <typename T, typename S>
T cast(S x) {
	return static_cast<T>(x);
}

// Create a shared pointer
template<typename T>
shared_ptr<T> make_shared_ptr(T* p) {
	return shared_ptr<T> (p);
}

// *** LinearFilter ***
void LinearFilter::RunSequential(const MatF& input, MatF& output) {
	assert(input.Cols() <= output.Cols());
	assert(input.Rows() <= output.Rows());

	for (int r = 0; r < input.Rows(); r++) {
		float* out = output[r];
		for (int c = 0; c < input.Cols(); c++) {
			out[c] = RunAt(input, r, c);
		}
	}
}

int LinearFilter::RunAt(const MatF& input, int r, int c) const {
	int sum = 0;
	int xrad = mask.Cols() / 2;
	int yrad = mask.Rows() / 2;
	for (int dr = -yrad; dr <= yrad; dr++) {
		for (int dc = -xrad; dc <= xrad; dc++) {
			int rr = Clamp(r+dr, 0, input.Rows()-1);
			int cc = Clamp(c+dc, 0, input.Cols()-1);
			sum += input[rr][cc] * mask[dr+yrad][dc+xrad];
		}
	}
	return sum;
}

// *** SeperatedFilter ***
void SeperatedFilter::RunSequential(const MatF& input, MatF& output) {
	assert(input.Cols() <= output.Cols());
	assert(input.Rows() <= output.Rows());

	const int w = input.Cols();
	const int h = input.Rows();
	const int xrad = xmask.Size()/2;
	const int yrad = ymask.Size()/2;

	// Only reallocate if the temporary storage is smaller than the
	// input image. In any other case we can simply use a sub-section
	// of the temporary buffer and avoid the expensive reallocation.
	if (temp.get() == NULL || temp->Cols() < w || temp->Rows() < h) {
		if (temp.get() != NULL) {
			DLOG << "Efficiency warning: Forced to reallocate"
				" temp image in a SeperatedFilter\n";
		}
		temp.reset(new MatF(h, w));
	}

	temp->Fill(0.0);
	for (int r = 0; r < h; r++) {
		const float* inrow = input[r];
		float* outp = (*temp)[r];
		for (int c = 0; c < w; c++) {
			for (int dc = -xrad; dc <= xrad; dc++) {
				*outp += xmask[dc+xrad] * inrow[ Clamp(c+dc, 0, w-1) ];
			}
			outp++;
		}
	}

	output.Fill(0.0);
	for (int r = 0; r < h; r++) {
		float* outp = output[r];
		for (int c = 0; c < w; c++) {
			for (int dr = -yrad; dr <= yrad; dr++) {
				*outp += ymask[dr+yrad] * (*temp)[ Clamp(r+dr, 0, h-1) ][c];
			}
			outp++;
		}
	}
}

void SeperatedFilter::RunOnHardware(HwConvolver& convolver, MatF& output) {
	VecF kx = xmask.ApplyF<float>(&cast<float, double>);
	VecF ky = ymask.ApplyF<float>(&cast<float, double>);
	convolver.Convolve(kx, ky, output);
}

// *** DiffFilter ***
void DiffFilter::RunSequential(const MatF& input, MatF& output) {
	ExpandBuffer(input.Cols(), input.Rows());
	filter1->RunSequential(input, output);
	filter2->RunSequential(input, *temp);
	Subtract(output, *temp);
}

void DiffFilter::RunOnHardware(HwConvolver& convolver, MatF& output) {
	VecF kx1 = filter1->xmask.ApplyF<float>(&cast<float, double>);
	VecF ky1 = filter1->ymask.ApplyF<float>(&cast<float, double>);
	VecF kx2 = filter2->xmask.ApplyF<float>(&cast<float, double>);
	VecF ky2 = filter2->ymask.ApplyF<float>(&cast<float, double>);
	convolver.ConvolveDiff(kx1, ky1, kx2, ky2, output);
}

void DiffFilter::Subtract(MatF& x, const MatF& y) {
	for (int r = 0; r < x.Rows(); r++) {
		float* xrow = x[r];
		const float* yrow = y[r];
		for (int c = 0; c < x.Cols(); c++) {
			xrow[c] -= yrow[c];
		}
	}
}

void DiffFilter::ExpandBuffer(int w, int h) {
	// Only reallocate if the temporary storage is smaller than the
	// input image. In any other case we can simply use a subset of the
	// temporary buffer and avoid the expensive reallocation.
	if (temp.get() == NULL || temp->Cols() < w || temp->Rows() < h) {
		if (temp.get() != NULL) {
			DLOG << "Efficiency warning: Reallocating image in DiffFilter\n";
		}
		temp.reset(new MatF(h, w));
	}
}


// *** GaussFunction ***
double GaussFunction::operator()(double x) const {
	return exp(-x*x / denom) / nrm1d;
}

double GaussFunction::operator()(double x, double y) const {
	return exp(-(x*x + y*y) / denom) / nrm2d;
}

SeperatedFilter* GaussFunction::MakeSeperatedFilter() const {
	int rad = static_cast<int>(ceil(sigma*3.0));
	return SeperatedFilter::MakeFromFuncs(rad, rad, *this, *this);
}

// *** GaborFunction ***
double GaborFunction::operator()(double x, double y) const {
	return gauss(x, y) * cos(x*kcostheta + y*ksintheta);
}

DiffFilter* GaborFunction::MakeSeperatedFilter() const {
	// The gabor function can be represented as the difference between
	// two seperable funcions, which is much faster than a full
	// convolution:
	//   gabor(x,y) = gauss2d(x, y) * cos(k * (x*cos(theta) + y*sin(theta) ))
	//              = gauss2d(x, y) * cos(a*x + b*y)
	//                                  { where a=k*cos(theta), b=k*sin(theta) }
	//              = gauss1d(x)*gauss1d(y) * [cos(a*x)*cos(b*y) - sin(a*x)*sin(b*y)]
	//              = gauss1d(x)*cos(a*x) * gauss1d(y)*cos(b*y) -
	//                   gauss1d(x)*sin(a*x) * gauss1d(y)*sin(b*y)

	struct SepFuncs {
		static double gtcosbt(const GaussFunction& g, double b, double t) {
			return g(t)*cos(b*t);
		}
		static double gtsinbt(const GaussFunction& g, double b, double t) {
			return g(t)*sin(b*t);
		}
	};

	int rad = static_cast<int>(ceil(gauss.sigma*3.0));
	SeperatedFilter* cospart = SeperatedFilter::MakeFromFuncs
			(rad, rad,
			 bind(&SepFuncs::gtcosbt, gauss, kcostheta, _1),
			 bind(&SepFuncs::gtcosbt, gauss, ksintheta, _1));
	SeperatedFilter* sinpart = SeperatedFilter::MakeFromFuncs
			(rad, rad,
			 bind(&SepFuncs::gtsinbt, gauss, kcostheta, _1),
			 bind(&SepFuncs::gtsinbt, gauss, ksintheta, _1));
	return new DiffFilter(cospart, sinpart);
}

// *** FilterBank ***
FilterBank::FilterBank() : nworkers(0) {
}

FilterBank::~FilterBank() {
	for (int i = 0; i < nworkers; i++) {
		workers[i].Join();
	}
}

void FilterBank::RunOneFilter(int filter,
                              const MatF& input,
                              MatF& output) const {
	output.Resize(input.Rows(), input.Cols());
	filters[filter]->RunSequential(input, output);
}

void FilterBank::RunSequential(const MatF& input,
															 vector<shared_ptr<MatF> >* output) const {
	for (int j = 0; j < filters.size(); j++) {
		output->push_back(make_shared_ptr(new MatF));
		RunOneFilter(j, input, *output->back());
	}
}

void FilterBank::RunOnHardware(const MatF& input,
															 vector<shared_ptr<MatF> >* outputs) const {
	// Initialize the hardware convolver
	hw.reset(new HwConvolver(input.Cols(), input.Rows()));
	hw->LoadImage(input);

	// Run the convolutions
	for (int j = 0; j < filters.size(); j++) {
		MatF* output = new MatF(input.Rows(), input.Cols());
		outputs->push_back(make_shared_ptr(output));
		filters[j]->RunOnHardware(*hw, *output);
	}
}

void FilterBank::RunParallel(const MatF& input,
                             vector<shared_ptr<MatF> >* output) const {
	// We must only have one thread per filter at any one time since the
	// filters are not thread-safe (they store contain temporary buffers
	// used during the convolutions).
	if (nworkers != filters.size()) {
		workers.reset(new Worker[filters.size()]);
		nworkers = filters.size();
	}

	// Do the filtering
	for (int j = 0; j < filters.size(); j++) {
		output->push_back(make_shared_ptr(new MatF));
		workers[j].Add(bind(&FilterBank::RunOneFilter,
												this, j, ref(input), ref(*output->back())));
	}

	// Wait for the jobs to finish
	for (int i = 0; i < nworkers; i++) {
		workers[i].Wait();
	}
}


// *** FilterPyramid ***
FilterPyramid::FilterPyramid() : num_scales(0) {
}

FilterPyramid::FilterPyramid(int nscales) {
	Configure(nscales);
}

void FilterPyramid::Configure(int nscales) {
	num_scales = nscales;
	GaussFunction g(1.0);
	lowpass.reset(g.MakeSeperatedFilter());
}

MatF* FilterPyramid::NextLevel(const MatF& cur) const {
	MatF smoothed(cur.Rows(), cur.Cols());
	lowpass->RunSequential(cur, smoothed);
	MatF* next_level = new MatF(cur.Rows()/2, cur.Cols()/2);
	Downsample(smoothed, 2, *next_level);
	return next_level;
}

void FilterPyramid::RunSequential(const MatF& input,
																	vector<shared_ptr<MatF> >* outputs) const {
	CHECK_GT(num_scales, 0);
	scoped_ptr<MatF> level;
	const MatF* cur = &input;
	for (int i = 0; i < num_scales; i++) {
		filter_bank.RunSequential(*cur, outputs);
		level.reset(NextLevel(*cur));
		cur = level.get();
	}
}

void FilterPyramid::RunOnHardware(const MatF& input,
																	vector<shared_ptr<MatF> >* outputs) const {
	CHECK_GT(num_scales, 0);
	scoped_ptr<MatF> level;
	const MatF* cur = &input;
	for (int i = 0; i < num_scales; i++) {
		filter_bank.RunOnHardware(*cur, outputs);
		level.reset(NextLevel(*cur));
		cur = level.get();
	}
}

void FilterPyramid::RunParallel(const MatF& input,
																vector<shared_ptr<MatF> >* outputs) const {
	CHECK_GT(num_scales, 0);
	vector<shared_ptr<MatF> > pyramid;
	const MatF* cur = &input;
	for (int i = 0; i < num_scales; i++) {
		filter_bank.RunParallel(*cur, outputs);
		pyramid.push_back(make_shared_ptr(NextLevel(*cur)));
		cur = pyramid.back().get();
	}
}










GaborFilters::GaborFilters() {
}

GaborFilters::GaborFilters(int ns, int no) {
	Configure(ns, no);
}

void GaborFilters::Configure(int ns, int no) {
	num_scales = ns;
	num_orients = no;
	pyramid.Configure(num_scales);
	MakeGaborFilters(num_orients, &pyramid.filter_bank.filters);
}

void GaborFilters::Run(const MatF& input) {
	Run(input, *gvDefaultParallelism);
}

void GaborFilters::Run(const MatF& input, const string& strategy) {
	if (strategy == "CPU-Sequential") {
		RunSequential(input);
	} else if (strategy == "CPU-Parallel") {
		RunParallel(input);
	} else if (strategy == "GPU") {
		RunParallel(input);
	} else {
		CHECK(false) << "Unknown parallelism strategy: " << strategy;
	}
}

void GaborFilters::RunSequential(const MatF& input) {
	// TODO: stop deleting and re-allocating all this memory
	responses.clear();
	pyramid.RunSequential(input, &responses);
}

void GaborFilters::RunParallel(const MatF& input) {
	// TODO: stop deleting and re-allocating all this memory
	responses.clear();
	pyramid.RunParallel(input, &responses);
}

void GaborFilters::RunOnHardware(const MatF& input) {
	// TODO: stop deleting and re-allocating all this memory
	responses.clear();
	pyramid.RunOnHardware(input, &responses);
}




void SmoothGaussian(const float sigma,
                    const MatF& input,
                    MatF& output) {
	output.Resize(input.Rows(), input.Cols());
	GaussFunction g(sigma);
	scoped_ptr<SeperatedFilter> filter(g.MakeSeperatedFilter());
	filter->RunSequential(input, output);
}

void MakeGaborFilters(int norients, vector<shared_ptr<DiffFilter> >* filters) {
	for (int i = 0; i < norients; i++) {
		GaborFunction g(1.2, i*M_PI/norients, 0.5);
		filters->push_back(make_shared_ptr(g.MakeSeperatedFilter()));
	}
}

void MakeFullGaborFilters(int norients,
                          vector<shared_ptr<LinearFilter> >* filters) {
	for (int i = 0; i < norients; i++) {
		GaborFunction g(1.2, i*M_PI/norients, 0.5);
		LinearFilter* filter = LinearFilter::MakeFromFunc(4, 4, g);
		filters->push_back(make_shared_ptr(filter));
	}
}

}
