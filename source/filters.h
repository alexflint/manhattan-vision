#pragma once

#include <cmath>
#include <iostream>

#include "worker.h"
#include "common_types.h"
#include "hw_convolver.h"

namespace indoor_context {

// Represents a filter that takes an input image and produces an
// output image of the same size
class Filter2D {
public:
	// Run the filter
	virtual void Run(const ImageF& input, ImageF& output) = 0;

	// Run on dedicated hardware
	virtual void RunOnHardware(HwConvolver& convolver,
	                           ImageF& output) {
		cerr << "Error: RunOnHardware not implemented for this filter type\n";
	}
};

// Represents a filter that uses a 2D correlation with a filter mask
// to perform its transformation.
class LinearFilter;
class LinearFilter : public Filter2D {
private:
	MatD mask;
public:
	LinearFilter(int xrad, int yrad) : mask(xrad*2+1, yrad*2+1) { }
	LinearFilter(const MatD& mat) : mask(mat) { }
	virtual void Run(const ImageF& input, ImageF& output);
	int RunAt(const ImageF& input, int r, int c) const;

	template<typename Func>
	static LinearFilter* MakeFromFunc(int xrad, int yrad, Func f) {
		LinearFilter* filter = new LinearFilter(xrad, yrad);
		for (int r = -yrad; r <= yrad; r++) {
			for (int c = -xrad; c <= xrad; c++) {
				filter->mask[r+yrad][c+xrad] = f(c, r);
			}
		}
		return filter;
	}
};

// Represents a filter that uses two 1D filters: one in the X
// direction and one in the Y direction. NOT THREAD SAFE (due to use
// of "temp" member)
class SeperatedFilter;
class SeperatedFilter : public Filter2D {
private:
	scoped_ptr<ImageF> temp;  // used for storing intermediate results
public:
	VecD xmask;
	VecD ymask;
	SeperatedFilter(int xrad, int yrad)
	: xmask(xrad*2+1),
	  ymask(yrad*2+1) { }
	SeperatedFilter(int rad)
	: xmask(rad*2+1),
	  ymask(rad*2+1) { }
	SeperatedFilter(const VecD& xs, const VecD& ys)
	: xmask(xs),
	  ymask(ys) { }
	virtual void Run(const ImageF& input, ImageF& output);
	virtual void RunOnHardware(HwConvolver& convolver, ImageF& output);

	template<typename XFunc, typename YFunc>
	static SeperatedFilter* MakeFromFuncs(int xrad, int yrad, XFunc xf, YFunc yf) {
		SeperatedFilter* filter = new SeperatedFilter(xrad, yrad);
		for (int x = -xrad; x <= xrad; x++) {
			filter->xmask[x+xrad] = xf(x);
		}
		for (int y = -yrad; y <= yrad; y++) {
			filter->ymask[y+yrad] = yf(y);
		}
		return filter;
	}
};

// Represents a filter that takes the signed difference between two
// filters. NOT THREAD SAFE (due to use of "temp1" and "temp2" members)
class DiffFilter : public Filter2D {
private:
public:
	scoped_ptr<SeperatedFilter> filter1, filter2;
	scoped_ptr<ImageF> temp;

	// Note that this constructor takes ownership of the two filters
	DiffFilter(SeperatedFilter* a, SeperatedFilter* b) :
		filter1(a),
		filter2(b) { }
	virtual void Run(const ImageF& input, ImageF& output);

	virtual void RunOnHardware(HwConvolver& convolver, ImageF& output);
	static void Subtract(ImageF& x, const ImageF& y);
	void ExpandBuffer(int min_width, int min_height);
};

// Represents a gaussian of two variables
class GaussFunction {
public:
	double sigma, denom, nrm1d, nrm2d;
	GaussFunction(double s) : sigma(s),
			denom(2 * s * s),
			nrm2d(denom * M_PI) {
		nrm1d = sqrt(nrm2d);  // don't make this an initializer, the order
		// is confusing
	}
	double operator()(double x) const;
	double operator()(double x, double y) const;
	SeperatedFilter* MakeSeperatedFilter() const;
};

// Represents a 2D gabor function
class GaborFunction {
public:
	GaussFunction gauss;
	double theta, freq;
	double kcostheta, ksintheta;

	GaborFunction(double sigma, double th, double f)
	: gauss(sigma),
	  theta(th),
	  freq(f),
	  kcostheta(2.0 * M_PI * freq * cos(th)),
	  ksintheta(2.0 * M_PI * freq * sin(th)) { }

	double operator()(double x, double y) const;
	DiffFilter* MakeSeperatedFilter() const;
};

// Represents a set of filters operating over scales
class FilterBank {
public:
	int nscales;
	scoped_ptr<SeperatedFilter> lowpass;  // applied before downsampling
	vector<shared_ptr<DiffFilter> > filters;

	mutable int nworkers;  // number of workers for parallel impl
	mutable scoped_array<Worker> workers;

	// Hardware handles for cuda impl
	mutable vector<shared_ptr<HwConvolver> > convs;

	// Initialize a filter bank with the specified nubmer of scales
	FilterBank(int nscales);
	// Kill the worker threads when this filterbank is destroyed
	~FilterBank();

	// Get the number of output images to be produced
	inline int size() const { return nscales * filters.size(); }
	// Run the filter bank, storing output images in the supplied vector
	void Run(const ImageF& input, vector<shared_ptr<ImageF> >* output) const;
	// Run the filter bank using multiple threads. Exactly the same
	// result as above, but faster.
	void RunParallel(const ImageF& input,
	                 vector<shared_ptr<ImageF> >* output) const;

	// Run the filter bank using multiple threads. Same result as above
	// (except for perhaps some small differences due to different
	// hardware).
	void RunOnHardware(const ImageF& input,
	                   vector<shared_ptr<ImageF> >* output) const;
private:
	void RunOneFilter(int filter,
	                  const ImageF& input,
	                  ImageF& output) const;
};

class GaborFilterBank {
public:
	int num_scales;
	int num_orients;
	scoped_ptr<FilterBank> filterbank;
	vector<shared_ptr<ImageF> > responses;

	// Initialize empty
	GaborFilterBank();
	// Initialize and configure the filter bank
	GaborFilterBank(int scales, int orientations);
	// Configure the filter bank to have the specified number of scales
	// and orientations
	void Configure(int scales, int orientations);
	// Run the filter bank, storing output images in the supplied vector
	void Run(const ImageF& input);
	// Run the filter bank using multiple threads. Exactly the same
	// result as above, but faster.
	void RunParallel(const ImageF& input);
	// Run the filter bank using multiple threads. Same result as above
	// (except for perhaps small differences arising from hardware
	// differences)
	void RunOnHardware(const ImageF& input);
};

// Smooth an image with a gaussian kernel of radius sigma
void SmoothGaussian(const float sigma,
                    const ImageF& input,
                    ImageF& output);

// Make a bank of gabor filters
void MakeGaborFilters(int norients,
                      vector<shared_ptr<DiffFilter> >* filters);

// Make a bank of non-separated gabor filters. These are slow but
// convenient for evaluating at specific locations (rather than over
// an entire image).
void MakeFullGaborFilters(int norients,
                          vector<shared_ptr<LinearFilter> >* filters);

}
