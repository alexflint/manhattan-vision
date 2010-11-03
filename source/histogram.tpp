#pragma once

#include <vector>

#include <boost/format.hpp>

#include "common_types.h"
#include "range_utils.tpp"
//#include "numeric_utils.tpp"
#include "image_utils.tpp"
#include "lazyvar.h"

namespace indoor_context {

static const int kTextWidth = 60;

// Interface for histogram bin maps
template <typename T>
class IBinLayout {
public:
	// Get the number of bins
	virtual int GetBinCount() const = 0;
	// Get the index for a given bin
	virtual int GetBinIndex(const T& x) const = 0;
	// Get the centre (or other exemplar) for a given bin
	virtual T GetBinExemplar(int bin) const = 0;
};

// A bin map for scalars
template <typename T>
class ScalarBinLayout : public IBinLayout<T> {
public:
	ScalarBinLayout() { }
	ScalarBinLayout(const T& lo, const T& hi, int bin_count) {
		Configure(lo, hi, bin_count);
	}

	void Configure(const T& lo, const T& hi, int bin_count) {
		lo_ = lo;
		hi_ = hi;
		if (hi_ == lo_) hi_++;  // simple hack for degenerate cases
		n_ = bin_count;
	}

	int GetBinCount() const {
		return n_;
	}

	int GetBinIndex(const T& x) const {
		return (x - lo_ - 1e-5) * n_ / (hi_ - lo_);
	}

	T GetBinExemplar(int bin) const {
		return lo_ + bin * (hi_ - lo_) / n_;
	}
private:
	T lo_, hi_;
	int n_;
};

// Multi even-spaced bin layout
template <int N=-1, typename T=double>
class VectorBinLayout : public IBinLayout<toon::Vector<N, T> > {
public:
	typedef toon::Vector<N,T> Measurement;
	VectorBinLayout() { }
	VectorBinLayout(const Measurement& lo,
	                const Measurement& hi,
	                const toon::Vector<N,int>& dims) {
		Configure(lo, hi, dims);
	}

	int Configure(const Measurement& lo,
	              const Measurement& hi,
	              const toon::Vector<N,int>& dims) {
		lo_ = lo;
		hi_ = hi;
		dims_ = dims;
		n_ = 1;
		for (int i = 0; i < dims_.size(); i++) {
			n_ *= dims_[i];
		}
	}

	int GetBinCount() const {
		return n_;
	}

	int GetBinIndex(const Measurement& x) const {
		int bin = 0;
		int pv = 1;
		for (int i = x.size()-1; i >= 0; i--) {
			int a = (x[i] - lo_[i] - 1e-9) * dims_[i] / (hi_[i] - lo_[i]);
			bin += Clamp<int>(a, 0, dims_[i]-1) * pv;
			pv *= dims_[i];
		}
		return bin;
	}

	Measurement GetBinExemplar(int bin) const {
		Measurement exemplar;
		int pv = n_;
		for (int i = 0; i < dims_.size(); i++) {
			pv /= dims_[i];
			exemplar[i] = lo_[i] + (0.5+(int)(bin/pv)) * (hi_[i] - lo_[i]) / dims_[i];
			bin %= pv;
		}
		return exemplar;
	}
private:
	int n_;
	VecI dims_;
	Measurement& lo_, hi_;
	toon::Vector<N,double>& a_, b_;  // w is precomputed coefficients for efficiency
	toon::Vector<N,int>& N_;
};

// Bin layout for RGB colorspace
class RgbLayout : public IBinLayout<PixelRGB<byte> > {
public:
	RgbLayout();
	RgbLayout(int na);
	RgbLayout(int nr, int ng, int nb);
	void Configure(int na);
	void Configure(int nr, int ng, int nb);
	int GetBinCount() const;
	int GetBinIndex(const PixelRGB<byte>& p) const;
	PixelRGB<byte> GetBinExemplar(int bin) const;
private:
	int n_, nr_, ng_, nb_;
};

// Represents a general histogram with an arbitrary measurement->bin mapping
template <typename T, typename BinLayout=IBinLayout<T> >
class Histogram {
public:
	Histogram() { }
	Histogram(const BinLayout& layout) {
		Configure(layout);
	}

	// Configure this histogram with a new layout
	void Configure(const BinLayout& layout) {
		layout_ = layout;
		bins_.resize(layout_.GetBinCount());
		Clear();
	}

	// Get the bin layout scheme
	const BinLayout& layout() const {
		return layout_;
	}

	// Add a value to the histogram (exits if out of range)
	void Add(const T& x) {
		IncrementBin(layout_.GetBinIndex(x));
	}

	// Add a value to a specific bin by its index
	inline void IncrementBin(int bin) {
		bins_[bin]++;
	}

	// Reset all histogram bins to zero
	void Clear() {
		fill_all(bins_, 0);
	}

	// Get the value in a bin
	inline const int& bin(int i) const { return bins_[i]; }
	inline int& bin(int i) { return bins_[i]; }

	// Get the bins vector
	const vector<int>& bins() const { return bins_; }
	vector<int>& bins() { return bins_; }
private:
	vector<int> bins_;
	BinLayout layout_;
};

// Represnts a 1D histogram
template <typename T>
class ScalarHistogram {
public:
	// Accessors
	const T& lo() const { return lo_; }
	const T& hi() const { return hi_; }
	const int bin_count() const { return n_; }

	// Initialize an empty histogram (cannot add anything)
	ScalarHistogram() { }
	// Initialize a histogram spanning the range [lo,hi] with bin_count bins
	ScalarHistogram(const T& lo, const T& hi, int bin_count) {
		Configure(lo, hi, bin_count);
	}

	// Configure a histogram spanning the range [lo,hi] with bin_count bins
	void Configure(const T& lo, const T& hi, int bin_count) {
		lo_ = lo;
		hi_ = hi;
		if (hi_ == lo_) hi_++;  // simple hack for degenerate cases
		n_ = bin_count;
		bins_.resize(bin_count);
		Clear();
	}

	// Get the index of the bin into which x falls. Does not check bounds.
	inline int GetBinIndex(const T& x) const {
		return (x - lo_ - 1e-9) * n_ / (hi_ - lo_);
	}

	// Add a value to the histogram (exits if out of range)
	void Add(const T& x) {
		bins_[GetBinIndex(x)]++;
	}

	// Reset all histogram bins to zero
	void Clear() {
		fill_all(bins_, 0);
	}

	// Get the value in a bin
	inline const int& bin(int i) const { return bins_[i]; }
	inline int& bin(int i) { return bins_[i]; }

	// Get the bins vector
	const vector<int>& bins() const { return bins_; }
	vector<int>& bins() { return bins_; }

	// Estimate the number of items in a given range
	double Integrate(double a, double b) const {
		CHECK_LE(a, b);
		double bin_width = 1.0*(hi_ - lo_) / n_;
		double a_loc = (a-lo_) / bin_width;
		double b_loc = (b-lo_) / bin_width;
		double a_pos = Clamp<double>(a_loc, 0.0, n_-1e-5);
		double b_pos = Clamp<double>(b_loc, 0.0, n_-1e-5);
		int a_bin = floori(a_pos);
		int b_bin = floori(b_pos);
		if (a_bin == b_bin) {
			return (b_pos-a_pos) * bin(a_bin);
		} else {
			double sum = (a_bin+1.0-a_pos)*bin(a_bin) + (b_pos-b_bin)*bin(b_bin);
			for (int i = a_bin+1; i < b_bin; i++) {
				sum += bin(i);
			}
			return sum;
		}
	}

	static string hashes(int k, int n) {
		return string(k, '#')+string(n-k, ' ');
	}

	// Output
	void Draw(ImageRGB<byte>& canvas) const {
		DrawHistogram(canvas, bins_, 1.0);
	}
	void OutputViz(const string& filename) const {
		ImageRGB<byte> canvas;
		Draw(canvas);
		WriteImage(filename, canvas);
	}
	void RenderText() const {
		int max_count = max(*max_element(bins_),1);
		for (int i = 0; i < n_; i++) {
			T a = lo_ + i*(hi_-lo_)/n_;
			T b = lo_ + (i+1)*(hi_-lo_)/n_;
			int nchars = bins_[i] * kTextWidth / max_count;
			DLOG << (format("(%|-3.2|-%|3.2|) %8d|")%a%b%bins_[i])
								 << hashes(nchars, kTextWidth) << "\n";
		}
	}

	// Render two histograms side-by-side
	static void RenderTextSbs(const ScalarHistogram<T>& h1,
	                          const ScalarHistogram<T>& h2,
	                          int bin_count = -1) {
		if (bin_count ==-1) {
			bin_count = h1.bin_count();
		}
		double lo = min(h1.lo(), h2.lo());
		double hi = max(h1.hi(), h2.hi());

		// Compute the integrated counts
		vector<int> h1_vals;
		vector<int> h2_vals;
		for (int i = 0; i < bin_count; i++) {
			double a = lo + i*(hi-lo)/bin_count;
			double b = lo + (i+1)*(hi-lo)/bin_count;
			h1_vals.push_back(h1.Integrate(a,b));
			h2_vals.push_back(h2.Integrate(a,b));
		}

		// Find largest bin
		int max_count = max(max(*max_element(h1_vals), *max_element(h2_vals)), 1);

		// Output
		int w = kTextWidth / 2;
		for (int i = 0; i < bin_count; i++) {
			T a = lo + i*(hi-lo)/bin_count;
			T b = lo + (i+1)*(hi-lo)/bin_count;
			int n1 = h1_vals[i] * w / max_count;
			int n2 = h2_vals[i] * w / max_count;
			DLOG << (format("(%|-3.2|-%|3.2|) %8d")%a%b%h1_vals[i])
								 << '|' << hashes(n1, w) << (format("%8d")%h2_vals[i])
								 << '|' << hashes(n2, w) << "\n";
		}
	}

private:
	vector<int> bins_;
	T lo_, hi_;
	int n_;
};



// Represents a lazily-evaluated histogram. All measurements are
// stored internally until output, so this is quite inefficient but
// convenient for profiling.
template <typename T>
class LazyHistogram {
public:
	LazyHistogram() : dirty_(true) { }

	// Add a measurement
	void Add(const T& x) {
		measurements_.push_back(x);
		dirty_ = true;
	}

	// Clears all measurements
	void Clear() {
		measurements_.clear();
		dirty_ = true;
	}

	// Return the number of measurements
	inline int size() const { measurements_.size(); }

	// Builds an internal histogram with the specified number of bins
	// and returns a reference to it. Will overwrtite
	ScalarHistogram<T>& Eval(int bin_count) {
		if (dirty_) {
			T lo, hi;
			if (measurements_.empty()) {
				lo = 0;
				hi = 1;
			} else {
				lo = *max_element(measurements_, greater<T>());
				hi = *max_element(measurements_);
			}
			hist_.Configure(lo, hi, bin_count);
			BOOST_FOREACH(const T& x, measurements_) {
				hist_.Add(x);
			}
		}
		dirty_ = false;
		return hist_;
	}

private:
	bool dirty_;
	vector<T> measurements_;
	ScalarHistogram<T> hist_;
};
}
