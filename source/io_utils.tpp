#pragma once

#include <sstream>
#include <iomanip>

#include <boost/lexical_cast.hpp>

#include "common_types.h"
#include "safe_stream.h"

#include "range_utils.tpp"

namespace indoor_context {
	using boost::lexical_cast;

	// Convert an integer to a string of length W, padding on the left
	// with '0' chars.
	template <typename T>
	string itoa(const T& x, int width=1) {
		stringstream ss;
		ss << setfill('0') << setw(width) << x;
		return ss.str();
	}

	// Read an object from a file
	template <typename T>
	void ReadFile(const char* file, T& x) {
		sifstream input(file);
		input >> x;
		input.close();
	}

	// Read an object from a file
	template <typename T>
	void ReadFile(const string& file, T& x) {
		ReadFile(file.c_str(), x);
	}

	// Write an object to a file
	template <typename T>
	void WriteFile(const char* file, const T& x) {
		sofstream output(file);
		output << x;
		output.close();
	}

	// Write an object to a file
	template <typename T>
	void WriteFile(const string& file, const T& x) {
		WriteFile(file.c_str(), x);
	}

	// Write a matrix to a file
	template <typename T>
	void WriteMatrix(const char* file, const VNL::Matrix<T>& m) {
		sofstream output(file);
		output << m.Rows() << " " << m.Cols() << endl;
		output << m;
		output.close();
	}

	// Write a matrix to a file
	template <typename T>
	void WriteMatrix(const string& file, const VNL::Matrix<T>& m) {
		WriteMatrix(file.c_str(), m);
	}

	// Read a matrix from a file
	template <typename T>
	VNL::Matrix<T>* ReadMatrix(const char* file) {
		int r, c;
		sifstream input(file);
		input >> r >> c;
		VNL::Matrix<T>* m = new VNL::Matrix<T>(r, c);
		input >> *m;
		input.close();
		return m;
	}

	// Read a matrix from a file
	template <typename T>
	VNL::Matrix<T>* ReadMatrix(const string& file) {
		return ReadMatrix<T>(file.c_str());
	}

	// Read a matrix from a file
	template <typename T>
	void ReadMatrix(const char* file, VNL::Matrix<T>& m) {
		int r, c;
		sifstream input(file);
		input >> r >> c;
		if (m.Rows() > 0 || m.Cols() > 0) {
			assert(r == m.Rows());
			assert(c == m.Cols());
		} else {
			m.Resize(r, c);
		}
		input >> m;
		input.close();
	}

	// Read a matrix from a file
	template <typename T>
	void ReadMatrix(const string& file, VNL::Matrix<T>& m) {
		ReadMatrix(file.c_str(), m);
	}

	// Read a list of vectors from a file
	template <typename T>
	void ReadVecs(const char* file, vector<VNL::Vector<T> >& vs) {
		sifstream input(file);
		int dim;
		input >> dim;
		VNL::Vector<T> v(dim);
		while (input >> v) {
			vs.push_back(v);
		}
		input.close();
	}

	// Read a list of vectors from a file
	template <typename T>
	void ReadVecs(const string& file, vector<VNL::Vector<T> >& vs) {
		ReadVecs(file.c_str(), vs);
	}

	// Write a list of vectors to a file
	template <typename T>
	void WriteVecs(const char* file, const vector<VNL::Vector<T> >& vs) {
		sofstream output(file);
		if (!vs.empty()) {
			output << vs[0].Size() << endl;
			BOOST_FOREACH(const VNL::Vector<T>& v, vs) {
				if (v.Size() != vs[0].Size()) {
					cout << "Warning: vector length mismatch "
							 << "(" << v.Size() << " vs " << vs[0].Size() << ")"
							 << " when writing to " << file << endl;
				}
				output << v << endl;
			}
		}
	}

	// Write a list of vectors to a file
	template <typename T>
	void WriteVecs(const string& file, const vector<VNL::Vector<T> >& vs) {
		WriteVecs(file.c_str(), vs);
	}




	// Read a list of vectors from a file
	template <typename T>
	void ReadVecs(const char* file, vector<toon::Vector<toon::Dynamic, T>  >& vs) {
		sifstream input(file);
		int dim;
		input >> dim;
		toon::Vector<toon::Dynamic, T> v(dim);
		while (input >> v) {
			vs.push_back(v);
		}
		input.close();
	}

	// Read a list of vectors from a file
	template <typename T>
	void ReadVecs(const string& file, vector<toon::Vector<toon::Dynamic, T> >& vs) {
		ReadVecs(file.c_str(), vs);
	}

	// Write a list of vectors to a file
	template <typename T>
	void WriteVecs(const char* file, const vector<toon::Vector<toon::Dynamic, T> >& vs) {
		typedef toon::Vector<toon::Dynamic, T> toonVecT;
		sofstream output(file);
		if (!vs.empty()) {
			output << vs[0].size() << endl;
			BOOST_FOREACH(const toonVecT& v, vs) {
				if (v.size() != vs[0].size()) {
					cout << "Warning: vector length mismatch "
							 << "(" << v.size() << " vs " << vs[0].size() << ")"
							 << " when writing to " << file << endl;
				}
				output << v << endl;
			}
		}
	}

	// Write a list of vectors to a file
	template <typename T>
	void WriteVecs(const string& file, const vector<toon::Vector<toon::Dynamic, T> >& vs) {
		WriteVecs(file.c_str(), vs);
	}

	// Convert one object to another by passing through a
	// stringstream. The result is returned by copy, and both a stream
	// insertion and extraction are performed along the way, so this is
	// not very efficient.
	template <typename T, typename U>
	T stream_to(const U& x) {
		T y;
		stringstream ss;
		ss << x;
		ss >> y;
		return y;
	}

	// Convert one object to another by passing through a
	// stringstream. The result is returned in the second argument. A
	// stream insertion and extraction are performed along the way, so
	// this is not very efficient.
	template <typename T, typename U>
	void stream_into(const T& x, U& y) {
		stringstream ss;
		ss << x;
		ss >> y;
	}

	// Consume an item from a stream using stream extraction
	template <typename T, typename Stream>
	T consume(Stream& s) {
		T x;
		s >> x;
		return x;
	}

	// Construct an EOF iterator
	template <typename InputIterator>
	InputIterator end_iterator() {
		InputIterator eof;  // The EOF iterator for istream_iterators is
		// created with no constructor params
		return eof;
	}

	// Represents a container wrapper that outputs each element in the
	// container to a stream
	template <typename Range>
	struct iowrapper {
		const Range& range;
		const string sep;
		iowrapper(const Range& range_, const string& sep_ = " ")
			: range(range_), sep(sep_) { }
	};

	template <typename Range>
	ostream& operator<<(ostream& o, const iowrapper<Range>& x) {
		write_all(x.range, o, x.sep);
		return o;
	}

	template <typename Range>
	iowrapper<Range> iowrap(const Range& range) {
		return iowrapper<Range>(range);
	}

	template <typename Range>
	iowrapper<Range> iowrap(const Range& range, const string& sep) {
		return iowrapper<Range>(range, sep);
	}

	template <typename T>
	void Print(const T& x) {
		DLOG << x;
	}

	// Parse a string of the form "1-6 10 11 24:2:50"
	template <typename T>
	void ParseMultiRange(const string& s, vector<T>& v) {
		int a = 0, pos;
		do {
			pos = min(s.find(',', a), s.find(' ', a));
			int b = (pos == string::npos) ? s.size() : pos;
			string token = s.substr(a, b-a);
			int dash = token.find('-');
			int colon = token.find(':');
			if (dash != string::npos && dash != 0) {  // if dash == 0 then we have a negative number
				T first = lexical_cast<T>(token.substr(0, dash));
				T last = lexical_cast<T>(token.substr(dash+1));
				for (int j = first; j <= last; j++) {
					v.push_back(j);
				}
			} else if (colon != string::npos) {
				int colon2 = token.find(':', colon+1);
				T first, last, stride;
				if (colon2 == string::npos) {
					first = lexical_cast<T>(token.substr(0, colon));
					last = lexical_cast<T>(token.substr(colon+1));
					stride = 1;
				} else {
					first = lexical_cast<T>(token.substr(0, colon));
					stride = lexical_cast<T>(token.substr(colon+1, colon2-colon-1));
					last = lexical_cast<T>(token.substr(colon2+1));
				}
				for (T j = first; j <= last; j += stride) {
					v.push_back(j);
				}
			} else {
				v.push_back(lexical_cast<T>(token));
			}
			a = b+1;
		} while (pos != string::npos);
	}

	template <typename T>
	vector<T> ParseMultiRange(const string& s) {
		vector<T> v;
		ParseMultiRange(s, v);
		return v;
	}
}
