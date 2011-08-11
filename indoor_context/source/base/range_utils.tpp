// Provides STL-esque functions that operate on ranges rather than
// iterators, allowing much more concise code.

#pragma once

#include <algorithm>
#include <numeric>

#include <boost/iterator.hpp>
#include <boost/range.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>

#include "common_types.h"

namespace indoor_context {

	// Use anonymous namespace to include only within this file
	namespace {	using namespace boost; }

// Get the next iterator
template <typename Iterator>
Iterator successor(const Iterator& it) {
	Iterator jt = it;
	jt++;
	return jt;
}

// Get the previous iterator
template <typename Iterator>
Iterator predecessor(const Iterator& it) {
	Iterator jt = it;
	jt--;
	return jt;
}

// Get the first element of a pair
template <typename Pair>
typename Pair::first_type select1st(const Pair& a) {
	return a.first;
}

// Get the second element of a pair
template <typename Pair>
typename Pair::second_type select2nd(const Pair& a) {
	return a.second;
}

// Get the maximum element
template <class Iterator>
Iterator max_element_n(Iterator begin, const int n) {
	Iterator end = begin;
	advance(end, n);
	return max_element(begin, end);
}

// Get the maximum element
template <class Iterator, class Pred>
Iterator max_element_n(Iterator begin, const int n, Pred comp) {
	Iterator end = begin;
	advance(end, n);
	return max_element(begin, end, comp);
}

// Get the index of the maximum element
template <class Iterator>
unsigned max_index(Iterator begin, Iterator end) {
	return distance(begin, max_element(begin, end));
}

// Get the index of the maximum element
template <class Iterator, class Pred>
unsigned max_index(Iterator begin, Iterator end, Pred comp) {
	return distance(begin, max_element(begin, end, comp));
}

// Get the index of the maximum element
template <class Iterator>
unsigned max_index_n(Iterator begin, const int n) {
	Iterator end = begin;
	advance(end, n);
	return max_index(begin, end);
}

// Get the index of the maximum element
template <class Iterator, class Pred>
unsigned max_index_n(Iterator begin, const int n, Pred comp) {
	Iterator end = begin;
	advance(end, n);
	return max_index(begin, end, comp);
}

// Get an iterator for the maximum element in a range (const version)
template <typename Range>
typename range_iterator<const Range>::type
max_element(const Range& r) {
	return max_element(begin(r), end(r));
}

// Get an iterator for the maximum element in a range (non-const version)
template <typename Range>
typename range_iterator<Range>::type
max_element(Range& r) {
	return max_element(begin(r), end(r));
}

// Get an iterator for the maximum element in a range (const version)
template <typename Range, typename Pred>
typename range_iterator<const Range>::type
max_element(const Range& r, Pred comp) {
	return max_element(begin(r), end(r), comp);
}

// Get an iterator for the maximum element in a range (non-const version)
template <typename Range, typename Pred>
typename range_iterator<Range>::type
max_element(Range& r, Pred comp) {
	return max_element(begin(r), end(r), comp);
}

// Get an iterator for the minimum element in a range (const version)
template <typename Range>
typename range_iterator<const Range>::type
min_element(const Range& r) {
	return max_element(r, greater<typename range_value<Range>::type>());
}

// Get an iterator for the minimum element in a range (non-const version)
template <typename Range>
typename range_iterator<Range>::type
min_element(Range& r) {
	return max_element(r, greater<typename range_value<Range>::type>());
}

// Turn an array into a range
template <typename T>
iterator_range<T*> ptr_range(T* a, T* b) {
	return iterator_range<T*>(a, b);
}

// Turn an array into a const range
template <typename T>
iterator_range<const T*> ptr_range(const T* a, const T* b) {
	return iterator_range<const T*>(a, b);
}

// Turn an array into a range
template <typename T>
iterator_range<T*> array_range(T* a, unsigned n) {
	return ptr_range(a, a+n);
}

// Turn an array into a range
template <typename T>
iterator_range<const T*> array_range(const T* a, unsigned n) {
	return ptr_range(a, a+n);
}

// We often want to sort a list according to some function of its
// elements. For example, we might have a list of Cat instances and we
// want to sort based on the return value of the num_kittens()
// member. We can achieve this by passing a comparison function to
// std::sort that applies some transformation to both of its arguments
// and then compares the results using some other comparison
// function. If the transform is a one-argument function G and the
// comparison between results is a two-argument function F then the
// comparator will have the form
//   comp(x,y) = F(G(x), G(y)).
// Use compose_twice to achieve this.

// Compose a two-parameter function F and a one-parameter function G
// together to create H(x,y) = F(G(x), G(y))
template <typename F, typename G>
class composer_twice {
private:
	F f;
	G g;
public:
	composer_twice(F ff, G gg) : f(ff), g(gg) { }
	template <typename T>
	bool operator()(const T& x, const T& y) const {
		return f(g(x), g(y));
	}
};

// Compose a two-parameter function F and a one-parameter function G
// together to create H(x,y) = F(G(x), G(y))
template <typename F, typename G>
composer_twice<F, G> compose_twice(F f = F(), G g = G()) {
	return composer_twice<F, G>(f, g);
}

// Transform all elements in a container
template <typename InputRange,
typename OutputIterator,
typename Operation>
void transform_all(const InputRange& input,
                   OutputIterator output,
                   const Operation op) {
	transform(begin(input), end(input), output, op);
}

// Transform all elements in a container
template <typename InputRange,
typename OutputContainer,
typename Operation>
void transform_all_into(const InputRange& input,
                        OutputContainer& output,
                        const Operation op) {
	transform_all(input, back_inserter(output), op);
}

// Copy all elements in a container
template <typename InputRange, typename OutputIterator>
void copy_all(const InputRange& input, OutputIterator output) {
	copy(begin(input), end(input), output);
}


// Copy all elements in a container
template <typename InputRange, typename OutputContainer>
void copy_all_into(const InputRange& input, OutputContainer& output) {
	copy_all(input, back_inserter(output));
}

template <typename Range>
typename range_iterator<const Range>::type
find_all(const Range& r, const typename range_value<Range>::type& x) {
	return find(begin(r), end(r), x);
}

// Sort all elements of a contains
template <typename Range>
void sort_all(Range& range) {
	sort(begin(range), end(range));
}

// Sort all elements of a container
template <typename Range, typename Comp>
void sort_all(Range& range, Comp comp) {
	sort(begin(range), end(range), comp);
}

// Fill a range with a value
template <typename Range, typename T>
void fill_all(Range& range, const T& x) {
	fill(begin(range), end(range), x);
}

// Fill a range with start,start+1,start+2,...
template <typename OutputIterator, typename T>
void iota_n(OutputIterator it, int n, T start = 0) {
	for (int i = 0; i < n; i++) {
		it++ = start++;
	}
}

// Fill a range with start,start+1,start+2,...
template <typename Range, typename T>
void iota_all(Range& range, T start = 0) {
	iota(begin(range), end(range), start);
}

// Shuffle a range
template <typename Range>
void random_shuffle_all(Range& range) {
	random_shuffle(begin(range), end(range));
}

// Accumulate all elements in a range
template <typename Range, typename T>
T accumulate_all(const Range& range, const T& init) {
	return accumulate(begin(range), end(range), init);
}

// Represents a function object that maps indices to values according
// to a specified range
template <typename Range>
struct mapper {
	typedef typename range_value<Range>::type Element;
	const Range& map;
	mapper(const Range& m) : map(m) { }
	Element operator()(const int i) const {
		return map[i];
	}
};

// Make a function object that maps keys to values according to a
// specified map
template <typename Range>
mapper<Range> make_mapper(const Range& map) {
	return mapper<Range>(map);
}

// Get the sorted order of elements in a range without modifying the
// range itself
template<typename Range>
void sorted_order(const Range& range, vector<int>& order) {
	typedef typename range_value<Range>::type Element;
	sorted_order(range, order, less<Element>());
}

// Get the sorted order of elements in a range without modifying the
// range itself
template<typename Range, typename Comp>
void sorted_order(const Range& range, vector<int>& order, Comp comp) {
	iota_n(back_inserter(order), size(range), 0);
	sort_all(order, compose_twice(comp, make_mapper(range)));
}

// Get the sorted order of elements in a range without modifying the
// range itself
template<typename Range, typename Comp>
void sorted_order(const Range& range, vector<int>& order) {
	typedef typename range_value<Range>::type Element;
	sorted_order(range, order, less<Element>());
}

// Perform a binary search on a range
template <typename Range, typename T>
bool binary_search_all(const Range& range, const T& x) {
	return binary_search(begin(range), end(range), x);
}

// Perform a binary search on a range
template <typename Range, typename T, typename Comp>
bool binary_search_all(const Range& range, const T& x, Comp comp) {
	return binary_search(begin(range), end(range), x, comp);
}




// Copy from an input stream until EOF
template <typename Stream, typename Container>
void read_all(Stream& stream, Container& container) {
	typedef typename range_value<Container>::type Element;
	istream_iterator<Element> eof;
	copy(istream_iterator<Element>(stream), eof, back_inserter(container));
}

// Write all elements of a range to a stream
template <typename Range, typename Stream>
void write_all(const Range& range, Stream& stream, const char* sep = " ") {
	typedef typename range_value<Range>::type Element;
	ostream_iterator<Element> iter(stream, sep);
	copy_all(range, iter);
}

// Write all elements of a vector to a stream
template <typename Range>
void write_all(const Range& range, const char* sep = " ") {
	typedef typename range_value<Range>::type Element;
	ostream_iterator<Element> iter(cout, sep);
	copy_all(range, iter);
}

// Write all elements of a range to a stream
template <typename Range, typename Stream>
void write_all(const Range& range, Stream& stream, const string& sep) {
	write_all(range, stream, sep.c_str());
}

// Write all elements of a vector to a stream
template <typename Range>
void write_all(const Range& range, const string& sep) {
	write_all(range, sep.c_str());
}

// Divide a sequence in two by cutting it in the middle
template<typename Range, typename OutputIterator>
void PartitionCut(const Range& range, OutputIterator out1, OutputIterator out2,
                  const int& w1, const int& w2) {
	typedef typename range_const_iterator<Range>::type Iterator;
	int n = size(range);
	int cut = w1 * n / (w1 + w2);
	Iterator it = const_begin(range);
	for (int i = 0; i < n; i++) {
		*(i < cut ? out1 : out2)++ = *it++;
	}
}

// Divide a sequence in two by alternating throughout the sequence
template<typename Range, typename OutputIterator>
void PartitionInterleaved(const Range& range, OutputIterator out1,
                          OutputIterator out2, const int& w1, const int& w2) {
	typedef typename range_value<Range>::type T;
	int n1 = 0, n2 = 0;
	BOOST_FOREACH(const T& x, range) {
		bool flag;
		if (w2*n1 == w1*n2) {
			flag = w1 > w2;
		} else {
			flag = w2*n1 < w1*n2;
		}
		*(flag ? out1 : out2)++ = x;
		(flag ? n1 : n2)++;
	}
}
}


// Tell boost how to treat TooN vectors as ranges.
namespace boost {
// Putting this stuff into the boost namespace is actually the
// _reccommended_ way to do this according to
// http://www.boost.org/doc/libs/1_39_0/libs/range/doc/boost_range.html#method2

// An iterator for a vector is a pointer
template <int N, typename T>
struct range_mutable_iterator<TooN::Vector<N,T> > {
	typedef T* type;
};

// An iterator for a vector is a pointer
template <int N, typename T>
struct range_const_iterator<TooN::Vector<N,T> > {
	typedef const T* type;
};
}

namespace TooN {
// Putting this stuff into the same namespace as the original class
// _is_ actually the reccommended way to do this (according to
// http://www.boost.org/doc/libs/1_39_0/libs/range/doc/boost_range.html#method2)

// Here we deal with statically sized vectors. As usual, the "end"
// is one past the last element.
template <int N, typename T>
inline T* range_begin(TooN::Vector<N,T>& v) {
	return &v[0];
}
template <int N, typename T>
inline const T* range_begin(const TooN::Vector<N,T>& v) {
	return &v[0];
}
template <int N, typename T>
inline T* range_end(TooN::Vector<N,T>& v) {
	return &v[0]+N;
}
template <int N, typename T>
inline const T* range_end(const TooN::Vector<N,T>& v) {
	return &v[0]+N;
}

// Here we deal with dynamically sized vectors, in which we have to
// call size() rather then use the size template parameter. As
// usual, the "end" is one past the last element.
template <typename T>
inline T* range_begin(TooN::Vector<TooN::Dynamic,T>& v) {
	return &v[0];
}
template <typename T>
inline const T* range_begin(const TooN::Vector<TooN::Dynamic,T>& v) {
	return &v[0];
}
template <typename T>
inline T* range_end(TooN::Vector<TooN::Dynamic,T>& v) {
	return &v[0]+v.size();
}
template <typename T>
inline const T* range_end(const TooN::Vector<TooN::Dynamic,T>& v) {
	return &v[0]+v.size();
}
}
