#pragma once

#include <string>
#include <iostream>

// Provides a generic object wrapper that allows streaming the wrapped
// object to output streams.
namespace indoor_context {
	// A thin wrapper around boost::ptr_vector<Streamable>
	class Streamable;
	class VectorOfStreamableImpl;
	class VectorOfStreamable {
	public:
		VectorOfStreamable();
		VectorOfStreamable(int capacity);
		~VectorOfStreamable();
		int Size() const;
		void Append(Streamable* x);
		Streamable& Get(int i) const;
	private:
		VectorOfStreamableImpl* pimpl;
	};

	// Represents a streamable object
	class Streamable {
	public:
		// vector is actually boost::ptr_vector<Streamable>
		// We hide it here to speed up compilation
		typedef VectorOfStreamable vector;
		virtual ~Streamable();
		virtual std::ostream& write(std::ostream& o) const = 0;

		// Factory method
		template <typename T> static Streamable* New(const T& x);

		// Factory methods for vectors of streamable objects
		template <typename A>
		static vector* NewVector(const A& a);

		template <typename A, typename B>
		static vector* NewVector(const A& a, const B& b);

		template <typename A, typename B, typename C>
		static vector* NewVector(const A& a,	const B& b,	const C& c);

		template <typename A, typename B, typename C, typename D>
		static vector* NewVector(const A& a, const B& b, const C& c, const D& d);

		template <typename A, typename B, typename C, typename D,
							typename E>
		static vector* NewVector(const A& a, const B& b, const C& c, const D& d,
														 const E& e);

		template <typename A, typename B, typename C, typename D,
							typename E, typename F>
		static vector* NewVector(const A& a, const B& b, const C& c, const D& d,
														 const E& e, const F& f);

		template <typename A, typename B, typename C, typename D,
							typename E, typename F, typename G>
		static vector* NewVector(const A& a, const B& b, const C& c, const D& d,
														 const E& e, const F& f, const G& g);

		template <typename A, typename B, typename C, typename D,
							typename E, typename F, typename G, typename H>
		static vector* NewVector(const A& a, const B& b, const C& c, const D& d,
														 const E& e, const F& f, const G& g, const H& h);
	};

	// Stream insertion
	std::ostream& operator<<(std::ostream& o, const Streamable& s);

	// Streamable implementation template
	template <typename T>
	class StreamableImpl : public Streamable {
	public:
		T x_;
		StreamableImpl(const T& x) : x_(x) { }
		~StreamableImpl() { }
		std::ostream& write(std::ostream& o) const { return o << x_; }
	};

	template <typename T>	Streamable* Streamable::New(const T& x) {
		return new StreamableImpl<T>(x);
	}

	template <typename A>
	Streamable::vector* Streamable::NewVector(const A& a) {
		Streamable::vector* v = new Streamable::vector(1);
		v->Append(Streamable::New(a));
		return v;
	}

	template <typename A, typename B>
	Streamable::vector* Streamable::NewVector(const A& a,
																						const B& b) {
		Streamable::vector* v = new Streamable::vector(2);
		v->Append(Streamable::New(a));
		v->Append(Streamable::New(b));
		return v;
	}

	template <typename A, typename B, typename C>
	Streamable::vector* Streamable::NewVector(const A& a,
																						const B& b,
																						const C& c) {
		Streamable::vector* v = new Streamable::vector(3);
		v->Append(Streamable::New(a));
		v->Append(Streamable::New(b));
		v->Append(Streamable::New(c));
		return v;
	}

	template <typename A, typename B, typename C, typename D>
	Streamable::vector* Streamable::NewVector(const A& a,
																						const B& b,
																						const C& c,
																						const D& d) {
		Streamable::vector* v = new Streamable::vector(4);
		v->Append(Streamable::New(a));
		v->Append(Streamable::New(b));
		v->Append(Streamable::New(c));
		v->Append(Streamable::New(d));
		return v;
	}

	template <typename A, typename B, typename C, typename D, typename E>
	Streamable::vector* Streamable::NewVector(const A& a,
																						const B& b,
																						const C& c,
																						const D& d,
																						const E& e) {
		Streamable::vector* v = new Streamable::vector(5);
		v->Append(Streamable::New(a));
		v->Append(Streamable::New(b));
		v->Append(Streamable::New(c));
		v->Append(Streamable::New(d));
		v->Append(Streamable::New(e));
		return v;
	}

	template <typename A, typename B, typename C, typename D, typename E, typename F>
	Streamable::vector* Streamable::NewVector(const A& a,
																						const B& b,
																						const C& c,
																						const D& d,
																						const E& e,
																						const F& f) {
		Streamable::vector* v = new Streamable::vector(6);
		v->Append(Streamable::New(a));
		v->Append(Streamable::New(b));
		v->Append(Streamable::New(c));
		v->Append(Streamable::New(d));
		v->Append(Streamable::New(e));
		v->Append(Streamable::New(f));
		return v;
	}

	template <typename A, typename B, typename C, typename D, typename E, typename F, typename G>
	Streamable::vector* Streamable::NewVector(const A& a,
																						const B& b,
																						const C& c,
																						const D& d,
																						const E& e,
																						const F& f,
																						const G& g) {
		Streamable::vector* v = new Streamable::vector(7);
		v->Append(Streamable::New(a));
		v->Append(Streamable::New(b));
		v->Append(Streamable::New(c));
		v->Append(Streamable::New(d));
		v->Append(Streamable::New(e));
		v->Append(Streamable::New(f));
		v->Append(Streamable::New(g));
		return v;
	}

	template <typename A, typename B, typename C, typename D,
						typename E, typename F, typename G, typename H>
	Streamable::vector* Streamable::NewVector(const A& a,
																						const B& b,
																						const C& c,
																						const D& d,
																						const E& e,
																						const F& f,
																						const G& g,
																						const H& h) {
		Streamable::vector* v = new Streamable::vector(8);
		v->Append(Streamable::New(a));
		v->Append(Streamable::New(b));
		v->Append(Streamable::New(c));
		v->Append(Streamable::New(d));
		v->Append(Streamable::New(e));
		v->Append(Streamable::New(f));
		v->Append(Streamable::New(g));
		v->Append(Streamable::New(h));
		return v;
	}
}
