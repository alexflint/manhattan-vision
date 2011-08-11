#include "streamable.tpp"

#include <boost/ptr_container/ptr_vector.hpp>

namespace indoor_context {
	typedef boost::ptr_vector<Streamable> VectorImpl;


	class VectorOfStreamableImpl {
	public:
		VectorOfStreamableImpl() { }
		VectorOfStreamableImpl(int capacity) : v(capacity) { }
		boost::ptr_vector<Streamable> v;
	};



	Streamable::~Streamable() {
	}

	std::ostream& operator<<(std::ostream& o, const Streamable& s) {
		return s.write(o);
	}



	VectorOfStreamable::VectorOfStreamable() :
		pimpl(new VectorOfStreamableImpl) {
	}

	VectorOfStreamable::VectorOfStreamable(int capacity) :
		pimpl(new VectorOfStreamableImpl(capacity)) {
	}

	VectorOfStreamable::~VectorOfStreamable() {
		// This is one of the few places that I don't use
		// boost::scoped_ptr due to the very large number of files that
		// include this file.
		delete pimpl;
	}

	int VectorOfStreamable::Size() const {
		return pimpl->v.size();
	}

	void VectorOfStreamable::Append(Streamable* x) {
		pimpl->v.push_back(x);
	}

	Streamable& VectorOfStreamable::Get(int i) const {
		return pimpl->v[i];
	}
}
