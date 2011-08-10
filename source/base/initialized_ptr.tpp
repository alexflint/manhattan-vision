#include <boost/scoped_ptr.hpp>

namespace indoor_context {
	// An initialized_ptr is just like a scoped_ptr except that it
	// initializes itself with "new T" upon construction. This means
	// that T must have a default constructor. These two are exactly the
	// same:
	//   initialized_ptr<FooType> x;
	//   scoped_ptr<FooType> x(new FooType);
	template <typename T>
	class initialized_ptr : public scoped_ptr<T> {
	public:
		explicit initialized_ptr() : scoped_ptr<T>(new T) { };
		explicit initialized_ptr(T* p) : scoped_ptr<T>(p) { };
  };
}
