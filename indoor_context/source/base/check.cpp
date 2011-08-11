#include "check.tpp"

namespace indoor_context {
	// static
  int AssertionManager::ErrorMode(ErrorModes mode) {
		static int cur_mode = kErrorModeExit;  // default to exiting on assertion failure
		if (mode != kErrorModeCurrent) {
			cur_mode = mode;
		}
		return cur_mode;
	}

	// static
	/*bool AssertionManager::IsLiteral(const string& expr) {
		return expr[0] == '"' || expr[0] == '\'' || expr[0] == '-' || isdigit(expr[0]);
		}*/

	// virtual
	DelayedError::~DelayedError() {
		if (AssertionManager::ErrorMode() == AssertionManager::kErrorModeThrow) {
			// It is bad practice (though perfectly legal C++) to throw exceptions
			// from destructors. Unfortunately, it's the only way to accomplish
			// the syntax that looks like:
			//     CHECK(foo) << "some extra information";
			// DelayedError is never used except from CHECK() macros,
			// guaranteeing that it will never be in a std::vector or
			// similar situation in which it might cause memory leaks. Never
			// the less, CHECK(...)  should never appear in another object's
			// destructor.
			throw AssertionFailedException(ss->str());
		} else {
			cerr << ss->str() << endl;
			exit(-1);
		}
	}
}
