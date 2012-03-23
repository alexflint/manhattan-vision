#include "common_types.h"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	toon::Matrix<3,4> as_matrix(const toon::SE3<>& se3) {
		toon::Matrix<3,4> m;
		m.slice<0,0,3,3>() = se3.get_rotation().get_matrix();
		m.slice<0,3,3,1>() = se3.get_translation().as_col();
		return m;
	}

	toon::SO3<> RandomRotation(double size) {
		return SO3<>::exp(RandomVector<3>()*size);
	}
}
