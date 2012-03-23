#include <TooN/so3.h>
#include <TooN/se3.h>

#include "common_types.h"

#include "vector.pb.h"

namespace indoor_context {
	// Construct a full 3x4 matrix representing a rigid 3D transform
	toon::Matrix<3,4> as_matrix(const toon::SE3<>& se3);

	// Construct a random rotation
	toon::SO3<> RandomRotation(double size);
}
