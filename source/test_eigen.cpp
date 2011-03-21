#include <iostream>
#include <Eigen/Core>

#include "entrypoint_types.h"
#include "timer.h"

int main(int argc, char **argv) {
	Eigen::Matrix3f m = Eigen::Matrix3f::Random();
	Eigen::Vector3f x = Eigen::Vector3f::Random();

	TIMED("eigen")
		for (int i = 0; i < 1000000; i++) {
			Eigen::Vector3f y = m * x;
			if (y == Eigen::Vector3f::Zero()) {
				std::cout << "bar\n";
				x = Eigen::Vector3f::Zero();
			}
		}

	Mat3 mm = Identity;
	Vec3 xx = Ones;
	Vec3 bb = Zeros;
	TIMED("toon") {
		for (int i = 0; i < 1000000; i++) {
			Vec3 y = mm * xx;
			if (y == bb) {
				DLOG << "foo";
			}
		}
	}

	return 0;
}
