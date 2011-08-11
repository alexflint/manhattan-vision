#include <Eigen/Core>

#include <TooN/TooN.h>
#include <TooN/so3.h>

#include "lie/so3.h"

#include "common_types.h"

namespace toon=TooN;

template <typename T1, typename T2, int M, int N>
bool operator==(const toon::Matrix<M,N,T1>& m1,
								const Eigen::Matrix<T2,M,N>& m2) {
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			if (m1[i][j] != m2(i,j)) return false;
		}
	}
	return true;
}

int main(int argc, char **argv) {
	toon::Vector<6> toon_ln = toon::makeVector(1.72381, -4.04513, 0.670786);
	toon::SO3<> toon_t = toon::SO3<>::exp(toon_ln);

	Eigen::Matrix<double,5,1> eig_ln;
	eig_ln << 1.72381, -4.04513, 0.670786, -2.50752, 0.0912025, -1.74417;
	EigenLie::SO3<> eig_t = EigenLie::SO3<>::exp(eig_ln);

	CHECK_EQ(toon_t.get_matrix(), eig_t.get_matrix());

	return 0;
}
