#include <iostream>
#include <fstream>

#include <se3.h>
#include <so3.h>

using namespace std;
using namespace TooN;

int main(int argc, char **argv) {
	if (argc != 2) {
		cout << "Usage: "<<argv[0]<<" POSEFILE";
		return -1;
	}

	ifstream input(argv[1]);
	Vector<6> v;
	input >> v;
	SE3<> cfromw = SE3<>::exp(v);

	SO3<> m = cfromw.get_rotation();
	SO3<> minv = m.inverse();

	cout << v << "\n\n" << cfromw << "\n\n" << m << "\n\n" << minv << endl;

	return 0;
}
