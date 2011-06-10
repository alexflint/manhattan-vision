#include <iostream>
#include <array>
using namespace std;
int main(int argc, char **argv) {
	array<int> xs(15);
	xs[0] = 13;
	xs[14] = 2;
	cout << *xs.begin() << endl;
	return 0;
}
