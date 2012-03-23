#include <boost/bind.hpp>

#include "common_types.h"

double f(double a) {
	return a*a;
}

double g(double x) {
	return x+1;
}

int main(int argc, char **argv) {
	boost::function<double(double)> fg =
		boost::bind(f, boost::bind(g, _1));
	DREPORT(fg(3));
	return 0;
}

