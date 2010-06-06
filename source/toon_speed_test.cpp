/*
 * toon_speed_test.cpp
 *
 *  Created on: 2 Jun 2010
 *      Author: alexf
 */

#include <TooN/TooN.h>
//#include "common_types.h"
//#include "timer.h"

using namespace TooN;
//using namespace indoor_context;

const int kRepeat = 10000000;

int main(int argc, char **argv) {
	Vector<2> v = makeVector(0.0, 1.0);
	//TIMED("toon")
	for (int i = 0; i < kRepeat; i++) {
		v[0] = v[1];
		v[1]++;
		if (v[1]==v[0]) v[1]++;
	}

	double u[] = {0.0, 1.0};
	//TIMED("array")
	for (int i = 0; i < kRepeat; i++) {
		u[0] = u[1];
		u[1]++;
		if (u[1]==u[0]) u[1]++;
	}

	return 0;
}
