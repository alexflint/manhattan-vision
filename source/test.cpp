/*
 * test.cpp
 *
 *  Created on: 2 Aug 2010
 *      Author: alexf
 */

#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <exception>

#include "common_types.h"

using namespace std;
using namespace indoor_context;

int main(int argc, char **argv) {
	if (argc > 1) {
		AssertionManager::ErrorMode(AssertionManager::kErrorModeThrow);
	}

	int x = 1, y = 2, z = 3;

	try {
		CHECK_EQ(x+y, z) << "this bit is okay";
		CHECK_LT(y, x) << "this bit is bad";
	} catch (const std::exception& ex) {
		cout << "Caught exception:\n" << ex.what() << endl;
	}

	AssertionManager::ErrorMode(AssertionManager::kErrorModeExit);
	CHECK(false) << "this bit exits";

	cout << "Finished.\n";

	return 0;
}
