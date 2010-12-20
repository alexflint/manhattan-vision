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
using namespace toon;

int main(int argc, char **argv) {
	Vector<-1> v = makeVector(1);
	Vector<-1> u = makeVector(2,2);
	if (v == u) {
		return 1;
	} else {
		return 0;
	}
	cout << "done";
}
