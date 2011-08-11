#include <iostream>

#include "matlab_sink.h"

#include <unistd.h>

#include <string.h>
#include <mex.h>

std::streamsize MatlabSink::write(const char* s, std::streamsize n) {
	char* buf = new char[n+1];
	strncpy(buf, s, n);
	buf[n] = '\0';
	mexPrintf(buf);
	fsync(STDOUT_FILENO);  // only works when matlab is in console mode,
												 // but in windows mode this does no harm.
	delete[] buf;
	return n;
}
