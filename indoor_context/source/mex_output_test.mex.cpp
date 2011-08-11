#include <iostream>
#include <mex.h>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

#include "entrypoint_types.h"

namespace bi=boost::iostreams;

struct repeating_filter {
	typedef char char_type;
	struct category : bi::output_filter_tag /*, bi::flushable_tag*/ { };

	template<typename Sink>
	bool put(Sink& dest, int c)  {
		//bi::put(dest, c);
		bi::put(dest, c);
		return true;
	}

	/*
	template<typename Sink>
	bool flush(Sink& dest) {
		return true;
		}*/
};

void _mexFunction(int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]) {
	repeating_filter filter;
	bi::filtering_ostream o;
	o.push(filter);
	o.push(bi::file_descriptor_sink(STDOUT_FILENO, bi::never_close_handle));
	
	//o << "abc\n";
	DLOG << "first line\n";
	//bool success = o.strict_sync();
	//LogManager::Flush();
	mexPrintf("<-- mexPrintf -->\n");

	//cout << "(flush " << (success ? "succeeded" : "failed") << ")\n";
}
