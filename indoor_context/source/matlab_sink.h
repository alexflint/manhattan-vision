#include <iostream>

#include "log.tpp"

// Prints chunks of characters to matlab using mexPrintf
class MatlabSink : public GenericCharSink {
	virtual std::streamsize write(const char* s, std::streamsize n);
};

