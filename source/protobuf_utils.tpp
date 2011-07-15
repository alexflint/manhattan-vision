#include <stdio.h>

#include "common_types.h"

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace indoor_context {
	// Write a protocol buffer to a file
	template <typename Proto>
	void WriteProto(const string& path, const Proto& p) {
		ofstream s(path);
		CHECK(p.SerializeToOstream(&s)) << "Failed to write protocol buffer to " << path;
	}

	// Read a protocol buffer from a file
	template <typename Proto>
	void ReadProto(const string& path, Proto& p) {
		ifstream s(path.c_str(), ios::binary);
		CHECK(p.ParseFromIstream(&s)) << "Failed to read protocol buffer from " << path;
	}

	// Read a protocol buffer larger than 64MB (ignores security restrictions)
	template <typename Proto>
	void ReadLargeProto(const string& path, Proto& p) {
		// Open the file and get the file number
		FILE* fd = fopen(path.c_str(), "r");
		CHECK_NOT_NULL(fd) << "Failed to open " << path;
		int fno = fileno(fd);
		CHECK(fno != -1) << "Could not get file number for " << path;

		// Construct and configure a Coded Input Stream
		google::protobuf::io::FileInputStream filestr(fno);
		google::protobuf::io::CodedInputStream codedstr(&filestr);
		codedstr.SetTotalBytesLimit(1e+9, 1e+9);  // Don't warn about large file sizes

		// Parse the message
		CHECK(p.ParseFromCodedStream(&codedstr)) 
			<< "Failed to read protocol buffer from " << path;
	}
}
