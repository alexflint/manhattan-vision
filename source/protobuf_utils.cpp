#include "protobuf_utils.h"

#include <stdio.h>

#include <fstream>

#include <google/protobuf/message.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "common_types.h"
#include "vector.pb.h"

namespace indoor_context {
	using namespace toon;

	void WriteProto(const string& path, const google::protobuf::Message& p) {
		ofstream s(path.c_str());
		CHECK(p.SerializeToOstream(&s))
			<< "Failed to write protocol buffer to " << path;
	}

	// Read a protocol buffer from a file
	void ReadProto(const string& path, google::protobuf::Message& p) {
		ifstream s(path.c_str(), ios::binary);
		CHECK(p.ParseFromIstream(&s))
			<< "Failed to read protocol buffer from " << path;
	}

	// Read a protocol buffer larger than 64MB (ignores security restrictions)
	void ReadLargeProto(const string& path, google::protobuf::Message& p) {
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

		// Close file
		fclose(fd);
	}

	Vec2 asToon(const proto::Vec2& v) {
		return makeVector(v.x1(), v.x2());
	}

	Vec3 asToon(const proto::Vec3& v) {
		return makeVector(v.x1(), v.x2(), v.x3());
	}

	Vec4 asToon(const proto::Vec4& v) {
		return makeVector(v.x1(), v.x2(), v.x3(), v.x4());
	}

	Vec5 asToon(const proto::Vec5& v) {
		return makeVector(v.x1(), v.x2(), v.x3(), v.x4(), v.x5());
	}

	Vec6 asToon(const proto::Vec6& v) {
		return makeVector(v.x1(), v.x2(), v.x3(), v.x4(), v.x5(), v.x6());
	}

	proto::Vec2 asProto(const Vec2& v) {
		proto::Vec2 u;
		u.set_x1(v[0]);
		u.set_x2(v[1]);
		return u;
	}

	proto::Vec3 asProto(const Vec3& v) {
		proto::Vec3 u;
		u.set_x1(v[0]);
		u.set_x2(v[1]);
		u.set_x3(v[2]);
		return u;
	}

	proto::Vec4 asProto(const Vec4& v) {
		proto::Vec4 u;
		u.set_x1(v[0]);
		u.set_x2(v[1]);
		u.set_x3(v[2]);
		u.set_x4(v[3]);
		return u;
	}

	proto::Vec5 asProto(const Vec5& v) {
		proto::Vec5 u;
		u.set_x1(v[0]);
		u.set_x2(v[1]);
		u.set_x3(v[2]);
		u.set_x4(v[3]);
		u.set_x5(v[4]);
		return u;
	}

	proto::Vec6 asProto(const Vec6& v) {
		proto::Vec6 u;
		u.set_x1(v[0]);
		u.set_x2(v[1]);
		u.set_x3(v[2]);
		u.set_x4(v[3]);
		u.set_x5(v[4]);
		u.set_x6(v[5]);
		return u;
	}
}
