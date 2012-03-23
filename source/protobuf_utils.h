#include "common_types.h"

// Forward declarations
namespace google { namespace protobuf { class Message; } }

namespace indoor_context {
	// Forward declarations
	namespace proto {
		class Vec2;
		class Vec3;
		class Vec4;
		class Vec5;
		class Vec6;
	}

	// Write a protocol buffer to a file
	void WriteProto(const string& path, const google::protobuf::Message& msg);
	// Read a protocol buffer from a file
	void ReadProto(const string& path, google::protobuf::Message& msg);
	// Read a protocol buffer larger than 64MB (ignores security restrictions)
	void ReadLargeProto(const string& path, google::protobuf::Message& p);

	// Converters for vectors
	Vec2 asToon(const proto::Vec2& x);
	Vec3 asToon(const proto::Vec3& x);
	Vec4 asToon(const proto::Vec4& x);
	Vec5 asToon(const proto::Vec5& x);
	Vec6 asToon(const proto::Vec6& x);
	proto::Vec2 asProto(const Vec2& x);
	proto::Vec3 asProto(const Vec3& x);
	proto::Vec4 asProto(const Vec4& x);
	proto::Vec5 asProto(const Vec5& x);
	proto::Vec6 asProto(const Vec6& x);
}
