#include "common_types.h"
#include "vector_utils.tpp"


namespace indoor_context {
using namespace toon;

Vector<2> asToon(const proto::Vec2& v) {
	return makeVector(v.x1(), v.x2());
}

Vector<3> asToon(const proto::Vec3& v) {
	return makeVector(v.x1(), v.x2(), v.x3());
}

Vector<4> asToon(const proto::Vec4& v) {
	return makeVector(v.x1(), v.x2(), v.x3(), v.x4());
}

Vector<5> asToon(const proto::Vec5& v) {
	return makeVector(v.x1(), v.x2(), v.x3(), v.x4(), v.x5());
}

Vector<6> asToon(const proto::Vec6& v) {
	return makeVector(v.x1(), v.x2(), v.x3(), v.x4(), v.x5(), v.x6());
}

proto::Vec2 asProto(const Vector<2>& v) {
	proto::Vec2 u;
	u.set_x1(v[0]);
	u.set_x2(v[1]);
	return u;
}

proto::Vec3 asProto(const Vector<3>& v) {
	proto::Vec3 u;
	u.set_x1(v[0]);
	u.set_x2(v[1]);
	u.set_x3(v[2]);
	return u;
}

proto::Vec4 asProto(const Vector<4>& v) {
	proto::Vec4 u;
	u.set_x1(v[0]);
	u.set_x2(v[1]);
	u.set_x3(v[2]);
	u.set_x4(v[3]);
	return u;
}

proto::Vec5 asProto(const Vector<5>& v) {
	proto::Vec5 u;
	u.set_x1(v[0]);
	u.set_x2(v[1]);
	u.set_x3(v[2]);
	u.set_x4(v[3]);
	u.set_x5(v[4]);
	return u;
}

proto::Vec6 asProto(const Vector<6>& v) {
	proto::Vec6 u;
	u.set_x1(v[0]);
	u.set_x2(v[1]);
	u.set_x3(v[2]);
	u.set_x4(v[3]);
	u.set_x5(v[4]);
	u.set_x6(v[5]);
	return u;
}

toon::Matrix<3,4> as_matrix(const toon::SE3<>& se3) {
	toon::Matrix<3,4> m;
	m.slice<0,0,3,3>() = se3.get_rotation().get_matrix();
	m.slice<0,3,3,1>() = se3.get_translation().as_col();
	return m;
}
}
