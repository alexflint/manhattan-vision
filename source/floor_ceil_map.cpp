#include "floor_ceil_map.h"

#include "common_types.h"
#include "camera.h"
#include "math_utils.tpp"

#include <SVD.h>
#include <LU.h>

namespace indoor_context {
using namespace toon;

void FloorCeilMap::Compute(const Vector<3>& ret_floor_pt,
                           const Vector<3>& ret_ceil_pt,
                           const PosedCamera& cam) {
	up = cam.GetRetinaVpt(2);
	pc = &cam;

	Vector<3> pf = ret_floor_pt;
	Vector<3> pc = ret_ceil_pt;

	CHECK(BelowHorizon(pf) != BelowHorizon(pc))
	<< "pf and pc must be on opposite sides of the horizon\n"
	<< "pf = " << pf << "\n"
	<< "pc = " << pc;

	if (AboveHorizon(pf)) {
		swap(pf,pc);
		DLOG << "Note: swapping floor and ceiling points in FloorCeilMap";
	}

	// These should never fail now...
	CHECK_PRED1(BelowHorizon, pf);
	CHECK_PRED1(AboveHorizon, pc);

	// Solve for depth of floor and ceiling
	Matrix<3,2> m;
	m.slice<0,0,3,1>() = pc.as_col();
	m.slice<0,1,3,1>() = -pf.as_col();
	Vector<2> soln = SVD<3,2>(m).backsub(up);
	zFloor = soln[1]*pf[2];
	zCeil = soln[1]*pc[2];

	// Construct the homography
	FtoC = (soln[1]*up*pf)*Identity + up.as_col()*up.as_row();
	CtoF = LU<3>(FtoC).get_inverse();
}

bool FloorCeilMap::BelowHorizon(const Vector<3>& ret_x) {
	return Sign(ret_x*up) == Sign(up[1]);
}

bool FloorCeilMap::AboveHorizon(const Vector<3>& ret_x) {
	return !BelowHorizon(ret_x);
}

bool FloorCeilMap::BelowHorizonIm(const Vector<3>& im_x) {
	return BelowHorizon(pc->ImToRet(im_x));
}

bool FloorCeilMap::AboveHorizonIm(const Vector<3>& im_x) {
	return AboveHorizon(pc->ImToRet(im_x));
}

Vector<3> FloorCeilMap::Transfer(const Vector<3>& ret_x) {
	return (BelowHorizon(ret_x) ? FtoC : CtoF) * ret_x;
}

Vector<3> FloorCeilMap::TransferIm(const Vector<3>& im_x) {
	return pc->RetToIm(Transfer(pc->ImToRet(im_x)));
}
}
