#pragma once

#include "common_types.h"
#include "camera.h"

namespace indoor_context {
	// Maps the projection of a point on the ceiling to the projection
	// of the corresponding point on the floor, given the camera
	// orientation and the position of one point on the ceiling and its
	// corresponding floor point.
	class FloorCeilMap {
	public:
		toon::Vector<3> up;  // vertical vanishing point
		double zFloor, zCeil;  // z coord of floor and ceiling in the world
		toon::Matrix<3> FtoC, CtoF;  // the homography and its inverse
		const PosedCamera* pc;  // intrinsic camera parameters

		// Compute the homography
		void Compute(const toon::Vector<3>& ret_floor_pt,
								 const toon::Vector<3>& ret_ceil_pt,
								 const PosedCamera& pc);

		// Determine whether a point is above or the below the horizon
		bool BelowHorizon(const toon::Vector<3>& ret_x);
		bool AboveHorizon(const toon::Vector<3>& ret_x);
		bool BelowHorizonIm(const toon::Vector<3>& im_x);
		bool AboveHorizonIm(const toon::Vector<3>& im_x);

		// Transfer between the floor and ceiling
		toon::Vector<3> Transfer(const toon::Vector<3>& ret_x);
		toon::Vector<3> TransferIm(const toon::Vector<3>& im_x);
	};
}
