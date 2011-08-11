#include "entrypoint_types.h"
#include "simple_renderer.h"

#include "image_utils.tpp"
#include "vector_utils.tpp"

int main(int argc, char **argv) {
	// Camera matrices
	int scale = 500;
	Vec2I sz = makeVector(scale, scale);
	Mat3 intr = Identity;  // intrinsics
	intr[0][2] = intr[1][2] = .5;
	intr *= scale;
	intr[2][2] = 1;

	toon::Matrix<3,4> extr = Zeros;
	extr[0][0] = extr[1][2] = extr[2][1] = 1.;
	Matrix<3,4> cam = intr*extr;  // full camera matrix
	DREPORT(intr, extr, cam);

	// Do the rendering
	SimpleRenderer r(cam, sz);
	r.RenderInfinitePlane(1., 2);
	r.RenderInfinitePlane(-1., 2);
	r.SmoothInfiniteDepths();
	//r.Render(verts[0], verts[1], verts[2], 1);
	//r.Render(verts[0], verts[2], verts[3], 1);

	// Check against old version
	SimpleRenderer r_old(cam, sz);
	r_old.OldRenderInfinitePlane(1., 2);
	r_old.OldRenderInfinitePlane(-1., 2);

	for (int y = 0; y < scale; y+=10) {
		for (int x = 0; x < scale; x+=10) {
			double v = r.depthbuffer()[y][x];
			double v_old = r_old.depthbuffer()[y][x];
			if (abs(v-v_old) > 1e-8*abs(v)) {
				DLOG << "Depth buffers differ at "<<x<<","<<y<<": "<<v<< " vs "<<v_old;
			}
		}
	}

	// Visualize
	WriteOrientationImage("out/frame.png", r.framebuffer());
	WriteMatrixImageRescaled("out/depth.png", r.depthbuffer());

	return 0;
}
