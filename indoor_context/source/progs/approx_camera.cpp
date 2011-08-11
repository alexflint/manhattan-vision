#include "camera.h"
#include "common_types.h"
#include "vars.h"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	InitVars(argc, argv);
	ImageRef sz(640, 480);

	Camera cam(sz);
	scoped_ptr<LinearCamera> lincam(LinearCamera::Approximate(cam));

	DREPORT(CameraBase::GetMaxDeviation(cam, *lincam));

	return 0;
}
