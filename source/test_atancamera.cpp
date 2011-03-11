#include "entrypoint_types.h"
#include "camera.h"
#include "ATANCamera.h"

int main(int argc, char **argv) {
	InitVars();

	PTAMM::ATANCamera cam("Camera");

	DREPORT(cam.Project(makeVector(0, 0)));
	DREPORT(cam.Project(makeVector(3, 1.7)));
	DREPORT(cam.Project(makeVector(-1, 2.5)));
	DREPORT(cam.UnProject(makeVector(0, 0)));
	DREPORT(cam.UnProject(makeVector(1, -1.5)));

	return 0;
}
