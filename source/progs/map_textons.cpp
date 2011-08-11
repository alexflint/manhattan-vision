#include "common_types_vw.h"
#include "textons.h"
#include "args_meta.h"

int main(int argc, char **argv) {
	ImageBundle image(argv[1]);
	TextonMapper textons(image);
	

	return 0;
}
