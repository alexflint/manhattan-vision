#include "entrypoint_types.h"

#include "colors.h"
#include "image_bundle.h"
#include "vw_image_io.h"

#include "image_utils.tpp"
#include "vw_image.tpp"

using namespace indoor_context;

int main(int argc, char** argv) {
	CHECK_EQ(argc, 2);
	ImageBundle im(argv[1]);

	im.BuildMono();
	WriteImage("out/rgb.png", im.rgb);

	MatF mat;
	ImageToMatrix(im.mono, mat);
	WriteMatrixImage("out/mono.png", mat);
	//WriteImage("out/mono.png", im.mono);

	/*int h = 10, w = 10, bpp=4;
	byte* mem = new byte[w*h*bpp];
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			mem[y*w*bpp + x*bpp] = x%3==0 ? 255 : 1;
			mem[y*w*bpp + x*bpp + 1] = x%3==1 ? 255 : 1;
			mem[y*w*bpp + x*bpp + 2] = x%3==2 ? 255 : 1;
			mem[y*w*bpp + x*bpp + 3] = 255;
		}
	}

	rgba8c_view_t  src = interleaved_view(w,h,(const rgba8_pixel_t*)mem, w*bpp);
	png_write_view("test.png", src);*/

	return 0;
}
