#include "hw_convolver.h"
#include "common_types.h"
#include "cuda_conv.h"

namespace indoor_context {

HwConvolver::HwConvolver(int w, int h) {
#ifdef HAVE_CUDA
	cudaconv_AllocImage(w, h, &input);
	cudaconv_AllocImage(w, h, &temp);
	cudaconv_AllocImage(w, h, &output);
#endif
}

HwConvolver::~HwConvolver() {
#ifdef HAVE_CUDA
	cudaconv_FreeImage(&input);
	cudaconv_FreeImage(&temp);
	cudaconv_FreeImage(&output);
#endif
}

void HwConvolver::Warmup() {
#ifdef HAVE_CUDA
	HwConvolver conv(100, 100);
	MatF image(100, 100);
	VecF kx(5), ky(5);
	conv.LoadImage(image);
	conv.Convolve(kx, ky, image);
#endif
}

void HwConvolver::EnsureInitialized() {
#ifdef HAVE_CUDA
	static bool inited = false;
	if (!inited) {
		cudaconv_Init();
		inited = true;
	}
#endif
}

void HwConvolver::LoadImage(const MatF& image) {
#ifdef HAVE_CUDA
	cudaconv_LoadImage(image[0], &input);
#else
	DLOG << "HwConvolver::LoadImage: CUDA support was not compiled" << endl;
	exit(-1);
#endif
}

void HwConvolver::Convolve(const VecF& kernelx,
													 const VecF& kernely,
													 MatF& out_image) {
#ifdef HAVE_CUDA
	float *kx, *ky;
	int radius;
	CheckKernels(kernelx, kernely, &kx, &ky, &radius);
	cudaconv_Convolve(&input, radius, kx, ky, &temp, &output, false);
	cudaconv_RetrieveImage(&output, out_image[0]);
#else
	DLOG << "HwConvolver::Convolve: CUDA support was not compiled" << endl;
	exit(-1);
#endif
}


void HwConvolver::ConvolveDiff(const VecF& kernelx1,
															 const VecF& kernely1,
															 const VecF& kernelx2,
															 const VecF& kernely2,
															 MatF& out_image) {
#ifdef HAVE_CUDA
	float *kx1, *ky1, *kx2, *ky2;
	int r1, r2;
	CheckKernels(kernelx1, kernely1, &kx1, &ky1, &r1);
	CheckKernels(kernelx2, kernely2, &kx2, &ky2, &r2);

	cudaconv_Convolve(&input, r1, kx1, ky1, &temp, &output, false);
	cudaconv_Convolve(&input, r2, kx2, ky2, &temp, &output, true);
	cudaconv_RetrieveImage(&output, out_image[0]);
#else
	DLOG << "HwConvolver::ConvolveDif: CUDA support was not compiled" << endl;
	exit(-1);
#endif
}

// static
void HwConvolver::CheckKernels(const VecF& kx, const VecF& ky,
															 float** px, float** py, int* rad) {
	assert(kx.Size() % 2 == 1);
	assert(ky.Size() == kx.Size());
	*px = const_cast<float*>(kx.DataBlock());
	*py = const_cast<float*>(ky.DataBlock());
	*rad = kx.Size()/2;  // note that kx.Size() is always odd
}

}
