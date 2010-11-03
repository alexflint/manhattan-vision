#include "common_types_vw.h"

#include <VW/Utils/timer.h>
#include <cvd/image.h>
#include <cvd/image_interpolate.h>
#include <LU.h>

#include "unwarped_image.h"
#include "misc.h"
#include "vars.h"
#include "ptam_include.h"

#include <VW/Image/imageio.tpp>

#include "image_utils.tpp"
//#include "numeric_utils.tpp"

using namespace indoor_context;

typedef TooN::Vector<2> tvec2;

void DrawGradient(ImageBundle& image,
									const tvec2& p,
									const tvec2& d,
									const PixelRGB<byte> color) {
	DrawLineClipped(image.rgb, p-d, p+d, color);
}

// Efficiently projects gradients through an ATANCamera model. Caches
// per-pixel Jacobians so the memory overhead is equivalent to four
// keyframes.
//
// Note: Don't share an ATANCamera because it stores so much internal
// state.
class GradientProjector {
public:
	// We use NN interpolation because we the Jacobians really change
	// very very little from one pixel to the next. We are already
	// over-doing it by computing a jacobian for every pixel, really.
	typedef CVD::Interpolate::NearestNeighbour NN;

	ptam::ATANCamera cam;
	ImageRef& kf_size;
	CVD::Image<toon::Matrix<2> > jacobians;
	CVD::image_interpolate<NN, toon::Matrix<2> > jacs;
	bool configured;

	toon::Matrix<3> image_to_retina;
	toon::Matrix<3> retina_to_image;

	// cameraName determines the basename for the GVars
	GradientProjector(const string& cameraName, ImageRef& kfsize)
		: cam(cameraName),
			kf_size(kfsize),
			jacobians(CVD::ImageRef(kfsize.x, kfsize.y)),
			jacs(jacobians),
			configured(false) { }
	// Initialize: computes thousands of jacobians. May take some time.
	void Configure();
	// Transform a vector v at image location p into the unwarped domain
	tvec2 Transform(const tvec2& v, const tvec2& p);
};

void GradientProjector::Configure() {
	// TODO: we could substatially downsize the jacobian image and
	// probably still get away with NN interpolation

	// Compute the mapping between retina plane and image pixels
	UnwarpedImage::ComputeRetinaMap(cam, kf_size, image_to_retina, retina_to_image);

	// Compute per-pixel jacobians
	Timer t;
	toon::Matrix<2> derivs;
	toon::LU<2> lu(toon::Matrix<2>(toon::Identity));
	for (int y = 0; y < kf_size.y; y++) {
		for (int x = 0; x < kf_size.x; x++) {
			cam.UnProject(toon::makeVector(x, y));
			lu.compute(cam.GetProjectionDerivs());
			jacobians[y][x] = lu.get_inverse();
		}
	}
	DLOG << "Computing jacobians took " << t;
	configured = true;
}

tvec2 GradientProjector::Transform(const tvec2& v, const tvec2& p) {
	CHECK(configured);
	if (!jacs.in_image(p)) {
		DLOG << "Error: p is outside image in GradientProjector::Transform: " << p << endl;
	}
	return jacs[p] * v;
}

int main(int argc, char **argv) {
	InitVars(argc, argv);
	if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" INPUT";
		return -1;
	}

	ImageBundle image(argv[1]);
	UnwarpedImage unwarped(image);

	ImageRef ir(image.sz());
	GradientProjector gproj("Camera", ir);
	gproj.Configure();

	vector<tvec2> edges;
	edges.push_back(toon::makeVector(5,0));
	edges.push_back(toon::makeVector(0,5));
	static const int stride = 28;
	
	for (int y = 5; y < image.ny(); y += stride) {
		for (int x = 5; x < image.nx(); x += stride) {
			for (int i = 0; i < edges.size(); i++) {
				tvec2 p = toon::makeVector(x, y);
				tvec2 d = edges[i];
				tvec2 pd = unwarped.cam.UnProject(p);
				tvec2 dd = gproj.Transform(d, p);
				tvec2 pd_im = HProj(unwarped.retina_to_image * HUnproj(pd));
				tvec2 dd_im = unwarped.retina_to_image.slice<0,0,2,2>() * dd;
				DrawGradient(image, p, d, BrightColors::Get(0));
				if (unwarped.image.contains(toIR(pd_im))) {
					DrawGradient(unwarped.image, pd_im, dd_im, BrightColors::Get(0));
				}
			}
		}
	}

	WriteImage("out/orig_gradient.png", image.rgb);
	WriteImage("out/unwarped_gradient.png", unwarped.image.rgb);

	return 0;
}
