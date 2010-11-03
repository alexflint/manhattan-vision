//#include "ptam_include.h"
//#undef Bool // this gets defined somewhere in CVD, TooN, PTAM, or GKTools

#include <LU.h>

#include <VW/Improc/interpolatergb.h>

#include "common_types.h"
#include "unwarped_image.h"

#include "image_utils.tpp"
//#include "numeric_utils.tpp"

// TODO: the use of "retina" is completely wrong, it should be "retina"
// TODO: use SL<3> rather than Matrix<3> for image_to_retina and retina_to_image

namespace indoor_context {
using namespace toon;

UndistortMap::UndistortMap() : cam("Camera") {
}

UndistortMap::UndistortMap(const ImageRef& size) : cam("Camera") {
	Compute(size);
}

void UndistortMap::Prepare(const ImageRef& size) {
	if (input_size != size) {
		Compute(size);
	}
}

void UndistortMap::Compute(const ImageRef& size) {
	DLOG << "Computing the undistort map for a " << size << " image ...";
	input_size = size;

	// Determine crop bounds
	tl = makeVector(-INFINITY, -INFINITY);
	br = makeVector(INFINITY, INFINITY);
	for (int y = 0; y < size.y; y++) {
		tl[0] = max(tl[0], cam.UnProject(makeVector(0, y))[0]);
		br[0] = min(br[0], cam.UnProject(makeVector(size.x-1, y))[0]);
	}
	for (int x = 0; x < size.x; x++) {
		tl[1] = max(tl[1], cam.UnProject(makeVector(x, 0))[1]);
		br[1] = min(br[1], cam.UnProject(makeVector(x, size.y-1))[1]);
	}
	tr = makeVector(br[0], tl[1]);
	bl = makeVector(tl[0], br[1]);

	// Compute the image-to-retina-plane homography
	image_to_retina[0][0] = (br[0]-tl[0])/size.x;
	image_to_retina[0][1] = 0;
	image_to_retina[0][2] = tl[0];
	image_to_retina[1][0] = 0;
	image_to_retina[1][1] = (br[1]-tl[1])/size.y;
	image_to_retina[1][2] = tl[1];
	image_to_retina[2][0] = 0;
	image_to_retina[2][1] = 0;
	image_to_retina[2][2] = 1;

	// Invert it
	LU<3> lu(image_to_retina);
	retina_to_image = lu.get_inverse();

	// Compute the line equations for the bounds poly
	retina_bounds.clear();
	retina_bounds.push_back(unproject(tr)^unproject(tl));  // top boundary
	retina_bounds.push_back(unproject(tl)^unproject(bl));  // left boundary
	retina_bounds.push_back(unproject(bl)^unproject(br));  // bottom boundary
	retina_bounds.push_back(unproject(br)^unproject(tr));  // right boundary

	// Compute the undistort map
	ResizeImage(xmap, size);
	ResizeImage(ymap, size);
	Vec2 p;
	for (int y = 0; y < size.y; y++) {
		PixelF* xrow = xmap[y];
		PixelF* yrow = ymap[y];
		for (int x = 0; x < size.x; x++) {
			p = project(image_to_retina * makeVector(x,y,1));
			p = cam.Project(p);
			xrow[x] = p[0];
			yrow[x] = p[1];
		}
	}
}

Vec2 UndistortMap::operator()(const ImageRef& ir) const {
	// Note that ".y" here just "intensity of the pixel" not "y coordinate";
	return makeVector(xmap[ir].y, ymap[ir].y);
}

void UndistortMap::Apply(const ImageBundle& input, ImageBundle& out) const {
	ResizeImage(out.rgb, input.sz());
	// "x" and "y" are used for at least 3 different things in this function, beware!
	for (int y = 0; y < input.ny(); y++) {
		PixelRGB<byte>* row = out.rgb[y];
		const PixelF* xrow = xmap[y];
		const PixelF* yrow = ymap[y];
		for (int x = 0; x < input.nx(); x++) {
			// Note that ".y" here means "intensity of the pixel" not "y coordinate";
			row[x] = VW::InterpolateRGB<byte>::GetInterpolatedPixel
					(input.rgb, xrow[x].y, yrow[x].y);
		}
	}
	out.Invalidate();
}

Mat2 UndistortMap::ComputeDerivsAt(const Vec2& pos) {
	cam.UnProject(pos);  // ATANCamera precomputes a bunch of stuff in here
	Mat2 Jac_pd = cam.GetProjectionDerivs();
	LU<2> lu(Jac_pd);
	return lu.get_inverse();
}

Vec2 UndistortMap::ProjectGradient(const Vec2& grad,
                                   const Vec2& pos) {
	return grad * ComputeDerivsAt(pos);
}




UnwarpedImage::UnwarpedImage() {	}

UnwarpedImage::UnwarpedImage(const ImageBundle& input,
                             const UndistortMap& undistmap) {
	Compute(input, undistmap);
}

void UnwarpedImage::Compute(const ImageBundle& input,
                            const UndistortMap& undist) {
	CHECK_EQ(input.sz(), undist.input_size);
	DLOG << "Unwarping a " << input.sz() << " image ...";
	prev_input = &input;
	map = &undist;

	// Undistort the image
	map->Apply(input, image);

	// TODO: remove these, have users just access the map directly
	image_to_retina = map->image_to_retina;
	retina_to_image = map->retina_to_image;
}

Vec2 UnwarpedImage::TLinRetina() const {
	// TODO: return map->tl
	return project(image_to_retina * unproject(makeVector(0, 0)));
}

Vec2 UnwarpedImage::TRinRetina() const {
	return project(image_to_retina * unproject(makeVector(1.0*image.nx(), 0.0)));
}

Vec2 UnwarpedImage::BRinRetina() const {
	return project(image_to_retina * unproject(makeVector(1.0*image.nx(), image.ny())));
}

Vec2 UnwarpedImage::BLinRetina() const {
	return project(image_to_retina * unproject(makeVector(0.0, image.ny())));
}
}
