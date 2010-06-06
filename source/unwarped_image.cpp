#include "ptam_include.h"
#undef Bool // this gets defined somewhere in CVD, TooN, PTAM, or GKTools

#include <LU.h>

#include <VW/Improc/interpolatergb.h>

#include "common_types.h"
#include "misc.h"
#include "unwarped_image.h"

#include "image_utils.tpp"
#include "math_utils.tpp"

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
		tl = toon::makeVector(-INFINITY, -INFINITY);
		br = toon::makeVector(INFINITY, INFINITY);
		for (int y = 0; y < size.y; y++) {
			tl[0] = max(tl[0], cam.UnProject(toon::makeVector(0, y))[0]);
			br[0] = min(br[0], cam.UnProject(toon::makeVector(size.x-1, y))[0]);
		}
		for (int x = 0; x < size.x; x++) {
			tl[1] = max(tl[1], cam.UnProject(toon::makeVector(x, 0))[1]);
			br[1] = min(br[1], cam.UnProject(toon::makeVector(x, size.y-1))[1]);
		}
		tr = toon::makeVector(br[0], tl[1]);
		bl = toon::makeVector(tl[0], br[1]);

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
		toon::LU<3> lu(image_to_retina);
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
		toon::Vector<2> p;
		for (int y = 0; y < size.y; y++) {
			PixelF* xrow = xmap[y];
			PixelF* yrow = ymap[y];
			for (int x = 0; x < size.x; x++) {
				p = project(image_to_retina * toon::makeVector(x,y,1));
				p = cam.Project(p);
				xrow[x] = p[0];
				yrow[x] = p[1];
			}
		}
	}

	Vector<2> UndistortMap::operator()(const ImageRef& ir) const {
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

	toon::Matrix<2> UndistortMap::ComputeDerivsAt(const toon::Vector<2>& pos) {
		cam.UnProject(pos);  // ATANCamera precomputes a bunch of stuff in here
		toon::Matrix<2> Jac_pd = cam.GetProjectionDerivs();
		toon::LU<2> lu(Jac_pd);
		return lu.get_inverse();
	}

	toon::Vector<2> UndistortMap::ProjectGradient(const toon::Vector<2>& grad,
																								const toon::Vector<2>& pos) {
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

	toon::Vector<2> UnwarpedImage::TLinRetina() const {
		// TODO: return map->tl
		return project(image_to_retina * unproject(toon::makeVector(0, 0)));
	}

	toon::Vector<2> UnwarpedImage::TRinRetina() const {
		return project(image_to_retina * unproject(toon::makeVector(1.0*image.nx(), 0.0)));
	}

	toon::Vector<2> UnwarpedImage::BRinRetina() const {
		return project(image_to_retina * unproject(toon::makeVector(1.0*image.nx(), image.ny())));
	}

	toon::Vector<2> UnwarpedImage::BLinRetina() const {
		return project(image_to_retina * unproject(toon::makeVector(0.0, image.ny())));
	}







	/*
	toon::Matrix<2> UnwarpedImage::ComputeDerivsOld(const toon::Vector<2>& pos) {
		// I_d = original distorted image
		// I_u = unwarped image
		// p_d = (x_d, y_d) = position in original distorted image
		// p_u = (x_u, y_u) = position in unwarped image
		// r_d = ||p_d|| = dist from center in original distorted image
		// r_u = ||p_u|| = dist from center in unwarped image
		//
		// From "Straight lines have to be straight", Faugeras et al:
		// r_u = tan(W*r_d) / (2*tan(W/2))   { W is a calibrated camera parameter)
		// p_u = (r_u/r_d) p_d
		//
		// In the following, Jac mean jacobian wrt p_d, i.e. it is a 2-vector
		// Jac(r_d) = (2/r_d) * p_d          { from expanding r_d = sqrt(...) }
		// Jac(r_u) = Jac{ tan(W*r_d) / (2*tan(W/2)) }
		//          = 1/(2*tan(W/2)) * sec^2(W*r_d)*w*Jac(r_d)  { chain rule }
		//
		// let a = r_a/r_d
		// Jac(a) = ( Jac(r_u)*r_d - Jac(r_d)*r_u ) / r_u^2    { quotient rule }
		//        = ( W/(2*tan(W/2)) * sec^2(W*r_d)*r_d*Jac(r_d)
		//                                      - 2*(r_u/r_d)*pd ) / r_u^2
		//              { chain rule, derivative of tan(x) is sec^2(x) }
		//
		// x_u = a * x_d
		// Jac(x_u) = Jac(a)*x_d + a*Jac(x_d)
		//          = [ Jac(a)*x_d + a,   Jac(a)*x_d ]
		// Similarly for y_u


		// Unproject, collect the inputs
		cam.GetProjectionDerivs();
		const toon::Vector<2>& pd = cam.mvLastDistCam;
		const toon::Vector<2>& pu = cam.mvLastCam;
		const double& rd = cam.mdLastDistR;
		const double& ru = cam.mdLastR;
		const double& a = ru / rd;

		// Compute Jacobians
		toon::Vector<2> Jac_a;
		if (cam.mdW < 1e-5) {
			Jac_a = toon::makeVector(0,0);
		} else {
			const toon::Vector<2> Jac_rd = (2.0/rd) * pd;
			const toon::Vector<2> Jac_ru = cam.mdW*cam.mdOneOver2Tan
				* secsqr(cam.mdW*rd) * cam.mdW*Jac_rd;
			Jac_a = (Jac_ru*rd - Jac_rd*ru) / (ru*ru);
		}

		//DREPORT(Jac_a);

		toon::Matrix<2> Jac_pu;
		Jac_pu[0][0] = Jac_a[0] * pd[0] + a;
		Jac_pu[0][1] = Jac_a[0] * pd[0];
		Jac_pu[1][0] = Jac_a[1] * pd[1];
		Jac_pu[1][1] = Jac_a[1] * pd[1] + a;

		return Jac_pu;
		}*/
}
