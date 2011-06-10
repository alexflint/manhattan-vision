#include "common_types_vw.h"
#include "canny.h"
#include "range_utils.tpp"

lazyvar<int> gvThetaBins("Edgels.NumThetaBins");
lazyvar<int> gvRhoBins("Edgels.NumRhoBins");
lazyvar<int> gvPhiBins("Edgels.NumPhiBins");
lazyvar<int> gvThresh("Edgels.Threshold");
lazyvar<int> gvRoiSize("Edgels.RoiSize");
lazyvar<int> gvRoiStep("Edgels.RoiStep");


inline int Sign(float x) {
	return x == 0 ? 0 : (x > 0 ? 1 : -1);
}

inline int BSign(float x) {
	return x == 0 ? 0 : (x > 0 ? 1 : -1);
}

inline int PointSign(const Vec3F& point, const Vec3F& line) {
	return Sign(DotProduct(line, point)) * (point[2] < 0 ? -1 : 1);
}

void ClipToPositive(const Vec3F& line,
										const Vec3F& clip,
										Vec3F& start,
										Vec3F& end) {
	const Vec3F isct = Cross3D(line, clip);
	if (PointSign(line, start) == -1) {
		start = isct;
	}
	if (PointSign(line, end) == -1) {
		end = isct;
	}
}

void GetImageBounds(const ImageRef& size,
										vector<Vec3F>& bounds) {
	bounds.push_back(Vec3F(1.0, 0.0, 0.0)); // left
	bounds.push_back(Vec3F(0.0, 1.0, 0.0)); // top
	bounds.push_back(Vec3F(-1.0, 0.0, size.x-1)); // right
	bounds.push_back(Vec3F(0.0, -1.0, size.y-1)); // bottom
}

void GetROIBounds(const ROI& roi,
									vector<Vec3F>& bounds) {
	bounds.push_back(Vec3F(1.0, 0.0, -roi.Left())); // left
	bounds.push_back(Vec3F(0.0, 1.0, -roi.Top())); // top
	bounds.push_back(Vec3F(-1.0, 0.0, roi.Right()-1)); // right
	bounds.push_back(Vec3F(0.0, -1.0, roi.Bottom()-1)); // bottom
}
							 
void ClipLineToPoly(const Vec3F& line,
										const vector<Vec3F>& poly,
										Vec3F& a,
										Vec3F& b) {
	a.Fill(0.0);
	b.Fill(0.0);
	const int n = poly.size();
	INDENTED for (int i = 1; i <= n; i++) {
		const Vec3F isct = Cross3D(line, poly[i%n]);
		const float dp1 = PointSign(isct, poly[ (i-1)%n ]);
		const float dp2 = PointSign(isct, poly[ (i+1)%n ]);
		if (dp1 >= 0 && dp2 >= 0) {
			b = a;
			a = isct;
		}
	}
}


void ClipLineToImage(const Vec3F& line,
										 const ImageRef& size,
										 Vec3F& a,
										 Vec3F& b) {
	vector<Vec3F> bounds;
	GetImageBounds(size, bounds);
	ClipLineToPoly(line, bounds, a, b);
}

void ClipLineToImage(const Vec3F& line,
										 const ImageRef& size,
										 Vec2F& a,
										 Vec2F& b) {
	Vec3F ah, bh;
	ClipLineToImage(line, size, ah, bh);
	a = Project(ah);
	b = Project(bh);
}

void ClipLineToROI(const Vec3F& line,
									 const ROI& roi,
									 Vec3F& a,
									 Vec3F& b) {
	vector<Vec3F> bounds;
	GetROIBounds(roi, bounds);
	ClipLineToPoly(line, bounds, a, b);
}

void ClipLineToROI(const Vec3F& line,
									 const ROI& roi,
									 Vec2F& a,
									 Vec2F& b) {
	Vec3F ah, bh;
	ClipLineToROI(line, roi, ah, bh);
	a = Project(ah);
	b = Project(bh);
}

// Theta and rho are in local (ROI) coordinate frame, line is in
// global (whole image) coordinate frame
struct Edgel {
	float theta, rho, support;
	Vec3F line;
	Vec2F start, end;
	Edgel(float t, float r, float s, const Vec3F& v)
		: theta(t), rho(r), support(s), line(v) { }

	void OutputViz(const ImageRGB<byte>& orig,
								 const string& filename) const {
		ImageRGB<byte> canvas;
		ImageCopy(orig, canvas);
		Draw(canvas, PixelRGB<byte>(255, 0, 0));
		WriteImage(filename, canvas);
	}

	void Draw(ImageRGB<byte>& canvas, const PixelRGB<byte>& color) const {
		const int nx = canvas.GetWidth();
		const int ny = canvas.GetWidth();
		const float theta = theta;
		const float rho = rho;

		canvas.SetPenColour(color);
		canvas.DrawLine(start[0], start[1], end[0], end[1]);
	}
};

class EdgeHistogram {
public:
	MatF hist;
	float diaglen;
	vector<Edgel> edgels;

	int RhoToBin(const float rho) {
		return roundi((rho + diaglen/2) * *gvRhoBins / diaglen);
	}
	int ThetaToBin(const float theta) {
		return roundi((M_PI+theta) * *gvThetaBins / M_PI) % *gvThetaBins;
	}
	float BinToRho(const int bin) {
		return diaglen * bin / *gvRhoBins - diaglen/2;
	}
	float BinToTheta(const int bin) {
		return M_PI * bin / *gvThetaBins;
	}

	void Compute(const ROI& roi,
							 const MatI& mask,
							 const MatF& magsqr,
							 const MatF& orient) {
		const float cx = roi.CenterX();
		const float cy = roi.CenterY();		
		diaglen = sqrtf(roi.Width()*roi.Width() + roi.Height()*roi.Height());

		edgels.clear();
		hist.Resize(*gvThetaBins, *gvRhoBins);
		hist.Fill(0);

		// Compute the (theta,rho) histogram
		for (int y = roi.Top(); y < roi.Bottom(); y++) {
			const float* orientrow = orient[y];
			const float* magrow = magsqr[y];
			for (int x = roi.Left(); x < roi.Right(); x++) {
				if (mask[y-roi.Top()][x-roi.Left()]) {
					const float theta = OpenUpAngle(orientrow[x]);
					const float rho = (x-cx) * cos(theta)	+ (y-cy) * sin(theta);
					const int theta_bin = ThetaToBin(theta);
					const int rho_bin = RhoToBin(rho);
					const float mag = sqrtf(magrow[x]);
					hist[theta_bin][rho_bin] += mag;
				}
			}
		}

		// Find edgels
		int nn = 0;
		for (int y = 0; y < hist.Rows(); y++) {
			for (int x = 0; x < hist.Cols(); x++) {
				if (hist[y][x] >= *gvThresh) {
					const float theta = BinToTheta(y);
					const float rho = BinToRho(x);
					const float costh = cos(theta);
					const float sinth = sin(theta);
					const Vec3F line(costh, sinth, -cx*costh - cy*sinth - rho);
					Edgel cur(theta, rho, hist[y][x], line);
					ClipLineToROI(cur.line, roi, cur.start, cur.end);
					edgels.push_back(cur);
				}
			}
		}
	}
};

class Edgels {
public:
	vector<Edgel> edgels;
	EdgeHistogram edgehist;
	Gradients gradients;

	Edgels() { }
	Edgels(const ImageF& image) {
		Compute(image);
	}

	void Compute(const ImageF& image) {
		const int nx = image.GetWidth();
		const int ny = image.GetHeight();
		const int len = *gvRoiSize;

		// Compute gradients
		gradients.Compute(image);
		MatF& magsqr = gradients.magnitude_sqr;
		magsqr /= magsqr.MaxValue();

		// Create a circular mask to remove the bias for diagonal lines
		MatI mask(len, len);
		for (int y = 0; y < len; y++) {
			for (int x = 0; x < len; x++) {
				const float xx = x-len/2;
				const float yy = y-len/2;
				const float rad = sqrt(xx*xx + yy*yy);
				mask[y][x] = (rad <= len/2) ? 1 : 0;
			}
		}

		// Compute the histograms
		for (int top = 0; top <= ny - *gvRoiSize; top += *gvRoiStep) {
			for (int left = 0; left <= nx - *gvRoiSize; left += *gvRoiStep) {
				ROI roi(left, top, left+*gvRoiSize, top+*gvRoiSize);
				edgehist.Compute(roi, mask, magsqr, gradients.orient);
				copy_all_into(edgehist.edgels, edgels);
			}
		}
	}
};

class VotedEdgels {
public:
	vector<Edgel> edgels;
	Gradients gradients;
	Table<3,float> votes;

	int RhoToBin(const float rho) {
		return roundi((rho + diaglen/2) * *gvRhoBins / diaglen);
	}
	int PhiToBin(const float phi) {
		return roundi((phi + diaglen/2) * *gvPhiBins / diaglen);
	}
	int ThetaToBin(const float theta) {
		return roundi((M_PI+theta) * *gvThetaBins / M_PI) % *gvThetaBins;
	}
	float BinToRho(const int bin) {
		return diaglen * bin / *gvRhoBins - diaglen/2;
	}
	float BinToRho(const int bin) {
		return diaglen * bin / *gvPhiBins - diaglen/2;
	}
	float BinToTheta(const int bin) {
		return M_PI * bin / *gvThetaBins;
	}

	VotedEdgels() { }
	VotedEdgels(const ImageF& image) {
		Compute(image);
	}

	void Compute(ImageF& image) {
		const int nx = image.GetWidth();
		const int ny = image.GetHeight();
		const int len = *gvRoiSize;

		// Compute gradients
		gradients.Compute(image);
		MatF& magsqr = gradients.magnitude_sqr;
		magsqr /= magsqr.MaxValue();

		// Create the histogram
		votes.Resize(ny, nx, *gvThetaBins);


		// Compute the (theta,rho,phi) histogram
		for (int y = roi.Top(); y < roi.Bottom(); y++) {
			const float* orientrow = orient[y];
			const float* magrow = magsqr[y];
			for (int x = roi.Left(); x < roi.Right(); x++) {
				if (mask[y-roi.Top()][x-roi.Left()]) {
					const float theta = OpenUpAngle(orientrow[x]);
					const float rho = (x-cx) * cos(theta)	+ (y-cy) * sin(theta);
					const int theta_bin = ThetaToBin(theta);
					const int rho_bin = RhoToBin(rho);
					const float mag = sqrtf(magrow[x]);
					hist[theta_bin][rho_bin] += mag;
				}
			}
		}

		// Find edgels
		int nn = 0;
		for (int y = 0; y < hist.Rows(); y++) {
			for (int x = 0; x < hist.Cols(); x++) {
				if (hist[y][x] >= *gvThresh) {
					const float theta = BinToTheta(y);
					const float rho = BinToRho(x);
					const float costh = cos(theta);
					const float sinth = sin(theta);
					const Vec3F line(costh, sinth, -cx*costh - cy*sinth - rho);
					Edgel cur(theta, rho, hist[y][x], line);
					ClipLineToROI(cur.line, roi, cur.start, cur.end);
					edgels.push_back(cur);
				}
			}
		}

int main(int argc, char **argv) {
	InitVars(argc, argv, "edgels.cfg");

	ImageF image;
	ImageRGB<byte> imagergb;
	ReadImage(argv[1], imagergb);
	ImageCopy(imagergb, image);
	ResetAlpha(imagergb);
	const int nx = image.GetWidth();
	const int ny = image.GetHeight();

	Edgels edgels(image);
	DREPORT(edgels.edgels.size());

	ImageRGB<byte> canvas;
	ImageCopy(imagergb, canvas);
	BrightColors colors;
	BOOST_FOREACH(const Edgel& e, edgels.edgels) {
		e.Draw(canvas, colors.Next());
	}
	WriteImage("out/all_edgels.png", canvas);

	return 0;
}
