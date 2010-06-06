#include "guided_line_detector.h"

#include "common_types.h"
#include "line_sweeper.h"
#include "timer.h"
#include "geom_utils.h"
#include "clipping.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
#include "math_utils.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	void DrawAxes(ImageRGB<byte>& canvas, const PosedCamera& pc) {
		Vec2 image_ctr = pc.RetToIm(makeVector(0.0, 0.0));
		Vec3 scene_ctr = pc.pose.inverse() * makeVector(0,0,1);
		for (int j = 0; j < 3; j++) {
			Vec3 retina_ej = pc.pose * (scene_ctr + GetAxis<3>(j));
			Vec2 image_ej = pc.RetToIm(project(retina_ej));
			ClipLineToImage(image_ctr, image_ej, canvas.GetSize());
			DrawLine(canvas, image_ctr, image_ej, BrightColors::Get(j));
		}
	}



	int GuidedLineDetector::GetBinForTheta(double theta, int assoc) {
		double t = Ring(theta+theta_offsets[assoc], M_PI) / theta_spans[assoc];
		return floori(t*num_t_bins);
	}

	void GuidedLineDetector::Compute(const PosedImage& image) {
		input = &image;
		image.BuildMono();

		ComputeUndist(image.sz(), image.pc.camera);
		TIMED("Compute gradients") ComputeGradients();
		ComputeVptProjs();
		TIMED("Compute assocs") ComputeAssocs();
		//TIMED("Compute vpt windows") ComputeVptWindows();
		TIMED("Build histograms") ComputeHistograms();
		TIMED("Identify line segments") ComputeSegments();
	}

	void GuidedLineDetector::ComputeUndist(const ImageRef& sz, const CameraBase& cam) {
		if (undist[0].Rows() != sz.y || undist[0].Cols() != sz.x) {
			for (int i = 0; i < 3; i++) {
				undist[i].Resize(sz.y, sz.x);
			}
			Vec3 retina_pt;
			for (int y = 0; y < sz.y; y++) {
				float* xrow = undist[0][y];
				float* yrow = undist[1][y];
				float* zrow = undist[2][y];
				for (int x = 0; x < sz.x; x++) {
					retina_pt = unit(unproject(cam.ImToRet(makeVector(1.0*x,y))));
					xrow[x] = retina_pt[0];
					yrow[x] = retina_pt[1];
					zrow[x] = retina_pt[2];
				}
			}
		}
	}

	void GuidedLineDetector::ComputeGradients() {
		gradients.ComputeSobel(input->mono);
		gradients.ComputeMagSqr();
	}

	void GuidedLineDetector::ComputeVptProjs() {
		for (int j = 0; j < 3; j++) {
			retina_vpts[j] = unit(col(input->pc.pose.get_rotation(), j));
			image_vpts[j] = input->pc.RetToIm(retina_vpts[j]);
		}
	}

	void GuidedLineDetector::ComputeAssocs() {
		// TODO: speed this up by getting Canny to store a vector
		// of image locations with gradient magnitude above the threshold

		// DistThresh is distance from point to line but we maximize
		// projection of point onto line, so convert threshold.
		const double kDistThresh = GV3::get<double>("GuidedLineDetector.DistThresh");
		const double kProjThresh = sqrt(1.0 - kDistThresh*kDistThresh);

		const double kMagThresh = GV3::get<double>("GuidedLineDetector.MagThresh");
		const double kMagSqrThresh = kMagThresh * kMagThresh;

		assocs.Resize(input->ny(), input->nx());
		for (int j = 0; j < 3; j++) {
			responses[j].Resize(input->ny(), input->nx());
			responses[j].Fill(0.0);
		}
			
		// Compute normalizing factors for efficiency
		Vector<5> quad_norm[3];
		for (int j = 0; j < 3; j++) {
			const Vec3& v = image_vpts[j];
			quad_norm[j] = makeVector(v[0]*v[0]+v[1]*v[1],
																-2*v[0]*v[2],
																v[2]*v[2],
																-2*v[1]*v[2],
																v[2]*v[2]);
		}

		for (int y = 0; y < input->ny(); y++) {
			// Determine vpt associations: this must be done in the
			// raw image because we are computing errors in terms of
			// pixels, which makes sense since this image is where the
			// measurements were actually made. 

			// Watch out here, "x" and "y" have three seperate
			// meanings!
			const PixelF* dxrow = gradients.diffx[y];
			const PixelF* dyrow = gradients.diffy[y];
			const float* magrow = gradients.magnitude_sqr[y];
			int* arow = assocs[y];
			for (int x = 0; x < input->nx(); x++) {
				if (magrow[x] > kMagSqrThresh) {
					double norm = sqrt(magrow[x]);
					double dx = dxrow[x].y/norm;
					double dy = dyrow[x].y/norm;
					double proj_max = kProjThresh;
					int assoc = -1;
					for (int j = 0; j < 3; j++) {
						const Vec3& v = image_vpts[j];
						const Vector<5>& n = quad_norm[j];
						// We write out these dot products for (significant) efficiency
						double line_norm = sqrt(n[0] + n[1]*x + n[2]*x*x + n[3]*y + n[4]*y*y);
						double proj = abs(((v[0]-x*v[2])*(-dy) + (v[1]-y*v[2])*dx) / line_norm);
						if (proj > proj_max) {
							proj_max = proj;
							assoc = j;
						}
					}
					responses[assoc][y][x] = (1.0+sqrt(magrow[x]))*pow(proj_max,10.0);
					arow[x] = assoc;
				} else {
					arow[x] = -2;
				}
			}
		}
	}

	int GuidedLineDetector::GetBin(int x, int y, int vpt_index) {
		const Vec3& v0 = retina_vpts[ vpt_index ];
		const Vec3& v1 = retina_vpts[ (vpt_index+1)%3 ];
		const Vec3& v2 = retina_vpts[ (vpt_index+2)%3 ];
		Vec3 p = unit(unproject(input->pc.camera.ImToRet(makeVector(1.0*x,y))));
		//Vec3 p = makeVector(undist[0][y][x], undist[1][y][x], undist[2][y][x]);
		// Unroll these dot products for (significant) efficiency
		double d0 = p[0]*v0[0] + p[1]*v0[1] + p[2]*v0[2];
		double d1 = p[0]*v1[0] + p[1]*v1[1] + p[2]*v1[2];
		double d2 = p[0]*v2[0] + p[1]*v2[1] + p[2]*v2[2];
		double theta = atan2(d1,d2) + M_PI;
		double t = Ring(theta+theta_offsets[vpt_index], 2*M_PI) / theta_spans[vpt_index];
		return floori(t*num_t_bins);
	}

	void GuidedLineDetector::ComputeHistograms() {
		// TODO: speed this up by getting ComputeAssocs to store a vector
		// of image locations that are actually associated

		thetas.Resize(input->ny(), input->nx());
		d0s.Resize(input->ny(), input->nx());
		for (int i = 0; i < 3; i++) {
			bins[i].Resize(input->ny(), input->nx());
		}

		// Clear the histograms
		num_t_bins = max(input->nx(), input->ny())/2;  // fairly arbitrary choice
		for (int i = 0; i < 3; i++) {
			histogram[i].resize(num_t_bins);
			BOOST_FOREACH(LineBin& bin, histogram[i]) {
				bin.pixels.clear();
				bin.pixels.reserve(max(input->nx(), input->ny()));  // worst-case size
				bin.start_d0 = INFINITY;
				bin.end_d0 = -INFINITY;
				bin.support = 0;
			}
			min_d0[i] = INFINITY;
			max_d0[i] = -INFINITY;
		}


		// Compute bounds on the angles for clustering
		Bounds2D<double> bounds = Bounds2D<double>::FromSize(input->sz());
		Vec3 retina_cnrs[] = { unit(unproject(input->pc.ImToRet(bounds.tl()))),
																unit(unproject(input->pc.ImToRet(bounds.tr()))),
																unit(unproject(input->pc.ImToRet(bounds.br()))),
																unit(unproject(input->pc.ImToRet(bounds.bl()))) };


		for (int j = 0; j < 3; j++) {
			double thmin = INFINITY, thmax = -INFINITY;
			const Vec3& vpt = image_vpts[j];
			if (input->contains(ImageRef(vpt[0]/vpt[2], vpt[1]/vpt[2]))) {
				// vpt is inside the image, all angles are possible
				theta_offsets[j] = 0;
				theta_spans[j] = 2*M_PI;
			} else {
				double corner_thetas[4];
				for (int k = 0; k < 4; k++) {
					double d0 = retina_cnrs[k] * retina_vpts[j];
					double u = retina_cnrs[k] * retina_vpts[ (j+1)%3 ];
					double v = retina_cnrs[k] * retina_vpts[ (j+2)%3 ];
					double th = atan2(u, v) + M_PI;
					if (th < thmin) thmin = th;
					if (th > thmax) thmax = th;
					if (d0 < min_d0[j]) min_d0[j] = d0;
					if (d0 > max_d0[j]) max_d0[j] = d0;
					corner_thetas[k] = th;
				}
				if (thmin < thmax-M_PI) {
					// the interval spans 0 so invert it
					for (int k = 0; k < 4; k++) {
						if (corner_thetas[k] < thmax-M_PI && corner_thetas[k] > thmin) {
							thmin = corner_thetas[k];
						}
						if (corner_thetas[k] > thmin+M_PI && corner_thetas[k] < thmax) {
							thmax = corner_thetas[k];
						}
					}
					theta_offsets[j] = 2*M_PI - thmax;
					theta_spans[j] = 2*M_PI - (thmax-thmin);
				} else {
					// the interval is "normal". offset should always be positive
					theta_offsets[j] = 2*M_PI - thmin;
					theta_spans[j] = thmax-thmin;
				}
			}

			// Add a small extra padding to the offset and the span for border Ring() cases
			theta_offsets[j] += 1e-5;
			theta_spans[j] += 2e-5;
		}


		// Accumulate the histograms
		for (int y = 0; y < input->ny(); y++) {
			const int* arow = assocs[y];
			const float* xrow = undist[0][y];
			const float* yrow = undist[1][y];
			const float* zrow = undist[2][y];
			const float* magrow = gradients.magnitude_sqr[y];
			float* trow = thetas[y];
			float* d0row = d0s[y];
			for (int x = 0; x < input->nx(); x++) {
				// Compute angle in a plane normal to the vpt.
				// This part must be done in the calibrated retina domain
				// where the vanishing points are actually orthogonal to
				// each other.
				if (arow[x] >= 0) {
					int assoc = arow[x];
					// Expanding out this matrix multiplication significantly
					// speeds up this inner loop.
					const Vec3& v0 = retina_vpts[ assoc ];
					const Vec3& v1 = retina_vpts[ (assoc+1)%3 ];
					const Vec3& v2 = retina_vpts[ (assoc+2)%3 ];
					const float& px = xrow[x];
					const float& py = yrow[x];
					const float& pz = zrow[x];
					double d0 = px*v0[0] + py*v0[1] + pz*v0[2];
					double d1 = px*v1[0] + py*v1[1] + pz*v1[2];
					double d2 = px*v2[0] + py*v2[1] + pz*v2[2];
					double theta = atan2(d1,d2) + M_PI;
					double t = Ring(theta+theta_offsets[assoc], 2*M_PI) / theta_spans[assoc];
					int t_bin = floori(t*num_t_bins);

					if (assoc<0 || assoc>=3 || t_bin<0 || t_bin>=num_t_bins) {
						DLOG << "t_bin or assoc out of range!";
						INDENTED {
							DREPORT(assoc, t_bin, num_t_bins);
							DREPORT(t, theta, theta_offsets[assoc], theta_spans[assoc]);
							DREPORT(x, y, px, py, pz);
							DREPORT(d1,d2);
							DREPORT(v0, v1 ,v2, image_vpts[assoc]);
							DREPORT(theta + theta_offsets[assoc]);
							DREPORT(Ring(theta+theta_offsets[assoc], M_PI));
							DREPORT(Ring(theta+theta_offsets[assoc], M_PI) / theta_spans[assoc]);

							ImageRGB<byte> canvas;
							ImageCopy(input->rgb, canvas);
							DrawSceneAxes(canvas);
							PixelRGB<byte> black(0,0,0);
							PixelRGB<byte> white(255,255,255);
							Vec2 p = makeVector(x,y);
							DrawSpot(canvas, p, black, 5);
							DrawSpot(canvas, p, white, 3);
							DrawLineClipped(canvas, project(image_vpts[assoc]), p, white);
							WriteImage("out/error_axes.png", canvas);
							OutputAssociationViz("out/error_assoc.png");
						}
						exit(-1);
					}
					d0row[x] = d0;
					trow[x] = theta;

					LineBin& bin = histogram[assoc][t_bin];
					bin.support += sqrt(magrow[x]);
					bin.pixels.push_back(LinePixel(d0, x, y));
					if (d0 < bin.start_d0) {
						bin.start_d0 = d0;
						bin.start[0] = x;
						bin.start[1] = y;
					}
					if (d0 > bin.end_d0) {
						bin.end_d0 = d0;
						bin.end[0] = x;
						bin.end[1] = y;
					}
				}
			}
		}
	}

	void GuidedLineDetector::ComputeSegments() {
		const int kMinPeak = GV3::get<int>("GuidedLineDetector.MinPeak");
		const double kCutThresh = 8; // in pixels
		const double kMinSegLength = 25;  // in pixels

		for (int i = 0; i < 3; i++) {
			peaks[i].clear();
			detections[i].clear();

			vector<LineBin>& hist = histogram[i];
			for (int j = 0; j < num_t_bins; j++) {
				if (hist[j].support > kMinPeak &&
						(j == 0 || hist[j].support > hist[j-1].support) &&
						(j == num_t_bins-1 || hist[j].support > hist[j+1].support)) {

					// Compute theta from bin index
					double t = j*theta_spans[i] / num_t_bins - theta_offsets[i];
					if (t < 0) {
						t += 2*M_PI;
					}

					// Record the ray
					peaks[i].push_back(make_pair(t, j));

					// Break the ray into line segments
					sort_all(hist[j].pixels);
					Vec2 dir = unit(hist[j].end - hist[j].start);
					int j0 = 0;
					const vector<LinePixel>& pix = hist[j].pixels;
					for (int j = 1; j < pix.size(); j++) {	
						// proj_dist may be a small negative number because "dir" is
						// computed from the two most distant pixels, which may not
						// correspond exactly to the line specified by
						// line.theta. proj_dist will never be large and negative.
						double proj_dist = (pix[j].pos - pix[j-1].pos) * dir;
						if (j == pix.size()-1 || proj_dist > kCutThresh) {
							Vec2 total_diff = pix[j-1].pos - pix[j0].pos;
							double len = dir * total_diff;
							if (len >= kMinSegLength) {
								LineDetection det(unproject(pix[j0].pos), unproject(pix[j-1].pos));
								det.axis = i;
								det.confidence = j-j0;
								for (int k = j0; k < j; k++) {
									det.pixels->push_back(round_pos(pix[k].pos));
								}
								detections[i].push_back(det);
							}
							j0 = j;
						}
					}
				}
			}
		}
	}

	void GuidedLineDetector::ComputeVptWindows() {
		// Compute the vanishing points for each window location
		const int kRadius = 10;
		const int kNWin = (kRadius*2+1) * (kRadius*2+1);
		const int kNumSamples = 250;
		const int kScanStride = 3;
		const double kThetaAtten = 2000;  // parameter for scoring vpt/pixel agreement
		const double kStepDist = 4.0;  // number of (approx) pixels per vpt window

		// Compute the size in the retina of a pixel at the image centre
		Vec2 c = makeVector(input->nx()/2, input->ny()/2);
		Vec2 d = c+makeVector(1,1);
		double step = kStepDist * norm(input->pc.ImToRet(c) - input->pc.ImToRet(d));

		vector<Vector<5> > quad_norms[3];
	
		// Generate the vanishing point windows
		for (int i = 0; i < 3; i++) {
			vpt_windows[i].resize(kNWin);
			vpt_votes[i].resize(kNWin);
			fill_all(vpt_votes[i], 0);
			quad_norms[i].resize(kNWin);

			Vec3 v0 = unit(retina_vpts[i]);
			Vec3 v1 = unit(retina_vpts[(i+1)%3]);
			Vec3 v2 = unit(retina_vpts[(i+2)%3]);
			int j = 0;
			for (int d1 = -kRadius; d1 <= kRadius; d1++) {
				for (int d2 = -kRadius; d2 <= kRadius; d2++) {
					Vec3 retina_vpt = v0 + step*d1*v1 + step*d2*v2;
					Vec3 v = unproject(input->pc.RetToIm(project(retina_vpt)));
					vpt_windows[i][j] = v;
					quad_norms[i][j] = makeVector(v[0]*v[0]+v[1]*v[1],
																				-2*v[0]*v[2],
																				v[2]*v[2],
																				-2*v[1]*v[2],
																				v[2]*v[2]);
					j++;
				}
			}
		}

		// Scan the image
		vector<ImageRef> pixels[3];
		vector<double> wcum[3];
		double wtotal[] = {0,0,0};
		for (int y = 0; y < input->ny(); y += kScanStride) {
			const int* arow = assocs[y];
			const float* magrow = gradients.magnitude_sqr[y];
			for (int x = 0; x < input->nx(); x += kScanStride) {
				if (arow[x] >= 0) {
					int assoc = arow[x];
					pixels[assoc].push_back(ImageRef(x,y));
					wtotal[assoc] += 1.0;//magrow[x];
					wcum[assoc].push_back(wtotal[assoc]);
				}
			}
		}

		// Sample pixels, build the windows
		for (int i = 0; i < 3; i++) {
			vpt_samples[i].clear();

			double wtotal = wcum[i].back();
			for (int j = 0; j < kNumSamples; j++) {
				double r = rand() * wtotal / RAND_MAX;
				int index = lower_bound(wcum[i].begin(), wcum[i].end(), r) - wcum[i].begin();
				const ImageRef& pos = pixels[i][index];
				vpt_samples[i].push_back(pos);

				int x = pos.x;
				int y = pos.y;
				double mag = sqrt(gradients.magnitude_sqr[pos.y][pos.x]);
				double dx = gradients.diffx[pos].y/mag;
				double dy = gradients.diffy[pos].y/mag;
				// do not let large magnitudes bias the vote
				double weight = sqrt(mag);

				for (int k = 0; k < vpt_windows[i].size(); k++) {
					const Vec3& v = vpt_windows[i][k];
					const Vector<5>& n = quad_norms[i][k];
					// We write out these dot products for (significant) efficiency
					double line_norm = sqrt(n[0] + n[1]*x + n[2]*x*x + n[3]*y + n[4]*y*y);
					double proj = abs(((v[0]-x*v[2])*(-dy) + (v[1]-y*v[2])*dx) / line_norm);
					// apply sqrt here a second time
					double score = weight * pow(proj, kThetaAtten);
					vpt_votes[i][k] += score;
				}
			}
		}
	}

	void GuidedLineDetector::DrawVptWindows(ImageRGB<byte>& canvas) {
		for (int i = 0; i < 3; i++) {
			PixelRGB<byte> color(i==0?255:0, i==1?255:0, i==2?255:0);

			double max_score = 0;
			for (int j = 0; j < vpt_windows[i].size(); j++) {
				if (vpt_votes[i][j] > max_score) {
					max_score = vpt_votes[i][j];
				}
			}

			// Draw the sample locations
			BOOST_FOREACH(const ImageRef& pos, vpt_samples[i]) {
				DrawSpot(canvas, makeVector(pos.x, pos.y), color, 1);
			}

			// Draw the windows
			for (int j = 0; j < vpt_windows[i].size(); j++) {
				Vec2 im_v = project(vpt_windows[i][j]);
				if (input->contains(ImageRef(im_v[0], im_v[1]))) {
					int ins = 255 * vpt_votes[i][j] / max_score;
					PixelRGB<byte> win_color(i==0?ins:0, i==1?ins:0, i==2?ins:0);
					if (vpt_votes[i][j] == max_score) {
						win_color = PixelRGB<byte>(255,255,255);
					}
					DrawSpot(canvas, makeVector(im_v[0], im_v[1]), win_color, 2);
				}
			}
		}
	}

	void GuidedLineDetector::OutputVptWindowViz(const string& filename) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		DrawVptWindows(canvas);
		WriteImage(filename, canvas);
	}




	void GuidedLineDetector::DrawSceneAxes(ImageRGB<byte>& canvas) {
		DrawAxes(canvas, input->pc);
	}

	void GuidedLineDetector::OutputSceneAxesViz(const string& filename) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		DrawSceneAxes(canvas);
		WriteImage(filename, canvas);
	}

	void GuidedLineDetector::DrawBins(ImageRGB<byte>& canvas) {
		for (int y = 0; y < canvas.GetHeight(); y++) {
			const int* arow = assocs[y];
			const float* trow = thetas[y];
			for (int x = 0; x < canvas.GetWidth(); x++) {
				if (arow[x] == -1) {
					canvas[y][x] = PixelRGB<byte>(255,255,255);
				} else if (arow[x] >= 0) {
					int t_bin = GetBinForTheta(trow[x], arow[x]);
					int ins = ReverseBits(t_bin%256);
					PixelRGB<byte> color = BrightColors::Get(arow[x]);
					color.r = (color.r * ins)/256;
					color.g = (color.g * ins)/256;
					color.b = (color.b * ins)/256;
					canvas[y][x] = color;
				}
			}
		}
	}

	void GuidedLineDetector::OutputBinViz(const string& filename) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		DrawBins(canvas);
		WriteImage(filename, canvas);
	}

	void GuidedLineDetector::DrawAssociations(ImageRGB<byte>& canvas) {
		for (int y = 0; y < canvas.GetHeight(); y++) {
			const int* arow = assocs[y];
			for (int x = 0; x < canvas.GetWidth(); x++) {
				if (arow[x] == -1) {
					canvas[y][x] = PixelRGB<byte>(255,255,255);
				} else if (arow[x] >= 0) {
					canvas[y][x] = BrightColors::Get(arow[x]);
				}
			}
		}
	}

	void GuidedLineDetector::OutputAssociationViz(const string& filename) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		DrawAssociations(canvas);
		WriteImage(filename, canvas);
	}

	void GuidedLineDetector::DrawDists(ImageRGB<byte>& canvas) {
		for (int y = 0; y < canvas.GetHeight(); y++) {
			const int* arow = assocs[y];
			const float* d0row = d0s[y];
			for (int x = 0; x < canvas.GetWidth(); x++) {
				if (arow[x] == -1) {
					canvas[y][x] = PixelRGB<byte>(255,255,255);
				} else if (arow[x] >= 0) {
					const float& m = min_d0[arow[x]];
					const float& M = max_d0[arow[x]];
					canvas[y][x] = BrightColors::Get(arow[x]);
					canvas[y][x].r *= (d0row[x]-m)/(M-m);
					canvas[y][x].g *= (d0row[x]-m)/(M-m);
					canvas[y][x].b *= (d0row[x]-m)/(M-m);
				}
			}
		}
	}

	void GuidedLineDetector::OutputDistsViz(const string& filename) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		DrawDists(canvas);
		WriteImage(filename, canvas);
	}

	void GuidedLineDetector::OutputThetaViz(const string& basename) {
		ImageRGB<byte> canvases[3];
		for (int j = 0; j < 3; j++) {
			ImageCopy(input->rgb, canvases[j]);
		}
		for (int y = 0; y < input->ny(); y++) {
			const int* arow = assocs[y];
			const float* trow = thetas[y];
			for (int x = 0; x < input->nx(); x++) {
				if (arow[x] >= 0) {
					double t = Ring(trow[x]+theta_offsets[arow[x]], 2*M_PI) / theta_spans[arow[x]];
					canvases[ arow[x] ][y][x] = PixelRGB<byte>(255*t, 255*(1-t), 0);
				}
			}
		}
		for (int j = 0; j < 3; j++) {
			WriteImage(basename+PaddedInt(j,1)+".png", canvases[j]);
		}
	}

	void GuidedLineDetector::DrawRays(ImageRGB<byte>& canvas, int axis) {
		Vec2 retina_tl = input->pc.ImToRet(makeVector(0.0, 0.0));
		Vec2 retina_br = input->pc.ImToRet(makeVector(canvas.GetWidth()-1.0,
																											 canvas.GetHeight()-1.0));
		Vec3 left = makeVector(1, 0, -retina_tl[0]);
		Vec3 right = makeVector(-1, 0, retina_br[0]);
		Vec3 top = makeVector(0, 1, -retina_tl[1]);
		Vec3 bottom = makeVector(0, -1, retina_br[1]);

		for (int i = 0; i < 3; i++) {
			if (i != axis && axis != -1) continue;
			float max_support = 0;
			typedef pair<float, int> Peak;
			BOOST_FOREACH(const Peak& peak, peaks[i]) {
				int support = histogram[i][peak.second].support;
				if (support > max_support) {
					max_support = support;
				}
			}

			const Vec3& v0 = retina_vpts[i];
			const Vec3& v1 = retina_vpts[(i+1)%3];
			const Vec3& v2 = retina_vpts[(i+2)%3];

			COUNTED_FOREACH(int j, const Peak& peak, peaks[i]) {
				// TODO: this will only draw rays in a 180 degree arc
				double t = peak.first-M_PI;
				// Intersect a ray from canon_x in the direction of v0 with the image plane.
				Vec3 canon_x = 100.0 * (sin(t)*v1 + cos(t)*v2);
				Vec3 retina_x = canon_x + (1-canon_x[2])*atretina(v0);
				Vec3 retina_vpt = v0;  // copy here because we will modify during clipping

				ClipAgainstLine(retina_x, retina_vpt, left, 1);
				ClipAgainstLine(retina_x, retina_vpt, right, 1);
				ClipAgainstLine(retina_x, retina_vpt, top, 1);
				ClipAgainstLine(retina_x, retina_vpt, bottom, 1);

				Vec2 image_x = input->pc.RetToIm(project(retina_x));
				Vec2 image_vpt = input->pc.RetToIm(project(retina_vpt));

				int ins = 127 + floori(128 * histogram[i][peak.second].support / max_support);
				PixelRGB<byte> color(i==0?ins:0, i==1?ins:0, i==2?ins:0);
				if (draw_thick_lines) {
					DrawThickLineClipped(canvas, image_vpt, image_x, color, 2);
				} else {
					DrawLine(canvas, image_vpt, image_x, color);
				}
			}
		}
	}

	void GuidedLineDetector::OutputRayViz(const string& filename, int axis) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		DrawRays(canvas, axis);
		WriteImage(filename, canvas);
	}

	void GuidedLineDetector::DrawSegments(ImageRGB<byte>& canvas, int axis) {
		for (int i = 0; i < 3; i++) {
			if (i != axis && axis != -1) continue;
			BOOST_FOREACH(const LineDetection& det, detections[i]) {
				det.DrawLine(canvas, Colors::primary(i), Zeros, draw_thick_lines?2:1);
			}
		}
	}

	void GuidedLineDetector::OutputSegmentsViz(const string& filename, int axis) {
		ImageRGB<byte> canvas;
		canvas.AllocImageData(640, 480);
		ImageCopy(input->rgb, canvas);
		DrawSegments(canvas, axis);
		WriteImage(filename, canvas);
	}

	void GuidedLineDetector::DrawSegPixels(ImageRGB<byte>& canvas) {
		for (int i = 0; i < 3; i++) {
			BOOST_FOREACH(const LineDetection& det, detections[i]) {
				det.DrawPixels(canvas, Colors::primary(i), Zeros);
			}
		}
	}

	void GuidedLineDetector::OutputSegPixelsViz(const string& filename) {
		ImageRGB<byte> canvas;
		ImageCopy(input->rgb, canvas);
		DrawSegPixels(canvas);
		WriteImage(filename, canvas);
	}
}
