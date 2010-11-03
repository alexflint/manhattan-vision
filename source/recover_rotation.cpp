#include <cmath>

#include <iostream>
#include <sstream>

#include <se3.h>
#include <LU.h>
#include <Cholesky.h>
#include <helpers.h>

#include "misc.h"
#include "common_types.h"
#include "vanishing_points.h"
#include "timer.h"
#include "image_bundle.h"
#include "unwarped_image.h"
#include "map_widgets.h"
#include "vars.h"
#include "clipping.h"
#include "progress_reporter.h"
#include "hotspot.h"

#include "image_utils.tpp"
#include "io_utils.tpp"
//#include "numeric_utils.tpp"
#include "range_utils.tpp"

using namespace indoor_context;

//int axis_map[3];
Vector<3> vmin, vmax;

bool InBounds(const Vector<3>& v, const Vector<3>& lower, const Vector<3>& upper) {
	for (int i = 0; i < 3; i++) {
		if (v[i] < lower[i] || v[i] > upper[i]) return false;
	}
	return true;
}

void RenderAxes();
void RenderAxesHistogram(const Matrix<3,Dynamic>& hist);
void RenderPeakPlanes(const Matrix<3,Dynamic>& hist, int axis);
void RenderProjectedLines(const Map& map, const Vector<4>& plane);
void RenderUnProjections(MapViz& mapviz);
void RenderMatchedLines(const MapViz& mapviz);

void RenderSphereAt(const Vector<3>& v, double radius);

Vector<4> AppendZero(const Vector<3>& v) {
	Vector<4> y = Zeros;
	y.slice<0,3>() = v;
	return y;
}

// Compute the closest two points on two lines
pair<Vector<3>, Vector<3> > LineIsct3D(const Vector<3>& a1, const Vector<3>& a2,
																			 const Vector<3>& b1, const Vector<3>& b2) {
	double na = ((b2-b1) ^ (a1-b1)) * ((a2-a1) ^ (b2-b1));
	double nb = ((a2-a1) ^ (a1-b1)) * ((a2-a1) ^ (b2-b1));
	double d = ((a2-a1) ^ (b2-b1)) * ((a2-a1) ^ (b2-b1));
	Vector<3> a0 = a1 + (na/d)*(a2-a1);
	Vector<3> b0 = b1 + (nb/d)*(b2-b1);
	return make_pair(a0,b0);
}









struct Ray {
	const KeyFrame* kf;
	const LineSegment* line;
	int line_num;
	Vector<3> start;
	Vector<3> end;
	float len;
	Vector<3> eqn;
	
	// temp...
	Vector<3> world_isct;
	float idist, jdist;
};

struct RayIsct {
	int axis;
	vector<Ray> rays;
	float spread;
	Vector<3> location;
};
	

void ToggleBool(bool& b) { b = !b; }

class ProjectedRays : public Widget3D {
public:
	static const int kMinMatches = 4;
	static const float kMaxSpread = 0.005;

	MapViz& mapviz;
	int axis;

	vector<Ray> rays;
	vector<RayIsct> filtered;
	Matrix<2,3> perm;

	bool draw_rays;
	HotSpots hotspots;

	ProjectedRays(MapViz& mv, int ax) : mapviz(mv), axis(ax), draw_rays(true) {
		mapviz.viewer.Add(hotspots);
		mapviz.viewer.BindKey('r', bind(&ToggleBool, ref(draw_rays)));
	}
	
	void Initialize() {
		static const double kClosest = 0.001;
		static const double kFurthest = 1000;

		int other1 = (axis+1)%3;
		int other2 = (axis+2)%3;
		perm = Zeros;
		perm[0][other1] = 1;
		perm[1][other2] = 1;

		rays.clear();
		filtered.clear();

		// Project each keyframe's retinal plane into the projection plane
		map<const KeyFrame*, Vector<3> > retina_projs;
		BOOST_FOREACH(const KeyFrame& kf, mapviz.map.kfs) {
			// Z coordinates define infrontness/behindness in camera
			// frame. On the projected plane, the current axis is zero, so
			// don't need that coefficient. We end up with a homogeneous
			// line equation on the projection plane.
			const Vector<2>& Rperm = perm * kf.CfW.get_rotation().get_matrix()[axis];
			retina_projs[&kf] = makeVector(Rperm[0], Rperm[1], kf.CfW.get_translation()[axis]);
		}

		// Compute the rays
		ProgressReporter rayprog(mapviz.map.kfs.size(), "Computing rays");
		BOOST_FOREACH(const KeyFrame& kf, mapviz.map.kfs) {
			COUNTED_FOREACH(int i, const LineSegment& line, kf.vpts.lines.segments) {
				if (mapviz.map.axis_map[kf.vpts.detector.owners[i]] == axis) {
					// Compute points endpoints of the rays
					Vector<3> a = kf.unwarped.image_to_retina * unproject(line.start);
					Vector<3> world_p = kf.CfW.inverse() * (a*kClosest);
					Vector<3> world_q = kf.CfW.inverse() * (a*kFurthest);

					// Project them onto the orthogonal axis
					Vector<3> proj_p = unproject(perm*world_p);
					Vector<3> proj_q = unproject(perm*world_q);

					// Add the ray to the list
					world_p[axis] = 0;
					world_q[axis] = 0;
					Ray r = {&kf, &line, i, world_p, world_q, norm_sq(world_p-world_q), proj_p^proj_q};
					rays.push_back(r);
					rayprog.Increment();
				}
			}
		}

		// Index the keyframes
		int next_ind = 0;
		map<const KeyFrame*, int> kf_inds;
		BOOST_FOREACH(const KeyFrame& kf, mapviz.map.kfs) {
			kf_inds[&kf] = next_ind++;
		}

		// Compute the intersections
		ProgressReporter isctprog(rays.size(), "Computing intesections");
		for (int i = 0; i < rays.size(); i++) {
			vector<pair<float, const Ray*> > iscts;
			for (int j = 0; j < rays.size(); j++) {
				if (j == i || rays[j].kf == rays[i].kf) continue;
				// Intersect the rays
				Vector<3> isct = rays[i].eqn ^ rays[j].eqn;
				Vector<3> world_isct = perm.T()*project(isct);

				// Compute distance from camera centre
				float idist1 = norm_sq(world_isct - rays[i].start);
				float idist2 = norm_sq(world_isct - rays[i].end);
				float jdist1 = norm_sq(world_isct - rays[j].start);
				float jdist2 = norm_sq(world_isct - rays[j].end);
				if (idist1 < rays[i].len && idist2 < rays[i].len
						&& jdist1 < rays[j].len && jdist2 < rays[j].len) {
					iscts.push_back(make_pair(idist1, &rays[j]));
				}
			}

			// Sort intersections by distance
			sort_all(iscts, compose_twice(less<double>(), &select1st<pair<float, const Ray*> >));

			// Find clusters of intersections
			vector<RayIsct> candidate_iscts;
			for (int j = 0; j < iscts.size(); j++) {
				Vector<> foundkfs(mapviz.map.kfs.size());
				foundkfs = Zeros;
				float cutoff = iscts[j].first + kMaxSpread*2;

				int k;
				int num_found = 0;
				for (k = j+1; k < iscts.size() && iscts[k].first < cutoff; k++) {
					int kfi = kf_inds[iscts[k].second->kf];
					if (!foundkfs[kfi]) {
						foundkfs[kfi] = 1;
						num_found++;
					}
				}

				if (num_found >= kMinMatches-1) {  // minus one because ray[i] is the first
					RayIsct isct;
					isct.axis = axis;
					isct.spread = iscts[k-1].first - iscts[j].first;
					for (int a = j; a < k; a++) {
						isct.rays.push_back(*iscts[a].second);
					}
					// TODO: take average or something smarter
					isct.location = rays[i].eqn ^ iscts[j].second->eqn;
					Vector<3> world_loc = perm.T()*project(isct.location);
					hotspots.Add(perm.T()*project(isct.location))
						<< "i=" << i << "\n"
						<< "j=" << j << "\n"
						<< "ray[i]=" << rays[i].eqn << "\n"
						<< "ray[other]=" << iscts[j].second->eqn << "\n"
						<< "isct=" << world_loc << "\n"
						<< "iplane=" << retina_projs[rays[i].kf] << "\n"
						<< "jplane=" << retina_projs[iscts[j].second->kf] << "\n"
						<< "idist=" << (retina_projs[rays[i].kf] * world_loc) << "\n"
						<< "jdist=" << (retina_projs[iscts[j].second->kf] * world_loc) << "\n"
						<< "num_found=" << num_found << "\n"
						<< "tally=" << foundkfs;
					candidate_iscts.push_back(isct);
					j=k;  // skip over the intersections we've just included
				}
			}

			// Add the candidate with minimum spread
			if (candidate_iscts.size() > 0) {
				int best = 0;
				for (int j = 1; j < candidate_iscts.size(); j++) {
					if (candidate_iscts[j].spread < candidate_iscts[best].spread) {
						best = j;
					}
				}
				filtered.push_back(candidate_iscts[best]);
			}
			isctprog.Increment();
		}

		DREPORT(rays.size());
		DREPORT(filtered.size());
	}


	void OnRender() {
		glLineWidth(1.2);
		glColor3f(axis!=0, axis!=1, axis!=2);

		map<const KeyFrame*, PixelRGB<byte> > colormap;
		BrightColors bc;
		BOOST_FOREACH(const KeyFrame& kf, mapviz.map.kfs) colormap[&kf] = bc.Next();

		// Draw rays
		if (draw_rays) {
			GL_PRIMITIVE(GL_LINES) {
				BOOST_FOREACH(const Ray& r, rays) {
					glColorP(colormap[r.kf]);
					glVertexV(r.start);
					glVertexV(r.end);
				}
			}
		}

		// Draw intersection points and vertical edges
		BOOST_FOREACH(const RayIsct& isct, filtered) {
			glColor4f(1, 1, 1, 0.5);
			Vector<3> pt = perm.T()*project(isct.location);
			RenderSphereAt(pt, 0.05);
		}

		// Draw the vertical lines
		BOOST_FOREACH(const RayIsct& isct, filtered) {
			glColor4f(1, 1, 1, 0.5);
			Vector<3> pt = perm.T()*project(isct.location);
			glColor3f(axis==0, axis==1, axis==2);
			GL_PRIMITIVE(GL_LINES) {
				pt[axis] = vmin[axis];
				glVertexV(pt);
				pt[axis] = vmax[axis];
				glVertexV(pt);
			}
		}
	}
};









struct CandidatePillar {
	Vector<3> bottom;
	Vector<3> top;

	int support;
	int unique_kfs;
	double residual;
	vector<int> support_kfs;
	vector<int> support_lines;
	bool is_best;

	CandidatePillar(const Vector<3>& b, const Vector<3>& t) 
		: bottom(b),
			top(t),
			support(0),
			unique_kfs(0),
			residual(0),
			is_best(false) {
	}
};

static const double kRecurseThresh = 3.0;  // in pixels
static const double kLineMargin = 3.0;  // in pixels
static const int kMinSupport = 4;  // to include a pillar
static const double kRayStart = 1.5;  // in retina lengths
static const double kRayEnd = 10;  // in retina lengths
static const double kMinPillarZ = 0.5;  // minimum Z distance for a line to be searched for
static const double kMaxPillarDist = 2.0;  // maximum distance from the image center

class ResolvedLine : public Widget3D {
public:
	MapViz& mapviz;
	int src_frame;
	int src_line;

	int axis;
	Matrix<2,3> perm;

	Widget3D pillar_container, proj_container;

	vector<CandidatePillar> cands;

	ResolvedLine(MapViz& m) : mapviz(m) {
		Add(pillar_container);
		Add(proj_container);
	}

	void GenerateCandidates(double near,
													double far,
													const Vector<3>& retina_pt,
													const SE3<>& rootkf_CfW_inv,
													vector<double>& out_samples) {
		// Recursing criteria is the maximum length of the ray segment in
		// any keyframe. This ensures that we sample at no less than every
		// k pixels in the most sparsely sampled image (where
		// k=kRecurseThresh)

		double max_dsq = 0;
		const Matrix<3>& ret_to_im = mapviz.map.undistorter.retina_to_image;
		Vector<3> a = rootkf_CfW_inv * (near*retina_pt);
		Vector<3> b = rootkf_CfW_inv * (far*retina_pt);
		BOOST_FOREACH(const KeyFrame& kf, mapviz.map.kfs) {
			const Vector<2> im_b = project(ret_to_im * (kf.CfW * b));
			// only check that it contains b since that is the only point we'll actually add
			if (kf.unwarped.image.contains(ImageRef(im_b[0], im_b[1]))) {
				const Vector<2> im_a = project(ret_to_im * (kf.CfW * a));
				max_dsq = max(max_dsq, norm_sq(im_a-im_b));
			}
		}
		if (max_dsq == 0) return; // we're outside all keyframes so skip
		if (max_dsq > kRecurseThresh) {
			double mid = 0.5*(near+far);
			GenerateCandidates(near, mid, retina_pt, rootkf_CfW_inv, out_samples);
			GenerateCandidates(mid, far, retina_pt, rootkf_CfW_inv, out_samples);
		} else {
			out_samples.push_back(far);
		}
	}

	void Compute(int kf_index, int line_index) {
		const Matrix<3>& im_to_ret = mapviz.map.undistorter.image_to_retina;
		const Matrix<3>& ret_to_im = mapviz.map.undistorter.retina_to_image;

		src_frame = kf_index;
		src_line = line_index;

		const KeyFrame& rootkf = mapviz.map.kfs[kf_index];
		const LineSegment& rootline = rootkf.vpts.lines.segments[line_index];

		int owner = mapviz.map.kfs[src_frame].vpts.detector.owners[line_index];
		axis = mapviz.map.axis_map[owner];
		int other1 = (axis+1)%3;
		int other2 = (axis+2)%3;
		perm = Zeros;
		perm[0][other1] = 1;
		perm[1][other2] = 1;

		SE3<> root_inv = rootkf.CfW.inverse();
		Vector<3> origin = root_inv*makeVector(0,0,0);

		// Compute the points on the line in camera coords
		Vector<3> start_ret = rootkf.unwarped.image_to_retina * unproject(rootline.start);
		Vector<3> end_ret = rootkf.unwarped.image_to_retina * unproject(rootline.end);
		Vector<3> midp_ret = 0.5*(start_ret + end_ret);

		// Generate sample points
		vector<double> samples;
		samples.push_back(kRayStart);
		GenerateCandidates(kRayStart, kRayEnd, midp_ret, rootkf.CfW.inverse(), samples);

		// Select candidates
		cands.clear();
		int best_support = 0;

		BOOST_FOREACH(double depth, samples) {
			// Unproject the start and end points
			Vector<3> midp_raw = root_inv*(depth*midp_ret);
			Vector<3> top_raw = root_inv*(depth*start_ret);
			Vector<3> bottom_raw = root_inv*(depth*end_ret);

			// Compute the top and bottom of the pillar. Here we slightly
			// perturb top_raw and bottom_raw so that their orth coordinates
			// are identical.
			Vector<3> pillar_top = midp_raw;
			pillar_top[axis] = top_raw[axis];
			Vector<3> pillar_bottom = midp_raw;
			pillar_bottom[axis] = bottom_raw[axis];
			
			CandidatePillar pillar(pillar_top, pillar_bottom);
			COUNTED_FOREACH(int kfi, const KeyFrame& kf, mapviz.map.kfs) {
				Vector<3> proj_a = kf.CfW * pillar.top;
				Vector<3> proj_b = kf.CfW * pillar.bottom;

				// Skip the line if its behind the camera or too close to the camera plane
				if (proj_a[2] < kMinPillarZ || proj_b[2] < kMinPillarZ) continue;

				Vector<3> im_a = ret_to_im * proj_a;
				Vector<3> im_b = ret_to_im * proj_b;

				// Skip the line if its too far from the image center
				ImageRef sz = kf.unwarped.image.sz();
				double max_dist = kMaxPillarDist * norm_sq(makeVector(sz.x, sz.y));
				if (norm_sq(project(im_a)) > max_dist || norm_sq(project(im_b)) > max_dist) {
					continue;
				}

				Vector<3> im_line = im_a ^ im_b;
				im_line /= norm(im_line.slice<0,2>());  // normalize for euclidean dists
				COUNTED_FOREACH(int segi, const LineSegment& seg, kf.vpts.lines.segments) {
					Vector<3> start = unproject(seg.start);
					Vector<3> end = unproject(seg.end);
					double dstart = abs(start * im_line);
					double dend = abs(end * im_line);
					if (dstart < kLineMargin && dend < kLineMargin) {
						pillar.support++;
						pillar.support_kfs.push_back(kfi);
						pillar.support_lines.push_back(segi);
					}
				}
			}

			for (int i = 0; i < pillar.support; i++) {
				if (i == 0 || pillar.support_kfs[i] != pillar.support_kfs[i-1]) {
					pillar.unique_kfs++;
				}
			}

			if (pillar.unique_kfs >= kMinSupport) {
				cands.push_back(pillar);
				if (pillar.unique_kfs > best_support) {
					best_support = pillar.unique_kfs;
				}
			}
		}

		// Choose the candidates with best support
		DREPORT(best_support);
		pillar_container.ClearChildren();
		proj_container.ClearChildren();
		BOOST_FOREACH(CandidatePillar& pillar, cands) {
			if (pillar.unique_kfs == best_support) {
				LineWidget* linew = new LineWidget(pillar.top, pillar.bottom, 2.5, 1,0,1);
				pillar_container.AddOwned(linew);
				linew->Click.add(bind(&ResolvedLine::pillar_Click, this, pillar));
				pillar.is_best = true;
			}
		}

		if (pillar_container.children().empty()) {
			DLOG << "Found no depths with sufficient support";
		} else {
			DLOG << "Found " << pillar_container.children().size() << " candidates";
		}

		Invalidate();
	}


	void pillar_Click(const CandidatePillar& pillar) {
		const Matrix<3>& im_to_ret = mapviz.map.undistorter.image_to_retina;

		proj_container.ClearChildren();
		for (int i = 0; i < pillar.support; i++) {
			int kfi = pillar.support_kfs[i];
			KeyFrameWidget* kfw = mapviz.kf_widgets[kfi];
			if (i == 0 || kfi != pillar.support_kfs[i-1]) {
				kfw->border_color = PixelRGB<byte>(0,255,255);
			} else {
				kfw->border_color = PixelRGB<byte>(127,127,127);
			}

			int segi = pillar.support_lines[i];
			KeyFrame& kf = mapviz.map.kfs[kfi];
			LineSegment& seg = kf.vpts.lines.segments[segi];
			Vector<3> ret_a = atretina(im_to_ret * unproject(seg.start));
			Vector<3> ret_b = atretina(im_to_ret * unproject(seg.end));
			Vector<3> wa = kf.WfC * (kfw->retina_z * ret_a);
			Vector<3> wb = kf.WfC * (kfw->retina_z * ret_b);
			proj_container.AddOwned(new PointWidget(wa, 6.0, PixelRGB<byte>(0,255,255), -5));
			proj_container.AddOwned(new PointWidget(wb, 6.0, PixelRGB<byte>(0,255,255), -5));


			/*const KeyFrame& kf = mapviz.map.kfs[kfi];
				int segi = pillar.support_lines[i];
				double z = mapviz.kf_widgets[i]->retina_z;
				Vector<3> a = kf.WfC * (z*atretina(kf.CfW*pillar.top));
				Vector<3> b = kf.WfC * (z*atretina(kf.CfW*pillar.bottom));
				proj_container.AddOwned(new LineWidget(a, b,  2.5,  0,1,1));*/
		}
		mapviz.viewer.Invalidate();
	}
};







int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map
	MapViz mapviz;
	mapviz.map.RotateToSceneFrame();
	DREPORT(mapviz.map.scene_to_slam.ln());

	// Accumulate along each dimension
	vector<double> ps[3];
	vmin = vmax = mapviz.map.pts[0];
	int i = 0;

	BOOST_FOREACH(const Vector<3>& v, mapviz.map.pts) {
		for (int i = 0; i < 3; i++) {
			if (v[i] < vmin[i]) vmin[i] = v[i];
			if (v[i] > vmax[i]) vmax[i] = v[i];
			ps[i].push_back(v[i]);
		}
	}

	// Generate bounds and histogram
	int histSize = 100;
	Matrix<3, Dynamic> hists(3, histSize);
	hists = Zeros;
	for (int i = 0; i < 3; i++) {
		sort_all(ps[i]);

		int a, b;
		for (a = 0; ps[i][a] < 1.1*ps[i][a+1]; a++);
		vmin[i] = ps[i][a];
		for (b = ps[i].size()-1; ps[i][b] > 1.1*ps[i][b-1]; b--);
		vmax[i] = ps[i][b];

		for (int j = a; j <= b; j++) {
			int bin = floori( (ps[i][j]-vmin[i])*histSize / (vmax[i]-vmin[i]) );
			hists[i][bin] += 1.0;
		}
	}

	// Render the axes with point histograms along them
	mapviz.viewer.Add(bind(&RenderAxesHistogram, ref(hists)));

	// Render planes at peaks in the above histogram
	for (int i = 0; i < 3; i++) {
		Widget3D& w = mapviz.viewer.AddToggleable(bind(&RenderPeakPlanes, ref(hists), i), 'x'+i);
		w.Hide();  // hide by default
	}

	// Render lines in keyframes
	if (GV3::get<int>("MapViz.RenderProjectedLines")) {
		mapviz.viewer.Add(bind(&RenderProjectedLines, ref(mapviz.map), makeVector(0,0,1,0)));
	}



	ResolvedLine rline(mapviz);
	mapviz.viewer.Add(rline);

	for (int i = 0; i < mapviz.map.kfs.size(); i++) {
		mapviz.kf_widgets[i]->ConfigureLineWidgets();
		for (int j = 0; j < mapviz.map.kfs[i].vpts.lines.segments.size(); j++) {
			mapviz.kf_widgets[i]->line_widgets[j]->Click.add(bind(&ResolvedLine::Compute,
																														ref(rline), i, j));
		}
	}


	mapviz.Run();

	return 0;
}

























void RenderAxes() {
	WITHOUT(GL_LINE_SMOOTH) {
		glColor3f(0,1,1);
		glLineWidth(2);
		glPointSize(10);
		WITH(GL_BLEND) WITHOUT(GL_DEPTH_TEST) {
			for (int i = 0; i < 3; i++) {
				toon::Vector<3> a = vmin[i] * GetAxis<3>(i);
				toon::Vector<3> b = vmax[i] * GetAxis<3>(i);
				GL_PRIMITIVE(GL_POINTS) {
					glVertexV(a);
					glVertexV(b);
				}
				GL_PRIMITIVE(GL_LINES) {
					glVertexV(a);
					glVertexV(b);
				}
			}
		}
	}
}


void RenderAxesHistogram(const Matrix<3,Dynamic>& hist) {
	int histSize = hist.num_cols();
	glColor3f(0,1,1);
	glLineWidth(5);
	glPointSize(10);
	WITH(GL_BLEND) {
		for (int i = 0; i < 3; i++) {
			toon::Vector<3> a = vmin[i] * GetAxis<3>(i);
			toon::Vector<3> b = vmax[i] * GetAxis<3>(i);
			float histmax = *max_element(&hist[i][0], &hist[i][histSize]);
			GL_PRIMITIVE(GL_LINE_STRIP) {
				for (int j = 0; j < histSize; j++) {
					float f = 1.0*j/histSize;
					float y = hist[i][j]/histmax;
					float yd = 0.5+0.5*y;
					glColor3f(i==0 ? yd:y, i==1 ? yd:y, i==2 ? yd:y);
					glVertexV((1-f)*a + f*b);
				}
				glVertexV(b);
			}

			GL_PRIMITIVE(GL_POINTS) {
				glVertexV(a);
				glVertexV(b);
			}
		}
	}
}

 void RenderPeakPlanes(const Matrix<3,Dynamic>& hist, int axis) {
	int histSize = hist.num_cols();
	glColor3f(0,1,1);
	glLineWidth(5);
	glPointSize(10);
	WITH(GL_BLEND) {
		toon::Vector<3> a = vmin[axis] * GetAxis<3>(axis);
		toon::Vector<3> b = vmax[axis] * GetAxis<3>(axis);

		int other1 = (axis+1)%3;
		int other2 = (axis+2)%3;
		Vector<8> corners = makeVector(vmin[other1],vmin[other2],
																	 vmin[other1],vmax[other2],
																	 vmax[other1],vmax[other2],
																	 vmax[other1],vmin[other2])/10;
		Matrix<3,2> perm = Zeros;
		perm[other1][0] = 1;
		perm[other2][1] = 1;

		Vector<> v = hist[axis];
		double maxv = *max_element(&v[0], &v[0]+v.size());
		glColor4f(axis!=0, axis!=1, axis!=2, 0.6);
		for (int j = 0; j < v.size(); j++) {
			if ((j==0 || v[j]>v[j-1]) && (j==v.size()-1 || v[j]>v[j+1]) && v[j] > 0.2*maxv) {
				float f = 1.0*j/histSize;

				GL_MATRIX_SCOPE {
					Vector<3> pos = (1-f)*a + f*b;
					glTranslatef(pos[0], pos[1], pos[2]);
					GL_PRIMITIVE(GL_QUADS) {
						for (int k = 0; k < 4; k++) {
							glVertexV(perm * makeVector(corners[k*2], corners[k*2+1]));
						}
					}
				}
			}
		}
	}
}


void RenderProjectedLines(const Map& map, const Vector<4>& plane) {
	glLineWidth(1.2);
	BOOST_FOREACH(const KeyFrame& kf, map.kfs) {
		Matrix<4> m;
		m.slice<0,0,3,3>() = kf.CfW.get_rotation().get_matrix();
		m.slice<0,3,3,1>() = kf.CfW.get_translation().as_col();
		m.slice<3,0,1,4>() = plane.as_row();
		Matrix<4> minv = LU<4>(m).get_inverse();
		GL_PRIMITIVE(GL_LINES) {
			BOOST_FOREACH(const LineSegment& line, kf.vpts.lines.segments) {
				Vector<4> pa = minv * AppendZero(kf.unwarped.image_to_retina * unproject(line.start));
				Vector<4> pb = minv * AppendZero(kf.unwarped.image_to_retina * unproject(line.end));
				glColor3f(0,1,1);
				glVertexV(pa);
				glVertexV(pb);
			}
		}
	}
}


void RenderSphereAt(const Vector<3>& v, double radius) {
	GLfloat cur_color[4];
	glGetFloatv(GL_CURRENT_COLOR, cur_color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, cur_color);
	WITH2(GL_LIGHTING, GL_LIGHT0) GL_MATRIX_SCOPE {
		glTranslatef(v[0], v[1], v[2]);
		glutSolidSphere(radius, 10, 10);
	}
}

void RenderUnProjections(MapViz& mapviz) {
	glLineWidth(1.2);
	glPointSize(2.5);
	glColor3f(1,1,1);

	const KeyFrame& kf = mapviz.map.kfs[0];
	const KeyFrame& kf_o = mapviz.map.kfs[1];
	const LineSegment& line = kf.vpts.lines.segments[0];

	Vector<2> a = project(kf.unwarped.image_to_retina * unproject(line.start));
	Vector<2> b = project(kf.unwarped.image_to_retina * unproject(line.end));
	
	GL_MATRIX_SCOPE {
		mapviz.kf_widgets[0]->GLTransformToCameraCoords();
		GL_PRIMITIVE(GL_LINES) {
			glVertex3f(0,0,0);
			glVertexV(10*unproject(a));
		}
		GL_PRIMITIVE(GL_LINES) {
			glVertex3f(0,0,0);
			glVertexV(10*unproject(b));
		}

		RenderSphereAt(unproject(a), 0.02);
		RenderSphereAt(unproject(b), 0.02);
	}


	Vector<3> c_o = kf_o.CfW.inverse() * makeVector(0,0,0);

	for (int z = 2; z < 10; z++) {
		Vector<3> wa = kf.CfW.inverse() * (z*unproject(a));
		Vector<3> wb = kf.CfW.inverse() * (z*unproject(b));
		Vector<3> wc = kf.CfW.inverse() * makeVector(0,0,0);

		pair<Vector<3>,Vector<3> > p = LineIsct3D(wa,wa+GetAxis<3>(2),wb,wc);

		GL_PRIMITIVE(GL_LINES) {
			glColor3f(0,1,0);
			glVertexV(wa);
			glVertexV(p.first);

			glColor3f(1,1,1);
			glVertexV(wa);
			glVertexV(c_o);
			glVertexV(p.first);
			glVertexV(c_o);
		}

		GL_MATRIX_SCOPE {
			mapviz.kf_widgets[1]->GLTransformToCameraCoords();
			glColor3f(0,0,1);
			RenderSphereAt(unproject(project(kf_o.CfW * wa)), 0.01);
			RenderSphereAt(unproject(project(kf_o.CfW * p.first)), 0.01);
		}
	}
}

void RenderMatchedLines(const MapViz& mapviz) {
	glLineWidth(1.2);
	glPointSize(2.5);
	glColor3f(1,1,1);

	const KeyFrame& kf = mapviz.map.kfs[0];
	const KeyFrame& kf_o = mapviz.map.kfs[1];
	const LineSegment& line = kf.vpts.lines.segments[0];

	// Line endpoints in the first image
	Vector<3> a = kf.unwarped.image_to_retina * unproject(line.start);
	Vector<3> b = kf.unwarped.image_to_retina * unproject(line.end);
	// and their world coordinates (doesn't matter what z value we choose)
	Vector<3> a_w = kf.CfW.inverse() * a;
	Vector<3> b_w = kf.CfW.inverse() * b;
	// and their projections (directly) into the second image
	Vector<3> a_o = atretina(kf_o.CfW * a_w);
	Vector<3> b_o = atretina(kf_o.CfW * b_w);

	// The camera center of the first and its projection into the second image
	Vector<3> c = kf.CfW.inverse() * makeVector(0,0,0);
	Vector<3> c_o = atretina(kf_o.CfW * c);

	glColor3f(1,1,1);
	GL_PRIMITIVE(GL_LINES) {
		glVertexV(kf.CfW.inverse() * makeVector(0,0,0));
		glVertexV(kf_o.CfW.inverse() * c_o);

		glVertexV(kf_o.CfW.inverse() * makeVector(0,0,0));
		glVertexV(kf_o.CfW.inverse() * a_o);

		glVertexV(kf_o.CfW.inverse() * makeVector(0,0,0));
		glVertexV(kf_o.CfW.inverse() * b_o);

 		glVertexV(kf_o.CfW.inverse() * c_o);
		glVertexV(kf_o.CfW.inverse() * a_o);

		glVertexV(kf_o.CfW.inverse() * c_o);
		glVertexV(kf_o.CfW.inverse() * b_o);
	}

	// Clip the two epipolar lines to the image poly
	Vector<3> cl_a1, cl_a2, cl_b1, cl_b2;
	ClipLineToPoly(a_o^c_o, kf.unwarped.map->retina_bounds, cl_a1, cl_a2);
	ClipLineToPoly(b_o^c_o, kf.unwarped.map->retina_bounds, cl_b1, cl_b2);
	glColor3f(1,1,0);
	glLineWidth(2.5);
	GL_PRIMITIVE(GL_LINES) {
		glVertexV(kf_o.CfW.inverse() * atretina(cl_a1));
		glVertexV(kf_o.CfW.inverse() * atretina(cl_a2));

		glVertexV(kf_o.CfW.inverse() * atretina(cl_a1));
		glVertexV(kf_o.CfW.inverse() * atretina(cl_a2));
		glVertexV(kf_o.CfW.inverse() * atretina(cl_b1));
		glVertexV(kf_o.CfW.inverse() * atretina(cl_b2));
	}
}
