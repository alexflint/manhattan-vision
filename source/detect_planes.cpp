#include <iomanip>

#include "common_types.h"
#include "map_widgets.h"
#include "map.h"
#include "textons.h"
#include "vars.h"
#include "viewer3d.h"

using namespace indoor_context;

void ToggleTextons(MapViz& mapviz) {
	BOOST_FOREACH(KeyFrameWidget* w, mapviz.kf_widgets) {
		w->show_textons = !w->show_textons;
	}
	mapviz.viewer.Invalidate();
}

lazyvar<int> gvSearchDensity("MIPlanes.SearchDensity");
lazyvar<int> gvSampleDensity("MIPlanes.SampleDensity");

class ResolvedPlane : public Widget3D {
public:
	MapViz& mapviz;

	int axis, other1, other2;
	Vector<3> axisvec;
	Matrix<2,3> perm;
	float s_min, s_max, u_min, u_max, v_min, v_max;

	ResolvedPlane(MapViz& mv) : mapviz(mv) {
	}

	void Compute(int ax) {
		ClearChildren();

		axis = ax;
		other1 = (axis+1)%3;
		other2 = (axis+2)%3;
		axisvec = GetAxis<3>(axis);

		perm = Zeros;
		perm[0][other1] = 1;
		perm[1][other2] = 1;

		s_min = mapviz.map.vmin[axis] - 0.5;
		s_max = mapviz.map.vmax[axis] + 0.5;
		u_min = mapviz.map.vmin[other1];
		u_max = mapviz.map.vmax[other1];
		v_min = mapviz.map.vmin[other2];
		v_max = mapviz.map.vmax[other2];

		const int kSearchPts = *gvSearchDensity;
		const int kSamplePts = *gvSampleDensity;

		Vector<3> pos;
		PixelRGB<byte> ptcolor = BrightColors::Get(1);

		int num_textons = mapviz.map.kfs[0].texton_map.vocab.words.size();

		double mis[kSearchPts];

		Matrix<> joint_hist(num_textons, num_textons);
		Vector<> u_hist(num_textons), v_hist(num_textons);
		for (int i = 0; i < kSearchPts; i++) {
			float s = s_min + i * (s_max-s_min)/(kSearchPts-1);

			for (int a = 0; a < num_textons; a++) {
				u_hist[a] = 0;
				v_hist[a] = 0;
				for (int b = 0; b < num_textons; b++) {
					joint_hist[a][b] = 0;
				}
			}

			// Accumulate the histograms
			int num_samples = 0;
			AccumulateHistogram(s, u_min, u_max, kSamplePts,
													v_min, v_max, kSamplePts,
													u_hist, v_hist, joint_hist, num_samples);

			// Compute individual entropies
			if (num_samples > 100) {
				double hu = GetEntropy(u_hist);
				double hv = GetEntropy(v_hist);
				double hjoint = GetEntropy(joint_hist);
				mis[i] = hu + hv - hjoint;
				DLOG << "MI for i=" << i << ": " << mis[i] << " (" << num_samples << " samples)";
			} else {
				mis[i] = 0;
			}

			//DREPORT(u_hist, v_hist, joint_hist);

			// Compute mutual information
			//DREPORT(hu, hv, hjoint);
		}

		AddPlaneAt(s_min, 1,0,0);
		AddPlaneAt(s_max, 1,1,1);

		double mi_max = *max_element(&mis[0], &mis[kSearchPts]);
		for (int i = 0; i < kSearchPts; i++) {
			if ((i==0 || mis[i]>mis[i-1]) && (i==kSearchPts-1 || mis[i]>mis[i+1])) {

				DLOG << "Peak at i="<<i<<": MI="<<mis[i];
				float s = s_min + i * (s_max-s_min)/kSearchPts;
				//AddPlaneAt(s, 1,1,0);

				for (int j = 0; j < 5; j++) {
					double va = v_min + j*(v_max-v_min)/5;
					double vb = v_min + (j+1)*(v_max-v_min)/5;
					for (int k = 0; k < 5; k++) {
						double ua = u_min + k*(u_max-u_min)/5;
						double ub = u_min + (k+1)*(u_max-u_min)/5;

						// Re-zero
						u_hist *= 0;
						v_hist *= 0;
						joint_hist *= 0;

						int num_samples = 0;
						AccumulateHistogram(s, ua, ub, kSamplePts/3,
																va, vb, kSamplePts/3,
																u_hist, v_hist, joint_hist, num_samples);
						
						// Compute mutual information
						if (num_samples > 100) {
							double hu = GetEntropy(u_hist);
							double hv = GetEntropy(v_hist);
							double hjoint = GetEntropy(joint_hist);
							double mi_cur = hu + hv - hjoint;

							float y = mi_cur / mi_max;
							AddPlaneAt(s, ua, ub, va, vb,
												 y, 0, 1-y);
						}
					}
				}
			}
		}
	}

	double GetEntropy(const Vector<>& hist) {
		double h = 0, sum = 0;
		for (int j = 0; j < hist.size(); j++) {
			if (hist[j] > 0) {
				// Note that the limit as x goes to 0 of x*log(x) is 0 so this
				// is the correct thing to do.
				h -= hist[j]*log(hist[j]);
				sum += hist[j];
			}
		}
		return log(sum) + h/sum;
	}

	double GetEntropy(const Matrix<>& hist) {
		double h = 0, sum = 0;
		for (int j = 0; j < hist.num_cols(); j++) {
			for (int k = 0; k < hist.num_cols(); k++) {
				if (hist[j][k] > 0) {
					// Note that the limit as x goes to 0 of x*log(x) is 0 so this
					// is the correct thing to do.
					h -= hist[j][k] * log(hist[j][k]);
					sum += hist[j][k];
				}
			}
		}
		return log(sum) + h/sum;
	}

	void AccumulateHistogram(float s,
													 float ua, float ub, int nu,
													 float va, float vb, int nv,
													 Vector<>& u_hist, Vector<>& v_hist,
													 Matrix<>& joint_hist, int& num_samples) {
		int textons[2];
		Vector<3> pos = Zeros;
		pos[axis] = s;
		for (int i = 0; i < nu; i++) {
			pos[other1] = ua + i*(ub-ua) / (nu-1);

			for (int j = 0; j < nv; j++) {
				pos[other2] = va + j*(vb-va) / (nv-1);

				// Get the textons at this position
				textons[0] = textons[1] = -1;
				for (int a = 0; a < 2; a++) {
					const KeyFrame& kf = mapviz.map.kfs[a];
					Vector<2> impos = project(kf.unwarped.retina_to_image * (kf.CfW * pos));
					ImageRef ir(roundi(impos[0]), roundi(impos[1]));
					if (kf.unwarped.image.contains(ir)) {
						textons[a] = kf.texton_map.map[ir.y][ir.x];
					}
				}

				// Add to histogram
				if (textons[0] != -1 && textons[1] != -1) {
					u_hist[textons[0]]++;
					v_hist[textons[1]]++;
					joint_hist[ textons[0] ][ textons[1] ]++;
					num_samples++;
				}
			}
		}
	}

	void AddPlaneAt(float s, float r, float g, float b) {
		AddPlaneAt(s, u_min, u_max, v_min, v_max, r, g, b);
	}

	void AddPlaneAt(float s, float ua, float ub, float va, float vb,
									float r, float g, float b) {
		Vector<3> A = perm.T() * makeVector(ua,va) + s*axisvec;
		Vector<3> B = perm.T() * makeVector(ub,va) + s*axisvec;
		Vector<3> C = perm.T() * makeVector(ub,vb) + s*axisvec;
		Vector<3> D = perm.T() * makeVector(ua,vb) + s*axisvec;
		QuadWidget* w = new QuadWidget(A, B, C, D);
		w->color.r = r*255;
		w->color.g = g*255;
		w->color.b = b*255;
		AddOwned(w);
	}

	void OnRender() {
		
	}
};

int main(int argc, char **argv) {
	InitVars(argc, argv);
	Viewer3D::Init(&argc, argv);
	if (argc != 1) {
		DLOG << "Usage: "<<argv[0];
		return 0;
	}

	// Load the map and rotate to canonical frame
	MapViz mapviz;
	mapviz.map.RotateToSceneFrame();

	// Load the texton vocab
	TextonVocab vocab;
	vocab.Load(GV3::get<string>("Textons.VocabFile"));

	// Compute textons
	BOOST_FOREACH(KeyFrame& kf, mapviz.map.kfs) {
		kf.ComputeTextonMap(vocab);
	}

	// Toggleable textons
	mapviz.viewer.BindKey('t', bind(&ToggleTextons, ref(mapviz)));

	// Resolved planes
	ResolvedPlane rplane(mapviz);
	mapviz.viewer.Add(rplane);

	mapviz.viewer.BindKey('x', bind(&ResolvedPlane::Compute, ref(rplane), 0));
	mapviz.viewer.BindKey('y', bind(&ResolvedPlane::Compute, ref(rplane), 1));
	mapviz.viewer.BindKey('z', bind(&ResolvedPlane::Compute, ref(rplane), 2));
	rplane.Compute(2);

	// Enter the event loop
	mapviz.Run();

	return 0;
}
