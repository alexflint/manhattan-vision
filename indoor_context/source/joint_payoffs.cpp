#include "entrypoint_types.h"

#include "joint_payoffs.h"
#include "timer.h"

namespace indoor_context {
	// Call these "coef" to avoid confusion with Texton.Features.MonoWeight etc
	lazyvar<double> gvMonoCoef("JointPayoffs.Mono.Weight");
	lazyvar<double> gvOcclusionCoef("JointPayoffs.3D.OcclusionWeight");
	lazyvar<double> gvAgreementCoef("JointPayoffs.3D.AgreementWeight");
	lazyvar<double> gvStereoCoef("JointPayoffs.Stereo.Weight");

	JointPayoffGen::JointPayoffGen() {
		RevertWeights();
	}

	JointPayoffGen::JointPayoffGen(const PosedImage& image,
																 const DPGeometryWithScale& geometry,
																 const vector<Vec3>& point_cloud,
																 const vector<const PosedImage*>& aux_images) {
		RevertWeights();
		Compute(image, geometry, point_cloud, aux_images);
	}

	void JointPayoffGen::RevertWeights() {
		mono_weight = *gvMonoCoef;
		occlusion_weight = *gvOcclusionCoef;
		agreement_weight = *gvAgreementCoef;
		stereo_weight = *gvStereoCoef;
	}

	void JointPayoffGen::Compute(const PosedImage& image,
															 const DPGeometryWithScale& geom,
															 const vector<Vec3>& point_cloud,
															 const vector<const PosedImage*>& aux_images) {
		input = &image;
		geometry = geom;

		// Compute monocular payoffs
		TIMED("Mono payoffs") {
			objective_gen.Compute(image);
			mono_gen.Compute(objective_gen.objective, geom);
		}

		// Compute 3D payoffs
		TIMED("3D payoffs") {
			point_cloud_gen.Compute(point_cloud, image.pc(), geom);
		}

		// Compute stereo payoffs
		TIMED("Stereo payoffs") {
			stereo_gens.resize(aux_images.size());
			for (int i = 0; i < aux_images.size(); i++) {
				stereo_gens[i].Compute(image, *aux_images[i], geom);
			}
		}

		// Combine payoffs
		payoffs.Resize(geom.grid_size);  // always forces a Clear(0)
		payoffs.Add(mono_gen.payoffs, mono_weight);
		payoffs.Add(point_cloud_gen.agreement_payoffs, agreement_weight);
		payoffs.Add(point_cloud_gen.occlusion_payoffs, occlusion_weight);
		if (!aux_images.empty()) {
			double stereo_weight_per_aux = stereo_weight / aux_images.size();
			BOOST_FOREACH(const StereoPayoffGen& stereo_gen, stereo_gens) {
				// TODO: pass gradient images to stereo (See stereo_dp.cpp)
				payoffs.Add(stereo_gen.payoffs, stereo_weight_per_aux);
			}
		}
	}

	void JointPayoffGen::Compute(const PosedImage& image,
															 const DPGeometryWithScale& geometry,
															 const vector<Vec3>& point_cloud,
															 const vector<PosedImage*>& aux_images) {
		Compute(image,
						geometry,
						point_cloud,
						reinterpret_cast<const vector<const PosedImage*>&>(aux_images));
	}
}
