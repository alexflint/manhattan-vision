#!/bin/bash

SEQUENCES="exeter_bursary exeter_mcr1 lab_foyer1 lab_foyer2 lab_ground1 lab_kitchen1 magd_bedroom magd_living som_corr1"

SEQUENCES="lab_foyer1 exeter_mcr1"

SEQUENCE_ARGS=""
for i in $SEQUENCES; do
		SEQUENCE_ARGS="$SEQUENCE_ARGS --sequence=$i";
done

time -f "%E elapsed (real)" \
		./dp_optimization_wrapper \
		--corner_penalty=400 \
		--occlusion_penalty=300 \
		--mono_weight=0.001 \
		--stereo_weight=50 \
		--3d_agreement_sigma=1.5 \
		--3d_agreement_weight=1000 \
		--3d_occlusion_weight=40 \
		--frame_stride=10 \
		$SEQUENCE_ARGS \
		$*
