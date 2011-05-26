#!/bin/bash

IDEAL='20 20 20 20'
FALLOFF='.1 .1 .1 .1'  # with respect to _normalized_ coordinates in [0,1]
NOISE='8 8 8 8'
MIX='.2 .2 .2 .2'

./compute_loglik_wrapper.sh \
		--weights="$IDEAL $FALLOFF $NOISE $MIX" \
		--corner_penalty=1 \
		--occlusion_penalty=1 \
		--features=PrecomputedData/features.protodata \
		$*

## uncomment for logistic version
#./compute_loglik \
#		--weights='2.2 1.5 10 0.4' \
#		--corner_penalty=1 \
#		--occlusion_penalty=1 \
#		--features=PrecomptuedData/features.protodata \
#   --logit \
#		$*
