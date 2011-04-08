#!/bin/bash

OUTFILE=PrecomptuedData/features.protodata;

./compute_payoff_features \
		--sequence lab_kitchen1 \
		--frame_stride 10 \
		--store_features \
		--output $OUTFILE \
		$*

# Report file size
du -hs $OUTFILE