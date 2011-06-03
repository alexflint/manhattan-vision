#!/bin/bash

OUTFILE=PrecomputedData/features.protodata;

# TODO: change frame_stride back (it is 100 to keep evaluation fast during testing)
LD_PRELOAD=/usr/lib/liblapack.so ./compute_payoff_features \
		--sequence lab_kitchen1 \
		--frame_stride 10 \
		--store_features \
		--output $OUTFILE \
		$*

# Report file size
du -hs $OUTFILE