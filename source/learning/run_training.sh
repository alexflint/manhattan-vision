#!/bin/sh

SVMLEARN=~/Code/svm-python-v204/svm_python_learn
LEARNMODULE=mm_manhattan

env LD_PRELOAD=/usr/lib/liblapack.so \
		PYTHONPATH=/home/alex/Code/indoor_context/build:. \
		MANHATTAN_CONFIG=/home/alex/Code/indoor_context/config/common.cfg \
		$SVMLEARN \
		-w 4 \
		--m $LEARNMODULE \
		-c 1 \
		blah out.model
